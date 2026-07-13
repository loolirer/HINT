# mission_planner

The semantic mission planner: it turns a human/LLM-authored mission into sequential,
hierarchical, self-prompted navigation. It walks an ordered to-do of **areas** (bigger
tasks tied to a discrete environment) each holding ordered **steps** (concrete visual
instructions), hands each step's instruction to the existing
`trajectory_planner → waypoint_tracker → pursuit_servo` pipeline via the BT, and uses the
**reasoner** (`/reasoner_node/reason`, text-in / JSON-out) for the deliberation a
single-frame VLM check can't do — judging completion from a run log, revising an
instruction after a failure, and compressing a completed area into a summary.

> **Status.** Fully implemented and buildable: the data contracts (mission format, structured
> log, prompt templates, brief), the `mission_planner_node` that drives them, and the `hint_bt`
> `RunMission` loop tree (a single `MissionAdvance` leaf) — see
> [BT integration](#bt-integration). You can also drive the node directly with
> `ros2 action send_goal` (see [Test](#test)).

## Layout

```
mission_planner/
  mission_planner/mission_planner_node.py  # the state-authority node
  missions/apartment_tidy.yaml             # worked-example mission (schema by example)
  templates/judge.txt                      # completion-judgement prompt template
  templates/replan.txt                     # failure-replan prompt template
  templates/compress.txt                   # area-summary prompt template
  config/brief.md                          # permanent context: capabilities + rules + ambiguity policy
  README.md                                # this file — single source of truth
```

```bash
colcon build --symlink-install --packages-select hint_interfaces mission_planner
source install/setup.bash
```

At runtime the node writes two sibling files next to the mission YAML: `<mission>.state.yaml`
(the live checkpoint — status/attempts/result/summary) and `<mission>.log.jsonl` (the
structured log). On startup it **resumes** from `<mission>.state.yaml` if it exists; delete
that file to start the mission fresh.

## Mission file (`missions/*.yaml`)

Hierarchy: `mission → areas[] → steps[]`. An **area** is a bigger task bound to a discrete
environment (a room); a **step** is a small, concrete visual instruction fed *verbatim* to
`trajectory_planner` as its `description`. Order is authoritative — areas are visited in
order, steps within an area in order.

| Field | Level | Type | Meaning |
|---|---|---|---|
| `mission` | root | string | One-line mission statement; permanent context on every reasoner call |
| `id` | area / step | string | Stable identifier (`step.id` conventionally `"<area>.<n>"`) |
| `goal` | area | string | What "done" means for the area — the judge's target |
| `instruction` | step | string | The actionable path description handed to `trajectory_planner` |
| `verify` | step | string | Optional yes/no the judge grounds on; may cite a logged VLM answer (empty = judge on the outcome alone) |
| `status` | area / step | enum | `pending` \| `active` \| `done` \| `failed` |
| `attempts` | step | int | Best-effort re-plan counter (incremented on each failed try) |
| `result` | step | string | Last outcome message |
| `summary` | area | string | Filled by the compress step when the area completes |

**Status semantics**

| Status | Meaning |
|---|---|
| `pending` | Not yet reached |
| `active` | Currently being worked (set when handed to the executor) |
| `done` | Completed and verified — **kept, never deleted** ("done but not forgotten") |
| `failed` | Gave up after exhausting best-effort re-planning — **aborts the whole mission** (its area is marked `failed` too) |

**Persistence / resume.** The node writes `status` / `attempts` / `result` / `summary` back
to this file (or a sibling `*.state.yaml`) after every transition, so the file doubles as the
checkpoint: a restart resumes where it left off, and the retained `done` entries *are* the
"not forgotten" memory. (Write path is the node pass; here the fields are defined.)

## Structured log (`<mission>.log.jsonl`)

Append-only, one JSON object per line — machine-parseable so the judge reads it reliably and
the compress step rolls it up. Perception stays in the VLM nodes; their grounded outputs land
here **verbatim** in `observation`, and the reasoner reasons *over* the log, it does not
re-perceive.

| Field | Type | Value |
|---|---|---|
| `ts` | string | ISO-8601 timestamp |
| `area` | string | Area id |
| `step_id` | string | Step id |
| `event` | enum | `plan` \| `follow` \| `judge` \| `replan` \| `compress` |
| `prompt` | string | What was sent for this event (instruction / reasoner prompt), if any |
| `result` | string | Action result / verdict / message |
| `observation` | string | Any VLM answer or description, verbatim (empty if none) |
| `status` | string | The step/area status *after* this event |

```jsonl
{"ts":"2026-07-13T10:00:00Z","area":"bedroom","step_id":"bedroom.1","event":"plan","prompt":"Walk to the foot of the bed...","result":"planned 4 waypoints","observation":"","status":"active"}
{"ts":"2026-07-13T10:00:31Z","area":"bedroom","step_id":"bedroom.1","event":"follow","prompt":"","result":"reached last waypoint","observation":"","status":"active"}
{"ts":"2026-07-13T10:00:34Z","area":"bedroom","step_id":"bedroom.1","event":"judge","prompt":"","result":"completed=true","observation":"VLM: yes, the bed is beside the robot","status":"done"}
```

## Prompt templates (`templates/*.txt`)

Prompts are **data, not code** — the node fills them and calls the reasoner, so they iterate
without a rebuild. Placeholders are **literal `{name}` tokens replaced by substring
substitution** (not Python `str.format`), so the JSON braces inside the templates are safe.
Each file's header comment names its tokens and the exact JSON `schema` string the node passes
to the reasoner's `schema` field.

| Template | Purpose | Tokens | Reasoner `schema` |
|---|---|---|---|
| `judge.txt` | Did the step complete? | `brief`, `mission`, `area_goal`, `step_instruction`, `step_verify`, `outcome`, `log` | `{"completed": bool, "reason": string, "summary": string}` |
| `replan.txt` | Revise a failed instruction | `brief`, `mission`, `area_goal`, `step_instruction`, `log` | `{"revised_instruction": string, "reason": string}` |
| `compress.txt` | Summarize a finished area | `brief`, `mission`, `area_goal`, `log` | `{"summary": string}` |

`{brief}` is `config/brief.md` verbatim, present in every template — the permanent context.

## Robot brief (`config/brief.md`)

The permanent context prepended to every reasoner call. Three sections: **Capabilities** (the
actions the robot actually has and their limits), **Semantic navigation rules** (keep to open
floor, honor preferences, doorways bound areas…), and an **actionable Ambiguity policy** (each
ambiguity maps to a concrete move — e.g. unsure a step completed → `completed:false` so it
re-plans; under-specified instruction → take the conservative open-floor path — never
prose-only).

## Node

`mission_planner_node` is the state authority. It loads the mission YAML, `brief.md`, and the
templates; owns the status hierarchy, the log, and area compression; delegates every LLM job
to `/reasoner_node/reason` (holding no `genai`/API key); and persists after every transition.
It exposes **one** BT-facing action server, `~/advance`, called in a `advance → execute` loop.

`advance` is a **report-and-advance** step: the caller reports the outcome of the step it just
executed, the node judges that step, then returns the next directive — one round-trip per step
instead of a separate `next_step` + `record_outcome`.

Retries live **inside the node**: a step that the judge rules incomplete stays `active` and is
re-served by the next `advance` — with its instruction revised via `replan.txt` — until it
either completes or hits `max_attempts`, at which point it is marked `failed`. **A failed step
aborts the whole mission**: its area is marked `failed`, `advance` refuses to advance past it
and reports `mission_failed`, and no further steps are served. So the caller's loop is a dumb
`advance → execute` cycle; the node handles judging, re-planning, advancement, and the abort.

### Interfaces

| Interface | Type | Direction |
|---|---|---|
| `~/advance` | `hint_interfaces/action/MissionAdvance` | Action server |
| `/reasoner_node/reason` (see `reasoner_action`) | `hint_interfaces/action/Reason` | Action client — judge / replan / compress |

**`advance`** — Goal: `success` (did the step just executed succeed?) and `observation`
(grounded VLM feedback, verbatim — e.g. the planner's path reasoning). The node judges the
active step from that outcome (`judge.txt`), logs it, updates status, compresses the area
(`compress.txt`) on a boundary, and aborts on a failed step; then it selects the next step
(revising the instruction via `replan.txt` on a retry). Result: `mission_done`, `mission_failed`,
`description` (next instruction for `trajectory_planner`), `area`, `step_id`, `message`. On the
first call nothing has executed (`success` defaults true, `observation` empty), so it just
hands out the first step.

> **Feeding the judge grounded feedback.** The judge is only as good as the `observation` it
> gets. In the `RunMission` tree, `PlanTrajectoryAction` exposes the trajectory planner's VLM
> reasoning about the chosen path (its result `message`), which is routed into `advance`'s
> `observation` — so the judge sees *why the planner went where it did*, not just "the follow
> reached its last waypoint". Any VLM leaf's reasoning field (a `VisualQuestion` `rationale`, a
> `GroundDescription` label) can be routed the same way; a dedicated verify-VLM leaf is the
> natural next addition (see the note below).

### Parameters

| Parameter | Default | Effect |
|---|---|---|
| `mission_path` | share `missions/apartment_tidy.yaml` | Mission YAML to run |
| `brief_path` | share `config/brief.md` | Permanent-context brief |
| `templates_dir` | share `templates/` | Directory of the three prompt templates |
| `state_path` | `""` | Checkpoint file; empty → `<mission>.state.yaml` sibling |
| `log_path` | `""` | Log file; empty → `<mission>.log.jsonl` sibling |
| `reasoner_action` | `/reasoner_node/reason` | Reasoner action name |
| `reasoner_timeout` | `30.0` | Seconds to wait on each reasoner phase |
| `max_attempts` | `3` | Best-effort re-plan cap before a step is marked `failed` |

### Test

Run the node (the `reasoner` node must be up for judge/replan/compress to resolve):

```bash
ros2 run mission_planner mission_planner_node
```

Drive the loop manually. The first call reports nothing and returns step one; each subsequent
call reports the previous step's outcome and returns the next directive:

```bash
# first call — just get step one (nothing executed yet)
ros2 action send_goal /mission_planner_node/advance hint_interfaces/action/MissionAdvance \
  "{success: true, observation: ''}" --feedback

# report that step's outcome + get the next
ros2 action send_goal /mission_planner_node/advance hint_interfaces/action/MissionAdvance \
  "{success: true, observation: 'planner: routed along the open floor to the foot of the bed'}" \
  --feedback
```

The checkpoint and log are written next to the mission YAML — with the defaults, tail
`missions/apartment_tidy.state.yaml` and `missions/apartment_tidy.log.jsonl` in the installed
`share/mission_planner/` (or wherever `mission_path` points). Delete the `.state.yaml` to
restart the mission from scratch.

## BT integration

The mission loop is **visible in the BT** — it lives in XML ticked by `bt_executor_node`
(`hint_bt/behaviors/run_mission.xml`, tree ID `RunMission`). A **single** thin leaf,
`MissionAdvance` (→ `~/advance`), is the whole node interface: each tick it reports the last
step's outcome and returns the next directive, writing `{description}`/`{area}`/`{step_id}`/
`{mission_failed}` to the blackboard and returning `FAILURE` when the mission is over.

```
Fallback
  KeepRunningUntilFailure                        # ends when MissionAdvance reports mission over
    Sequence
      MissionAdvance(success={last_ok}, observation={plan_message}) → {description}, {mission_failed}
      Fallback                                    # capture follow success/failure into {last_ok}
        Sequence: FollowPlannedTrajectory(description)→{plan_message} ; SetBlackboard(last_ok := true)
        SetBlackboard(last_ok := false)
  Precondition if="mission_failed" else="SUCCESS" # abort → overall FAILURE, clean → SUCCESS
    AlwaysFailure
```

Since the node owns judging/retries/advancement/abort, the loop needs no `RetryUntilSuccessful`
— it repeats until `MissionAdvance` ends it. The only custom term in the loop is `MissionAdvance`;
`Fallback`/`Precondition`/`SetBlackboard`/`AlwaysFailure` are all stock BT.cpp. The planner's VLM
path reasoning (`{plan_message}`) is fed straight into `MissionAdvance`'s `observation`, so it
lands in the log as the grounded feedback the judge reasons over; the `SetBlackboard` pair
captures whether the follow succeeded (`{last_ok}`) for the next tick. The outer `Precondition`
maps a clean finish to overall `SUCCESS` and an abort (`{mission_failed}`) to `FAILURE`.
