# mission_planner

The semantic mission planner: it turns a human/LLM-authored mission into sequential,
hierarchical, self-prompted navigation. It walks an ordered to-do of **areas** (bigger
tasks tied to a discrete environment) each holding ordered **steps** (concrete visual
instructions), hands each step's instruction to the existing
`trajectory_planner → waypoint_tracker → pursuit_servo` pipeline via the BT, and uses the
**reasoner** (`/reasoner_node/reason`, text-in / JSON-out) for the deliberation a
single-frame VLM check can't do — judging completion from a run log, revising an
instruction after a failure, and compressing a completed area into a summary.

> **Status.** The data contracts (mission format, structured log, prompt templates, brief)
> **and** the `mission_planner_node` that drives them are implemented and buildable. Still
> deferred to the next pass: the `hint_bt` leaves (`MissionNextStep` / `MissionRecordOutcome`)
> and the `run_mission.xml` loop tree — see [BT integration](#bt-integration-deferred). Until
> those land, drive the node directly with `ros2 action send_goal` (see [Test](#test)).

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
| `failed` | Gave up after exhausting best-effort re-planning; non-fatal — the mission advances |

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
It exposes two BT-facing action servers, meant to be called in a `next_step → execute →
record_outcome` loop.

Retries live **inside the node**: a step that the judge rules incomplete stays `active` and is
re-served by the next `next_step` — with its instruction revised via `replan.txt` — until it
either completes or hits `max_attempts`, at which point it is marked `failed` and skipped
(best-effort, never fatal). So the caller's loop is a dumb `next → execute → record` cycle;
the node handles all re-planning and advancement.

### Interfaces

| Interface | Type | Direction |
|---|---|---|
| `~/next_step` | `hint_interfaces/action/MissionStep` | Action server |
| `~/record_outcome` | `hint_interfaces/action/MissionOutcome` | Action server |
| `/reasoner_node/reason` (see `reasoner_action`) | `hint_interfaces/action/Reason` | Action client — judge / replan / compress |

**`next_step`** — hand out the next actionable step. Goal is empty. Result: `mission_done`,
`description` (the instruction to feed `trajectory_planner`), `area`, `step_id`, `message`.
On a retry it revises the instruction (`replan.txt`) before returning it.

**`record_outcome`** — Goal: `step_id`, `success`, `message` (execution detail), `observation`
(a grounded perception verbatim, e.g. a verify-VLM answer — optional). It logs the outcome,
judges completion (`judge.txt`), updates status, and compresses the area (`compress.txt`) on a
boundary. Result: `completed`, `mission_done`, `message` (judge reason).

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

Drive one step manually — get the next instruction, then report an outcome for it:

```bash
ros2 action send_goal /mission_planner_node/next_step hint_interfaces/action/MissionStep "{}" --feedback

ros2 action send_goal /mission_planner_node/record_outcome hint_interfaces/action/MissionOutcome \
  "{step_id: 'bedroom.1', success: true, message: 'reached last waypoint', observation: 'VLM: yes, the bed is beside the robot'}" \
  --feedback
```

The checkpoint and log are written next to the mission YAML — with the defaults, tail
`missions/apartment_tidy.state.yaml` and `missions/apartment_tidy.log.jsonl` in the installed
`share/mission_planner/` (or wherever `mission_path` points). Delete the `.state.yaml` to
restart the mission from scratch.

## BT integration (deferred)

The mission loop stays **visible in the BT** — the loop lives in XML ticked by
`bt_executor_node`; the leaves are thin `RosActionNode` wrappers over the node's actions,
matching every other `hint_bt` leaf:

- `MissionNextStep` (→ `~/next_step`) — outputs `{description}`, `{step_id}`; `FAILURE` when
  `mission_done` (breaks the loop).
- `MissionRecordOutcome` (→ `~/record_outcome`) — reports the executed step's outcome.

**`behaviors/run_mission.xml`** — the loop:

```
KeepRunningUntilFailure               # FAILURE from MissionNextStep = mission complete
  Sequence
    MissionNextStep      → {description}, {step_id}
    FollowPlannedTrajectory(description)     # existing subtree: plan + follow
    MissionRecordOutcome({step_id}, outcome) # records + judges; loop repeats
```

Since the node owns retries/advancement, the loop needs no `RetryUntilSuccessful` — it simply
repeats until `MissionNextStep` reports `mission_done`. Two things to settle in that pass:
capturing the `FollowPlannedTrajectory` success/failure to pass into `MissionRecordOutcome`
(even on failure), and mapping "mission complete" (the loop's terminal `FAILURE`) to an overall
`SUCCESS` at the top of the tree.
