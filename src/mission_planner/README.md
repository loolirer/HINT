# mission_planner

The semantic mission planner — a **narrative director**. It starts from a static **Semantic
Plan** (a prior: which environments to visit, in order, and a brief intent for each) and drives
navigation by maintaining a rolling **Narrative State** that it **recompiles every cycle** with
a single `reasoner` call: fold the last move's outcome (the trajectory planner's own VLM
reasoning) into the narrative, and emit the next instruction.

There are **no discrete steps, statuses, retries or pass/fail judging**. The old model outlined
a happy path and read every divergence as failure, forcing constant replanning; here divergence
is the normal material the narrative absorbs. Completion is a narrative judgment, not a checklist
exhausted.

> **Status.** Fully implemented and buildable: the data contracts, the `mission_planner_node`,
> and the `hint_bt` `RunMission` loop tree (a single `MissionAdvance` leaf) — see
> [BT integration](#bt-integration). You can also drive the node directly with
> `ros2 action send_goal` (see [Test](#test)).

## Layout

```
mission_planner/
  mission_planner/mission_planner_node.py  # the narrative-director node
  missions/apartment_tidy.yaml             # a Semantic Plan (the static prior)
  templates/compile.txt                    # the single narrative-compile prompt
  config/brief.md                          # permanent context: capabilities + rules + policy
  README.md                                # this file — single source of truth
```

```bash
colcon build --symlink-install --packages-select hint_interfaces mission_planner
source install/setup.bash
```

At runtime the node writes two append-only siblings next to the mission YAML:
`<mission>.narrative.jsonl` (the versioned narrative history) and `<mission>.log.jsonl` (the raw
action log). On startup it **resumes** from the tail of `<mission>.narrative.jsonl` if it exists;
delete that file to start the mission fresh.

## The loop, in one line

Per cycle there are **two** model calls with a clean division of labour:
- **trajectory_planner (vision):** the narrative's `next` + the image → waypoints **and** a
  reasoning `message` (what it saw / why it went there). That VLM reasoning *is* the grounding —
  no separate verify call.
- **reasoner (text):** one `compile.txt` call folds that reasoning into the narrative and emits
  the next instruction + completion.

The `reasoner` is the director (memory + intent); the trajectory planner is the actor-with-eyes.

## Data contract 1 — Semantic Plan (`missions/*.yaml`)

The static prior, authored once (by a human or an LLM). Environments are **hard rails**: visited
in order, never skipped or reordered. Each carries only a brief **intent** — not steps.

```yaml
mission: "<one-line mission statement>"
environments:
  - name: bedroom
    description: "A regular bedroom"     # initial belief — enriched as the robot explores
    intent: "Pass through the bedroom to the doorway into the living room, keeping to open floor."
  - name: living_room
    description: "A regular living room"
    intent: "Enter the living room and stop on the floor in front of the couch."
```

`description` is the **initial** visual belief for each environment. It is not just
documentation: each cycle the reasoner enriches the *current* environment's description with
what the planner observed (accumulating detail, frozen once the robot moves on). The enrichment
lives in the in-memory plan and the narrative snapshots — the **source YAML is never
mutated** — so a finished mission's narrative tail holds a learned visual map of every visited
environment.

## Data contract 2 — Narrative State (`<mission>.narrative.jsonl`)

The living memory, recompiled every cycle: prose, recency-weighted, lossy by design, and the
**primary context for the next plan** (`narrative.next` is fed to `trajectory_planner`).

It is stored as a **versioned, git-like history** — each recompile **appends a full snapshot**
(one JSON record per line) rather than overwriting, so the whole belief evolution is retained
and any version reconstructs directly. Each record embeds the outcome that *triggered* it, so
the history is self-explaining.

| Field | Value |
|---|---|
| `version` | monotonic snapshot index (0, 1, 2 …) |
| `ts` | ISO-8601 timestamp |
| `current_environment` | the environment being worked (advances only in plan order) |
| `mission_complete` | `true` once the last environment's intent is satisfied |
| `trigger` | `{success, observation}` that caused this recompile (`null` on version 0) |
| `environments` | `{name: description}` — the evolving per-environment visual beliefs at this version |
| `narrative.done` | recency-weighted history; older info abstracted, newer sharp |
| `narrative.trying` | present intent, reconciling the plan with new info |
| `narrative.next` | the immediate next instruction — fed to `trajectory_planner` |

```jsonl
{"version":0,"ts":"…","current_environment":"bedroom","mission_complete":false,"trigger":null,"environments":{"bedroom":"A regular bedroom","living_room":"A regular living room"},"narrative":{"done":"Nothing yet.","trying":"Enter the bedroom and head for the far doorway.","next":"Drive forward across the open floor toward the doorway on the far wall."}}
{"version":1,"ts":"…","current_environment":"bedroom","mission_complete":false,"trigger":{"success":true,"observation":"planner: a wall is directly ahead, no doorway visible; turning left to look"},"environments":{"bedroom":"A regular bedroom; a wall directly ahead, no doorway that way","living_room":"A regular living room"},"narrative":{"done":"Drove forward and met a wall — no doorway that way.","trying":"Find the doorway by looking left.","next":"Turn toward the left side of the room and drive along the open floor."}}
```

- **Current state** = the last record. **Resume** = read the tail, continue the `version` counter.
- **Reconstruct history** = read records `0..N` (e.g. `jq . <mission>.narrative.jsonl`, or diff
  consecutive `narrative` objects to see how the belief changed at each step).

## Data contract 3 — Pure Log (`<mission>.log.jsonl`)

Every move's raw outcome, append-only, one JSON object per line
(`{ts, event, result, observation}`) — the debug trail **and** the per-cycle increment folded
into the narrative. Kept distinct from the narrative history: this is the *raw actions*, that is
the *compiled belief*.

## Prompt template (`templates/compile.txt`)

The single reasoner prompt (replacing the old judge/replan/compress). Placeholders are literal
`{name}` tokens the node substitutes (not `str.format` — the body has JSON braces):

| Token | Filled with |
|---|---|
| `{brief}` | `config/brief.md`, verbatim (permanent context) |
| `{semantic_plan}` | the environments (hard rails, in order) + their **evolving** descriptions + intents |
| `{narrative}` | the current narrative (`current_environment` + `done`/`trying`/`next`) |
| `{outcome}` | this cycle's move outcome + the planner's VLM reasoning (empty on cycle 0) |

Reasoner `schema`:
`{"done": string, "trying": string, "next": string, "current_environment": string, "environment_description": string, "mission_complete": bool}`
— `environment_description` is the enriched description of the current environment, folded back
into the plan each cycle.

`config/brief.md` is the permanent-context prefix (capabilities, navigation preferences, ambiguity
policy) prepended on every call.

## Node

`mission_planner_node` loads the Semantic Plan, `brief.md`, and `compile.txt`; holds the current
narrative in memory; and exposes **one** action server, `~/advance`, called in an
`advance → execute` loop. Each `advance` is one reasoner call.

### Interfaces

| Interface | Type | Direction |
|---|---|---|
| `~/advance` | `hint_interfaces/action/MissionAdvance` | Action server |
| `/reasoner_node/reason` (see `reasoner_action`) | `hint_interfaces/action/Reason` | Action client — the narrative recompile |

**`advance`** — Goal: `success` (did the last move execute?) + `observation` (the planner's VLM
reasoning, verbatim). The node appends the outcome to the pure log, recompiles the narrative
(`compile.txt` → reasoner), appends the new snapshot to `<mission>.narrative.jsonl`, and returns
Result: `mission_done` (the narrative's `mission_complete`), `description` (= `narrative.next`),
`area` (= `current_environment`), `message` (= `narrative.trying`). On the first call nothing has
executed (`success` defaults true, `observation` empty) so it just emits the opening instruction.

### Parameters

| Parameter | Default | Effect |
|---|---|---|
| `mission_path` | share `missions/apartment_tidy.yaml` | Semantic Plan to run |
| `brief_path` | share `config/brief.md` | Permanent-context brief |
| `templates_dir` | share `templates/` | Directory holding `compile.txt` |
| `narrative_path` | `""` | Narrative history; empty → `<mission>.narrative.jsonl` sibling |
| `log_path` | `""` | Raw log; empty → `<mission>.log.jsonl` sibling |
| `reasoner_action` | `/reasoner_node/reason` | Reasoner action name |
| `reasoner_timeout` | `30.0` | Seconds to wait on the reasoner call |

### Test

Run the node (the `reasoner` node must be up):

```bash
ros2 run mission_planner mission_planner_node
```

Drive the loop manually. The first call reports nothing and returns the opening instruction;
each subsequent call reports the previous move's outcome (with the planner's reasoning) and
returns the next:

```bash
# cycle 0 — nothing executed yet
ros2 action send_goal /mission_planner_node/advance hint_interfaces/action/MissionAdvance \
  "{success: true, observation: ''}" --feedback

# report the move + get the next
ros2 action send_goal /mission_planner_node/advance hint_interfaces/action/MissionAdvance \
  "{success: true, observation: 'planner: routed to the far wall, a doorway is now visible ahead'}" \
  --feedback
```

Watch the narrative evolve — every call appends one snapshot:

```bash
tail -f missions/apartment_tidy.narrative.jsonl | jq .
```

Delete the `.narrative.jsonl` to restart the mission from scratch.

## BT integration

The loop is **visible in the BT** — `hint_bt/behaviors/run_mission.xml` (tree ID `RunMission`),
ticked by `bt_executor_node`. A **single** thin leaf, `MissionAdvance` (→ `~/advance`), is the
whole node interface: each tick it reports the last move's outcome and returns the next directive.

```
Fallback
  KeepRunningUntilFailure                       # ends when MissionAdvance reports mission_complete
    Sequence
      MissionAdvance(success={last_ok}, observation={plan_message}) → {description}, {area}
      Fallback                                   # capture follow success/failure into {last_ok}
        Sequence: FollowPlannedTrajectory(description)→{plan_message} ; SetBlackboard(last_ok := true)
        SetBlackboard(last_ok := false)
  AlwaysSuccess                                  # completion → overall SUCCESS
```

The planner's VLM reasoning (`{plan_message}`) is fed straight into `MissionAdvance`'s
`observation`, so the narrative reasons over grounded feedback. There is **no abort path** —
divergence is absorbed by the narrative, so the loop only ends on completion, and the outer
`AlwaysSuccess` maps that to overall `SUCCESS`. The only custom term in the loop is
`MissionAdvance`; `Fallback`/`KeepRunningUntilFailure`/`SetBlackboard`/`AlwaysSuccess` are stock
BT.cpp.

> **Future refinements (deferred):** the trajectory planner returning *no* waypoints as an
> explicit "arrived" signal, and a scan/rotate primitive so the planner can look where the goal
> isn't currently in view (the current forward-arc action space can't turn around). Movement work
> is out of scope for now; completion is inferred from the planner's reasoning.
