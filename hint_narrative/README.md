# hint_narrative

The semantic mission planner — a **narrative director**. It starts from a static **Semantic
Plan** (a prior: which environments to visit, in order, and a brief intent for each) and drives
navigation by maintaining a rolling **Narrative State** that it **recompiles every cycle** with
a single `visual_reasoner` call: judge the last move from the before/after camera frames, fold that into
the narrative, and emit the next instruction.

There are **no discrete steps, statuses, retries or pass/fail judging**. The old model outlined
a happy path and read every divergence as failure, forcing constant replanning; here divergence
is the normal material the narrative absorbs.

**The environment queue.** Order is owned by *code*, not the model. The node holds a FIFO queue
of environments (head = current) plus a stack of visited ones. The model never names or reorders
environments — each cycle it emits `environment_action` ∈ `{stay, advance, back, insert}` and the
node applies it: `advance` pops the head (its intent is met), `back` restores the previous head
(advanced too early), `insert` splices a discovered intermediate the plan omitted (e.g. a corridor
between two rooms) in as the next environment, `stay` keeps working the head. This enforces the
plan order while letting reality *refine* it, keeps the prompt bounded (only the head + a one-line
peek are ever shown, regardless of queue length), and makes completion **code-derived** (the queue
empties). A per-environment cycle cap (`max_env_cycles`) is the one **failure** path: if the robot
never leaves a head within the cap, the mission fails (mapped to BT `FAILURE`) rather than dragging
on forever.

> **Status.** Fully implemented and buildable: the data contracts, the `narrative_navigation`,
> and the `hint_behavior` `RunMission` loop tree (a single `MissionAdvance` leaf) — see
> [BT integration](#bt-integration). You can also drive the node directly with
> `ros2 action send_goal` (see [Test](#test)).

## Layout

```
hint_narrative/
  hint_narrative/narrative_navigation.py  # the narrative-director node
  missions/<name>/mission.yaml             # a Semantic Plan (one dir per mission)
  prompts/compile.txt                      # the single narrative-compile prompt
  prompts/brief.txt                        # permanent context: capabilities + rules + policy
  README.md                                # this file — single source of truth
```

```bash
colcon build --symlink-install --packages-select hint_interfaces hint_narrative
source install/setup.bash
```

**There is no default mission.** The node starts **idle** and runs whichever mission a `~/mission_advance`
goal points it at (`mission_path`) — one running node serves any mission without a restart. (You
*can* preload one with the `mission_path` param, but that's optional.) Author missions with the
`/author-mission` command.

Each mission lives in its **own directory** (`missions/<name>/mission.yaml`) so its runtime
artifacts stay grouped with it. The node writes two append-only siblings next to the mission
YAML: `mission.narrative.jsonl` (the versioned narrative history) and `mission.log.jsonl` (the raw
action log). When it loads a mission it **resumes** from the tail of that mission's narrative if it
exists; delete that file to start fresh.

**Missions never resume across runs.** Every new tree run starts clean. The **first** `~/mission_advance` of
a run carries `first: true` — an **explicit** run-boundary flag the `MissionAdvance` BT leaf latches
on its first tick of the run (the leaf instance is rebuilt per `ExecuteTree` goal, so `first` is true
exactly once per run, false thereafter). On it the node **deletes** any existing `.narrative.jsonl` +
`.log.jsonl` and reseeds from the plan. This holds no matter how the previous run ended — clean
completion, failure, or a premature **Ctrl+C** mid-run — because the flag keys on the fresh run's
first tick, not on the old run's ending. The previous run's files persist *until* you launch the next
run, so they stay there for debugging in between; they're wiped only when a new run actually begins.

> Earlier this run-boundary was *inferred* from an empty report, but a mid-run move can legitimately
> report nothing, and that inference could then wipe a **live** mission mid-run (and livelock
> re-seeding). The signal is now the explicit `first` goal field, so a genuine empty report is never
> mistaken for a new run. Driving `~/mission_advance` by hand? Pass `first: true` on your first call
> (or just delete the `.narrative.jsonl`) to restart.

The siblings are written next to the **real** mission file: `os.path.realpath` resolves the
`--symlink-install` symlink back to the source tree, so in a dev workspace they appear in
`src/hint_narrative/missions/<name>/` (editor-visible); on a plain copied install they sit
beside the installed mission. They are `.gitignore`d. (Set `narrative_path`/`log_path` to
redirect them anywhere else.)

## The loop, in one line

This node is the mission's **cognition** layer: **one `~/mission_advance` cycle makes BOTH
per-cycle VLM calls** over the one frame buffer it owns, and returns a ready-to-drive
**trajectory** to the BT. The BT is then a thin executive over motor skills — it only reports
whether the last move `success`-ed, and drives the returned trajectory (`FollowVisualPath` +
`Spin`). There is no `PlanVisualPath` BT leaf; planning lives here.

Per cycle, in order:
- **reasoner (director-with-eyes):** one `compile.txt` call (`visual_reasoner`) that reasons
  over the **before/after frames of the move just executed** plus the narrative — it judges the
  move from the images, folds it in, and emits the next instruction (`next`) + completion.
- **path_planner (executor-with-eyes):** the narrative's `next` + the **same** frame buffer →
  ordered `markers` + a signed `turn_degrees` + a short reasoning `message`. This node is now
  `path_planner`'s **client** (mirroring the reasoner relationship), so both cognition calls run
  over one buffer with no cross-process routing — the trajectory rides out on the `advance`
  result (`markers` / `turn_degrees` / `stamp`).

If the compile reports the mission complete/stuck, no plan is made and `mission_done` is
returned. Both calls share the **retry-and-wait** resilience: a transient reasoner/planner/API
failure retries with backoff (the robot stays put in `RUNNING`), and only a *bounded, exhausted*
budget fails the mission (`compile_retries`, which now governs both). The planner's `message`
becomes the next cycle's trigger `observation` internally — it is **not** round-tripped through
the BT.

**Frame buffer (`history_frames` = N):** the node subscribes to the camera and latches the
current view each time it serves a move (a *move-start* frame), keeping a rolling buffer of the
last **N**. Each cycle it attaches those N past frames + the current view (oldest → current) to
**both** VLM calls, so the director sees the *sequence* of its recent views (judging moves against
ground truth — closing the old **one-action-behind lag**) and the planner plans over exactly the
same frames. `N=1` is the before/after pair (default), `N=0` is current-view-only, `N=3–4` is
deeper history (more image tokens = more latency/cost). `history_frames` is the single knob
governing frame depth for both calls. Only *images* are buffered — the director's text memory
(`done`) already carries the narrative history.

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
**primary context for the next plan** (`narrative.next` is fed to `path_planner`).

It is stored as a **versioned, git-like history** — each recompile **appends a full snapshot**
(one JSON record per line) rather than overwriting, so the whole belief evolution is retained
and any version reconstructs directly. Each record embeds the outcome that *triggered* it, so
the history is self-explaining.

| Field | Value |
|---|---|
| `version` | monotonic snapshot index (0, 1, 2 …) |
| `ts` | ISO-8601 timestamp |
| `current_environment` | the queue head's name (`""` once complete) — **derived**, not model-named |
| `mission_complete` | `true` once the queue is empty — **derived** |
| `mission_failed` | `true` when the mission ended stuck (a head exceeded `max_env_cycles`) |
| `action` | the queue edit applied this cycle: `stay` / `advance` / `back` / `insert` / `fail` |
| `trigger` | `{success, observation}` that caused this recompile (`null` on version 0) |
| `queue` | remaining environments `[{name, description, intent}, …]` — order shows any `insert`s |
| `visited` | completed environments (their **enriched** descriptions — the learned map) |
| `narrative.situation` | my current standing — where I am / what I face **right now**, rewritten fresh each cycle (the present moment, not history) |
| `narrative.done` | recency-weighted history; older info abstracted, newer sharp |
| `narrative.next` | the immediate next instruction — fed to `path_planner` |

```jsonl
{"version":1,"ts":"…","current_environment":"bedroom","mission_complete":false,"action":"insert","trigger":{"success":true,"observation":"planner: the only door leads to a hallway, not the living room"},"queue":[{"name":"bedroom","description":"a bedroom; door on the far wall opens to a hallway","intent":"pass through to the living room"},{"name":"corridor","description":"a hallway linking the rooms","intent":""},{"name":"living_room","description":"A regular living room","intent":"stop in front of the couch"}],"visited":[],"narrative":{"situation":"I am in the bedroom, facing the far wall; a door to a hallway is directly ahead.","done":"Found the bedroom's only exit is a hallway.","next":"Drive to the doorway on the far wall."}}
```

- **Current state** = the last record. **Resume** = read the tail (`queue` + `visited` restore the
  full plan state, including inserts), continue the `version` counter.
- **Reconstruct history** = read records `0..N` (e.g. `jq . <mission>.narrative.jsonl`) — the
  `action` + `queue` fields show exactly how and when the plan was refined (e.g. a corridor spliced in).

## Data contract 3 — Pure Log (`<mission>.log.jsonl`)

Every move's raw outcome, append-only, one JSON object per line
(`{ts, event, result, observation}`) — the debug trail **and** the per-cycle increment folded
into the narrative. Kept distinct from the narrative history: this is the *raw actions*, that is
the *compiled belief*.

## Mission rosbag + report (`<mission>.bag`, `mission_report.png`)

Every run auto-records a **minimal MCAP rosbag** (`<mission>.bag`, a sibling of the mission YAML),
started on a fresh run and closed when the mission ends (or on Ctrl+C). It is **overwritten each
run**, mirroring the jsonl semantics. The topic set is deliberately small (no images / clouds /
costmap — just `/tf`, `/tf_static`, `/odom`, the projector's followed path (`~/path`), `/map` if
present, and the `visual_reason` / `plan_visual_path` / `advance` / `follow_visual_path` / `spin`
`_action/status` topics). Toggle with the `record_bag` param; relocate with `bag_path`. Recording is a controlled
`ros2 bag record --storage mcap` subprocess (needs `ros-<distro>-rosbag2-storage-mcap`).

The offline **`mission_report`** script compiles that bag into one annotated figure plus stats,
written **into the mission's directory** (`mission_report.png` + `mission_stats.json`):

```bash
ros2 run hint_narrative mission_report missions/<name>/mission.bag
```

Reference frame is `map` if the bag has one, else `odom`. The figure shows the **actual** driven
path (continuous), the **truncated** (followed) path, the robot **footprint** (circle + heading)
drawn **only at VLM plan-call poses** and labelled with the **stop's sequence index** (0-based, in
visiting order), start/end markers, each **turn** as a post-spin heading line, and a stats box:
mission duration, VLM-processing time (the `advance` wrapper window — the whole stationary-cognition
span per cycle), movement time (`follow_visual_path` + `spin` windows), and the **VLM hit rate**
(successful/total of the real VLM calls — the `visual_reason` compile + `plan_visual_path` plan,
each by its `_action/status` terminal status, retries counted as separate calls). All displayed
paths share one opacity (`PATH_ALPHA`). Everything is derived from the recorded topics, so no
runtime node is touched.

## Prompt template (`prompts/compile.txt`)

The single reasoner prompt (replacing the old judge/replan/compress). Placeholders are literal
`{name}` tokens the node substitutes (not `str.format` — the body has JSON braces):

| Token | Filled with |
|---|---|
| `{brief}` | `prompts/brief.txt`, verbatim (permanent context) |
| `{environment}` | the **current** environment (name/description/intent) + a one-line peek at the next — bounded regardless of queue length |
| `{situation}` | last cycle's `situation` — my standing after the previous move (continuity for the fresh rewrite) |
| `{narrative}` | the memory carried forward — `done` only (`next` is regenerated; its result is read from the before/after images) |
| `{vision}` | how to read the attached camera image(s): the last is the current view, earlier ones are recent past views (buffer depth `history_frames`); one = current-only, none = no frame |

> The before/after frames themselves are **attached to the reasoner call** (the `VisualReason` goal's
> `images`), not substituted into the prompt text; `{vision}` is the caption that tells the model how
> to read them.

Reasoner `schema` — a **real JSON schema** (`NARRATIVE_SCHEMA`) passed to the reasoner, which uses it
as `response_schema` for **constrained decoding**, so the compile reply is always well-formed JSON
with exactly these fields:
`{situation, done, next, environment_description, environment_action ∈ {stay|advance|back|insert} (enum), new_environment: {name, description}}`
— the model reports its current `situation` + progress + enriches the current description, and edits the queue only via
`environment_action` (`new_environment` carries the room to splice on `insert`). It never names
the current environment; that's the queue head, owned by the node.

`prompts/brief.txt` is the permanent-context prefix (capabilities, navigation preferences, ambiguity
policy) prepended on every call.

## Node

`narrative_navigation` loads `brief.txt` and `compile.txt` at startup and then **waits idle** for a
mission. It holds the current narrative in memory and exposes **one** action server, `~/mission_advance`,
called in an `advance → execute` loop. Each `advance` makes **two VLM calls** — the narrative compile
(`visual_reasoner`) and the path plan (`path_planner`) — and returns the next move as a trajectory. The
first `~/mission_advance` that carries a `mission_path` loads that mission (and later ones switch it); an
`advance` with no mission loaded and none provided fails cleanly (`mission_failed`).

### Interfaces

| Interface | Type | Direction |
|---|---|---|
| `~/mission_advance` | `hint_interfaces/action/MissionAdvance` | Action server |
| `/visual_reasoner/visual_reason` (see `reasoner_action`) | `hint_interfaces/action/VisualReason` | Action client — the narrative recompile (with before/after frames) |
| `/path_planner/plan_visual_path` (see `planner_action`) | `hint_interfaces/action/PlanVisualPath` | Action client — the path plan (`next` + the same frame buffer → `markers` + `turn_degrees`) |
| `/camera/image_raw/compressed` (see `camera_topic`) | `sensor_msgs/CompressedImage` | Sub — latest frame; latched into the **unified** rolling image buffer (`history_frames`), attached to **both** VLM calls |

**`advance`** — Goal: `success` (did the last move execute?), `mission_path` (optional — the mission
to run; loads/switches it when it changes, else keeps the current one), and `first` (true only on the
run's first tick → wipe + reseed; see **Missions never resume across runs** above). The node appends
the outcome to the pure log, recompiles the narrative (`compile.txt` → reasoner) and applies the queue
edit, then — if the mission is not over — **plans the move** (`next` + the frame buffer → `path_planner`),
appends **one** snapshot for the cycle, and returns Result:
`mission_done` (queue empty **or** failed), `mission_failed` (stuck past the cap, an unrecoverable
compile/IO error, or a cognition call failed past its retry budget), `area` (= the current queue head),
`markers` / `turn_degrees` / `stamp` (the trajectory to drive — `markers` may be empty for a turn-only
move; empty/zero when `mission_done`), and `message` (the planner's brief path reasoning, or the
failure/closing text on `mission_done`). The planner's reasoning is recorded internally as the next
cycle's trigger `observation` — no longer round-tripped through the BT. On the first call nothing has
executed (`success` defaults true) so it just plans and serves the opening move.

### Parameters

| Parameter | Default | Effect |
|---|---|---|
| `mission_path` | `""` | Optional mission to preload at startup; empty → start **idle**. A `~/mission_advance` goal's `mission_path` selects/switches the mission per call (the node reloads on change, resuming that mission's narrative if it exists), so one running node serves any mission without a restart |
| `brief_path` | share `prompts/brief.txt` | Permanent-context brief |
| `prompts_dir` | share `prompts/` | Directory holding `compile.txt` |
| `narrative_path` | `""` | Narrative history; empty → sibling of the real mission file (`<mission>.narrative.jsonl`) |
| `log_path` | `""` | Raw log; empty → sibling of the real mission file (`<mission>.log.jsonl`) |
| `record_bag` | `true` | Auto-record the per-run minimal MCAP rosbag (start on a fresh run, close on mission end). Set `false` to disable (e.g. tests) |
| `bag_path` | `""` | Rosbag output dir; empty → sibling of the real mission file (`<mission>.bag`). Overwritten each fresh run |
| `reasoner_action` | `/visual_reasoner/visual_reason` | Reasoner (director) action name |
| `reasoner_timeout` | `30.0` | Seconds to wait on a single reasoner call |
| `planner_action` | `/path_planner/plan_visual_path` | Path-planner (executor) action name |
| `planner_timeout` | `30.0` | Seconds to wait on a single planner call |
| `compile_retries` | `-1` | Cognition-retry budget — governs **both** per-cycle VLM calls (compile **and** plan). On a reasoner/planner/API failure the move did **not** advance, so instead of driving on a stale belief / no plan the `~/mission_advance` call **waits and retries** — the robot stays put (the BT leaf sits in `RUNNING`; the follow only runs once advance returns). Retries after the first attempt: **`-1` = retry indefinitely** until it succeeds or the BT halts; `0` = one attempt; `N` = N retries. On a *bounded* budget being exhausted the mission **aborts** (`mission_failed` → BT `FAILURE`), never re-serving. Live-adjustable |
| `compile_retry_delay` | `2.0` | Backoff (s) between cognition retries (see `compile_retries`). The wait is cancellable — a BT halt / Ctrl+C breaks out immediately |
| `max_env_cycles` | `8` | Cycles on one environment before the mission fails (stuck backstop) |
| `camera_topic` | `/camera/image_raw/compressed` | Frame source for the director's image history |
| `history_frames` | `1` | Director vision-buffer depth N — how many past move-start frames to attach ahead of the current view. `0` = current view only (no move comparison), `1` = before/after of the last move, `3–4` = deeper history. Each extra frame adds image tokens → more latency/cost. Live-adjustable. Only images are buffered; the narrative (`done`) carries the text history |

### Test

Run the node (both the `visual_reasoner` **and** `path_planner` nodes must be up, since a cycle
calls each; the planner also needs camera frames arriving on `camera_topic`):

```bash
ros2 run hint_narrative narrative_navigation
```

Drive the loop manually. The first call plans and returns the opening move; each subsequent call
reports whether the previous move succeeded and returns the next trajectory
(`markers` / `turn_degrees` / `stamp`):

```bash
# cycle 0 — load a mission (via mission_path); first:true wipes any prior narrative
# and reseeds, then plans + returns the opening move
ros2 action send_goal /narrative_navigation/mission_advance hint_interfaces/action/MissionAdvance \
  "{success: true, first: true, mission_path: '/root/turtlebot3_ws/src/hint_narrative/missions/bedroom_to_living_room/mission.yaml'}" \
  --feedback

# report the move + get the next (first defaults false; mission_path can be omitted once loaded)
ros2 action send_goal /narrative_navigation/mission_advance hint_interfaces/action/MissionAdvance \
  "{success: true}" \
  --feedback
```

Watch the narrative evolve — every call appends one snapshot:

```bash
tail -f src/hint_narrative/missions/bedroom_to_living_room/mission.narrative.jsonl | jq .
```

Delete the `.narrative.jsonl` to restart the mission from scratch.

## BT integration

The loop is **visible in the BT** — `hint_behavior/behaviors/run_mission.xml` (tree ID `RunMission`),
ticked by `behavior_server`. `MissionAdvance` (→ `~/mission_advance`) is the whole **cognition**
interface: each tick it reports whether the last move succeeded and returns the next move as a
trajectory (`markers` + `turn_degrees` + `stamp`). The BT is a thin executive over motor skills —
it drives that trajectory with `FollowVisualPathAction` + `SpinAction`. There is **no**
`PlanVisualPath` leaf; planning happens inside the node.

```
Fallback
  KeepRunningUntilFailure                       # ends when MissionAdvance reports mission over
    Sequence
      MissionAdvance(success={last_ok}) → {markers}, {turn_degrees}, {stamp}, {area}
      Fallback                                   # capture follow/spin success into {last_ok}
        Sequence:
          FollowVisualPathAction(waypoints={markers}, stamp={stamp})
          SpinAction(yaw_degrees={turn_degrees})
          SetBlackboard(last_ok := true)
        SetBlackboard(last_ok := false)
  Precondition(if mission_failed → FAILURE, else SUCCESS)
```

The BT reports only `success`; the planner's reasoning stays inside the node (recorded as the next
cycle's trigger). Divergence is absorbed by the narrative, so the loop ends on completion or on a
failure path (stuck backstop, or a cognition call exhausting a *bounded* retry budget). The outer
`Precondition` maps a clean finish to overall `SUCCESS` and `mission_failed` to `FAILURE`. The only
custom terms are `MissionAdvance` / `FollowVisualPathAction` / `SpinAction`;
`Fallback`/`KeepRunningUntilFailure`/`SetBlackboard`/`Precondition` are stock BT.cpp.

> **Future refinements (deferred):** letting the cognition node **choose the skill** (not just
> plan a forward path) — e.g. an explicit "arrived" signal, a scan/rotate re-orient, or backtrack —
> now that the executive is a clean dispatch point. The current node always plans a forward path +
> end-of-move turn; completion is inferred from the narrative.
