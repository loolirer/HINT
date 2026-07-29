# hint_behavior

BT.cpp behavior trees for HINT, built on `behaviortree_ros2`.

## Executables

| Executable | Runtime node | Description |
|---|---|---|
| `behavior_server` | `hint_behavior_server` | Long-running action server (`HintBtExecutorNode`, a thin subclass of `behaviortree_ros2`'s `TreeExecutionServer`) — builds and ticks a tree per goal |

> The executable is named `behavior_server`, but its **runtime node name is `hint_behavior_server`** — deliberately distinct from Nav2's own `behavior_server` node (the Spin server in `hint_navigation`), which would otherwise collide.

## Leaf node registry

Every HINT-specific BT leaf type is registered once, in one place: `hint_behavior::registerHintNodes()` (`include/hint_behavior/register_nodes.hpp`, `src/register_nodes.cpp`). Any executable that builds/ticks a tree referencing these types links `hint_behavior_nodes` and calls this function against its own `rclcpp::Node`.

| Node type | Header | Action client |
|---|---|---|
| `FollowVisualPathAction` | `include/hint_behavior/nodes/follow_visual_path_action.hpp` | `/path_projector_node/follow_visual_path` |
| `SpinAction` | `include/hint_behavior/nodes/spin_action.hpp` | `/spin` (Nav2 behavior_server) |
| `VisualReasonAction` | `include/hint_behavior/nodes/visual_reason_action.hpp` | `/visual_reasoner/visual_reason` |
| `MissionAdvance` | `include/hint_behavior/nodes/mission_advance_action.hpp` | `/narrative_navigation/mission_advance` |

The action names are set in `registerHintNodes()`, one `BT::RosNodeParams` per type. Adding a new leaf: drop a `RosActionNode<...>` subclass header under `include/hint_behavior/nodes/`, register it inside `registerHintNodes()`. No executable needs to change — `behavior_server` picks up any tree that references it.

---

## behavior_server

`HintBtExecutorNode` (`src/behavior_server.cpp`) is a small subclass of `behaviortree_ros2`'s `BT::TreeExecutionServer` — rather than hand-rolling an action server, goal/cancel handling, thread lifecycle, and tick loop, we reuse the upstream implementation and only override four hooks:

- `registerNodesIntoFactory(factory)` — calls `hint_behavior::registerHintNodes(factory, node())`.
- `onGoalReceived(tree_name, payload)` — treats `payload` as a full tree XML document and calls `factory().registerBehaviorTreeFromText(payload)` before the base class's `createTree(tree_name, ...)` runs. This is what lets a caller send an **arbitrary, freshly composed tree per goal** instead of only invoking trees preloaded at startup from the `behavior_trees` ROS param. An empty `payload` falls back to a preloaded tree. The `<BehaviorTree ID="...">` inside `payload` must match `target_tree` in the goal, or `createTree` won't find it.
- `onLoopFeedback()` — reports the name of the currently `RUNNING` action leaf, via `tree().applyVisitor(...)`, and mirrors it to `~/state`.
- `onTreeExecutionCompleted(...)` — resets `~/state` to `IDLE` when a tree finishes.

Alongside the `ExecuteTree` feedback, the node publishes the running leaf name (or `IDLE`) on `~/state` (`/hint_behavior_server/state`, `std_msgs/String`, transient-local so late subscribers get the last value) — the BT-state source for `hint_navigation`'s `visual_debug` overlay.

Everything else (malformed-XML handling, single-goal-at-a-time execution, cancellation via `tree.haltTree()`, exception safety) is the base class's behavior — see `BehaviorTree.ROS2/behaviortree_ros2/tree_execution_server.md`.

### Action server

Served via `btcpp_ros2_interfaces/action/ExecuteTree` (not a custom HINT interface).

| Field | Direction | Notes |
|---|---|---|
| `target_tree` | Goal | ID that must match the `<BehaviorTree ID="...">` inside `payload` |
| `payload` | Goal | Full tree XML text (empty → run a preloaded tree) |
| `node_status` | Result | Final `BT::NodeStatus` |
| `return_message` | Result | Human-readable outcome, or the exception message on a malformed tree |
| `message` | Feedback | Name of the currently running leaf |

The server name comes from the `action_name` param, launched with the fully-qualified `/hint_behavior_server/execute_behavior_tree` so it follows the repo's `node_name/action_name` convention. (Don't use the `~/action_name` shorthand on a shell command line — an unquoted `~` is expanded by **bash**, not ROS2, silently renaming the server.)

---

## `behaviors/` — preloaded trees

Every `.xml` under `behaviors/` is installed to `share/hint_behavior/behaviors` and loaded into the node's BT factory once at startup, via the `behavior_trees` param (set to `["hint_behavior/behaviors"]` in `bringup.launch.py`). All files land in the **same** factory, so a `<BehaviorTree ID="...">` in one file can be referenced from another via `<SubTree ID="..."/>`.

| File | Tree ID | Description |
|---|---|---|
| `run_mission.xml` | `RunMission` | Drive a mission to completion: a `KeepRunningUntilFailure` loop of `MissionAdvance` → `FollowVisualPathAction` → `SpinAction` → `SetBlackboard`. `MissionAdvance` is the whole **cognition** interface — each tick reports only whether the last move succeeded (`{last_ok}`) and `narrative_navigation` makes **both** per-cycle VLM calls internally (recompiles its narrative *and* plans the path over one buffer), returning the next move as a trajectory: `{waypoints}` + `{turn_degrees}` + `{stamp}`. `FollowVisualPathAction` drives `{waypoints}` (empty = turn-only no-op) via `hint_navigation`'s `path_projector` → Nav2 `follow_path`/MPPI, then `SpinAction` rotates by `{turn_degrees}` (0° = instant success); the `SetBlackboard` pair captures the follow/spin outcome into `{last_ok}`. There is **no** `PlanVisualPath` leaf — planning moved into the node. The leaf also sends `first` (a per-run latch: true on its first tick of the run, false after — the leaf instance is rebuilt per `ExecuteTree` goal) so `narrative_navigation` wipes + reseeds exactly once per run. **Which mission runs is chosen at the call** via the `{mission_path}` port (a YAML path) — no default; bind it in the payload to run/switch missions without restart. Divergence is absorbed by the narrative; the failure paths are the stuck backstop and a cognition call exhausting a *bounded* retry budget, so the outer `Precondition` maps a clean finish to `SUCCESS` and `{mission_failed}` to `FAILURE`. Requires `narrative_navigation`, `visual_reasoner`, and `path_planner` running (plus the `path_projector` + Nav2 pipeline). |

Preloading happens once, in the constructor — editing a `behaviors/` file needs a **node restart** (no rebuild, `--symlink-install`).

## Usage

```bash
ros2 launch hint_bringup bringup.launch.py
```

`bringup.launch.py` starts `behavior_server` with `action_name` = `/hint_behavior_server/execute_behavior_tree` and `behavior_trees` = `["hint_behavior/behaviors"]`.

Standalone (iterating on `hint_behavior` alone):

```bash
ros2 run hint_behavior behavior_server --ros-args \
  -p action_name:=/hint_behavior_server/execute_behavior_tree \
  -p behavior_trees:="[hint_behavior/behaviors]"
```

### Run a mission

`RunMission` drives a mission to completion. Requires `narrative_navigation` and `visual_reasoner` (and the `path_planner` / `path_projector` + Nav2 pipeline) running. **No default mission** — choose it at the call by binding `mission_path` on a one-line wrapper `SubTree` into the preloaded `RunMission` (the node loads it, resuming that mission's narrative if it exists; no restart):

```bash
ros2 action send_goal /hint_behavior_server/execute_behavior_tree btcpp_ros2_interfaces/action/ExecuteTree "$(jq -n --arg tree Mission --arg xml '<root BTCPP_format="4"><BehaviorTree ID="Mission"><SubTree ID="RunMission" mission_path="/root/turtlebot3_ws/src/hint_narrative/missions/bedroom_to_living_room/mission.yaml"/></BehaviorTree></root>' '{target_tree: $tree, payload: $xml}')" --feedback
```

### Reason over text with `VisualReasonAction`

`VisualReasonAction` calls the `visual_reasoner` node (`/visual_reasoner/visual_reason`) — generic text(+optional-image)-in / JSON-out LLM reasoning. `prompt` is the text to reason over; the optional `schema` constrains the reply to a JSON shape; the reply lands on `response` (`SUCCESS`), or the failure reason does (`FAILURE`). It's the reasoning primitive the mission planner leans on, but stands alone for a one-off query (requires the `visual_reasoner` node running):

```bash
ros2 action send_goal /hint_behavior_server/execute_behavior_tree btcpp_ros2_interfaces/action/ExecuteTree "$(jq -n --arg tree AdHocReason --arg xml '<?xml version="1.0"?><root BTCPP_format="4"><BehaviorTree ID="AdHocReason"><VisualReasonAction prompt="In one sentence, is a hallway a good place to drive a robot?" response="{reply}"/></BehaviorTree></root>' '{target_tree: $tree, payload: $xml}')" --feedback
```

Bind `schema` to a JSON shape to force structured output; `{reply}` on `response` makes the JSON available to downstream leaves via the blackboard.
