# hint_bt

BT.cpp behavior trees for HINT, built on `behaviortree_ros2`.

## Executables

| Executable | Description |
|---|---|
| `bt_executor_node` | Long-running action server (`HintBtExecutorNode`, a thin subclass of `behaviortree_ros2`'s `TreeExecutionServer`) — builds and ticks a tree per goal |

## Leaf node registry

Every HINT-specific BT leaf type is registered once, in one place: `hint_bt::registerHintNodes()` (`include/hint_bt/register_nodes.hpp`, `src/register_nodes.cpp`). Any executable that wants to build/tick a tree referencing these node types links `hint_bt_nodes` and calls this function against its own `rclcpp::Node`.

| Node type | Header | Action client |
|---|---|---|
| `GroundDescriptionAction` | `include/hint_bt/nodes/ground_description_action.hpp` | `/description_detector_node/ground_description` |
| `ApproachTargetAction` | `include/hint_bt/nodes/approach_target_action.hpp` | `/visual_servoing_node/approach_target` |

Adding a new leaf: drop a new `RosActionNode<...>` subclass header under `include/hint_bt/nodes/`, register it inside `registerHintNodes()`. No executable needs to change — `bt_executor_node` picks up any tree that references it.

---

## bt_executor_node

`HintBtExecutorNode` (`src/bt_executor_node.cpp`) is a small subclass of `behaviortree_ros2`'s `BT::TreeExecutionServer` — rather than hand-rolling an action server, goal/cancel handling, thread lifecycle, and tick loop, we reuse the upstream implementation and only override three hooks:

- `registerNodesIntoFactory(factory)` — calls `hint_bt::registerHintNodes(factory, node())`.
- `onGoalReceived(tree_name, payload)` — treats `payload` as a full tree XML document and calls `factory().registerBehaviorTreeFromText(payload)` before the base class's `createTree(tree_name, ...)` runs. This is what lets a caller send an **arbitrary, freshly composed tree per goal** instead of only invoking trees preloaded at startup from the `behavior_trees` ROS param. An empty `payload` falls back to a preloaded tree. The `<BehaviorTree ID="...">` inside `payload` must match `target_tree` in the goal, or `createTree` won't find it.
- `onLoopFeedback()` — reports the name of the currently `RUNNING` action leaf, via `tree().applyVisitor(...)`.

Everything else (malformed-XML handling, single-goal-at-a-time execution, cancellation via `tree.haltTree()`, exception safety) is the base class's behavior — see `BehaviorTree.ROS2/behaviortree_ros2/tree_execution_server.md` and `src/tree_execution_server.cpp` for exact semantics.

### Action server

Served via `btcpp_ros2_interfaces/action/ExecuteTree` (not a custom HINT interface).

| Field | Direction | Notes |
|---|---|---|
| `target_tree` | Goal | ID that must match the `<BehaviorTree ID="...">` inside `payload` |
| `payload` | Goal | Full tree XML text |
| `node_status` | Result | Final `BT::NodeStatus` |
| `return_message` | Result | Human-readable outcome, or the exception message on a malformed tree |
| `message` | Feedback | Name of the currently running leaf |

The server name is set by the `bt_server.action_name` ROS parameter (read-only, default `bt_execution`, resolved as a **global** name — i.e. `/bt_execution`, not namespaced under the node). We launch it with the fully-qualified name `/bt_executor_node/execute_behavior_tree` explicitly, so it follows the rest of the repo's `node_name/action_name` convention — see below. (Don't use the `~/action_name` private-name shorthand here: passed unquoted on a shell command line, `~` gets expanded by **bash itself** to your home directory before ROS2 ever sees it, silently renaming the server to something like `/root/execute_behavior_tree`.)

Other `bt_server.*` parameters (`tick_frequency`, `groot2_port`, `plugins`, `behavior_trees`, `ros_plugins_timeout`) are documented in `BehaviorTree.ROS2/behaviortree_ros2/bt_executor_parameters.md`.

---

## `behaviors/` — preloaded trees

Every `.xml` file under `behaviors/` is installed to `share/hint_bt/behaviors` and loaded into the node's BT factory once at startup, via the `bt_server.behavior_trees` param (set to `["hint_bt/behaviors"]` in `bringup.launch.py`) — `RegisterBehaviorTrees` → `LoadBehaviorTrees` (`behaviortree_ros2/src/bt_utils.cpp`) iterates the directory and calls `factory.registerBehaviorTreeFromFile()` on each file. Since every file lands in the **same** factory, a `<BehaviorTree ID="...">` defined in one file can be referenced from another via `<SubTree ID="..."/>` without inlining it.

| File | Tree ID | Description |
|---|---|---|
| `approach_described_target.xml` | `ApproachDescribedTarget` | Grounds a text description to a bbox (`GroundDescriptionAction`), then drives to it (`ApproachTargetAction`). Its `description` port is left as an unbound blackboard reference (`{description}`) — it's meant to be driven as a `SubTree`, which binds `description` from the caller. |
| `sequential_bins.xml` | `SequentialBins` | Visits the blue trash bin, then the yellow trash bin — two `SubTree` calls into `ApproachDescribedTarget`, each binding its own `description`. |

Preloading happens once, in the node constructor, before it starts spinning — editing a file under `behaviors/` needs a **node restart** to take effect (no rebuild, since `--symlink-install` mirrors `install(DIRECTORY behaviors/ ...)` straight to the source file).

Because it's preloaded, a tree in `behaviors/` can be run directly with an **empty `payload`** — no XML needs to cross the wire at all:

```bash
ros2 action send_goal /bt_executor_node/execute_behavior_tree btcpp_ros2_interfaces/action/ExecuteTree \
  "{target_tree: 'SequentialBins', payload: ''}" \
  --feedback
```

## Run

```bash
ros2 launch hint bringup.launch.py
```

`bringup.launch.py` starts `bt_executor_node` with `action_name` set to the fully-qualified `/bt_executor_node/execute_behavior_tree` (not `~/execute_behavior_tree` — unquoted tildes get expanded by the shell, not ROS2, when passed on a command line, which silently renames the server; see `hint/launch/bringup.launch.py`) and `behavior_trees` set to `["hint_bt/behaviors"]` so everything under `behaviors/` is preloaded.

To run it standalone instead (e.g. while iterating on `hint_bt` without the rest of the stack):

```bash
ros2 run hint_bt bt_executor_node --ros-args \
  -p action_name:=/bt_executor_node/execute_behavior_tree \
  -p behavior_trees:="[hint_bt/behaviors]"
```

Drop `-p behavior_trees:=...` if you only want to send fully self-contained, ad-hoc trees per goal (see the first example below) and don't need the preloaded ones.

### Ground a description and approach it

Two-leaf `Sequence` — `GroundDescriptionAction` locates "the left trash bin" and hands its ROI to `ApproachTargetAction`, which drives the robot there. Paste as-is into a terminal once bringup is running:

```bash
ros2 action send_goal /bt_executor_node/execute_behavior_tree btcpp_ros2_interfaces/action/ExecuteTree '{target_tree: "ApproachDescribedTarget", payload: "<?xml version=\"1.0\"?><root BTCPP_format=\"4\"><BehaviorTree ID=\"ApproachDescribedTarget\"><Sequence><GroundDescriptionAction description=\"the left trash bin\" roi=\"{roi}\" stamp=\"{stamp}\"/><ApproachTargetAction roi=\"{roi}\" stamp=\"{stamp}\"/></Sequence></BehaviorTree></root>"}' --feedback
```

Swap `the left trash bin` for any other description — it's a literal string embedded directly in the XML, no blackboard indirection needed for a one-shot goal like this. `{roi}` / `{stamp}` remain blackboard placeholders so `GroundDescriptionAction`'s output feeds `ApproachTargetAction`'s input. `target_tree` must equal the `<BehaviorTree ID="...">` value inside `payload`.

### Run the preloaded sequential-bins mission

`SequentialBins` (`behaviors/sequential_bins.xml`) is preloaded by `bringup.launch.py`, so it needs no `payload` at all — see [`behaviors/` — preloaded trees](#behaviors--preloaded-trees) above:

```bash
ros2 action send_goal /bt_executor_node/execute_behavior_tree btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: 'SequentialBins', payload: ''}" --feedback
```

### Compose a one-off tree that reuses a preloaded `SubTree`

Because `ApproachDescribedTarget` is already registered in the factory (preloaded from `behaviors/`), a fresh ad-hoc tree sent as `payload` can `SubTree` into it without inlining its definition — this is the key benefit of splitting trees across files instead of writing one big document per goal:

```bash
ros2 action send_goal /bt_executor_node/execute_behavior_tree btcpp_ros2_interfaces/action/ExecuteTree "$(jq -n --arg tree AdHocApproach --arg xml '<?xml version="1.0"?><root BTCPP_format="4"><BehaviorTree ID="AdHocApproach"><SubTree ID="ApproachDescribedTarget" description="the red trash bin"/></BehaviorTree></root>' '{target_tree: $tree, payload: $xml}')" --feedback
```

`jq -n --arg tree ... --arg xml ... '{target_tree: $tree, payload: $xml}'` builds the JSON goal object with `jq` handling all quote escaping — no manual `\"` needed even for an inline XML string. `--rawfile xml <path>` (instead of `--arg xml '...'`) works the same way for sending a whole file's content verbatim; just don't point it at a tree ID that's already preloaded from `behaviors/` — `registerBehaviorTreeFromText` registers into the same factory as the preload step, and re-registering the same `<BehaviorTree ID="...">` a second time throws.