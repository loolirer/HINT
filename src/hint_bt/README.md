# hint_bt

BT.cpp behavior trees for HINT, built on `behaviortree_ros2`.

## Executables

| Executable | Description |
|---|---|
| `approach_described_target` | Grounds a text description to an ROI then drives the robot toward it |

---

## approach_described_target

Chains the VLM grounding and IBVS pipeline through a `Sequence` behavior tree: calls `description_detector`'s `GroundDescription` action to locate the target, then hands the resulting ROI and frame stamp directly to `visual_servoing`'s `ApproachTarget` action. The two subsystems are decoupled — data flows between them via the BT blackboard.

### Tree structure

```
Sequence
├── GroundDescriptionAction   → calls /description_detector_node/ground_description
│     in:  {description}      ← set from command-line argument before first tick
│     out: {roi}, {stamp}     → written to blackboard on success
└── ApproachTargetAction      → calls /visual_servoing_node/approach_target
      in:  {roi}, {stamp}     ← read from blackboard
```

The sequence short-circuits on the first failure: if grounding finds no match the robot never moves.

### Action clients

| Action | Server |
|---|---|
| `hint_interfaces/action/GroundDescription` | `/description_detector_node/ground_description` |
| `hint_interfaces/action/ApproachTarget` | `/visual_servoing_node/approach_target` |

### Tree file

`config/approach_described_target.xml` — can be edited without recompiling.

---

## Build

```bash
colcon build --symlink-install --packages-select hint_interfaces behaviortree_ros2 hint_bt
source install/setup.bash
```

`behaviortree_ros2` must be built from source (clone `BehaviorTree.ROS2` into `src/`); `hint_interfaces` must be built before `hint_bt`.

## Run

All three nodes must be running before launching the tree:

```bash
# terminal 1 — turtlebot bringup (tracker + servo + description_detector)
ros2 launch hint bringup.launch.py

# terminal 2 — drive toward a described target
ros2 run hint_bt approach_described_target "the left trash bin"
```

The executable exits with code `0` on SUCCESS and `1` on FAILURE.
