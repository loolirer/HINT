# hint_bringup

Bringup package for HINT — groups the launch file, RViz2 configuration, and teleop settings
in one place. It has no nodes of its own; `launch/bringup.launch.py` starts the full system.

## Contents

| Path | Purpose |
|---|---|
| `launch/bringup.launch.py` | Main bringup (see below) |
| `config/teleop.yaml` | `teleop_twist_joy` parameters (axes, scales, enable button) |
| `viz/hint.rviz` | RViz2 layout (ego view — `base_link` fixed frame) |

## What it launches

- `teleop_twist_joy`
- `ground_segmenter` (`hint_perception`) — semantic ground ONNX → binary ground mask on `/camera/ground` (image space only, no rig)
- `obstacle_projector` (`hint_navigation`) — ground mask → obstacle `PointCloud2` on `/obstacles`
- `trajectory_navigator` (`hint_navigation`) — grounds VLM markers → `odom` path → Nav2 `follow_path`
- `visual_debug` (`hint_navigation`) — composes one `/debug` image (mask overlay + navigator paths + BT state)
- the mapless Nav2 stack via `hint_navigation/launch/nav2.launch.py`: `controller_server` (FollowPath + MPPI), `behavior_server` (Spin), `nav2_lifecycle_manager`
- `trajectory_generator` (`hint_vlm`) — VLM ground-trajectory planner (+ end-of-move turn)
- `visual_reasoner` (`hint_vlm`) — generic text/vision → JSON reasoner (the narrative director)
- `narrative_navigation` (`hint_narrative`) — semantic mission planner
- `behavior_server` (`hint_behavior`; runtime node `hint_behavior_server`) — the BT executor running `RunMission`
- `rviz2`

The shared camera-rig geometry (`camera_height` / `camera_forward_offset` / `camera_tilt` /
`camera_hfov_deg`) is passed in one `camera_rig` dict to the three nodes that own metric space —
`obstacle_projector`, `trajectory_navigator`, and `visual_debug` (all in `hint_navigation`, via
`camera_rig.CameraRig`) — keep it matching the real rig. `ground_segmenter` no longer takes it
(perception is image-space only).

## Usage

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch hint_bringup bringup.launch.py
```

## Teleop configuration

`config/teleop.yaml` is passed to `teleop_twist_joy` at launch. Current mapping (Xbox controller):

| Parameter | Value | Effect |
|---|---|---|
| `axis_linear.x` | 1 | Left stick vertical → forward/back |
| `axis_angular.yaw` | 0 | Left stick horizontal → rotation |
| `scale_linear.x` | 0.26 m/s | Maximum linear speed |
| `scale_angular.yaw` | 0.91 rad/s | Maximum angular speed |
| `axis_linear_sign.x` | -1.0 | Inverts forward direction |
| `require_enable_button` | true | Deadman switch required |
| `enable_button` | 4 | LB button enables turbo |

## RViz2 layout

`viz/hint.rviz` opens in an **ego view** (`base_link` fixed frame). Useful displays for this
stack: `/local_costmap/costmap`, `/obstacles` (PointCloud2, Decay Time 0),
`/trajectory_navigator_node/path`, and — with MPPI `visualize` on —
`/controller_server/trajectories` + `/controller_server/transformed_global_plan`. Adjust and
save; the file is used on the next launch.
