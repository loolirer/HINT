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
- `path_projector` (`hint_navigation`) — grounds VLM waypoints → `odom` path → Nav2 `follow_path`
- `visual_debug` (`hint_navigation`) — composes one `/debug` image (mask overlay + projector paths + BT state)
- the mapless Nav2 stack via `hint_navigation/launch/nav2.launch.py`: `controller_server` (FollowPath + MPPI), `behavior_server` (Spin), `nav2_lifecycle_manager`
- **localization** via `hint_navigation/launch/localization.launch.py`: `map_server` + `amcl` on a saved map (`hint_navigation/maps/<region>/map.yaml`, chosen by the `region` launch arg). This publishes `map→odom` so the mission's actual trajectory lands in the map frame — it is **localization only**, HINT still drives with the mapless stack above. (SLAM is no longer run here; mapping moved to `hint_navigation`'s reference phase — see its README.)
- `visual_reasoner` (`hint_vlm`) — generic text/vision → JSON reasoner, launched **twice**: as `visual_reasoner` (the narrative director, temp 0) and as `path_planner` (the ground-path planner, temp 1.0). Both serve `VisualReason`; the prompt + schema that make one a director and the other a planner are owned by `hint_narrative`
- `narrative_navigation` (`hint_narrative`) — semantic mission planner
- `behavior_server` (`hint_behavior`; runtime node `hint_behavior_server`) — the BT executor running `RunMission`
- `rviz2`

The shared camera-rig geometry (`camera_height` / `camera_forward_offset` / `camera_tilt` /
`camera_hfov_deg`) is passed in one `camera_rig` dict to the three nodes that own metric space —
`obstacle_projector`, `path_projector`, and `visual_debug` (all in `hint_navigation`, via
`camera_rig.CameraRig`) — keep it matching the real rig. `ground_segmenter` no longer takes it
(perception is image-space only).

Similarly, a single `vlm_timeout` is shared across the VLM-facing nodes: it sets `api_timeout`
on `path_planner` and `visual_reasoner` and `reasoner_timeout` on `narrative_navigation`, and
`path_projector`'s `tf_buffer_time` is **derived** from it (`2 × vlm_timeout + margin`). That
coupling is deliberate: a path frame is captured, then flows through the director (reasoner)
call **and** the planner call before `path_projector` grounds it, so its stamp can be up to
~2× a single call old — the TF buffer must be large enough that the stamped `odom ← base_link`
lookup still resolves. Bumping `vlm_timeout` can't silently outrun the buffer.

`path_projector`'s `path_range` (max straight-line distance from the robot the grounded path is
clipped to) is set here too, default `5.0` m. It is kept **independent** of `obstacle_projector`'s
`bev_range` (the sensed horizon): you can set them equal, but the default leaves `path_range` a
little beyond `bev_range` so a move can reach just past the current obstacle window. See
`hint_navigation`'s `path_projector` docs for what the clip guards against.

## Usage

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch hint_bringup bringup.launch.py region:=<region>
```

`region` (default `default`) selects the saved map at `hint_navigation/maps/<region>/map.yaml`
that `map_server` + AMCL localize against — it must exist first (build it in the reference phase;
see `hint_navigation`'s README). After launch, set the robot's initial pose in RViz (2D Pose
Estimate) so AMCL converges.

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
`/path_projector_node/path`, and — with MPPI `visualize` on —
`/controller_server/trajectories` + `/controller_server/transformed_global_plan`. Adjust and
save; the file is used on the next launch.
