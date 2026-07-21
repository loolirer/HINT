# hint_navigation

Nav2 reactive-navigation integration for HINT — **mapless**. It bets on Nav2's mature MPPI
controller to follow the VLM-planned trajectory while flowing around obstacles that
`hint_perception` marks in a rolling local costmap. No SLAM, no global map: the robot
follows the path in `odom`, and the costmap is a short-lived rolling window.

Two parts:

| Part | What |
|---|---|
| `trajectory_navigator` (node) | Adapter — exposes the `hint_interfaces/FollowTrajectory` action the BT calls, grounds the VLM's normalized markers into a metric `odom` `nav_msgs/Path`, and drives Nav2's `follow_path` (MPPI) |
| `launch/nav2.launch.py` + `config/nav2_local.yaml` | Brings up the mapless Nav2 stack: `controller_server` (FollowPath + MPPI, rolling local costmap) + `behavior_server` (Spin) + `nav2_lifecycle_manager` |

```bash
colcon build --symlink-install --packages-select hint_interfaces hint_navigation
source install/setup.bash
```

Requires the Nav2 stack installed (rosdep pulls it): `nav2_controller`,
`nav2_mppi_controller`, `nav2_costmap_2d`, `nav2_behaviors`, `nav2_lifecycle_manager`.

## trajectory_navigator

The BT (`hint_behavior`'s `FollowTrajectoryAction`) still calls
`hint_interfaces/FollowTrajectory` with the VLM's **normalized image markers**; this node is
a transparent adapter that internally drives Nav2's `nav2_msgs/action/FollowPath`. The whole
chain stays action-based.

Per goal it:

1. **Grounds** the normalized markers (`x`/`y ∈ [-1, 1]`, nearest-first) onto the ground
   plane in `base_link` via the analytic camera model (same projection as `hint_perception`),
   then re-expresses them in **`odom`** using the odometry pose at the goal's stamp — so the
   path is anchored once in the world, and the robot follows it while it stays put in `odom`
   (Nav2 tracks the robot against it via the `odom → base_link` TF). Prepends the robot's own
   pose so the path starts at the robot; each pose's yaw is the path tangent.
2. **Calls** `follow_path` (`controller_id: FollowPath`), relays feedback, and maps the
   result: Nav2 `SUCCEEDED` → `success=true`; `ABORTED` (incl. `SimpleProgressChecker` firing
   on an unreachable goal — the "stall" backstop) / `CANCELED` / rejected → `success=false`.
3. Handles the **turn-only** move: an empty `waypoints` goal succeeds immediately (nothing to
   follow), so the BT's `SpinAction` that runs next performs the rotation.

It also republishes the grounded path (re-stamped, at control rate) on
`~/path` (`/trajectory_navigator_node/path`, latched) purely for RViz — re-stamping is what
lets it render correctly in an **ego (`base_link`) view** instead of freezing at plan time.

### Interfaces

| Interface | Type | Direction |
|---|---|---|
| `~/follow_trajectory` | `hint_interfaces/FollowTrajectory` | Action server (BT-facing) |
| `follow_path` (see `follow_path_action`) | `nav2_msgs/FollowPath` | Action client (Nav2 controller) |
| `/odom` (see `odom_topic`) | `nav_msgs/Odometry` | Sub — world anchor for grounding |
| `~/path` | `nav_msgs/Path` (latched) | Pub — the grounded path, for RViz |

### Key parameters

`camera_height` / `camera_forward_offset` / `camera_tilt` / `camera_hfov_deg` (match the rig,
same as `hint_perception`); `image_width` / `image_height` (marker normalization reference,
default 640×480); `follow_path_action` (default `/follow_path`), `controller_id`
(`FollowPath`), `goal_checker_id` (`goal_checker`), `progress_checker_id`
(`progress_checker`); `odom_topic`, `path_frame` (`odom`), `server_timeout`, `control_rate`.

## Nav2 config (`config/nav2_local.yaml`)

- **`controller_server`** — `FollowPath` = `nav2_mppi_controller::MPPIController`
  (`motion_model: DiffDrive`, forward-only `vx_min: 0`, Waffle-Pi limits). `SimpleProgressChecker`
  (the unreachable-goal backstop → `follow_path` ABORTs) and `SimpleGoalChecker` (positional
  arrival; yaw effectively ignored). `enable_stamped_cmd_vel: true` (TB3 consumes stamped
  `/cmd_vel`). `FollowPath.visualize` publishes MPPI's `trajectories` /
  `transformed_global_plan` markers for RViz.
- **`local_costmap`** — rolling, `global_frame: odom`, `robot_base_frame: base_link`,
  `obstacle_layer` fed by `/ground/obstacles` (`PointCloud2`) + `inflation_layer` (the keep-out
  margin). No static/voxel layer, no global costmap.
- **`behavior_server`** — the `spin` behavior only, mapless (its `global_*` costmap topics
  point at the local costmap). Drives the BT's end-of-move / scan turn via the `/spin` action.

Lifecycle order in `nav2.launch.py` is `["controller_server", "behavior_server"]` (controller
first, so its local costmap is up before the Spin server subscribes to it).

## Notes

- The costmap **remembers** obstacles within the rolling window until re-observed/scrolled out
  (a forward camera can't clear what falls behind it) — this is short-term memory, and MPPI
  plans against all of it. For a purely-reactive *view*, display `/ground/obstacles` (Decay
  Time 0) rather than the costmap.
- The path is anchored in `odom` at plan time and followed there; it is re-grounded fresh each
  mission cycle, which bounds odometry drift over a single move.
