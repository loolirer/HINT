# hint_navigation

Nav2 reactive-navigation integration for HINT — **mapless** — **and the image↔metric
bridge**. This package is the **sole owner of the camera rig** (`camera_rig.CameraRig`):
anything that needs the camera's physical placement lives here. `hint_perception` stays
purely image-space (it only labels pixels into a ground mask); everything metric — turning
that mask into obstacles, grounding VLM waypoints into a path, re-projecting paths for the
debug view — happens on this side.

It bets on Nav2's mature MPPI controller to follow the VLM-planned path while flowing
around obstacles in a rolling local costmap. No SLAM, no global map: the robot follows the
path in `odom`, and the costmap is a short-lived rolling window.

| Part | What |
|---|---|
| `camera_rig.py` (module) | `CameraRig` — the pinhole+tilt+height rig and both ground↔pixel projections; the single source of truth, imported by the three nodes below |
| `path_projector` (node) | Exposes the `hint_interfaces/FollowVisualPath` action the BT calls, grounds the VLM's normalized waypoints into a metric `odom` `nav_msgs/Path`, and drives Nav2's `follow_path` (MPPI) |
| `obstacle_projector` (node) | Streams `hint_perception`'s ground mask (`/camera/ground`) → obstacle `PointCloud2` (`/obstacles`) for the local costmap, via the ground-plane BEV homography |
| `visual_debug` (node) | Composes one `/debug` image from the system's real outputs (mask overlay + projector paths + BT state) |
| `launch/nav2.launch.py` + `config/nav2_local.yaml` | Brings up the mapless Nav2 stack: `controller_server` (FollowPath + MPPI, rolling local costmap) + `behavior_server` (Spin) + `nav2_lifecycle_manager` |
| `launch/mapping.launch.py` | **Reference phase step 1** — Cartographer SLAM + occupancy grid (stock `turtlebot3_cartographer` config) + `teleop_twist_joy` (you drive the region). Self-contained: a background `map_autosaver` re-saves `maps/<region>/map` every `save_interval` s (default 5) while the graph is alive, so a valid map is always on disk — no second terminal. Ctrl+C stops; the last autosave is your map |
| `config/teleop.yaml` | `teleop_twist_joy` parameters (axes, scales, enable button) — used by both `mapping.launch.py` here and `hint_bringup`'s bringup |
| `launch/reference.launch.py` + `config/nav2_reference.yaml` | **Reference phase step 2** — full Nav2 (AMCL + A* planner + DWB controller + bt_navigator) on the saved map, to drive an operator-clicked GoToGoal and record a ground-truth trajectory |
| `launch/localization.launch.py` + `config/localization.yaml` | **HINT run** — AMCL + map_server on the saved map. Publishes `map→odom` so the HINT run's trajectory lands in the map frame; **not** used for navigation (HINT still drives with the mapless stack). Included by `hint_bringup`'s bringup |
| `maps/<region>/` | Saved occupancy map (`map.pgm` + `map.yaml`) and the reference trajectory bag (`reference.bag`) per environment |

```bash
colcon build --symlink-install --packages-select hint_interfaces hint_navigation
source install/setup.bash
```

Requires the Nav2 stack installed (rosdep pulls it): `nav2_controller`,
`nav2_mppi_controller`, `nav2_costmap_2d`, `nav2_behaviors`, `nav2_lifecycle_manager`.

## camera_rig (`camera_rig.py`)

`CameraRig` is a small, stateless class holding the four rig parameters (`camera_height`,
`camera_forward_offset`, `camera_tilt`, `camera_hfov_deg`) and the pinhole + tilt + height
ground-plane model in **both directions**:

- `pixels_to_ground(pts, w, h)` → metric `base_link` (X fwd, Y left) — used by `path_projector`.
- `ground_to_pixels(gxy, w, h)` → `(pixels, in_front)` — used by `obstacle_projector` (to build
  the BEV homography) and `visual_debug` (to re-project odom paths).

The two directions are exact inverses of one model, so they can't drift apart (they used to
be hand-synced copies across three nodes). `CameraRig.from_node(node)` snapshots the rig from
a node's current parameters (declaring them if absent), called at the point of use so the rig
stays **live-adjustable** — `ros2 param set` a rig value while calibrating and the next frame
picks it up. The rig geometry is injected once by `hint_bringup` (the shared `camera_rig`
dict); the BEV *grid* geometry (`bev_*`) is **not** a rig concern and lives in
`obstacle_projector`.

## path_projector

The BT (`hint_behavior`'s `FollowVisualPathAction`) still calls
`hint_interfaces/FollowVisualPath` with the VLM's **normalized image waypoints**; this node is
a transparent adapter that internally drives Nav2's `nav2_msgs/action/FollowPath`. The whole
chain stays action-based.

Per goal it:

1. **Grounds** the VLM waypoints (`x`/`y ∈ [-1, 1]`, nearest-first) onto the ground plane
   in `base_link` via `CameraRig.pixels_to_ground`, then re-expresses them in **`odom`** using
   the robot pose at the goal's stamp — looked up from **TF** (`odom → base_link` at that
   stamp), the same transform the costmap and MPPI use. So the path is anchored once in the
   world, and the robot follows it while it stays put in `odom`. No `/odom` subscription: the
   pose source is TF, which also keeps grounding in step with the costmap. Because the frame
   is captured seconds before grounding (it flows through the director *and* planner VLM
   calls), the TF buffer's `cache_time` (`tf_buffer_time`) must cover that latency, or the
   stamped lookup falls out of the buffer and the goal aborts. Prepends the robot's own pose
   so the path starts at the robot; each pose's yaw is the path tangent.
1. **Distance-clips** the grounded path where it first leaves a circle of radius `path_range` m
   around the robot, interpolating the segment–circle crossing so the endpoint stays clean
   (`path_range ≤ 0` disables it). The bound is **straight-line distance from the robot**, not
   path length, because that is what projection error tracks: `pixels_to_ground` error grows with
   range (the horizon clamp sends a near-horizon waypoint on legitimately open floor to a wildly
   far, unreliable point), so a radial cutoff bounds *how unreliable* the farthest retained point
   can be. It is not obstacle safety — the costmap + MPPI handle that — but a guardrail against
   that projection blow-up and against a long move committing the robot to a big uncorrected drive
   past the sensed window (against the re-plan-every-cycle cadence). Independent of
   `obstacle_projector`'s `bev_range` (the sensed horizon); default `5.0` sits a little beyond it.
1. **Smooths + densifies** the grounded points before handing them to MPPI: a **centripetal
   Catmull-Rom** spline is fit through the (few, far-apart) waypoints and resampled at
   `path_resolution` m (default `0.05`, ≈ costmap resolution). MPPI's path critics
   (`offset_from_furthest`, path-align) are index-based and assume a costmap-resolution path;
   feeding them the raw sparse waypoints stalled the optimizer mid-path on long paths
   (the robot slowed and turned in place until `FollowPath` aborted). Centripetal
   parameterization keeps the smoothed curve close to the polyline (no cusps/overshoot), and
   the endpoints stay exactly on the robot pose and final waypoint.
2. **Calls** `follow_path` (`controller_id: FollowPath`), relays feedback, and maps the
   result: Nav2 `SUCCEEDED` → `success=true`; `ABORTED` (incl. `SimpleProgressChecker` firing
   on an unreachable goal — the "stall" backstop) / `CANCELED` / rejected → `success=false`.
3. Handles the **turn-only** move: an empty `waypoints` goal succeeds immediately (nothing to
   follow), so the BT's `SpinAction` that runs next performs the rotation.

It also republishes the grounded path purely for RViz, from a **`control_rate` timer that runs
continuously** — not only while a follow is active, but through the end-of-move Spin and idle,
until the next path replaces it. Each tick re-stamps the path to *now*, which is what lets RViz
transform it against the **live** `odom → base_link` and render it correctly in an **ego
(`base_link`) view**. (Republishing only inside the follow action froze the stamp the moment the
follow ended, so during the Spin RViz used a stale transform and the path rotated rigidly with
the robot instead of staying put in the world.)
- **`~/path`** — the VLM-projected path, both handed to MPPI and republished for RViz.

### Interfaces

| Interface | Type | Direction |
|---|---|---|
| `~/follow_visual_path` | `hint_interfaces/FollowVisualPath` | Action server (BT-facing) |
| `follow_path` (see `follow_path_action`) | `nav2_msgs/FollowPath` | Action client (Nav2 controller) |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | Sub (TF listener) — `path_frame ← robot_frame` at the goal stamp, the world anchor for grounding |
| `~/path` | `nav_msgs/Path` (latched) | Pub — the VLM-projected path handed to MPPI, for RViz |

### Key parameters

Rig: `camera_height` / `camera_forward_offset` / `camera_tilt` / `camera_hfov_deg` (owned by
this package; injected by bringup — see [camera_rig](#camera_rig-camera_rigpy)); `image_width`
/ `image_height` (waypoint normalization reference, default 640×480); `follow_path_action`
(default `/follow_path`), `controller_id` (`FollowPath`), `goal_checker_id` (`goal_checker`),
`progress_checker_id` (`progress_checker`); `path_frame` (`odom`) / `robot_frame`
(`base_link`) — the TF pair grounded against; `tf_buffer_time` (`90.0` s — TF buffer
`cache_time`; must cover the VLM latency from frame capture to grounding, i.e. the director
+ planner calls, so bringup derives it from `vlm_timeout`) and `tf_lookup_timeout` (`0.1` s —
brief blocking wait on the stamped lookup); `server_timeout`, `control_rate`;
`path_resolution` (`0.05` m — Catmull-Rom smooth-densification spacing for the path handed to
MPPI; live-adjustable); `path_range` (`5.0` m — max straight-line distance from the robot before the
grounded path is distance-clipped; `≤ 0` disables it. Deliberately independent of
`obstacle_projector`'s `bev_range`, though bringup can set them equal; the `5.0` default sits a
little beyond the sensed horizon).

## obstacle_projector

The streaming, image→metric half of the bridge. Subscribes to `hint_perception`'s ground mask
(`/camera/ground`, `mono8`, 255 = ground) and warps it through the ground-plane homography
(built from `CameraRig.ground_to_pixels` on the BEV window corners) into a top-down grid: a
cell that is **known** (inside the camera wedge) but **not ground** is an obstacle. Those
obstacle cell centres are published as a `PointCloud2` on `/obstacles` (`base_link`, z = 0),
one point per BEV cell.

**Latency compensation** is end-to-end: the cloud is stamped with the **mask's** header stamp
(which the segmenter inherits from the source camera frame), so Nav2's obstacle layer
TF-transforms `base_link → odom` at capture time — landing the points where they were seen,
not where the robot is now.

> Split out of the old fused segmenter so perception stays purely image-space. Kept a
> **separate node** from `path_projector` on purpose: the obstacle cloud is
> safety-critical streaming that must keep flowing while the projector's `follow_visual_path`
> action blocks for a whole path-follow, and separate processes get independent launch respawn.

### Interfaces

| Topic | Type | Direction |
|---|---|---|
| `/camera/ground` (see `mask_topic`) | `sensor_msgs/Image` (`mono8`) | Sub — the ground mask |
| `/obstacles` | `sensor_msgs/PointCloud2` | Pub — non-ground cell centres in `base_link` (z=0), consumed by the local costmap |

### Parameters

Rig (see [camera_rig](#camera_rig-camera_rigpy)), plus the BEV grid geometry:

| Parameter | Default | Effect |
|---|---|---|
| `bev_range` | `3.0` | Forward extent of the BEV window (m) |
| `bev_resolution` | `0.05` | BEV cell size (m) — one obstacle point per cell (~ costmap resolution) |
| `obstacle_frame` | `base_link` | Frame the obstacle cloud is published in |
| `mask_topic` | `/camera/ground` | Ground-mask input topic |

The window's **lateral** half-width is not a parameter — it is derived from the rig
(`camera_hfov_deg`, `camera_tilt`, `camera_height`, `camera_forward_offset`) and `bev_range` as
the lateral extent the camera actually sees at the far range. Widening it independently would only
add columns the camera never images (masked out as unknown); narrowing it would discard obstacle
pixels at the far sides. So `bev_range` is the single forward horizon, and the sides follow from
the FOV.

## visual_debug

The single place for live visualization. Instead of every node shipping its own debug image,
each node publishes only its **real output**, and this node layers those into one **`/debug`**
image (`sensor_msgs/Image`, `bgr8`). Rendering is **subscriber-gated** — nothing is composed
or published unless something subscribes to `/debug`. It lives here (not in perception)
because it needs the rig to re-project the projector's odom paths onto the frame.

Layers (each toggled by a `show_*` param):
1. **Backdrop** — the camera frame.
2. **Ground overlay** — the binary mask tinted (muted green = ground, coral = not), alpha-blended.
3. **Path** — the projector's `~/path` (the VLM-projected path), transformed
   `odom → current base_link` (via `CameraRig.ground_to_pixels`) and projected onto the frame,
   so it tracks as the robot moves. Teal.
4. **BT state** — the mission tree's live state (`/hint_behavior_server/state`) as text, top-left.

```bash
ros2 run rqt_image_view rqt_image_view /debug
ros2 param set /visual_debug_node show_mask false        # toggle any layer live
```

### Interfaces

| Topic | Type | Direction |
|---|---|---|
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | Sub — backdrop, drives the render |
| `/camera/ground` (see `mask_topic`) | `sensor_msgs/Image` (`mono8`) | Sub — ground overlay |
| `/path_projector_node/path` | `nav_msgs/Path` | Sub — the VLM-projected path |
| `/odom` | `nav_msgs/Odometry` | Sub — pose for re-projecting the paths |
| `/hint_behavior_server/state` | `std_msgs/String` | Sub — mission-tree state text |
| `/debug` | `sensor_msgs/Image` (`bgr8`) | Pub — the single composited debug image |

### Parameters

Rig (see [camera_rig](#camera_rig-camera_rigpy)); layer toggles `show_mask` / `show_path` /
`show_bt_state` (all default true); `overlay_alpha` (0.35); and a `*_topic`
name per input (`mask_topic` defaults to `/camera/ground`).

## Nav2 config (`config/nav2_local.yaml`)

- **`controller_server`** — `FollowPath` = `nav2_mppi_controller::MPPIController`
  (`motion_model: DiffDrive`, forward-only `vx_min: 0`, Waffle-Pi limits). `SimpleProgressChecker`
  (the unreachable-goal backstop → `follow_path` ABORTs) and `SimpleGoalChecker` (positional
  arrival; yaw effectively ignored). `enable_stamped_cmd_vel: true` (TB3 consumes stamped
  `/cmd_vel`). `FollowPath.visualize` publishes MPPI's `trajectories` /
  `transformed_global_plan` markers for RViz.
- **`local_costmap`** — rolling, `global_frame: odom`, `robot_base_frame: base_link`,
  `obstacle_layer` fed by `/obstacles` (`PointCloud2`) + `inflation_layer` (the keep-out
  margin). No static/voxel layer, no global costmap.
- **`behavior_server`** — the `spin` behavior only, mapless (its `global_*` costmap topics
  point at the local costmap). Drives the BT's end-of-move / scan turn via the `/spin` action.

Lifecycle order in `nav2.launch.py` is `["controller_server", "behavior_server"]` (controller
first, so its local costmap is up before the Spin server subscribes to it).

## Reference phase & localization (the mission test protocol)

A HINT mission is evaluated by **replicating a Nav2 ground-truth run**: map the region, drive an
arbitrary Nav2 GoToGoal on it (the *reference*), then describe that route semantically and have
HINT reproduce it — and finally overlay HINT's actual path against the reference on the same map
(`hint_narrative`'s `mission_report --reference`). This is a **two-session** protocol so the
reference and HINT trajectories share one fixed map frame: both localize with AMCL on the **same
saved map**.

Maps live in `hint_navigation/maps/<region>/` (one `<region>` per physical environment; several
missions can reuse a map). The `map`/`region` launch args resolve there by default; the reference
bag is a sibling (`reference.bag`).

**Step 1 — build + save the map** (Cartographer SLAM). Drive the region with teleop; a background
autosaver writes `maps/<region>/map` every `save_interval` s (default 5) so the map is always on
disk (self-contained — no second terminal). Ctrl+C to stop — the last autosave is your map, so
pause driving a moment before quitting:

```bash
ros2 launch hint_navigation mapping.launch.py region:=<region>
```

**Step 2 — record the reference GoToGoal** (full Nav2 on the saved map):

```bash
ros2 launch hint_navigation reference.launch.py region:=<region>
# in RViz: set the initial pose (2D Pose Estimate), then click a Nav2 goal — it drives there.
ros2 bag record --storage mcap -o hint_navigation/maps/<region>/reference.bag /tf /tf_static /odom
```

**Step 3 — run HINT** and compare: `hint_bringup`'s bringup includes `localization.launch.py`
(AMCL + map_server on the same `maps/<region>/map.yaml`, selected by its `region` arg), so the
mission bag records the actual trajectory in the map frame. Then:

```bash
ros2 run hint_narrative mission_report missions/<name>/mission.bag \
  --reference hint_navigation/maps/<region>/reference.bag
```

Requires (beyond the mapless-stack deps): `turtlebot3_cartographer`, `nav2_bringup`,
`nav2_map_server`, `nav2_amcl`. `reference.launch.py`/`localization.launch.py` reuse
`nav2_bringup`'s `bringup_launch.py` / `localization_launch.py`.

## Notes

- The costmap **remembers** obstacles within the rolling window until re-observed/scrolled out
  (a forward camera can't clear what falls behind it) — this is short-term memory, and MPPI
  plans against all of it. For a purely-reactive *view*, display `/obstacles` (Decay Time 0)
  rather than the costmap.
- The path is anchored in `odom` at plan time and followed there; it is re-grounded fresh each
  mission cycle, which bounds odometry drift over a single move.
