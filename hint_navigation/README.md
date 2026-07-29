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

0. **Ground-clips** the VLM pixel path: each normalized waypoint is tested against
   `hint_perception`'s binary ground mask (`/camera/ground`, matched to the goal's frame
   stamp); the **leading run** of on-ground waypoints is kept, and the **first waypoint that
   leaves the ground is dropped along with every waypoint after it** — so the robot never
   follows a path that runs off the floor. No fresh mask → the path passes through
   unclipped. If nothing survives (all off-ground, or the VLM sent none), the goal succeeds
   as a no-op so the BT's Spin still runs.
1. **Grounds** the surviving waypoints (`x`/`y ∈ [-1, 1]`, nearest-first) onto the ground plane
   in `base_link` via `CameraRig.pixels_to_ground`, then re-expresses them in **`odom`** using
   the robot pose at the goal's stamp — looked up from **TF** (`odom → base_link` at that
   stamp), the same transform the costmap and MPPI use. So the path is anchored once in the
   world, and the robot follows it while it stays put in `odom`. No `/odom` subscription: the
   pose source is TF, which also keeps grounding in step with the costmap. Because the frame
   is captured seconds before grounding (it flows through the director *and* planner VLM
   calls), the TF buffer's `cache_time` (`tf_buffer_time`) must cover that latency, or the
   stamped lookup falls out of the buffer and the goal aborts. Prepends the robot's own pose
   so the path starts at the robot; each pose's yaw is the path tangent.
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

It also republishes two grounded paths (re-stamped, at control rate) purely for RViz —
re-stamping is what lets them render correctly in an **ego (`base_link`) view** instead of
freezing at plan time:
- **`~/path`** — the ground-clipped path actually handed to MPPI.
- **`~/path_raw`** — the **full VLM path** as grounded (never followed), so you can
  see what the model intended vs what survived the clip. When nothing is clipped the two
  coincide.

### Interfaces

| Interface | Type | Direction |
|---|---|---|
| `~/follow_visual_path` | `hint_interfaces/FollowVisualPath` | Action server (BT-facing) |
| `follow_path` (see `follow_path_action`) | `nav2_msgs/FollowPath` | Action client (Nav2 controller) |
| `/camera/ground` (see `mask_topic`) | `sensor_msgs/Image` (`mono8`) | Sub — ground mask for clipping the pixel path |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | Sub (TF listener) — `path_frame ← robot_frame` at the goal stamp, the world anchor for grounding |
| `~/path` | `nav_msgs/Path` (latched) | Pub — the ground-clipped path handed to MPPI, for RViz |
| `~/path_raw` | `nav_msgs/Path` (latched) | Pub — the full VLM path grounded (debug; never followed) |

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
MPPI; live-adjustable).

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
| `bev_half_width` | `1.5` | Lateral extent each side (m) |
| `bev_resolution` | `0.05` | BEV cell size (m) — one obstacle point per cell (~ costmap resolution) |
| `obstacle_frame` | `base_link` | Frame the obstacle cloud is published in |
| `mask_topic` | `/camera/ground` | Ground-mask input topic |

## visual_debug

The single place for live visualization. Instead of every node shipping its own debug image,
each node publishes only its **real output**, and this node layers those into one **`/debug`**
image (`sensor_msgs/Image`, `bgr8`). Rendering is **subscriber-gated** — nothing is composed
or published unless something subscribes to `/debug`. It lives here (not in perception)
because it needs the rig to re-project the projector's odom paths onto the frame.

Layers (each toggled by a `show_*` param):
1. **Backdrop** — the camera frame.
2. **Ground overlay** — the binary mask tinted (muted green = ground, coral = not), alpha-blended.
3. **Paths** — the projector's `~/path_raw` (full VLM intent) and `~/path` (followed, ground-clipped),
   each transformed `odom → current base_link` (via `CameraRig.ground_to_pixels`) and projected
   onto the frame, so they track as the robot moves. Amber = intent, teal = followed.
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
| `/path_projector_node/path` | `nav_msgs/Path` | Sub — followed (clipped) path |
| `/path_projector_node/path_raw` | `nav_msgs/Path` | Sub — full VLM-intent path |
| `/odom` | `nav_msgs/Odometry` | Sub — pose for re-projecting the paths |
| `/hint_behavior_server/state` | `std_msgs/String` | Sub — mission-tree state text |
| `/debug` | `sensor_msgs/Image` (`bgr8`) | Pub — the single composited debug image |

### Parameters

Rig (see [camera_rig](#camera_rig-camera_rigpy)); layer toggles `show_mask` / `show_path` /
`show_path_raw` / `show_bt_state` (all default true); `overlay_alpha` (0.35); and a `*_topic`
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

## Notes

- The costmap **remembers** obstacles within the rolling window until re-observed/scrolled out
  (a forward camera can't clear what falls behind it) — this is short-term memory, and MPPI
  plans against all of it. For a purely-reactive *view*, display `/obstacles` (Decay Time 0)
  rather than the costmap.
- The path is anchored in `odom` at plan time and followed there; it is re-grounded fresh each
  mission cycle, which bounds odometry drift over a single move.
