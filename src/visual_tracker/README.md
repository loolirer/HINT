# visual_tracker

Classic-CV image-space trackers for egocentric navigation. The package ships two
executables that share a common tracking core (`tracking_common.py`: LK
forward-backward flow, feature detection, NCC, bbox geometry):

| Executable | Tracks | Started by | Publishes |
|---|---|---|---|
| `lk_tracker` | one landmark region (a bounding box) | `set_target` service | bbox + state |
| `waypoint_tracker` | an ordered set of ground-plane waypoints | `set_waypoints` service | normalized point stream + state |

```bash
colcon build --symlink-install --packages-select hint_interfaces visual_tracker
source install/setup.bash
```

---

## lk_tracker

Lucas-Kanade optical flow tracker for visual landmark tracking. Tracks a single
static scene region (not moving objects) from frame to frame and publishes the
tracked bounding box and tracking state.

### Usage

```bash
ros2 run visual_tracker lk_tracker
```

Start tracking a region (stamp `0` uses the next incoming frame):

```bash
ros2 service call /lk_tracker_node/set_target hint_interfaces/srv/SetTarget \
  "{roi: {x_offset: 220, y_offset: 140, width: 200, height: 200, do_rectify: false}, stamp: {sec: 0, nanosec: 0}}"
```

Stop tracking:

```bash
ros2 service call /lk_tracker_node/stop_tracking hint_interfaces/srv/StopTracking
```

Observe state and bbox:

```bash
ros2 topic echo /tracking/state
ros2 topic echo /tracking/bbox
```

### Algorithm

1. `goodFeaturesToTrack` + `cornerSubPix` inside the initial ROI to seed LK points.
2. Frame-to-frame LK with forward-backward consistency filtering — discards points whose round-trip error exceeds `fb_thresh`.
3. `estimateAffinePartial2D` (4 DOF: scale + rotation + translation) — full homography is intentionally avoided as it overfits noise.
4. NCC appearance check: warps the current frame back to the init viewpoint via `invertAffineTransform` and compares to the stored init patch. Catches visually-different obstacles and static coverage.
5. RANSAC inlier ratio check: a second independent occlusion signal that catches moving obstacles whose features disagree with the landmark's motion model.
6. EMA smoothing on output quad corners.
7. ORB descriptor fingerprint computed at init, used for re-detection when the tracker is OCCLUDED.

**State machine**

```
UNTRACKED ──► (set_target called) ──► TRACKING ──► (NCC or inlier fail) ──► OCCLUDED
                                          ▲                                       │
                                          └──────── (ORB re-detection) ───────────┘
```

Transitions back to UNTRACKED only if the initial ROI had too few features, or via `stop_tracking`.

### Interfaces

| Interface | Type | Direction |
|---|---|---|
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | Sub |
| `~/set_target` | `hint_interfaces/SetTarget` | Service server |
| `~/stop_tracking` | `hint_interfaces/StopTracking` | Service server |
| `/tracking/state` | `std_msgs/String` (latched) | Pub — `UNTRACKED` / `TRACKING` / `OCCLUDED` |
| `/tracking/bbox` | `sensor_msgs/RegionOfInterest` | Pub — axis-aligned bbox of the tracked quad |
| `/camera/tracking` | `sensor_msgs/Image` | Pub — debug overlay |

#### `set_target` service

| Field | Type | Notes |
|---|---|---|
| `roi` | `sensor_msgs/RegionOfInterest` | Initial tracking region |
| `stamp` | `builtin_interfaces/Time` | Specific frame to initialise on; `{sec: 0, nanosec: 0}` uses the next arriving frame |

The node keeps a ring buffer of the last 10 frames. If the requested stamp is found in the buffer, that frame is used for feature detection; otherwise the current frame is used.

### Parameters

All parameters are live-adjustable via `ros2 param set`.

| Parameter | Default | Effect |
|---|---|---|
| `ncc_thresh` | 0.60 | Occlusion floor — raise (0.6–0.7) to reject obstacles more aggressively |
| `ncc_redetect_thresh` | 0.75 | Feature re-detection gate — always keep above `ncc_thresh` |
| `inlier_ratio_thresh` | 0.60 | RANSAC inlier fraction required — raise (0.6–0.7) to catch moving obstacles stricter |
| `fb_thresh` | 2.0 | Forward-backward round-trip tolerance in pixels |
| `ema_alpha` | 0.20 | Corner smoothing weight (lower = smoother, more lag) |
| `max_features` | 300 | Max corners detected in the ROI at init |
| `min_features` | 10 | Minimum surviving points before declaring occlusion |

### Tuning

| Symptom | Adjustment |
|---|---|
| Box drifts onto obstacle | Raise `ncc_redetect_thresh` to 0.80, raise `inlier_ratio_thresh` to 0.65 |
| False occlusion on camera move | Lower `ncc_thresh` to 0.35, lower `inlier_ratio_thresh` to 0.35 |
| Box jitters on static camera | Lower `ema_alpha` to 0.10–0.15 |
| Loses track too easily | Lower `min_features` to 15, raise `fb_thresh` to 3.0 |
| Tracks garbage after init | Raise `qualityLevel` in `_detect_features` to 0.03–0.05 |

---

## waypoint_tracker

Tracks a VLM-planned ground trajectory in the image and re-publishes the
waypoints' current pixel positions as a **continuous stream**, so the robot
*remembers* where the waypoints are while it drives instead of navigating open
loop. It consumes the ordered normalized points produced by
`gemini_robotics_er/trajectory_planner` (`PlanTrajectory.markers`) — a planner
result feeds straight into `set_waypoints`.

### Algorithm — keyframe-anchored ground homography

All waypoints lie on the **same ground plane**, so one planar homography of the
ground describes how all of them move. Instead of advecting each waypoint by its
own (noisy) flow, the node tracks the *plane* and warps the whole waypoint set —
the pooling + plane constraint is what *corrects* the per-point noise that made
pure advection drift. Anchored to a **keyframe**, each frame:

1. **Dense flow over a path-masked grid.** DIS (`cv2.DISOpticalFlow`, preset via
   `dis_preset`) carries a grid (`grid_step`) from the keyframe to the current
   frame, both directions. The grid is **masked to a band around the waypoint
   path** — filled convex hull + a tube of half-width `roi_margin·span` along the
   ordered waypoints — so flanking walls and off-path moving objects stay out of
   the correspondences (which is what makes RANSAC hard to "kidnap"). The grid
   still gives plane correspondences even on blank floor.
2. **Forward-backward reject** grid vectors whose round trip exceeds `fb_thresh`
   (count shown as `reject=`).
3. **RANSAC transform.** Fit a **homography** (`findHomography`) when at least
   `min_homography_features` inliers support it, else a 4-DOF **affine** fallback.
   The inlier fraction is a global occlusion signal.
4. **Warp the waypoints** through the transform (`perspectiveTransform`). A
   covered or blank-floor waypoint is still placed correctly from texture
   elsewhere on the plane. If the fit is unreliable (`inlier_ratio_thresh`) or
   degenerate, the waypoints **hold** their last position instead of following it.
5. **Re-key** (advance the keyframe) only when the keyframe→current baseline grows
   past `rekey_flow_px` (median grid flow) or the flow breaks — so residual drift
   ticks at these infrequent events, not every frame.

The debug overlay shows the inlier grid (dots), the waypoint polyline, and
`model= inliers= reject= rekeys= kf_disp=`.

Waypoints are seeded (in pixels) from `set_waypoints` on the selected frame; one
carried off-image is still published, flagged `tracked=false`.

**State:** `UNTRACKED` before any waypoints (also if the ground ROI is too small
to seed a grid at init), `TRACKING` after `set_waypoints`. `stop_tracking` (or a
new `set_waypoints`) returns to `UNTRACKED`.

### Usage

```bash
ros2 run visual_tracker waypoint_tracker
```

Start tracking a trajectory (stamp `0` uses the next frame; points are normalized
`x`/`y ∈ [-1, 1]`, center = 0, ordered nearest → farthest). This example walks up
the middle of the frame, drifting slightly right, from near the bottom upward:

```bash
ros2 service call /waypoint_tracker_node/set_waypoints hint_interfaces/srv/SetWaypoints \
  "{waypoints: [{x: 0.0, y: 0.8, z: 0.0}, {x: 0.05, y: 0.2, z: 0.0}, {x: 0.1, y: -0.4, z: 0.0}], stamp: {sec: 0, nanosec: 0}}"
```

Stop tracking:

```bash
ros2 service call /waypoint_tracker_node/stop_tracking hint_interfaces/srv/StopTracking
```

Observe the state and the point stream:

```bash
ros2 topic echo /waypoint_tracking/state
ros2 topic echo /waypoint_tracking/points
```

### Interfaces

| Interface | Type | Direction |
|---|---|---|
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | Sub |
| `~/set_waypoints` | `hint_interfaces/SetWaypoints` | Service server |
| `~/stop_tracking` | `hint_interfaces/StopTracking` | Service server |
| `/waypoint_tracking/state` | `std_msgs/String` (latched) | Pub — `UNTRACKED` / `TRACKING` |
| `/waypoint_tracking/points` | `hint_interfaces/VisualWaypoints` | Pub — normalized waypoint stream |
| `/camera/waypoint_tracking` | `sensor_msgs/Image` | Pub — debug overlay |

**`VisualWaypoints` layout** — same `geometry_msgs/Point[]` element type as the
`set_waypoints` input, order preserved (index 0 nearest → last farthest):

| Field | Type | Value |
|---|---|---|
| `header` | `std_msgs/Header` | inherits the source frame's stamp/frame_id |
| `points` | `geometry_msgs/Point[]` | normalized image coords, `x`/`y ∈ [-1, 1]` (center 0), `z` unused |
| `tracked` | `bool[]` | parallel to `points`: `true` while the waypoint is in-frame, `false` once flow carries it off-image |

#### `set_waypoints` service

| Field | Type | Notes |
|---|---|---|
| `waypoints` | `geometry_msgs/Point[]` | Ordered ground waypoints, normalized `x`/`y ∈ [-1, 1]` (center 0), `z` unused; nearest first |
| `stamp` | `builtin_interfaces/Time` | Frame to initialise on; `{sec: 0, nanosec: 0}` uses the next arriving frame |

Like `lk_tracker`, the node keeps a ring buffer of the last 10 frames for
stamp-based initialisation.

### Parameters

| Parameter | Default | Effect |
|---|---|---|
| `dis_preset` | `ultrafast` | DIS accuracy/speed: `ultrafast` / `fast` / `medium` (more accurate = less drift, slower). Applied at startup, **not** live-adjustable. |
| `fb_thresh` | 2.0 | Forward-backward round-trip tolerance (px) for grid vectors |
| `rekey_flow_px` | 40.0 | Re-key when the median keyframe→current grid flow exceeds this (px). Lower = fresher flow but more frequent drift ticks; higher = larger baseline strains DIS |
| `grid_step` | 20 | Spacing (px) of the ground flow grid; lower = denser (more robust, slower) |
| `roi_margin` | 0.10 | Half-width of the path band (fraction of waypoint span) the grid is masked to. Narrower = fewer off-path/wall/mover vectors (harder to kidnap) but fewer features; wider = more support but more intrusion |
| `min_features` | 10 | Minimum surviving inlier grid points before the flow is treated as broken (re-anchor) |
| `min_homography_features` | 20 | Inlier count below which the estimator drops from homography to affine |
| `inlier_ratio_thresh` | 0.50 | RANSAC inlier fraction below which the fit is untrusted and waypoints hold |

> Remaining layer (odometry fusion — a scene-independent prior that gates
> occluders and coasts through blank stretches) is the planned follow-up. This
> layer *corrects* per-point noise via the pooled plane fit; residual drift still
> ticks at each re-key.
