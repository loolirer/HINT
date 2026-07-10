# visual_tracker

Classic-CV image-space trackers for egocentric navigation. The package ships two
executables that share a common tracking core (`tracking_common.py`: LK
forward-backward flow, feature detection, NCC, bbox geometry):

| Executable | Tracks | Started by | Publishes |
|---|---|---|---|
| `lk_tracker` | one landmark region (a bounding box) | `set_target` service | bbox + state |
| `waypoint_tracker` | an ordered set of ground-plane waypoints (DIS optical flow) | `set_waypoints` service | normalized point stream + state |
| `odom_waypoint_tracker` | the same ground-plane waypoints, but re-projected from `/odom` + camera geometry (no image flow) | `set_waypoints` service | normalized point stream + state |

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
   A candidate homography is additionally **sanity-checked** (orientation-preserving,
   bounded scale and anisotropy) and rejected to the affine fallback if degenerate,
   reflected or wildly sheared. RANSAC already rejects gross (e.g. far-occluded)
   correspondences as outliers, so the fit is robust to them.
4. **Re-anchor the plane on the near support.** The RANSAC inliers within
   `support_radius` of the nearest `priority_count` waypoints re-fit the plane
   (least-squares, no RANSAC) so residual **far**-plane noise doesn't tilt what the
   near waypoints ride on — the near end governs, the far end is extrapolated.
5. **Warp the waypoints** through the transform (`perspectiveTransform`). A
   covered or blank-floor waypoint is still placed correctly from texture elsewhere
   on the plane.
6. **Per-waypoint occlusion.** Each waypoint is *measured* when at least
   `min_support` inlier grid points fall within `support_radius` of its warped
   position, else *coasting* — still placed by the plane but flagged
   `tracked=false`. This is what stops a single far occlusion from poisoning the
   whole set: it only flips that waypoint's flag. The node's `TRACKING`/`OCCLUDED`
   state keys on the nearest `priority_count` **in-frame** waypoints, re-selected
   each frame — so waypoints scrolling off the frame bottom as the robot advances
   don't read as loss.
7. **Re-key** (advance the keyframe) only when the keyframe→current baseline grows
   past `rekey_flow_px` (median grid flow) **and** the current fit is high-confidence
   (inlier fraction ≥ `rekey_inlier_ratio`), or when the flow breaks. A re-key
   freezes the current estimate in as the new anchor, so gating it on fit quality
   keeps the error committed at each — hence residual drift — low. These are
   infrequent events, so drift ticks there, not every frame.

The debug overlay shows the inlier grid (dots), the waypoint polyline (measured
waypoints **filled**, coasting ones **hollow**), and
`meas= inliers= reject= rekeys= kf_disp=`.

Waypoints are seeded (in pixels) from `set_waypoints` on the selected frame; a
coasting or off-image waypoint is still published (while `TRACKING`), flagged
`tracked=false`.

**State:** mirrors `lk_tracker`'s dynamics, but the occlusion trigger is the
**nearest in-frame prefix**, not the global fit. `UNTRACKED` before any waypoints
(also if the ground ROI is too small to seed a grid at init); `TRACKING` after
`set_waypoints`; `OCCLUDED` the moment the **nearest `priority_count` in-frame
waypoints** all lose local support (or flow breaks / the warp is degenerate / no
waypoint is in-frame) — that flips `TRACKING → OCCLUDED` immediately (single frame,
no timeout — the give-up timeout on a prolonged occlusion belongs to the IBVS
consumer, exactly as `visual_servo` applies `occlusion_timeout` to `lk_tracker`'s
`OCCLUDED`). A far waypoint losing support does **not** trip the state; it just
publishes `tracked=false` while the near end keeps `TRACKING`. Following
`lk_tracker` (which stops publishing `/tracking/bbox` while occluded), **no points
are published while `OCCLUDED`** — the frozen positions go stale as the robot
moves, so `OCCLUDED` on the state topic is the "stop, don't coast" signal, not a
held target to chase. Regaining near support flips `OCCLUDED → TRACKING`
immediately. If the near end fails *before any* trusted fit was produced after
init, the track never established and it drops back to `UNTRACKED`. `stop_tracking`
(or a new `set_waypoints`) also returns to `UNTRACKED`.

```
UNTRACKED ──► (set_waypoints) ──► TRACKING ──► (near prefix lost) ──► OCCLUDED
                                     ▲                                    │
                                     └────────── (near support back) ─────┘
```

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
| `/waypoint_tracking/state` | `std_msgs/String` (latched) | Pub — `UNTRACKED` / `TRACKING` / `OCCLUDED` (the feedback topic; gate on it) |
| `/waypoint_tracking/points` | `hint_interfaces/VisualWaypoints` | Pub — normalized waypoint stream, **only while `TRACKING`** |
| `/camera/waypoint_tracking` | `sensor_msgs/Image` | Pub — debug overlay |

> **Consumer contract.** Points are published **only while `TRACKING`**; the stream
> goes silent on `OCCLUDED`/`UNTRACKED` (lk_tracker parity). A follower must gate on
> `/waypoint_tracking/state` **and** point-message freshness — treat "not `TRACKING`"
> or stale points as *stop*, never coast on the last message. Steer only by
> `tracked=true` waypoints; a `tracked=false` point is a plane extrapolation (and is
> clamped to the frame edge), not a measurement.

**`VisualWaypoints` layout** — same `geometry_msgs/Point[]` element type as the
`set_waypoints` input, order preserved (index 0 nearest → last farthest):

| Field | Type | Value |
|---|---|---|
| `header` | `std_msgs/Header` | inherits the source frame's stamp/frame_id |
| `points` | `geometry_msgs/Point[]` | normalized image coords, `x`/`y ∈ [-1, 1]` (center 0), `z` unused; **clamped** to `[-1, 1]` (a coasting point can extrapolate off-frame — the value is clamped to the frame edge) |
| `tracked` | `bool[]` | parallel to `points`: `true` when the waypoint is **measured** (≥ `min_support` inlier grid points within `support_radius`), `false` when it is **coasting** — placed by the plane fit but occluded / off-image / unsupported |

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
| `dis_preset` | `fast` | DIS accuracy/speed: `ultrafast` / `fast` / `medium` (more accurate = less drift, slower). Applied at startup, **not** live-adjustable. |
| `fb_thresh` | 1.0 | Forward-backward round-trip tolerance (px) for grid vectors |
| `rekey_flow_px` | 30.0 | Re-key when the median keyframe→current grid flow exceeds this (px). Lower = fresher flow but more frequent drift ticks; higher = larger baseline strains DIS |
| `grid_step` | 10 | Spacing (px) of the ground flow grid; lower = denser (more robust, slower) |
| `roi_margin` | 0.10 | Half-width of the path band (fraction of waypoint span) the grid is masked to. Narrower = fewer off-path/wall/mover vectors (harder to kidnap) but fewer features; wider = more support but more intrusion |
| `min_features` | 10 | Minimum surviving inlier grid points before the flow is treated as broken (re-anchor) |
| `min_homography_features` | 20 | Inlier count below which the estimator drops from homography to affine |
| `rekey_inlier_ratio` | 0.90 | Minimum (global) inlier fraction required to commit a re-key. A re-key freezes the current estimate in as the new anchor, so a mediocre-fit frame is deferred (keyframe held) until a cleaner one — keeps per-re-key error, hence drift, low |
| `priority_count` | 2 | Nearest N waypoints whose support governs the node state. The node stays `TRACKING` while ≥1 of these is measured; `OCCLUDED` only when the near end loses support. Higher = more near waypoints must all drop before `OCCLUDED` (more tolerant); 1 = the single nearest decides |
| `support_radius` | 60.0 | Radius (px) around a waypoint within which inlier grid points count as its local support — used both for per-waypoint measured/coasting and for the near-band plane re-fit. Larger = more forgiving (a waypoint stays "measured" with sparser nearby texture); smaller = stricter/more local |
| `min_support` | 3 | Inlier grid points required within `support_radius` for a waypoint to be **measured** (`tracked=true`); below it the waypoint coasts on the plane fit |

> Remaining layer (odometry fusion — a scene-independent prior that gates
> occluders and coasts through blank stretches) is the planned follow-up. This
> layer *corrects* per-point noise via the pooled plane fit; residual drift still
> ticks at each re-key.

---

## odom_waypoint_tracker

A **drop-in sibling** of `waypoint_tracker` with the *same tracking dynamics*
(state machine, per-waypoint measured/coasting flags, nearest-prefix occlusion,
under-robot retirement, publish-only-while-`TRACKING` contract) but a different
*measurement backend*: instead of estimating where the waypoints moved with DIS
optical flow, it **dead-reckons them from `/odom` and a pin-hole camera model**.
No image content is used to place the points — the image only drives the publish
rate, the header, and the debug overlay. It publishes on the **same topics** as
`waypoint_tracker` (`/waypoint_tracking/points`, `/waypoint_tracking/state`,
`/camera/waypoint_tracking`) and answers the same `~/set_waypoints` /
`~/stop_tracking` services, so `pursuit_servo` consumes it unchanged — the two are
**mutually exclusive** (never run both; they'd both drive `/waypoint_tracking`).

### Algorithm — ground-plane grounding + odometry re-projection

Every waypoint is assumed to lie on the ground plane, so:

1. **Ground the waypoints once.** On `set_waypoints`, each normalized image point is
   back-projected through the camera (`camera_height`, `camera_tilt`,
   `camera_hfov_deg`, `camera_forward_offset`) onto the ground, giving a fixed 2-D
   ground point `(X forward, Y left)` in the **reference frame** — the robot's
   odometry pose at the instant the trajectory arrived. The reference frame is
   re-anchored every time a new trajectory is received.
2. **Dead-reckon the camera.** Each camera frame, the current odometry pose is
   expressed relative to the reference pose (a planar rigid transform), so the fixed
   ground points are re-expressed in the *current* robot frame.
3. **Re-project.** The current-frame ground points are projected back through the
   same camera model to pixels and published as the normalized waypoint stream. The
   **full** set is re-projected and published every frame — the tracker **never
   retires** waypoints. Deciding when a waypoint has been *reached* (driven over) and
   advancing through the trajectory is the follower's job (`pursuit_servo`), since
   reaching is an act of the servo, not the tracker. Waypoints are kept in the exact
   order sent (nearest-first per the interface) — never re-sorted; the priority
   prefix and the published stream both walk the as-sent order.

Because placement is a geometric prediction rather than a visual measurement,
**"occlusion" means odometry loss**: the nearest `priority_count` in-frame
waypoints all leaving the frame / projecting behind the camera, or `/odom` going
stale (older than `odom_timeout`). A waypoint is *measured* (`tracked=true`) only
while it projects **in front of the camera and inside the frame**; otherwise it
*coasts* (`tracked=false`, still placed by the prediction, clamped to the frame
edge at publish time). Following `lk_tracker`/`waypoint_tracker`, **no points are
published while `OCCLUDED`** — the frozen prediction goes stale as the robot moves,
so `OCCLUDED` is the "stop, don't coast" signal.

**Camera convention:** OpenCV optical frame (x right, y down, z into scene); robot
frame REP-103 (x forward, y left, z up). The camera sits `camera_height` above and
`camera_forward_offset` ahead of the base origin, pitched `camera_tilt` radians
**down** from horizontal; principal point assumed at the image center, square
pixels, focal length derived from `camera_hfov_deg` and the frame width.

**State** — identical to `waypoint_tracker`:

```
UNTRACKED ──► (set_waypoints) ──► TRACKING ──► (near prefix off-frame / odom stale) ──► OCCLUDED
                                     ▲                                                       │
                                     └───────────────── (near end back in frame) ───────────┘
```

### Usage

```bash
ros2 run visual_tracker odom_waypoint_tracker \
  --ros-args -p camera_height:=0.14 -p camera_tilt:=0.35 -p camera_hfov_deg:=62.2
```

To swap it in for the DIS tracker so `pursuit_servo`'s `/waypoint_tracker_node/...`
service clients resolve, remap the node name:

```bash
ros2 run visual_tracker odom_waypoint_tracker --ros-args -r __node:=waypoint_tracker_node
```

Start / stop tracking and observe the stream exactly as `waypoint_tracker`:

```bash
ros2 service call /odom_waypoint_tracker_node/set_waypoints hint_interfaces/srv/SetWaypoints \
  "{waypoints: [{x: 0.0, y: 0.8, z: 0.0}, {x: 0.05, y: 0.2, z: 0.0}, {x: 0.1, y: -0.4, z: 0.0}], stamp: {sec: 0, nanosec: 0}}"
ros2 topic echo /waypoint_tracking/state
ros2 topic echo /waypoint_tracking/points
```

### Interfaces

| Interface | Type | Direction |
|---|---|---|
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | Sub — drives the publish loop, header, overlay |
| `/odom` (see `odom_topic`) | `nav_msgs/Odometry` | Sub — the sole pose source |
| `~/set_waypoints` | `hint_interfaces/SetWaypoints` | Service server |
| `~/stop_tracking` | `hint_interfaces/StopTracking` | Service server |
| `/waypoint_tracking/state` | `std_msgs/String` (latched) | Pub — `UNTRACKED` / `TRACKING` / `OCCLUDED` |
| `/waypoint_tracking/points` | `hint_interfaces/VisualWaypoints` | Pub — normalized waypoint stream, **only while `TRACKING`** |
| `/camera/waypoint_tracking` | `sensor_msgs/Image` | Pub — debug overlay |

The `set_waypoints` service and the `VisualWaypoints` output layout are **identical
to `waypoint_tracker`** (see above) — same normalized `[-1, 1]` nearest-first input,
same `points`/`tracked` output, same consumer contract (steer only by `tracked=true`
points; treat "not `TRACKING`" / stale as *stop*).

### Parameters

| Parameter | Default | Effect |
|---|---|---|
| `camera_height` | 0.14 | Camera height above the ground plane (m) |
| `camera_forward_offset` | 0.0 | Camera offset ahead of the base origin (m) |
| `camera_tilt` | 0.0 | Camera pitch **down** from horizontal (rad); larger = looks nearer/lower |
| `camera_hfov_deg` | 62.2 | Horizontal field of view (deg); sets the focal length (Pi cam v2 default) |
| `odom_topic` | `/odom` | Odometry topic (applied at startup, not live-adjustable) |
| `odom_timeout` | 0.5 | Odometry age (s) beyond which the tracker declares `OCCLUDED` |
| `priority_count` | 2 | Nearest N in-frame waypoints whose validity governs node state; `TRACKING` while ≥1 is measured, `OCCLUDED` when the near end all leaves the frame |

All parameters except `odom_topic` are live-adjustable via `ros2 param set`.

### Tuning

| Symptom | Adjustment |
|---|---|
| Waypoints project too near / too far | Correct `camera_tilt` and `camera_height` to the real rig — grounding is only as good as the geometry |
| Whole path skewed left/right | Check `camera_hfov_deg` (focal length) and that the principal point really is centered |
| Path lags / leads the robot | Verify `/odom` is well-calibrated and low-latency; lower `odom_timeout` if pose drops out |
| Points drift over a long run | Expected — pure dead-reckoning accumulates odometry drift with no visual correction |
