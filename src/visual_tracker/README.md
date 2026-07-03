# visual_tracker

Lucas-Kanade optical flow tracker for visual landmark tracking. Tracks static scene regions (not moving objects) from frame to frame and publishes the tracked bounding box and tracking state.

## Usage

```bash
colcon build --symlink-install --packages-select hint_interfaces visual_tracker
source install/setup.bash
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

## Algorithm

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

## Interfaces

| Interface | Type | Direction |
|---|---|---|
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | Sub |
| `~/set_target` | `hint_interfaces/SetTarget` | Service server |
| `~/stop_tracking` | `hint_interfaces/StopTracking` | Service server |
| `/tracking/state` | `std_msgs/String` (latched) | Pub — `UNTRACKED` / `TRACKING` / `OCCLUDED` |
| `/tracking/bbox` | `sensor_msgs/RegionOfInterest` | Pub — axis-aligned bbox of the tracked quad |
| `/camera/tracking` | `sensor_msgs/Image` | Pub — debug overlay |

### `set_target` service

| Field | Type | Notes |
|---|---|---|
| `roi` | `sensor_msgs/RegionOfInterest` | Initial tracking region |
| `stamp` | `builtin_interfaces/Time` | Specific frame to initialise on; `{sec: 0, nanosec: 0}` uses the next arriving frame |

The node keeps a ring buffer of the last 10 frames. If the requested stamp is found in the buffer, that frame is used for feature detection; otherwise the current frame is used.

## Parameters

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

## Tuning

| Symptom | Adjustment |
|---|---|
| Box drifts onto obstacle | Raise `ncc_redetect_thresh` to 0.80, raise `inlier_ratio_thresh` to 0.65 |
| False occlusion on camera move | Lower `ncc_thresh` to 0.35, lower `inlier_ratio_thresh` to 0.35 |
| Box jitters on static camera | Lower `ema_alpha` to 0.10–0.15 |
| Loses track too easily | Lower `min_features` to 15, raise `fb_thresh` to 3.0 |
| Tracks garbage after init | Raise `qualityLevel` in `_detect_features` to 0.03–0.05 |

---
