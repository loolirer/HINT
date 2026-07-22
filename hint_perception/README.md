# hint_perception

Perception + debug visualization for HINT. Two nodes:

| Node | Role |
|---|---|
| `ground_segmenter` | Semantic ground ONNX → obstacle `PointCloud2` (for the Nav2 costmap) + binary ground mask |
| `visual_debug` | Composes one `/debug` image from the other nodes' real outputs (no per-node debug topics) |

`ground_segmenter` runs a semantic-segmentation ONNX model on the iGPU (OpenVINO), takes
everything that is **not** traversable ground, projects those pixels onto the ground plane,
and publishes them as a `sensor_msgs/PointCloud2` — plus the binary ground mask.

```bash
colcon build --symlink-install --packages-select hint_perception
source install/setup.bash
```

## ground_segmenter

Fused inference + projection in one node:

1. **Infer** per-pixel ground probability. The model output layout is detected at load time,
   so most encoder-decoder segmentation models from Hugging Face work without code edits
   (per-class logits softmaxed with the `ground_class_ids` probabilities summed, a single
   sigmoid score, or an already-argmaxed class map). Export with `scripts/export_seg_onnx.py`.
2. **Project.** The mask is warped through the ground-plane homography (camera model
   `camera_height` / `camera_tilt` / `camera_hfov_deg` / `camera_forward_offset`, the same
   projection `hint_navigation` uses) into a metric top-down (BEV) grid; a cell that is
   **known** (inside the camera wedge) but **not ground** is an obstacle.
3. **Publish** those obstacle cell centres as a `PointCloud2` on `/ground/obstacles` in
   `base_link` (z = 0), stamped with the source frame's header stamp — so Nav2's obstacle
   layer TF-transforms it `base_link → odom` at that stamp (latency-compensated placement).
   One point per BEV cell keeps the cloud light. It also publishes the **binary ground
   mask** on `/camera/ground/mask` (`mono8`) at camera resolution.

> This node **owns the camera geometry** (it needs it to project). The costmap it feeds
> only sees the point cloud. Visual debug (the green/red overlay etc.) lives in
> `visual_debug`, not here.

### Model

Export a Hugging Face segmentation model into `models/` (gitignored) with
`scripts/export_seg_onnx.py` — it prints the model's label map and the `ground_class_ids`
it implies. SegFormer / DPT / BEiT / UPerNet / DeepLabV3 export cleanly;
Mask2Former / MaskFormer / OneFormer do not (query-based outputs).

```bash
python3 scripts/export_seg_onnx.py \
  --model-dir ~/turtlebot3_ws/src/hint_perception/models/segformer-b0-ade \
  --height 384 --width 512 \
  --output ~/turtlebot3_ws/src/hint_perception/models/ground-seg.onnx
```

> **`ground_class_ids` is the knob that matters.** The default `[3]` is `floor` in ADE20K's
> 150-class ordering. Wrong label set ⇒ ~0 % ground coverage — take the ids from the
> exporter's printout, not the default.

### Usage

```bash
ros2 run hint_perception ground_segmenter \
  --ros-args -p model_path:=/root/turtlebot3_ws/src/hint_perception/models/ground-seg.onnx
```

Watch it:

```bash
ros2 run rqt_image_view rqt_image_view /camera/ground/mask   # the binary ground mask
ros2 topic hz /ground/obstacles                              # obstacle cloud, per inference
```

### Interfaces

| Topic | Type | Direction |
|---|---|---|
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | Sub — latest-wins (best effort) |
| `/ground/obstacles` | `sensor_msgs/PointCloud2` | Pub — non-ground cell centres in `base_link` (z=0), consumed by `hint_navigation`'s local costmap |
| `/camera/ground/mask` | `sensor_msgs/Image` (`mono8`) | Pub — binary ground mask (255 = ground, 0 = not) at camera resolution, header from the source frame. Consumed by `hint_navigation`'s trajectory clipping and by `visual_debug`'s overlay |

### Parameters

`model_path` and `device` are read at startup; the rest are live-adjustable.

| Parameter | Default | Effect |
|---|---|---|
| `model_path` | share `models/segformer-b5-ade.onnx` | Path to the segmentation ONNX |
| `device` | `AUTO` | OpenVINO device (`AUTO`/`GPU`/`CPU`) — force `GPU` to fail loudly if the iGPU isn't available |
| `ground_class_ids` | `[3]` | Class ids counted as ground (ADE20K `floor`). Must match the model's label map |
| `ground_threshold` | `0.5` | Ground-probability cut |
| `morph_kernel` | `7` | Close+open kernel (px) to fill holes / drop specks |
| `camera_height` | `0.14` | Camera height above the ground plane (m) |
| `camera_forward_offset` | `0.0` | Camera offset ahead of the base origin (m) |
| `camera_tilt` | `0.0` | Camera pitch **down** from horizontal (rad) |
| `camera_hfov_deg` | `62.2` | Horizontal FOV (deg); sets the focal length (Pi cam v2) |
| `bev_range` | `3.0` | Forward extent of the BEV window (m) |
| `bev_half_width` | `1.5` | Lateral extent each side (m) |
| `bev_resolution` | `0.05` | BEV cell size (m) — one obstacle point per cell (~ costmap resolution) |
| `obstacle_frame` | `base_link` | Frame the obstacle cloud is published in |

---

## visual_debug

The single place for live visualization. Instead of every node shipping its own debug image,
each node publishes only its **real output**, and this node layers those into one **`/debug`**
image (`sensor_msgs/Image`, `bgr8`). Rendering is **subscriber-gated** — nothing is composed
or published unless something subscribes to `/debug`.

Layers (each toggled by a `show_*` param):
1. **Backdrop** — the camera frame.
2. **Ground overlay** — the binary mask tinted (muted green = ground, coral = not), alpha-blended.
3. **Paths** — the navigator's `~/path_raw` (full VLM intent) and `~/path` (followed, ground-clipped),
   each transformed `odom → current base_link` and projected onto the frame, so they track as the
   robot moves. Amber = intent, teal = followed.
4. **BT state** — the mission tree's live state (`/hint_behavior_server/state`) as text, top-left.

```bash
ros2 run rqt_image_view rqt_image_view /debug
ros2 param set /visual_debug_node show_mask false        # toggle any layer live
```

### Interfaces

| Topic | Type | Direction |
|---|---|---|
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | Sub — backdrop, drives the render |
| `/camera/ground/mask` | `sensor_msgs/Image` (`mono8`) | Sub — ground overlay |
| `/trajectory_navigator_node/path` | `nav_msgs/Path` | Sub — followed (clipped) path |
| `/trajectory_navigator_node/path_raw` | `nav_msgs/Path` | Sub — full VLM-intent path |
| `/odom` | `nav_msgs/Odometry` | Sub — pose for re-projecting the paths |
| `/hint_behavior_server/state` | `std_msgs/String` | Sub — mission-tree state text |
| `/debug` | `sensor_msgs/Image` (`bgr8`) | Pub — the single composited debug image |

### Parameters

Camera rig (`camera_height`/`camera_forward_offset`/`camera_tilt`/`camera_hfov_deg`, match the
segmenter); layer toggles `show_mask` / `show_path` / `show_path_raw` / `show_bt_state` (all
default true); `overlay_alpha` (0.35); and a `*_topic` name per input.
