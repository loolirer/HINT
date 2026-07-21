# hint_perception

Semantic ground perception for HINT: a single node, `ground_segmenter`, turns the camera
stream into an **obstacle point cloud** for the mapless Nav2 local costmap
(`hint_navigation`). It runs a semantic-segmentation ONNX model on the iGPU (OpenVINO),
takes everything that is **not** traversable ground, projects those pixels onto the ground
plane, and publishes them as a `sensor_msgs/PointCloud2`.

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
   One point per BEV cell keeps the cloud light. A green/red ground-overlay debug image is
   published on `/camera/ground/debug` (subscriber-gated).

> This node **owns the camera geometry** (it needs it to project). The costmap it feeds
> only sees the point cloud.

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
ros2 run rqt_image_view rqt_image_view /camera/ground/debug   # green = ground, red = not
ros2 topic hz /ground/obstacles                               # obstacle cloud, per inference
```

### Interfaces

| Topic | Type | Direction |
|---|---|---|
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | Sub — latest-wins (best effort) |
| `/ground/obstacles` | `sensor_msgs/PointCloud2` | Pub — non-ground cell centres in `base_link` (z=0), consumed by `hint_navigation`'s local costmap |
| `/camera/ground/debug` | `sensor_msgs/Image` (`bgr8`) | Pub — green/red ground overlay (subscriber-gated) |

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
| `overlay_alpha` | `0.4` | Debug ground-tint strength |
