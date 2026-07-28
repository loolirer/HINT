# hint_perception

**Image-space perception for HINT.** One node — it labels pixels and nothing more:

| Node | Role |
|---|---|
| `ground_segmenter` | Semantic ground ONNX → binary ground mask (`/camera/ground`, `mono8`) |

`ground_segmenter` runs a semantic-segmentation ONNX model on the iGPU (OpenVINO) and
publishes a binary **traversable-ground** mask. It owns **no** camera geometry: anything
metric — the obstacle cloud for the costmap, path grounding, the debug overlay — lives on
the navigation side (`hint_navigation`, the sole owner of the camera rig). Perception's job
ends at the pixel label.

```bash
colcon build --symlink-install --packages-select hint_perception
source install/setup.bash
```

## ground_segmenter

1. **Infer** per-pixel ground probability. The model output layout is detected at load time,
   so most encoder-decoder segmentation models from Hugging Face work without code edits
   (per-class logits softmaxed with the `ground_class_ids` probabilities summed, a single
   sigmoid score, or an already-argmaxed class map). Export with `scripts/export_seg_onnx.py`.
2. **Threshold + clean** on a small camera-aspect processing grid (`1/proc_scale` of camera
   res — the score comes off the model at a coarse stride anyway) and **publish** the binary
   mask on `/camera/ground` (`mono8`, 255 = ground). The header inherits the **source frame's
   stamp**, so navigation-side consumers can latency-compensate off the capture time. The mask
   is resolution-agnostic downstream (consumers use normalized coords or resize).

> This node is **purely image-space** — no rig, no projection. Its consumers all live in
> `hint_navigation`: `obstacle_projector` (mask → obstacle `PointCloud2`),
> `path_projector` (clips the VLM pixel path to the mask), and `visual_debug`
> (green/red overlay).

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
# `model` is a NAME, resolved against the package models/ dir (prefix match allowed)
ros2 run hint_perception ground_segmenter --ros-args -p model:=segformer-b2-ade
```

Watch it:

```bash
ros2 run rqt_image_view rqt_image_view /camera/ground   # the binary ground mask
ros2 topic hz /camera/ground                            # mask rate, per inference
```

### Interfaces

| Topic | Type | Direction |
|---|---|---|
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | Sub — latest-wins (best effort) |
| `/camera/ground` | `sensor_msgs/Image` (`mono8`) | Pub — binary ground mask (255 = ground, 0 = not) at processing-grid resolution, header from the source frame. Consumed by `hint_navigation`'s `obstacle_projector`, `path_projector` (clipping), and `visual_debug` (overlay) |

### Parameters

`model` and `device` are read at startup; the rest are live-adjustable.

| Parameter | Default | Effect |
|---|---|---|
| `model` | `segformer-b0-ade` | Model **name** (with/without `.onnx`, or a prefix like `segformer-b2`), always resolved against the package `models/` dir — never a path. First sorted match wins; unknown name errors listing what's available |
| `device` | `AUTO` | OpenVINO device (`AUTO`/`GPU`/`CPU`) — force `GPU` to fail loudly if the iGPU isn't available. **Note:** a heavy model (e.g. SegFormer-B5) can trip the Intel i915 GPU-reset (hangcheck), leaving the inference wedged (the hang holds the Python GIL, so no in-process recovery is possible — see the external watchdog below). `CPU` never hangs and is as fast as the iGPU here anyway |
| `ground_class_ids` | `[3]` | Class ids counted as ground (ADE20K `floor`). Must match the model's label map |
| `ground_threshold` | `0.5` | Ground-probability cut (inert in `argmax` score mode) |
| `score_mode` | `argmax` | `softmax` (summed ground-class probabilities, cut at `ground_threshold`) or `argmax` (ground iff the pixel's argmax class ∈ `ground_class_ids`; skips the full softmax) |
| `morph_kernel` | `3` | Close+open kernel (px, on the processing grid) to fill holes / drop specks |
| `proc_scale` | `4` | Post-processing + mask run on a camera-aspect grid at `1/proc_scale` of camera res |

### Reliability — naive node + external watchdog

`ground_segmenter` is **naive by design**: it decodes, infers, and publishes synchronously in
the callback, with no notion that inference can hang or that it can stop. This is deliberate —
an OpenVINO iGPU hang blocks in an uninterruptible driver call that **holds the Python GIL**, so
*any* in-process watchdog (same thread or a worker thread) is frozen too and can never fire.

Resilience is therefore **external**, assembled in `hint_bringup`:

- **`scripts/segmenter_watchdog.sh`** — a plain shell loop (its own process, immune to the
  node's GIL) launched via `ExecuteProcess`. If `/camera/ground` goes silent for `STALL`
  seconds while the camera is still publishing, the device has wedged, so it `pkill -9`s the
  segmenter. Tunable via env (`MASK_TOPIC`, `CAM_TOPIC`, `STALL`, `STARTUP`, `RESTART_GRACE`,
  `PROC_MATCH`).
- **`respawn=True`** on the `ground_segmenter` node in `bringup.launch.py` — the moment the
  watchdog kills it, launch restarts it (reloading the model fresh).

Net effect: a wedged iGPU self-heals in ~`STALL`+`RESTART_GRACE` seconds with no code inside the
segmenter. (`device=CPU` sidesteps the hang entirely and is the simplest choice.)

> **Debug view moved.** The composited `/debug` image (mask overlay + projector paths + BT
> state) is produced by `visual_debug`, which now lives in **`hint_navigation`** (it needs the
> camera rig to re-project the projector's odom paths). See that package's README.
