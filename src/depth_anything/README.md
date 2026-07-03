# depth_anything

Depth-Anything V3 monocular depth inference via OpenVINO, targeting the iGPU (`device=AUTO`), together with the tooling to export the model to ONNX. **Output convention**: `mono8`, **black = close, white = far** (raw disparity is percentile-normalized per frame, then inverted with `cv2.bitwise_not`).

## Usage

Export the ONNX model first, then build and run the node:

1. From the `tools/Depth-Anything-3` repository root, install the package in editable mode:
    ```bash
    pip install -e .
    ```

2. Get the pretrained model files from Hugging Face (choose between [small](https://huggingface.co/depth-anything/DA3-SMALL), [base](https://huggingface.co/depth-anything/DA3-BASE) and [large](https://huggingface.co/depth-anything/DA3-LARGE) models) and put their directories on a `models/` inside this package.

3. Convert the downloaded model to ONNX using the included exporter:
    ```bash
    python3 scripts/export_onnx.py --model-dir ~/turtlebot3_ws/src/turtlebot3/depth_anything/models/da3-small --height 224 --width 294 --output ~/turtlebot3_ws/src/turtlebot3/depth_anything/models/da3-small.onnx
    ```
    Height and width must be a multiple of 14 and must follow as closely as possible the image height and width proportions.

4. Build and run the node:
    ```bash
    colcon build --symlink-install --packages-select depth_anything
    source install/setup.bash
    ros2 run depth_anything depth_anything
    ```

## Interfaces

| Topic | Type | Direction |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | Sub — raw (not compressed) |
| `/camera/depth/image_raw` | `sensor_msgs/Image` (`mono8`) | Pub — black=close, white=far |

## Parameters

| Parameter | Default | Effect |
|---|---|---|
| `model_path` | package share `models/da3-base.onnx` | Path to the exported ONNX model |
| `device` | `AUTO` | OpenVINO device (`AUTO`/`GPU`/`CPU`) |
| `percentile_low` | 2.0 | Lower percentile for per-frame normalization |
| `percentile_high` | 98.0 | Upper percentile for per-frame normalization |
| `ema_alpha` | 1.0 | Temporal smoothing (1.0 = no smoothing) |

---
