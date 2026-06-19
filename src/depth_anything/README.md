# Depth-Anything

Tools and utilities for working with the Depth-Anything models, including exporting models to ONNX for inference.

## Usage

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

4. Run the node!

    ```bash
    ros2 run depth_anything depth_anything
    ```

---