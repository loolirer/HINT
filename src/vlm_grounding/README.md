# vlm_grounding

VLM-based visual grounding node for HINT. Converts a text description into a bounding box (ROI) on a provided image using the Gemini Robotics-ER model, then hands the ROI off to the IBVS pipeline.

## Action

`~/ground_description` (`hint_interfaces/action/GroundDescription`)

| Field | Type | Description |
|---|---|---|
| **Goal** `image` | `sensor_msgs/Image` | Image to ground the description on |
| **Goal** `description` | `string` | Natural-language description of the target region |
| **Result** `success` | `bool` | Whether a matching region was found |
| **Result** `message` | `string` | Label returned by the model, or error reason |
| **Result** `roi` | `sensor_msgs/RegionOfInterest` | Bounding box of the matched region |
| **Result** `stamp` | `builtin_interfaces/Time` | Stamp from the input image header |
| **Feedback** `state` | `string` | `"RUNNING"` while the API call is in flight |

## Topics

| Topic | Type | Direction |
|---|---|---|
| `~/debug` | `sensor_msgs/Image` | Pub — latest grounded bbox drawn on the input image |

## Parameters

| Parameter | Default | Effect |
|---|---|---|
| `api_key_path` | `""` | Path to a file containing the Gemini API key; falls back to `GEMINI_API_KEY` env var if empty |
| `model_id` | `gemini-robotics-er-1.6-preview` | Gemini model to use |
| `temperature` | `1.0` | Sampling temperature for the model |

## API key setup

Place your Gemini API key in `secrets/gemini_api_key.txt` at the repository root, then pass the path as a parameter:

```bash
ros2 run vlm_grounding vlm_grounding \
  --ros-args -p api_key_path:=/root/turtlebot3_ws/src/../secrets/gemini_api_key.txt
```

Alternatively, export `GEMINI_API_KEY` in the container's environment and omit the parameter.

## Build

```bash
colcon build --symlink-install --packages-select hint_interfaces vlm_grounding
source install/setup.bash
```

`hint_interfaces` must be built first (or in the same invocation) because the action definition lives there.

## Test

```bash
# Capture a frame and ground a description from the CLI
ros2 action send_goal /vlm_grounding_node/ground_description \
  hint_interfaces/action/GroundDescription \
  "{image: {}, description: 'the red door on the left'}"
```
