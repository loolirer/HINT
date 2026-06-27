# vlm_grounding

VLM-based visual grounding node for HINT. Converts a text description into a bounding box (ROI) on a subscribed camera frame using the Gemini Robotics-ER model (default), then hands the ROI off to the IBVS pipeline.

## Action

`~/ground_description` (`hint_interfaces/action/GroundDescription`)

| Field | Type | Description |
|---|---|---|
| **Goal** `stamp` | `builtin_interfaces/Time` | Stamp of the frame to ground on; `{sec: 0, nanosec: 0}` uses the latest received frame |
| **Goal** `description` | `string` | Natural-language description of the target region |
| **Result** `success` | `bool` | Whether a matching region was found |
| **Result** `message` | `string` | Label returned by the model, or error reason |
| **Result** `roi` | `sensor_msgs/RegionOfInterest` | Bounding box of the matched region |
| **Result** `stamp` | `builtin_interfaces/Time` | Stamp of the frame that was grounded |
| **Feedback** `state` | `string` | `"RUNNING"` while the API call is in flight |

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | Sub — buffered ring buffer of the last 30 frames |
| `~/debug` | `sensor_msgs/Image` | Pub — latest grounded bbox drawn on the input frame |

## Parameters

| Parameter | Default | Effect |
|---|---|---|
| `api_key_path` | `""` | Path to a file containing the Gemini API key; falls back to `GEMINI_API_KEY` env var if empty |
| `model_id` | `gemini-robotics-er-1.6-preview` | Gemini model to use |
| `temperature` | `0.0` | Sampling temperature (0.0 for deterministic output) |
| `api_timeout` | `10.0` | Seconds before the API call is abandoned and the action is aborted |

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
# Ground on the latest camera frame
ros2 action send_goal /vlm_grounding_node/ground_description \
  hint_interfaces/action/GroundDescription \
  "{stamp: {sec: 0, nanosec: 0}, description: 'the red door on the left'}"
```
