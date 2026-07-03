# gemini_robotics_er

ROS2 package of nodes powered by the Gemini Robotics-ER model for HINT.

## Nodes

| Executable | Description |
|---|---|
| `description_detector` | Grounds a natural-language description to a bounding box (ROI) on a camera frame |
| `visual_question` | Answers a yes/no question about a camera frame (VLM sanity-check fallback) |

Shared plumbing (API-key loading + client, stamped camera ring buffer, timeout-guarded API call, single-goal action lifecycle) lives in `gemini_robotics_er/gemini_base.py` as `GeminiActionNode`; both executables subclass it.

---

## description_detector

Converts a text description into a bounding box (ROI) using the Gemini Robotics-ER model, then hands the ROI off to the IBVS pipeline. Subscribes to `/camera/image_raw/compressed` and keeps a ring buffer of the last 30 frames; the action goal carries only a stamp to select the frame.

### Action

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

### Topics

| Topic | Type | Direction |
|---|---|---|
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | Sub — ring buffer of last 30 frames |
| `~/debug` | `sensor_msgs/Image` | Pub — latest grounded bbox drawn on the matched frame |

### Parameters

| Parameter | Default | Effect |
|---|---|---|
| `api_key_path` | `""` | Path to a file containing the Gemini API key; falls back to `GEMINI_API_KEY` env var if empty |
| `model_id` | `gemini-robotics-er-1.6-preview` | Gemini model to use |
| `temperature` | `0.0` | Sampling temperature (0.0 for deterministic output) |
| `api_timeout` | `10.0` | Seconds before the API call is abandoned and the action is aborted |

---

## visual_question

Answers a yes/no question about a camera frame with the VLM, returning a short rationale. Built as a sanity-check fallback for the approach pipeline: when a sequential approach fails (e.g. the tracker drops the target on close approach), the behavior tree can ask *"is the target still in view?"* and use the verdict to decide between retrying and giving up — turning an ambiguous tracker loss into an explicit yes/no.

Like `description_detector`, it keeps a ring buffer of the last 30 frames; the goal's `stamp` selects the frame (`0` → latest). The result carries **three** states, not two: `answered` is `false` whenever the check could not run (no frame, decode error, timeout, API error, unparseable reply), otherwise `affirmative` holds the yes/no verdict. **Every** path — verdict or "couldn't determine" — *succeeds* at the ROS layer, so the `rationale` always reaches the caller and a sanity check that cannot run never masquerades as a "no". See the `VisualQuestionAction` BT wrapper's `on_unknown` port in `hint_bt` for how the tree turns "couldn't determine" into an abstain policy.

### Action

`~/ask` (`hint_interfaces/action/VisualQuestion`)

| Field | Type | Description |
|---|---|---|
| **Goal** `stamp` | `builtin_interfaces/Time` | Stamp of the frame to reason about; `{sec: 0, nanosec: 0}` uses the latest received frame |
| **Goal** `question` | `string` | Yes/no question to ask about the frame |
| **Result** `answered` | `bool` | `false` if the VLM could not be reached / gave no usable answer |
| **Result** `affirmative` | `bool` | The yes/no verdict (only meaningful when `answered` is `true`) |
| **Result** `rationale` | `string` | One-sentence explanation of the answer, or the reason it couldn't answer |
| **Feedback** `state` | `string` | `"RUNNING"` while the API call is in flight |

### Topics

| Topic | Type | Direction |
|---|---|---|
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | Sub — ring buffer of last 30 frames |
| `~/debug` | `sensor_msgs/Image` | Pub — frame annotated with the verdict and rationale |

Parameters are the same as `description_detector` (`api_key_path`, `model_id`, `temperature`, `api_timeout`).

```bash
ros2 action send_goal /visual_question_node/ask \
  hint_interfaces/action/VisualQuestion \
  "{stamp: {sec: 0, nanosec: 0}, question: 'is a trash bin visible in the frame?'}"
```

---

## API key setup

Place your Gemini API key in `secrets/gemini_api_key.txt` at the repository root, then pass the path as a parameter:

```bash
ros2 run gemini_robotics_er description_detector \
  --ros-args -p api_key_path:=/root/turtlebot3_ws/src/../secrets/gemini_api_key.txt
```

Alternatively, export `GEMINI_API_KEY` in the container's environment and omit the parameter.

## Build

```bash
colcon build --symlink-install --packages-select hint_interfaces gemini_robotics_er
source install/setup.bash
```

`hint_interfaces` must be built first (or in the same invocation) because the action definition lives there.

## Test

```bash
ros2 action send_goal /description_detector_node/ground_description \
  hint_interfaces/action/GroundDescription \
  "{stamp: {sec: 0, nanosec: 0}, description: 'the door on the left'}"
```
