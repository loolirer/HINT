# hint_vlm

ROS2 package of VLM-powered nodes for HINT. The runtime nodes are powered by the Gemini
Robotics-ER model and live under `hint_vlm/gemini/`.

## Node

| Executable | Description |
|---|---|
| `visual_reasoner` | Generic text(+optional-image)-in / JSON-out LLM reasoner — the **only** VLM node |

`visual_reasoner` is the package's single node. `hint_narrative` launches it **twice**, as two independent instances differing only in parameters and the caller-supplied prompt/schema:
- **director** (`/visual_reasoner`, temperature 0) — recompiles the narrative each cycle;
- **path planner** (`/path_planner`, temperature 1.0) — plans the ground path (ordered waypoints + an end-of-move turn) for the same cycle.

Shared plumbing (API-key loading + client, per-goal frame decode, timeout-guarded API call, single-goal action lifecycle) lives in `hint_vlm/gemini/gemini_base.py` as `GeminiActionNode`; `visual_reasoner` subclasses it. It does **not** subscribe to a camera — frames arrive in the action goal (from `hint_narrative`'s single image buffer).

> **Gemini best practices applied** (per the [image-understanding](https://ai.google.dev/gemini-api/docs/image-understanding) and [robotics](https://ai.google.dev/gemini-api/docs/robotics-overview) docs): the contents list is **text-first, then image(s)** — the node calls `_call_api([prompt, *frames])` (with multi-frame order preserved so the prompt can say "the first / second image"). Coordinates follow the ER convention, `[y, x]` normalized `0–1000`. The ER model is tuned to **sample** for spatial reasoning, so the **planner instance** runs **`temperature 1.0`**, not `0.0`; the **director instance** (structured-JSON) stays deterministic.
>
> **Structured output — two strengths.** `_call_api(contents, json_output=True)` is **JSON mode** (`response_mime_type=application/json`): it forbids invalid-JSON tokens (killing the degenerate `"<td>"`-style corruption on long replies) while leaving field structure to the model, so reasoning quality is largely preserved. `_call_api(contents, response_schema=…)` is the **stricter** constrained decoding to an exact schema — always valid *and* shaped, but the hard grammar can **cost spatial-reasoning quality**. So the node defaults to **JSON mode** (see its `structured_output` param) and only uses the full schema on request. Plain `_call_api(contents)` stays fully unconstrained.

**Prompt-free.** `visual_reasoner` carries no prompt of its own — the prompt (and optional schema) arrive in the goal. `hint_narrative` owns both prompts as data (`compile.txt` for the director, `plan.txt` for the planner) and fills them; this node just reasons over whatever text + frames it is handed.

## Usage

`hint_interfaces` must be built first (or in the same invocation) because the action definitions live there:

```bash
colcon build --symlink-install --packages-select hint_interfaces hint_vlm
source install/setup.bash
```

**API key** — place your Gemini API key in `secrets/gemini_api_key.txt` at the repository root, then pass the path as a parameter:

```bash
ros2 run hint_vlm visual_reasoner \
  --ros-args -p api_key_path:=/root/turtlebot3_ws/src/../secrets/gemini_api_key.txt
```

Alternatively, export `GEMINI_API_KEY` in the container's environment and omit the parameter. The `secrets/gemini_api_key.txt` file is gitignored. Per-node run/test commands are in each node's section below.

---

## Common parameters

Every node subclasses `GeminiActionNode`, so they share this base parameter set (each node's own section lists any extras on top):

| Parameter | Default | Effect |
|---|---|---|
| `api_key_path` | `""` | Path to a file containing the Gemini API key; falls back to `GEMINI_API_KEY` env var if empty |
| `model_id` | `gemini-robotics-er-1.6-preview` | Gemini model to use |
| `temperature` | `0.0` | Sampling temperature. Default `0.0` (deterministic); the docs recommend **`1.0` for spatial reasoning** (pointing/trajectory), so bringup sets `1.0` for the planner instance and leaves the director at `0.0` |
| `api_timeout` | `30.0` | Seconds before the API call is abandoned and the action is aborted |
| `thinking_budget` | `0` | Gemini thinking budget in tokens (`0` = off). Per-instance — raise it for a reasoning-heavy call (e.g. the director's narrative compile), leave `0` for the planner's pointing/trajectory call |
| `prompts_dir` | package `share/prompts` | Directory the node loads its prompt template from |

---


## visual_reasoner

A **generic text-(and-optional-image)-in / JSON-out** LLM reasoner. It reasons
over the text it is handed and, when the goal carries `images`, over those frames
too. It is the model call the **semantic mission planner** relies on as its
**director**: each cycle the planner hands it the **before/after frames** of the
move just executed plus the running narrative, and it assesses the move against
what it actually sees and emits the next instruction. With an **empty `images`**
list it degrades to pure text reasoning, so any text-only caller still works
unchanged (empty prompt, timeout, API error, unparseable JSON all still abort).

It subclasses `GeminiActionNode` for the API client, camera-frame decode,
timeout-guarded call and single-goal lifecycle. The `images` are decoded (any
unreadable frame is skipped) and prepended to the prompt in order, so the prompt
can refer to "the first / second image".

**Contract.** The goal carries a `prompt`, an optional `schema` (a JSON shape the
reply must match), and an optional `images` list. When a `schema` is given the
reply is parsed and re-serialized, so the caller gets canonical JSON; when it is
empty, raw text is returned. A genuine reply **succeeds** (BT `SUCCESS`); anything
that stops the reasoning from running — empty prompt, timeout, API error,
unparseable JSON — **aborts** (BT `FAILURE`) with the reason placed in `response`.
Because the prompts belong to the caller, the mission planner keeps them as data
(template files) and this node stays prompt-free.

> `model_id` defaults to the Robotics-ER model (vision-capable). If you point it
> at a general Gemini model, keep it a **vision** model — the director now sends
> images. It does **not** subscribe to the camera itself; frames arrive in the
> goal (the mission planner captures and passes them).

### Interfaces

| Interface | Type | Direction |
|---|---|---|
| `~/visual_reason` | `hint_interfaces/action/VisualReason` | Action server |

> No `~/debug` publisher; frames are supplied in the goal rather than pulled from
> a camera subscription.

**`reason` action fields**

| Field | Type | Description |
|---|---|---|
| **Goal** `prompt` | `string` | Assembled prompt / context to reason over |
| **Goal** `schema` | `string` | Optional. How it's enforced depends on the `structured_output` param (below). A **real JSON schema** (parses to a dict) *can* drive `response_schema` constrained decoding (mode `schema`); otherwise (or for a loose hint like `'{"x": bool}'`) the shape is appended to the prompt as a hint, gated by JSON mode. Empty → free-form text |
| **Goal** `images` | `sensor_msgs/CompressedImage[]` | Optional frames to reason over (empty = text-only); order is meaningful (e.g. before, after) |
| **Result** `response` | `string` | The reply — canonical JSON when a schema was requested, else raw text; on `ABORTED` (failure) the reason. Success/failure is the action's terminal status — no `success` bool |
| **Result** `stamp` | `builtin_interfaces/Time` | Stamp of the frame reasoned over (`images[-1]`, the current view); `{sec: 0, nanosec: 0}` for a text-only call (no images) |
| **Feedback** `state` | `string` | `"RUNNING"` while the API call is in flight |

### Parameters

The [common parameters](#common-parameters) (`api_key_path`, `model_id`,
`temperature`, `api_timeout`, `thinking_budget`, `prompts_dir`). For the reasoner's
narrative-heavy compile you may want a non-zero `thinking_budget`. Plus:

| Parameter | Default | Effect |
|---|---|---|
| `structured_output` | `json` | Output control when a `schema` is requested, live-adjustable. **`json`** = JSON mode (valid JSON, shape hinted in the prompt, reasoning freedom kept — the default). **`off`** = unconstrained (best quality; a reply can be unparseable → the call fails). **`schema`** = constrained decoding to the schema when it's real JSON (enum-enforced `environment_action`), but the hard grammar can cost reasoning quality. Invalid `environment_action` values are clamped to `stay` by the node regardless, so `json` is safe |

### Test

Free-form:

```bash
ros2 action send_goal /visual_reasoner/visual_reason \
  hint_interfaces/action/VisualReason \
  "{prompt: 'In one sentence, is a hallway a good place to drive a robot?', schema: ''}" \
  --feedback
```

Structured (schema-constrained) — the shape the mission planner's completion
judge would use:

```bash
ros2 action send_goal /visual_reasoner/visual_reason \
  hint_interfaces/action/VisualReason \
  "{prompt: 'Log: planned a path to the door, followed it to the last waypoint, VLM confirmed a door is directly ahead. Did the robot reach the door?', schema: '{\"completed\": bool, \"reason\": string, \"summary\": string}'}" \
  --feedback
```

---

## References

Gemini best practices this package follows (see the **Gemini best practices applied** note near the top):

- Image understanding & technical details (tokenization, formats, text-before-images ordering): https://ai.google.dev/gemini-api/docs/image-understanding
- Gemini Robotics-ER overview (pointing/trajectory `[y,x]` `0–1000` format, `temperature 1.0` for spatial reasoning, thinking budget): https://ai.google.dev/gemini-api/docs/robotics-overview
