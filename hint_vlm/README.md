# hint_vlm

ROS2 package of VLM-powered nodes for HINT. The runtime nodes are powered by the Gemini
Robotics-ER model and live under `hint_vlm/gemini/`.

## Nodes

| Executable | Description |
|---|---|
| `path_planner` | Plans a ground-restricted path (ordered waypoints + an end-of-move in-place turn) from a text instruction |
| `visual_reasoner` | Generic text(+optional-image)-in / JSON-out LLM reasoner — the mission planner's director (sees the move's before/after frames) |

Shared plumbing (API-key loading + client, per-goal frame decode, timeout-guarded API call, single-goal action lifecycle, **prompt-template loading**) lives in `hint_vlm/gemini/gemini_base.py` as `GeminiActionNode`; each executable subclasses it. Neither node subscribes to a camera — both receive their frames in the action goal (from `hint_narrative`'s single image buffer).

> **Gemini best practices applied** (per the [image-understanding](https://ai.google.dev/gemini-api/docs/image-understanding) and [robotics](https://ai.google.dev/gemini-api/docs/robotics-overview) docs): the contents list is **text-first, then image(s)** — every node calls `_call_api([prompt, *frames])` (with multi-frame order preserved so the prompt can say "the first / second image"). Coordinates follow the ER convention, `[y, x]` normalized `0–1000`. The ER model is tuned to **sample** for spatial reasoning, so the pointing/trajectory node (`path_planner`) runs **`temperature 1.0`**, not `0.0`; the `visual_reasoner` structured-JSON director stays deterministic.
>
> **Structured output — two strengths.** `_call_api(contents, json_output=True)` is **JSON mode** (`response_mime_type=application/json`): it forbids invalid-JSON tokens (killing the degenerate `"<td>"`-style corruption on long replies) while leaving field structure to the model, so reasoning quality is largely preserved. `_call_api(contents, response_schema=…)` is the **stricter** constrained decoding to an exact schema — always valid *and* shaped, but the hard grammar can **cost spatial-reasoning quality**. So `path_planner` defaults to **JSON mode** (see its `structured_output` param) and only uses the full schema on request; the `visual_reasoner` director uses the schema for its narrative (enum-constrained `environment_action`). Plain `_call_api(contents)` stays fully unconstrained.

**Prompt templates.** Each node's prompt is an external `.txt` file under `prompts/` (installed to the package share), loaded and filled via `GeminiActionNode._fill_prompt(name, **tokens)` — the same convention as `hint_narrative`'s `compile.txt`: a `#` comment header (stripped), literal `{token}` substitution (not `str.format`, so the JSON braces in the body need no `{{ }}` escaping). Edit a prompt and restart the node (no rebuild, with `--symlink-install`). Point `prompts_dir` elsewhere to override.

## Usage

`hint_interfaces` must be built first (or in the same invocation) because the action definitions live there:

```bash
colcon build --symlink-install --packages-select hint_interfaces hint_vlm
source install/setup.bash
```

**API key** — place your Gemini API key in `secrets/gemini_api_key.txt` at the repository root, then pass the path as a parameter:

```bash
ros2 run hint_vlm path_planner \
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
| `temperature` | `0.0` | Sampling temperature. Default `0.0` (deterministic); the docs recommend **`1.0` for spatial reasoning** (pointing/trajectory), so bringup sets `1.0` for `path_planner` |
| `api_timeout` | `30.0` | Seconds before the API call is abandoned and the action is aborted |
| `thinking_budget` | `0` | Gemini thinking budget in tokens (`0` = off). Per-node — raise it for a reasoning-heavy node (e.g. the `visual_reasoner`), leave `0` for the pointing/trajectory node |
| `prompts_dir` | package `share/prompts` | Directory the node loads its prompt template from |

---

## path_planner

Plans a **ground-restricted path** from a natural-language instruction using Gemini Robotics-ER's point-defining capability. Given a description (e.g. *"walk to the door keeping to the right of the wall"* or *"reach the table without going over the mattress"*), it returns an ordered set of floor waypoints (`markers`) **and** a signed `turn_degrees` (an in-place rotation to apply at the end of the move — including a turn-only "scan" move with empty `markers`). The `FollowPlannedPath` behavior in `hint_behavior` chains plan → follow → turn: `hint_navigation`'s `path_projector` grounds the markers into a metric `odom` path and drives them via Nav2's `follow_path` (MPPI), then Nav2's Spin behavior applies `turn_degrees`. This opens room for semantic navigation preferences and constraint-aware waypoint generation.

Same API plumbing as `visual_reasoner` (both subclass `GeminiActionNode`) — and, like it, **frames arrive in the goal** (`images`) rather than from a camera subscription. The node holds **no buffer of its own**: `hint_narrative` owns the one image buffer and passes it in (last image = current view, earlier ones = continuity), so the planner and the mission director always reason over the *same* frames. Input is thus a text `description` + the `images` list; instead of a single result value it grounds an **ordered marker array in normalized image space** (`geometry_msgs/Point[]`, `x`/`y ∈ [-1, 1]` (center 0), `z` unused, `markers[0]` nearest → `markers[-1]` farthest) plus the current-view frame `stamp` (from `images[-1]`, so `FollowVisualPath` can ground the path). The model is prompted to keep points on the traversable ground plane, ordered nearest→farthest, and to honor any semantic preference in the instruction.

> **The `reasoning` field is a path note — what the model did and why (F2).** Since the
> `hint_narrative` director sees the frames directly, `reasoning` is not the narrative's eyes, so the
> prompt (`prompts/path_planner.txt`) asks for a short (1-2 sentence) note of the **path shape**
> and any constraint that forced it (e.g. *"a soft curve left around the chair toward the doorway"*),
> not a full scene report. It rides out on the result `message`; the mission BT logs it, but the
> director judges the move from the before/after images, not from this note. On an empty `waypoints`
> list (wall / already there) the node aborts, with the note in the message (via `hint_behavior`'s two-arg
> `onFailure`).
>
> **Continuity (from the goal's `images`).** When the `images` list carries more than one frame,
> the earlier ones (oldest first) are recent past views attached ahead of the current view, with a
> `{continuity}` note that labels them and says to continue the same approach across the view change —
> so each plan builds on the last instead of re-planning cold. Unlike before, the node keeps no buffer
> and no per-frame reasoning note travels with the frames (only the images do); the buffer — and its
> depth — now live in `hint_narrative` (its `history_frames` param), which passes those frames in via
> `MissionAdvance` → `PlanVisualPath.images`. `history_frames=0` there = current view only (stateless),
> `1` = last step, `3–4` = deeper history; each extra frame is more image tokens, so latency/cost rise
> with it. The `{continuity}` note also states that the **current instruction wins** if it redirects, so
> a stale plan can't trap the planner.

### Interfaces

| Interface | Type | Direction |
|---|---|---|
| `~/plan_visual_path` | `hint_interfaces/action/PlanVisualPath` | Action server |

> No camera subscription — frames arrive in the goal's `images` (the `hint_narrative` buffer). No debug image either — visualization is centralized in `hint_navigation`'s `visual_debug` node (it projects the projector's grounded paths onto the frame). This node publishes only its `markers` + `turn_degrees` result.

**`plan_visual_path` action fields**

| Field | Type | Description |
|---|---|---|
| **Goal** `description` | `string` | Natural-language navigation instruction |
| **Goal** `images` | `sensor_msgs/CompressedImage[]` | Frames to plan over (the `hint_narrative` buffer), oldest first; the **last** is the current view the path is planned on, earlier ones are continuity. Empty → the call aborts |
| **Result** `message` | `string` | The VLM's brief explanation of the chosen path, or (on `ABORTED`) the reason no path was found. Success/failure is the action's terminal status — no `success` bool |
| **Result** `markers` | `geometry_msgs/Point[]` | Ordered waypoints in normalized image space (`x`/`y ∈ [-1, 1]` (center 0), `z` unused). May be **empty** for a turn-only move (see `turn_degrees`) |
| **Result** `turn_degrees` | `float64` | Signed in-place rotation to apply after the path, from the heading the robot ends the path with. **+ = left (CCW)**, **− = right (CW)**, 0 = none. The `hint_behavior` tree feeds this to Nav2's Spin (`SpinAction`) |
| **Result** `stamp` | `builtin_interfaces/Time` | Stamp of the current-view frame that was planned on (`images[-1].header.stamp`) — `FollowVisualPath` grounds the path with the pose at this stamp |
| **Feedback** `state` | `string` | `"RUNNING"` while the API call is in flight |

### Parameters

The [common parameters](#common-parameters) (`api_key_path`, `model_id`, `temperature`, `api_timeout`, `thinking_budget`, `prompts_dir`), plus:

| Parameter | Default | Effect |
|---|---|---|
| `min_row` | `400` | Farthest image row (of 1000) a waypoint may occupy — caps forward reach. `1000` = right in front, smaller = farther/higher in the frame. The prompt asks the model to keep points at `y ≥ min_row`, and the node **clamps** any that overshoot (far points are where ground grounding is least reliable). Live-adjustable; raise it for shorter, more conservative steps |
| `n_candidates` | `1` | Consensus sampling in **one** call. `>1` asks the model for N candidate paths in a single reply — a `{"candidates": [...]}` list (this model **rejects** `candidate_count>1`, so N-sampling is done in-prompt, not via the API) — then keeps the **medoid**, the candidate whose arc-length-resampled path is closest to all the others. Robust to the model splitting between routes at `temperature > 0` (and drops spurious "no path" replies as long as one candidate finds a path). `1` = a single plan (today's behaviour). Live-adjustable |
| `structured_output` | `json` | Output control (quality vs validity), live-adjustable. **`json`** = JSON mode (`response_mime_type=application/json`) — valid JSON without the degenerate-token corruption, while keeping most of the model's reasoning freedom (the recommended default). **`off`** = unconstrained — best free-form quality, but a reply can occasionally be unparseable → that cycle aborts. **`schema`** = full constrained decoding to the waypoint schema — always valid *and* exactly-shaped, but the hard grammar can **cost spatial-reasoning quality** (paths got noticeably worse), so use only when validity matters more than path quality |

> **Why medoid, not average.** Two valid routes (left vs right of a table) *average* into a path straight through it. The medoid picks the most central *actual* candidate, so it snaps to the majority route instead of interpolating between conflicting ones. Candidates that return no waypoints (wall / already there) don't vote unless **all** of them decline, in which case the node aborts as before. Because the N are drawn in one autoregressive pass they're more **correlated** than independent API samples would be — the prompt asks for genuinely different routes only when the scene is ambiguous, so natural agreement still shows through.

### Test

The goal now carries the frames to plan over (`images`), so this node is normally driven through
the mission BT (`hint_narrative` supplies its buffer via `MissionAdvance` → `PlanVisualPath.images`);
hand-populating a `CompressedImage[]` on the command line is impractical. A bare CLI goal (no images)
exercises only the abort path:

```bash
ros2 action send_goal /path_planner/plan_visual_path \
  hint_interfaces/action/PlanVisualPath \
  "{description: 'walk toward the door keeping to the right side of the hallway', images: []}" \
  --feedback   # aborts: "No camera frame supplied in the goal."
```

For an end-to-end test run the mission BT (see `hint_behavior`'s **Run a mission**), which wires the
narrative buffer into the planner.

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
| `~/reason` | `hint_interfaces/action/Reason` | Action server |

> No `~/debug` publisher; frames are supplied in the goal rather than pulled from
> a camera subscription.

**`reason` action fields**

| Field | Type | Description |
|---|---|---|
| **Goal** `prompt` | `string` | Assembled prompt / context to reason over |
| **Goal** `schema` | `string` | Optional. How it's enforced depends on the `structured_output` param (below). A **real JSON schema** (parses to a dict) *can* drive `response_schema` constrained decoding (mode `schema`); otherwise (or for a loose hint like `'{"x": bool}'`) the shape is appended to the prompt as a hint, gated by JSON mode. Empty → free-form text |
| **Goal** `images` | `sensor_msgs/CompressedImage[]` | Optional frames to reason over (empty = text-only); order is meaningful (e.g. before, after) |
| **Result** `response` | `string` | The reply — canonical JSON when a schema was requested, else raw text; on `ABORTED` (failure) the reason. Success/failure is the action's terminal status — no `success` bool |
| **Result** `stamp` | `builtin_interfaces/Time` | Stamp of the frame reasoned over (`images[-1]`, the current view), mirroring `PlanVisualPath`; `{sec: 0, nanosec: 0}` for a text-only call (no images) |
| **Feedback** `state` | `string` | `"RUNNING"` while the API call is in flight |

### Parameters

The [common parameters](#common-parameters) (`api_key_path`, `model_id`,
`temperature`, `api_timeout`, `thinking_budget`, `prompts_dir`). For the reasoner's
narrative-heavy compile you may want a non-zero `thinking_budget`. Plus, mirroring
`path_planner`:

| Parameter | Default | Effect |
|---|---|---|
| `structured_output` | `json` | Output control when a `schema` is requested, live-adjustable. **`json`** = JSON mode (valid JSON, shape hinted in the prompt, reasoning freedom kept — the default). **`off`** = unconstrained (best quality; a reply can be unparseable → the call fails). **`schema`** = constrained decoding to the schema when it's real JSON (enum-enforced `environment_action`), but the hard grammar can cost reasoning quality. Invalid `environment_action` values are clamped to `stay` by the node regardless, so `json` is safe |

### Test

Free-form:

```bash
ros2 action send_goal /visual_reasoner/reason \
  hint_interfaces/action/Reason \
  "{prompt: 'In one sentence, is a hallway a good place to drive a robot?', schema: ''}" \
  --feedback
```

Structured (schema-constrained) — the shape the mission planner's completion
judge would use:

```bash
ros2 action send_goal /visual_reasoner/reason \
  hint_interfaces/action/Reason \
  "{prompt: 'Log: planned a path to the door, followed it to the last waypoint, VLM confirmed a door is directly ahead. Did the robot reach the door?', schema: '{\"completed\": bool, \"reason\": string, \"summary\": string}'}" \
  --feedback
```

---

## References

Gemini best practices this package follows (see the **Gemini best practices applied** note near the top):

- Image understanding & technical details (tokenization, formats, text-before-images ordering): https://ai.google.dev/gemini-api/docs/image-understanding
- Gemini Robotics-ER overview (pointing/trajectory `[y,x]` `0–1000` format, `temperature 1.0` for spatial reasoning, thinking budget): https://ai.google.dev/gemini-api/docs/robotics-overview
