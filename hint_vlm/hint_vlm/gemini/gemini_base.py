"""Shared plumbing for Gemini Robotics-ER action-server nodes.

Every node in this package follows the same pattern: take the camera frame(s)
handed to it **in the goal**, ship them to the Gemini model with a prompt under
a timeout, and expose the whole thing as a single-goal-at-a-time action server.
This base class factors out that plumbing so the concrete nodes only carry their
action type, prompt, and result mapping.

Frames arrive in the action goal (``Reason.images`` / ``PlanVisualPath.images``),
sourced from the one image buffer that lives in ``hint_narrative``. These nodes
therefore keep **no camera subscription or buffer of their own** — the single
buffer keeps the director and the planner reasoning over the same frames.
"""

import json
import os
import threading

import cv2
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from google import genai
from google.genai import types
from PIL import Image as PILImage
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

DEFAULT_MODEL_ID = "gemini-robotics-er-1.6-preview"


class GeminiActionNode(Node):
    """Base for Gemini Robotics-ER action-server nodes.

    A subclass passes its action type, the server name, and the name of the
    Result's text field (``message`` / ``response``); the base builds the
    single-goal action server and provides the shared feedback/abort/cancel
    helpers, so concrete nodes only implement :meth:`_run`. Frames are supplied
    per-goal and decoded with :meth:`_frame_to_pil`; there is no camera
    subscription here.
    """

    def __init__(self, node_name, action_type, action_name, result_text_field):
        super().__init__(node_name)

        share = get_package_share_directory("hint_vlm")
        self.declare_parameter("api_key_path", "")
        self.declare_parameter("model_id", DEFAULT_MODEL_ID)
        self.declare_parameter("temperature", 0.0)
        self.declare_parameter("api_timeout", 30.0)
        self.declare_parameter("thinking_budget", 0)   # 0 = off; per-node
        self.declare_parameter("prompts_dir", os.path.join(share, "prompts"))

        self._client = genai.Client(api_key=self._load_api_key())
        self._bridge = CvBridge()
        self._goal_lock = threading.Lock()

        # One single-goal action server for every subclass. `result_text_field` is
        # the Result member the abort/cancel helpers write the reason into
        # (`message` for PlanVisualPath, `response` for Reason).
        self._action_type = action_type
        self._result_text_field = result_text_field
        self._action_server = ActionServer(
            self, action_type, action_name,
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=ReentrantCallbackGroup(),
        )

    # ------------------------------------------------------------------
    # Action callbacks — one goal at a time, guarded by _goal_lock.

    def _goal_cb(self, goal_request):
        if not self._goal_lock.acquire(blocking=False):
            self.get_logger().warn(
                "Rejecting goal — a call is already running."
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        try:
            return self._run(goal_handle)
        finally:
            self._goal_lock.release()

    def _run(self, goal_handle):
        raise NotImplementedError

    # ------------------------------------------------------------------
    # Shared result helpers — feedback, abort, cancel (used by every subclass)

    def _publish_feedback(self, goal_handle, state):
        fb = self._action_type.Feedback()
        fb.state = state
        goal_handle.publish_feedback(fb)

    def _result(self, text):
        """A Result with the node's text field (`message`/`response`) set to `text`."""
        result = self._action_type.Result()
        setattr(result, self._result_text_field, text)
        return result

    def _abort(self, goal_handle, message):
        """Abort the goal — ABORTED status is the failure signal — with the reason."""
        self.get_logger().warn(message)
        goal_handle.abort()
        return self._result(message)

    def _cancel(self, goal_handle):
        goal_handle.canceled()
        return self._result("Cancelled")

    # ------------------------------------------------------------------
    # Frame decode

    def _frame_to_pil(self, compressed):
        """Decode a CompressedImage into ``(bgr_ndarray, PIL.Image)``."""
        cv_bgr = self._bridge.compressed_imgmsg_to_cv2(
            compressed, desired_encoding="bgr8"
        )
        pil_img = PILImage.fromarray(cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2RGB))
        return cv_bgr, pil_img

    # ------------------------------------------------------------------
    # Gemini API

    def _call_api(self, contents, response_schema=None, json_output=False):
        """Call ``generate_content`` under a timeout, returning ``.text``.

        ``contents`` is the list handed to the model (text first, then any images,
        e.g. ``[prompt, pil_img]``). Output control, lightest → strictest:
        - default: unconstrained (best free-form quality; may return fenced/loose text);
        - ``json_output=True``: **JSON mode** (``response_mime_type=application/json``) —
          forbids invalid-JSON tokens (kills degenerate `"<td>"`-style corruption) but
          does NOT pin fields, so the model keeps most of its reasoning freedom;
        - ``response_schema=…``: **constrained decoding** to that schema — always
          well-formed and exactly-shaped, but the hard grammar can cost spatial-
          reasoning quality (use only where validity matters more than quality).
        Raises ``TimeoutError`` if the call outlives ``api_timeout``, or re-raises
        any API exception.
        """
        timeout = float(self._p("api_timeout"))
        result = [None]
        error = [None]

        def _call():
            try:
                cfg = dict(
                    temperature=float(self._p("temperature")),
                    thinking_config=types.ThinkingConfig(
                        thinking_budget=int(self._p("thinking_budget"))),
                )
                if response_schema is not None:
                    cfg["response_mime_type"] = "application/json"
                    cfg["response_schema"] = response_schema
                elif json_output:
                    cfg["response_mime_type"] = "application/json"
                result[0] = self._client.models.generate_content(
                    model=self._p("model_id"),
                    contents=contents,
                    config=types.GenerateContentConfig(**cfg),
                )
            except Exception as e:  # noqa: BLE001 — surfaced to caller
                error[0] = e

        t = threading.Thread(target=_call, daemon=True)
        t.start()
        t.join(timeout=timeout)

        if t.is_alive():
            raise TimeoutError(f"Gemini API did not respond within {timeout}s")
        if error[0]:
            raise error[0]
        return result[0].text

    def _parse_json(self, raw):
        """Parse a JSON model response, tolerating markdown code fences."""
        text = raw.strip()
        lines = text.splitlines()
        for i, line in enumerate(lines):
            if line.strip().startswith("```"):
                text = "\n".join(lines[i + 1:]).split("```")[0]
                break
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Could not parse JSON response: {raw!r}")
            return None

    # ------------------------------------------------------------------
    # Config helpers

    def _load_api_key(self):
        path = self._p("api_key_path")
        if path:
            with open(path, "r") as f:
                return f.read().strip()
        key = os.environ.get("GEMINI_API_KEY", "")
        if not key:
            raise RuntimeError(
                "No Gemini API key found. "
                "Set the api_key_path parameter or the GEMINI_API_KEY env var."
            )
        return key

    def _p(self, name):
        return self.get_parameter(name).value

    def _fill_prompt(self, name, **tokens):
        """Load ``prompts_dir/name`` and substitute ``{token}`` placeholders.

        Mirrors the mission_planner: the leading ``#`` comment header is stripped
        and placeholders are literal ``{name}`` substrings (NOT ``str.format``), so
        the literal JSON braces in the body need no escaping. Editing a template
        takes effect on node restart (no rebuild, with ``--symlink-install``).
        """
        with open(os.path.join(self._p("prompts_dir"), name), "r") as f:
            text = f.read()
        body = "\n".join(ln for ln in text.splitlines()
                         if not ln.lstrip().startswith("#"))
        for key, value in tokens.items():
            body = body.replace("{" + key + "}", str(value))
        return body.strip()
