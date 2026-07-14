"""Shared plumbing for Gemini Robotics-ER action-server nodes.

Every node in this package follows the same pattern: keep a stamped ring
buffer of camera frames, resolve a goal stamp to one of those frames, ship it
to the Gemini model with a prompt under a timeout, and expose the whole thing
as a single-goal-at-a-time action server. This base class factors out that
plumbing so the concrete nodes only carry their action type, prompt, and
result mapping.
"""

import json
import os
import threading
from collections import deque

import cv2
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from google import genai
from google.genai import types
from PIL import Image as PILImage
from rclpy.action import CancelResponse, GoalResponse
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

DEFAULT_MODEL_ID = "gemini-robotics-er-1.6-preview"


class GeminiActionNode(Node):
    """Base for Gemini Robotics-ER action-server nodes.

    Subclasses declare their own action server (bound to :meth:`_execute_cb`,
    :meth:`_goal_cb`, :meth:`_cancel_cb`) and implement :meth:`_run`.
    """

    def __init__(
        self,
        node_name,
        *,
        buffer_size=30,
        camera_topic="/camera/image_raw/compressed",
    ):
        super().__init__(node_name)

        share = get_package_share_directory("gemini_robotics_er")
        self.declare_parameter("api_key_path", "")
        self.declare_parameter("model_id", DEFAULT_MODEL_ID)
        self.declare_parameter("temperature", 0.0)
        self.declare_parameter("api_timeout", 30.0)
        self.declare_parameter("thinking_budget", 0)   # 0 = off; per-node
        self.declare_parameter("prompts_dir", os.path.join(share, "prompts"))

        self._client = genai.Client(api_key=self._load_api_key())
        self._bridge = CvBridge()
        self._goal_lock = threading.Lock()
        # Ring buffer of the last N frames keyed by (sec, nanosec) for
        # stamp-based lookup.
        self._frame_buffer: deque = deque(maxlen=buffer_size)

        self.create_subscription(
            CompressedImage, camera_topic, self._camera_cb, 1
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
    # Camera subscriber / frame lookup

    def _camera_cb(self, msg):
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self._frame_buffer.append((key, msg))

    def _resolve_frame(self, stamp):
        """Return ``(CompressedImage, stamp)`` for ``stamp`` (0 → latest)."""
        if not self._frame_buffer:
            return None, None
        target_key = (stamp.sec, stamp.nanosec)
        if target_key == (0, 0):
            _, msg = self._frame_buffer[-1]
            return msg, msg.header.stamp
        for buf_key, msg in self._frame_buffer:
            if buf_key == target_key:
                return msg, msg.header.stamp
        self.get_logger().warn(
            f"Frame with stamp {target_key} not in buffer — using latest."
        )
        _, msg = self._frame_buffer[-1]
        return msg, msg.header.stamp

    def _frame_to_pil(self, compressed):
        """Decode a CompressedImage into ``(bgr_ndarray, PIL.Image)``."""
        cv_bgr = self._bridge.compressed_imgmsg_to_cv2(
            compressed, desired_encoding="bgr8"
        )
        pil_img = PILImage.fromarray(cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2RGB))
        return cv_bgr, pil_img

    # ------------------------------------------------------------------
    # Gemini API

    def _call_api(self, contents):
        """Call ``generate_content`` under a timeout, returning ``.text``.

        ``contents`` is the list handed to the model (e.g. ``[pil_img,
        prompt]``). Raises ``TimeoutError`` if the call outlives
        ``api_timeout``, or re-raises any API exception.
        """
        timeout = float(self._p("api_timeout"))
        result = [None]
        error = [None]

        def _call():
            try:
                result[0] = self._client.models.generate_content(
                    model=self._p("model_id"),
                    contents=contents,
                    config=types.GenerateContentConfig(
                        temperature=float(self._p("temperature")),
                        thinking_config=types.ThinkingConfig(
                            thinking_budget=int(self._p("thinking_budget"))),
                    ),
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
