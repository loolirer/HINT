import json
import os
import shutil
import signal
import subprocess
import threading
import time
from datetime import datetime, timezone

import yaml

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from action_msgs.msg import GoalStatus
from hint_interfaces.action import MissionAdvance, PlanVisualPath, VisualReason
from sensor_msgs.msg import CompressedImage

_LATEST_FRAME_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)

NARRATIVE_SCHEMA = json.dumps(
    {
        "type": "object",
        "properties": {
            "analysis": {"type": "string"},
            "situation": {"type": "string"},
            "done": {"type": "string"},
            "next": {"type": "string"},
            "environment_description": {"type": "string"},
            "environment_action": {
                "type": "string",
                "enum": ["stay", "advance", "back", "insert"],
            },
            "new_environment": {
                "type": "object",
                "properties": {
                    "name": {"type": "string"},
                    "description": {"type": "string"},
                },
            },
        },
        "required": [
            "analysis",
            "situation",
            "done",
            "next",
            "environment_description",
            "environment_action",
        ],
    }
)

ACTIONS = ("stay", "advance", "back", "insert")

_BAG_TOPICS = [
    "/tf",
    "/tf_static",
    "/odom",
    "/path_projector_node/path",
    "/map",
    "/visual_reasoner/visual_reason/_action/status",
    "/path_planner/plan_visual_path/_action/status",
    "/narrative_navigation/mission_advance/_action/status",
    "/path_projector_node/follow_visual_path/_action/status",
    "/spin/_action/status",
]


class MissionPlannerNode(Node):
    def __init__(self):
        super().__init__("narrative_navigation")

        share = get_package_share_directory("hint_narrative")
        self.declare_parameter(
            "mission_path", ""
        )  # empty = start idle; pick a mission per ~/mission_advance call
        self.declare_parameter(
            "brief_path", os.path.join(share, "prompts", "brief.txt")
        )
        self.declare_parameter("prompts_dir", os.path.join(share, "prompts"))
        self.declare_parameter(
            "narrative_path", ""
        )  # empty -> <mission>.narrative.jsonl
        self.declare_parameter("log_path", "")  # empty -> <mission>.log.jsonl
        self.declare_parameter("record_bag", True)
        self.declare_parameter("bag_path", "")  # empty -> <mission>.bag (sibling dir)
        self.declare_parameter("reasoner_action", "/visual_reasoner/visual_reason")
        self.declare_parameter("reasoner_timeout", 30.0)
        self.declare_parameter("planner_action", "/path_planner/plan_visual_path")
        self.declare_parameter("planner_timeout", 30.0)
        self.declare_parameter("max_env_cycles", 10)  # stuck backstop per environment
        self.declare_parameter("camera_topic", "/camera/image_raw/compressed")
        self.declare_parameter("history_frames", 1)
        self.declare_parameter("compile_retries", -1)
        self.declare_parameter("compile_retry_delay", 1.0)

        self._lock = threading.Lock()
        _mp = self._p("mission_path")
        self._mission_path = os.path.abspath(_mp) if _mp else ""
        self._plan = None  # loaded mission (None = idle, no mission)
        self._queue = []  # remaining environments (head = current)
        self._visited = []  # completed environments (for `back`)
        self._narrative = {"situation": "", "done": "", "next": ""}
        self._env_cycles = 0  # cycles spent on the current head
        self._failed = False  # mission stuck past the cycle cap
        self._version = -1
        self._served = False
        self._bag_proc = None  # running `ros2 bag record` subprocess, or None
        self._latest_frame = None
        self._frame_history = []  # move-start frames, oldest first, trimmed to N
        self._last_plan_message = ""

        self._brief = self._read(self._p("brief_path"))
        if self._mission_path:
            self._load_mission(self._mission_path)

        cbg = ReentrantCallbackGroup()
        self._reasoner = ActionClient(
            self, VisualReason, self._p("reasoner_action"), callback_group=cbg
        )
        self._planner = ActionClient(
            self, PlanVisualPath, self._p("planner_action"), callback_group=cbg
        )
        self._advance_srv = ActionServer(
            self,
            MissionAdvance,
            "~/mission_advance",
            execute_callback=self._advance_cb,
            cancel_callback=self._cancel_cb,
            callback_group=cbg,
        )

        self.create_subscription(
            CompressedImage,
            self._p("camera_topic"),
            self._camera_cb,
            _LATEST_FRAME_QOS,
            callback_group=cbg,
        )

        if self._plan is not None:
            self.get_logger().info(
                f"Mission planner ready — '{self._plan.get('mission', '')}' "
                f"(queue: {[e['name'] for e in self._queue]}), narrative v{self._version}."
            )
        else:
            self.get_logger().info(
                "Mission planner ready — no mission loaded; waiting for a mission_path."
            )

    def _current(self):
        return self._queue[0] if self._queue else None

    def _peek(self):
        return self._queue[1] if len(self._queue) > 1 else None

    def _complete(self):
        return not self._queue

    def _camera_cb(self, msg):
        self._latest_frame = msg

    def _history_window(self):
        k = max(0, int(self._p("history_frames")))
        return self._frame_history[-k:] if k else []

    def _push_frame(self):
        if self._latest_frame is not None:
            self._frame_history.append(self._latest_frame)
        k = max(0, int(self._p("history_frames")))
        self._frame_history = self._frame_history[-k:] if k else []

    def _vision_inputs(self, history, after):
        imgs = [f for f in history if f is not None]
        if after is not None:
            imgs.append(after)
        if not imgs:
            return [], "(No camera image is available this cycle.)"
        if len(imgs) == 1:
            return imgs, (
                "One camera image is attached: my current view. (Nothing yet to "
                "compare it against.)"
            )
        k = len(imgs) - 1
        return imgs, (
            f"{len(imgs)} camera images are attached, oldest first; the LAST is my "
            f"CURRENT view, the earlier {k} are what I saw before my recent move(s). "
            "I compare them to judge what my moves actually did — got closer, turned, "
            "or barely moved."
        )

    def _advance_cb(self, goal_handle):
        with self._lock:
            try:
                return self._advance_locked(goal_handle)
            except (
                Exception
            ) as e:  # noqa: BLE001 — deliberately broad: this is the backstop
                self.get_logger().error(f"Advance failed, aborting mission: {e!r}")
                try:
                    self._stop_recording()
                except Exception:
                    pass
                result = MissionAdvance.Result()
                result.mission_done = True
                result.mission_failed = True
                result.message = f"Mission planner error: {e}"
                if goal_handle.is_active:
                    goal_handle.succeed()
                return result

    def _advance_locked(self, goal_handle):
        req = goal_handle.request
        if req.mission_path and os.path.abspath(req.mission_path) != self._mission_path:
            self.get_logger().info(f"Switching mission -> {req.mission_path}")
            self._load_mission(req.mission_path)

        if self._plan is None:
            result = MissionAdvance.Result()
            result.mission_done = True
            result.mission_failed = True
            result.message = (
                "No mission loaded — pass mission_path in the advance goal."
            )
            goal_handle.succeed()
            self.get_logger().warn(result.message)
            return result

        if self._version >= 0 and not os.path.exists(self._narrative_path()):
            self.get_logger().info(
                "Narrative file missing — restarting the mission from scratch."
            )
            self._load_mission(self._mission_path)

        if req.first:
            self.get_logger().info("New tree run! Wiping old logs, starting fresh.")
            self._reset_mission()

        self._feedback(goal_handle, MissionAdvance, "RUNNING")

        trigger = None
        if self._served:
            trigger = {
                "success": bool(req.success),
                "observation": self._last_plan_message,
            }
            self._append_log(
                "follow",
                result="success" if req.success else "failure",
                observation=self._last_plan_message,
            )

        after = self._latest_frame
        history = self._history_window()
        images, vision = self._vision_inputs(history, after)

        # --- Cognition call 1: recompile the narrative (director) ---
        data = self._compile(vision, images, goal_handle)

        if goal_handle.is_cancel_requested:
            return self._cancelled_result(goal_handle)

        result = MissionAdvance.Result()
        if data is None:
            self._failed = True
            self._append_snapshot(trigger, "fail")
            cur = self._current()
            result.mission_done = True
            result.mission_failed = True
            result.area = cur["name"] if cur else ""
            result.message = (
                "Narrative compile failed past the retry budget — aborting "
                "(the narrative did not advance)."
            )
            self._stop_recording()
            goal_handle.succeed()
            self.get_logger().warn(result.message)
            return result

        action = self._apply(data)
        self._narrative = {
            "situation": str(data.get("situation", "")),
            "done": str(data.get("done", "")),
            "next": str(data.get("next", "")),
        }

        cur = self._current()
        result.area = cur["name"] if cur else ""

        if self._failed:
            self._append_snapshot(trigger, action)  # action == "fail" (stuck cap)
            result.mission_done = True
            result.mission_failed = True
            result.message = (
                f"Mission failed — stuck in '{result.area}' past the cycle cap."
            )
            self._stop_recording()
            goal_handle.succeed()
            self.get_logger().warn(result.message)
            return result

        if self._complete():
            self._append_snapshot(trigger, action)
            result.mission_done = True
            result.mission_failed = False
            result.message = self._narrative.get("done", "") or "Mission complete."
            self._stop_recording()
            goal_handle.succeed()
            self.get_logger().info(f"Mission complete (v{self._version}).")
            return result

        plan = self._plan_move(self._narrative.get("next", ""), images, goal_handle)

        if goal_handle.is_cancel_requested:
            return self._cancelled_result(goal_handle)

        if plan is None:
            self._failed = True
            self._append_snapshot(trigger, "fail")
            result.mission_done = True
            result.mission_failed = True
            result.message = (
                "Path plan failed past the retry budget — aborting "
                "(no trajectory to drive)."
            )
            self._stop_recording()
            goal_handle.succeed()
            self.get_logger().warn(result.message)
            return result

        self._append_snapshot(trigger, action)
        result.mission_done = False
        result.mission_failed = False
        result.waypoints = list(plan.waypoints)
        result.turn_degrees = float(plan.turn_degrees)
        result.stamp = plan.stamp
        result.message = plan.message
        self._last_plan_message = plan.message

        self._push_frame()
        self._served = True
        goal_handle.succeed()
        self.get_logger().info(
            f"v{self._version} [{result.area}] ({action}) "
            f"{len(result.waypoints)} wpt, turn {result.turn_degrees:+.0f}: {plan.message}"
        )
        return result

    def _cancelled_result(self, goal_handle):
        goal_handle.canceled()
        result = MissionAdvance.Result()
        result.message = "Cancelled."
        self.get_logger().info("Advance cancelled (BT halt).")
        return result

    def _apply(self, data):
        cur = self._current()
        desc = str(data.get("environment_description", "")).strip()
        if cur is not None and desc:
            cur["description"] = desc

        action = str(data.get("environment_action", "stay")).strip().lower()
        if action not in ACTIONS:
            action = "stay"
        head_changed = False
        if action == "advance":
            if self._queue:
                self._visited.append(self._queue.pop(0))
                head_changed = True
            else:
                action = "stay"
        elif action == "back":
            if self._visited:
                self._queue.insert(0, self._visited.pop())
                head_changed = True
            else:
                action = "stay"
        elif action == "insert":
            ne = data.get("new_environment") or {}
            name = str(ne.get("name", "")).strip()
            if name:
                env = {
                    "name": name,
                    "description": str(ne.get("description", "")),
                    "intent": "",
                }
                self._queue.insert(1 if self._queue else 0, env)
            else:
                action = "stay"

        if head_changed:
            self._env_cycles = 0
        else:
            self._env_cycles += 1
            if self._queue and self._env_cycles > int(self._p("max_env_cycles")):
                self.get_logger().warn(
                    f"Env cycle cap on '{self._queue[0]['name']}' — failing the mission."
                )
                self._failed = True
                action = "fail"
        return action

    def _compile(self, vision, images, goal_handle=None):
        prompt = self._fill(
            "compile.txt",
            {
                "brief": self._brief,
                "environment": self._context_text(),
                "situation": self._narrative.get("situation", "") or "(nothing yet)",
                "narrative": self._narrative_text(),
                "vision": vision,
            },
        )
        retries = int(self._p("compile_retries"))
        delay = max(0.0, float(self._p("compile_retry_delay")))
        attempt = 0
        while True:
            data = self._call_reasoner(prompt, NARRATIVE_SCHEMA, images, goal_handle)
            if isinstance(data, dict):
                return data
            if goal_handle is not None and goal_handle.is_cancel_requested:
                return None
            attempt += 1
            if retries >= 0 and attempt > retries:
                self.get_logger().warn(
                    f"Narrative compile failed after {attempt} attempt(s) — "
                    "giving up (aborting the mission)."
                )
                return None
            budget = (
                f"{attempt}/{retries + 1}" if retries >= 0 else f"{attempt}, indefinite"
            )
            self.get_logger().warn(
                f"Narrative compile failed (attempt {budget}) — the robot waits; "
                f"retrying in {delay:.1f}s (narrative not advanced)."
            )
            if not self._interruptible_sleep(delay, goal_handle):
                return None

    def _plan_move(self, description, images, goal_handle=None):
        retries = int(self._p("compile_retries"))  # -1 = retry indefinitely
        delay = max(0.0, float(self._p("compile_retry_delay")))
        attempt = 0
        while True:
            res = self._call_planner(description, images, goal_handle)
            if res is not None:
                return res
            if goal_handle is not None and goal_handle.is_cancel_requested:
                return None
            attempt += 1
            if retries >= 0 and attempt > retries:
                self.get_logger().warn(
                    f"Path plan failed after {attempt} attempt(s) — "
                    "giving up (aborting the mission)."
                )
                return None
            budget = (
                f"{attempt}/{retries + 1}" if retries >= 0 else f"{attempt}, indefinite"
            )
            self.get_logger().warn(
                f"Path plan failed (attempt {budget}) — the robot waits; "
                f"retrying in {delay:.1f}s."
            )
            if not self._interruptible_sleep(delay, goal_handle):
                return None  # cancelled during the backoff

    def _context_text(self):
        lines = [f"My mission: {self._plan.get('mission', '')}"]
        cur = self._current()
        if cur is None:
            lines.append("I have been through all my planned places.")
            return "\n".join(lines)
        desc = cur.get("description", "")
        lines.append(f"Where I am now: {cur['name']}" + (f" — {desc}" if desc else ""))
        if cur.get("intent"):
            lines.append(f"  What I need to do here: {cur['intent']}")
        peek = self._peek()
        if peek is not None:
            lines.append(
                f"Where I head next: {peek['name']}"
                + (f" — {peek['intent']}" if peek.get("intent") else "")
            )
        else:
            lines.append("Where I head next: (nowhere — I finish once I am done here)")
        return "\n".join(lines)

    def _narrative_text(self):
        return self._narrative.get("done", "") or "(nothing yet)"

    def _call_reasoner(self, prompt, schema, images=None, goal_handle=None):
        timeout = float(self._p("reasoner_timeout"))
        if not self._reasoner.wait_for_server(timeout_sec=timeout):
            self.get_logger().warn("Reasoner action server unavailable.")
            return None
        goal = VisualReason.Goal()
        goal.prompt = prompt
        goal.schema = schema
        goal.images = images or []
        handle = self._await(self._reasoner.send_goal_async(goal), timeout, goal_handle)
        if handle is None or not handle.accepted:
            self.get_logger().warn("Reasoner rejected the goal or timed out.")
            return None
        wrapped = self._await(handle.get_result_async(), timeout, goal_handle, handle)
        if wrapped is None:
            self.get_logger().warn("Reasoner result timed out or cancelled.")
            return None
        res = wrapped.result

        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f"Reasoner failed: {res.response}")
            return None
        try:
            return json.loads(res.response)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Reasoner returned non-JSON: {res.response!r}")
            return None

    def _call_planner(self, description, images, goal_handle=None):
        timeout = float(self._p("planner_timeout"))
        if not self._planner.wait_for_server(timeout_sec=timeout):
            self.get_logger().warn("Planner action server unavailable.")
            return None
        goal = PlanVisualPath.Goal()
        goal.description = description
        goal.images = images or []
        handle = self._await(self._planner.send_goal_async(goal), timeout, goal_handle)
        if handle is None or not handle.accepted:
            self.get_logger().warn("Planner rejected the goal or timed out.")
            return None
        wrapped = self._await(handle.get_result_async(), timeout, goal_handle, handle)
        if wrapped is None:
            self.get_logger().warn("Planner result timed out or cancelled.")
            return None
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f"Planner failed: {wrapped.result.message}")
            return None
        return wrapped.result

    @staticmethod
    def _await(future, timeout, goal_handle=None, reasoner_handle=None):
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        deadline = time.monotonic() + timeout
        while not done.wait(0.1):
            if goal_handle is not None and goal_handle.is_cancel_requested:
                if reasoner_handle is not None:
                    reasoner_handle.cancel_goal_async()
                return None
            if time.monotonic() >= deadline:
                return None
        return future.result()

    def _fill(self, template_name, tokens):
        text = self._read(os.path.join(self._p("prompts_dir"), template_name))
        body = "\n".join(
            ln for ln in text.splitlines() if not ln.lstrip().startswith("#")
        )
        for key, value in tokens.items():
            body = body.replace("{" + key + "}", value)
        return body.strip()

    def _load_mission(self, path):
        abspath = os.path.abspath(path)
        plan = self._load_plan(abspath)  # may raise — before any state is committed
        self._mission_path = abspath
        self._plan = plan
        self._load_or_seed()
        self._env_cycles = 0

    def _reset_mission(self):
        for path in (self._narrative_path(), self._log_path()):
            try:
                if os.path.exists(path):
                    os.remove(path)
            except OSError as e:
                self.get_logger().warn(f"Could not delete {path}: {e}")
        self._load_or_seed()  # narrative now absent -> seeds a fresh plan
        self._env_cycles = 0
        self._frame_history = []
        self._start_recording()  # fresh run -> fresh bag (overwrites the previous)

    def _load_plan(self, path):
        with open(path, "r") as f:
            plan = yaml.safe_load(f)
        if not isinstance(plan, dict):
            raise ValueError(f"Mission file {path} did not parse to a mapping.")
        self.get_logger().info(f"Loaded semantic plan from {path}.")
        return plan

    def _seed_queue(self):
        return [
            {
                "name": e.get("name", ""),
                "description": e.get("description", ""),
                "intent": e.get("intent", ""),
            }
            for e in self._plan.get("environments", [])
        ]

    def _load_or_seed(self):
        path = self._narrative_path()
        last = None
        if os.path.exists(path):
            with open(path, "r") as f:
                for line in f:
                    line = line.strip()
                    if line:
                        try:
                            last = json.loads(line)
                        except json.JSONDecodeError:
                            pass
        if last is not None:
            self._version = int(last.get("version", -1))
            self._queue = last.get("queue") or []
            self._visited = last.get("visited") or []
            nar = last.get("narrative", {})
            self._narrative = {
                "situation": nar.get("situation", ""),
                "done": nar.get("done", ""),
                "next": nar.get("next", ""),
            }
            self._failed = bool(last.get("mission_failed", False))
            self._served = True  # resuming mid-mission
            self.get_logger().info(f"Resumed from {path} at v{self._version}.")
        else:
            self._queue = self._seed_queue()
            self._visited = []
            self._narrative = {"situation": "", "done": "", "next": ""}
            self._version = -1
            self._served = False
            self._failed = False
            self._start_recording()  # fresh seed (no narrative on disk) -> start the bag

    def _append_snapshot(self, trigger, action):
        """Append a full snapshot — the versioned, git-like history (queue included).

        ``_version`` is bumped only AFTER the write succeeds, so an IO failure can't
        leave the in-memory counter ahead of what's on disk (a later resume reads the
        tail, so the two must agree)."""
        next_version = self._version + 1
        cur = self._current()
        rec = {
            "version": next_version,
            "ts": self._now(),
            "current_environment": cur["name"] if cur else "",
            "mission_complete": self._complete(),
            "mission_failed": self._failed,
            "action": action,
            "trigger": trigger,
            "queue": self._queue,  # remaining environments (order shows inserts)
            "visited": self._visited,  # completed environments (enriched descriptions)
            "narrative": {
                "situation": self._narrative.get("situation", ""),
                "done": self._narrative.get("done", ""),
                "next": self._narrative.get("next", ""),
            },
        }
        with open(self._narrative_path(), "a") as f:
            f.write(json.dumps(rec) + "\n")
        self._version = next_version  # commit only after the write succeeds
        return rec

    def _append_log(self, event, result="", observation=""):
        rec = {
            "ts": self._now(),
            "event": event,
            "result": result,
            "observation": observation,
        }
        with open(self._log_path(), "a") as f:
            f.write(json.dumps(rec) + "\n")
        return rec

    def _narrative_path(self):
        return self._p("narrative_path") or self._sibling(".narrative.jsonl")

    def _log_path(self):
        return self._p("log_path") or self._sibling(".log.jsonl")

    def _sibling(self, suffix):
        base, _ = os.path.splitext(os.path.realpath(self._mission_path))
        return base + suffix

    def _bag_dir(self):
        return self._p("bag_path") or self._sibling(".bag")

    def _start_recording(self):
        if not bool(self._p("record_bag")) or self._plan is None:
            return
        self._stop_recording()
        bag_dir = self._bag_dir()
        try:
            if os.path.exists(bag_dir):
                shutil.rmtree(bag_dir)
        except OSError as e:
            self.get_logger().warn(f"Could not delete old bag {bag_dir}: {e}")
        cmd = [
            "ros2",
            "bag",
            "record",
            "--storage",
            "mcap",
            "--include-hidden-topics",
            "-o",
            bag_dir,
            *_BAG_TOPICS,
        ]
        try:
            self._bag_proc = subprocess.Popen(cmd, start_new_session=True)
            self.get_logger().info(f"Recording mission bag -> {bag_dir}")
        except (OSError, ValueError) as e:
            self._bag_proc = None
            self.get_logger().warn(f"Could not start bag recording: {e}")

    def _stop_recording(self):
        proc = self._bag_proc
        self._bag_proc = None
        if proc is None or proc.poll() is not None:
            return
        try:
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=10.0)
            self.get_logger().info("Mission bag closed.")
        except subprocess.TimeoutExpired:
            self.get_logger().warn("Bag recorder did not exit on SIGINT — killing it.")
            proc.kill()
        except OSError as e:
            self.get_logger().warn(f"Error stopping bag recorder: {e}")

    @staticmethod
    def _interruptible_sleep(seconds, goal_handle=None):
        """Sleep up to `seconds`, returning False early if a BT-halt cancel arrives
        (True if it slept the full duration). Lets the compile backoff bail out the
        instant the tree halts, mirroring `_await`'s cancel polling."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if goal_handle is not None and goal_handle.is_cancel_requested:
                return False
            time.sleep(0.1)
        return True

    @staticmethod
    def _now():
        return (
            datetime.now(timezone.utc)
            .isoformat(timespec="seconds")
            .replace("+00:00", "Z")
        )

    def _p(self, name):
        return self.get_parameter(name).value

    @staticmethod
    def _read(path):
        with open(path, "r") as f:
            return f.read()

    def _cancel_cb(self, _goal_handle):
        # Accept BT-halt cancellations so an in-flight advance can bail out.
        return CancelResponse.ACCEPT

    @staticmethod
    def _feedback(goal_handle, action_type, state):
        fb = action_type.Feedback()
        fb.state = state
        goal_handle.publish_feedback(fb)


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlannerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_recording()  # flush the mission bag if a run was interrupted
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
