"""Semantic mission planner — the narrative director for HINT missions.

Consumes a Semantic Plan (environments to visit, in order, each a brief intent) and
drives navigation by maintaining a rolling Narrative State, recompiled every cycle
by one reasoner call. Exposes one BT-facing action server, ``~/advance``: the caller
reports the outcome of the move it just executed (``success`` + the planner's VLM
``observation``); the node folds it in and returns the next instruction, or
``mission_done``.

**The environment queue.** Order is owned by *code*, not the model. The node holds a
FIFO queue of environments (head = current) plus a stack of visited ones. The model
never names or reorders environments — each cycle it emits `environment_action` ∈
{stay, advance, back, insert}, and the node applies it deterministically:
- `advance` pops the head (its intent is met);
- `back` restores the previous head (advanced too early);
- `insert` splices a discovered intermediate (e.g. a corridor the plan omitted) in as
  the next environment;
- `stay` keeps working the head.
This enforces the plan order while letting reality refine it, keeps the model's prompt
bounded (only the head + a one-line peek are shown, regardless of queue length), and
makes `mission_complete` code-derived (the queue empties). A per-environment cycle cap
(`max_env_cycles`) is the one failure path: if the model never leaves a head, the mission
fails rather than dragging on.

Persistence (siblings of the mission YAML), both append-only:
- ``*.narrative.jsonl`` — one full snapshot per recompile (queue + visited + narrative
  + the triggering outcome + the action taken). Tail = current; 0..N reconstructs the
  whole belief + queue evolution; resume reads the tail.
- ``*.log.jsonl`` — the raw action log (debug).
"""

import json
import os
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

from hint_interfaces.action import MissionAdvance, Reason

NARRATIVE_SCHEMA = (
    '{"done": string, "next": string, "environment_description": string, '
    '"environment_action": "stay"|"advance"|"back"|"insert", '
    '"new_environment": {"name": string, "description": string}}')

ACTIONS = ("stay", "advance", "back", "insert")


class MissionPlannerNode(Node):
    def __init__(self):
        super().__init__("mission_planner_node")

        share = get_package_share_directory("mission_planner")
        self.declare_parameter("mission_path", "")   # empty = start idle; pick a mission per ~/advance call
        self.declare_parameter("brief_path", os.path.join(share, "config", "brief.md"))
        self.declare_parameter("prompts_dir", os.path.join(share, "prompts"))
        self.declare_parameter("narrative_path", "")   # empty -> <mission>.narrative.jsonl
        self.declare_parameter("log_path", "")          # empty -> <mission>.log.jsonl
        self.declare_parameter("reasoner_action", "/reasoner_node/reason")
        self.declare_parameter("reasoner_timeout", 30.0)
        self.declare_parameter("max_env_cycles", 10)     # stuck backstop per environment

        self._lock = threading.Lock()
        _mp = self._p("mission_path")
        self._mission_path = os.path.abspath(_mp) if _mp else ""   # runtime, switchable
        self._plan = None                       # loaded mission (None = idle, no mission)
        self._queue = []                        # remaining environments (head = current)
        self._visited = []                      # completed environments (for `back`)
        self._narrative = {"done": "", "next": ""}
        self._env_cycles = 0                    # cycles spent on the current head
        self._failed = False                    # mission stuck past the cycle cap
        self._version = -1
        self._served = False

        self._brief = self._read(self._p("brief_path"))
        if self._mission_path:
            self._load_mission(self._mission_path)

        cbg = ReentrantCallbackGroup()
        self._reasoner = ActionClient(
            self, Reason, self._p("reasoner_action"), callback_group=cbg)
        self._advance_srv = ActionServer(
            self, MissionAdvance, "~/advance",
            execute_callback=self._advance_cb,
            cancel_callback=self._cancel_cb, callback_group=cbg)

        if self._plan is not None:
            self.get_logger().info(
                f"Mission planner ready — '{self._plan.get('mission', '')}' "
                f"(queue: {[e['name'] for e in self._queue]}), narrative v{self._version}.")
        else:
            self.get_logger().info(
                "Mission planner ready — no mission loaded; waiting for a mission_path.")

    # ------------------------------------------------------------------
    # Derived state (the queue is the source of truth)

    def _current(self):
        return self._queue[0] if self._queue else None

    def _peek(self):
        return self._queue[1] if len(self._queue) > 1 else None

    def _complete(self):
        return not self._queue

    # ------------------------------------------------------------------
    # Action server — report-and-advance (one narrative recompile per cycle)

    def _advance_cb(self, goal_handle):
        with self._lock:
            req = goal_handle.request
            # Per-call mission selection: switch missions if the goal points at a
            # different YAML (resumes that mission's narrative if it already exists).
            if req.mission_path and os.path.abspath(req.mission_path) != self._mission_path:
                self.get_logger().info(f"Switching mission -> {req.mission_path}")
                self._load_mission(req.mission_path)

            # No mission loaded and none provided — nothing to do; fail cleanly so
            # the tree ends (FAILURE) rather than driving on an empty instruction.
            if self._plan is None:
                result = MissionAdvance.Result()
                result.mission_done = True
                result.mission_failed = True
                result.message = "No mission loaded — pass mission_path in the advance goal."
                goal_handle.succeed()
                self.get_logger().warn(result.message)
                return result

            if self._version >= 0 and not os.path.exists(self._narrative_path()):
                self.get_logger().info(
                    "Narrative file missing — restarting the mission from scratch.")
                self._load_mission(self._mission_path)

            self._feedback(goal_handle, MissionAdvance, "RUNNING")

            trigger = None
            outcome_text = "(nothing yet — this is the first cycle)"
            if self._served:
                trigger = {"success": bool(req.success), "observation": req.observation}
                outcome_text = self._outcome_text(req)
                self._append_log("follow",
                                 result="success" if req.success else "failure",
                                 observation=req.observation)

            data = self._compile(outcome_text, goal_handle)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                cancelled = MissionAdvance.Result()
                cancelled.message = "Cancelled."
                self.get_logger().info("Advance cancelled (BT halt).")
                return cancelled

            result = MissionAdvance.Result()
            if data is None:
                # Compile failed: keep state, append NO snapshot, re-serve current.
                cur = self._current()
                result.mission_done = self._complete()
                result.mission_failed = False
                result.area = cur["name"] if cur else ""
                result.description = self._narrative.get("next", "")
                result.message = "Compile failed — re-serving previous instruction."
                self._served = True
                goal_handle.succeed()
                self.get_logger().warn(result.message)
                return result

            action = self._apply(data)
            self._narrative = {"done": str(data.get("done", "")),
                               "next": str(data.get("next", ""))}
            self._append_snapshot(trigger, action)

            cur = self._current()
            result.area = cur["name"] if cur else ""

            if self._failed:
                result.mission_done = True
                result.mission_failed = True
                result.message = (
                    f"Mission failed — stuck in '{result.area}' past the cycle cap.")
                goal_handle.succeed()
                self.get_logger().warn(result.message)
                return result

            if self._complete():
                result.mission_done = True
                result.mission_failed = False
                result.message = self._narrative.get("done", "") or "Mission complete."
                goal_handle.succeed()
                self.get_logger().info(f"Mission complete (v{self._version}).")
                return result

            result.mission_done = False
            result.mission_failed = False
            result.description = self._narrative.get("next", "")
            result.message = self._narrative.get("done", "")
            self._served = True
            goal_handle.succeed()
            self.get_logger().info(
                f"v{self._version} [{result.area}] ({action}) next: {result.description}")
            return result

    def _apply(self, data):
        """Enrich the current environment, apply the queue edit + stuck-cap.

        Returns the action actually taken (for the snapshot).
        """
        cur = self._current()
        # 1) Enrich the current environment's description from this cycle's observation
        #    BEFORE any advance, so the final enrichment stays with the room being left.
        desc = str(data.get("environment_description", "")).strip()
        if cur is not None and desc:
            cur["description"] = desc

        # 2) Apply the model's queue edit — a fixed verb set on a code-owned queue.
        action = str(data.get("environment_action", "stay")).strip().lower()
        if action not in ACTIONS:
            action = "stay"
        head_changed = False
        if action == "advance" and self._queue:
            self._visited.append(self._queue.pop(0))
            head_changed = True
        elif action == "back" and self._visited:
            self._queue.insert(0, self._visited.pop())
            head_changed = True
        elif action == "insert":
            ne = data.get("new_environment") or {}
            name = str(ne.get("name", "")).strip()
            if name:
                env = {"name": name,
                       "description": str(ne.get("description", "")),
                       "intent": ""}
                self._queue.insert(1 if self._queue else 0, env)
            else:
                action = "stay"   # insert with no name is a no-op
        # else "stay"

        # 3) Stuck backstop: too many cycles on the same head -> fail the mission.
        if head_changed:
            self._env_cycles = 0
        else:
            self._env_cycles += 1
            if self._queue and self._env_cycles > int(self._p("max_env_cycles")):
                self.get_logger().warn(
                    f"Env cycle cap on '{self._queue[0]['name']}' — failing the mission.")
                self._failed = True
                action = "fail"
        return action

    # ------------------------------------------------------------------
    # Narrative recompile (the single reasoner call)

    def _compile(self, outcome_text, goal_handle=None):
        prompt = self._fill("compile.txt", {
            "brief": self._brief,
            "environment": self._context_text(),
            "narrative": self._narrative_text(),
            "outcome": outcome_text,
        })
        data = self._call_reasoner(prompt, NARRATIVE_SCHEMA, goal_handle)
        if not isinstance(data, dict):
            self.get_logger().warn("Narrative compile failed — keeping previous narrative.")
            return None
        return data

    def _outcome_text(self, req):
        base = ("I finished my last move" if req.success
                else "My last move failed or was interrupted")
        if req.observation:
            return f"{base}. What I saw: {req.observation}"
        return base + "."

    def _context_text(self):
        """Only the current environment + a one-line peek — bounded regardless of
        how long or refined the queue is."""
        lines = [f"My mission: {self._plan.get('mission', '')}"]
        cur = self._current()
        if cur is None:
            lines.append("I have been through all my planned places.")
            return "\n".join(lines)
        desc = cur.get("description", "")
        lines.append(f"Where I am now: {cur['name']}"
                     + (f" — {desc}" if desc else ""))
        if cur.get("intent"):
            lines.append(f"  What I need to do here: {cur['intent']}")
        peek = self._peek()
        if peek is not None:
            lines.append(f"Where I head next: {peek['name']}"
                         + (f" — {peek['intent']}" if peek.get("intent") else ""))
        else:
            lines.append("Where I head next: (nowhere — I finish once I am done here)")
        return "\n".join(lines)

    def _narrative_text(self):
        # Only the accumulated `done` carries forward; `next` is regenerated and its
        # result is already in {outcome}, so it is not echoed back. The compile
        # prompt frames this as "What I remember so far".
        return self._narrative.get("done", "") or "(nothing yet)"

    # ------------------------------------------------------------------
    # Reasoner client

    def _call_reasoner(self, prompt, schema, goal_handle=None):
        timeout = float(self._p("reasoner_timeout"))
        if not self._reasoner.wait_for_server(timeout_sec=timeout):
            self.get_logger().warn("Reasoner action server unavailable.")
            return None
        goal = Reason.Goal()
        goal.prompt = prompt
        goal.schema = schema
        handle = self._await(self._reasoner.send_goal_async(goal), timeout, goal_handle)
        if handle is None or not handle.accepted:
            self.get_logger().warn("Reasoner rejected the goal or timed out.")
            return None
        wrapped = self._await(handle.get_result_async(), timeout, goal_handle, handle)
        if wrapped is None:
            self.get_logger().warn("Reasoner result timed out or cancelled.")
            return None
        res = wrapped.result
        if not res.success:
            self.get_logger().warn(f"Reasoner failed: {res.response}")
            return None
        try:
            return json.loads(res.response)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Reasoner returned non-JSON: {res.response!r}")
            return None

    @staticmethod
    def _await(future, timeout, goal_handle=None, reasoner_handle=None):
        # Block the current server thread on a client future, polling so we can bail
        # out early on a BT-halt cancel. The MultiThreaded executor keeps spinning the
        # reasoner client's callbacks on other threads (reentrant group), so the
        # done-callback fires and sets the event. On timeout OR cancel we return None
        # without waiting further; on cancel we also cancel the reasoner goal so it
        # does not run orphaned server-side. (The in-flight Gemini call still finishes
        # in the reasoner's background thread; only its result is dropped — cooperative.)
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
        # Drop the leading '#' comment header so it isn't sent to the model, then
        # substitute literal {name} tokens (NOT str.format — the body has JSON braces).
        body = "\n".join(ln for ln in text.splitlines()
                         if not ln.lstrip().startswith("#"))
        for key, value in tokens.items():
            body = body.replace("{" + key + "}", value)
        return body.strip()

    # ------------------------------------------------------------------
    # Persistence / IO

    def _load_mission(self, path):
        """(Re)load the mission at ``path`` and reset runtime state — or resume it
        if that mission's narrative already exists. Switchable per ~/advance call."""
        self._mission_path = os.path.abspath(path)
        self._load_plan()
        self._load_or_seed()
        self._env_cycles = 0

    def _load_plan(self):
        with open(self._mission_path, "r") as f:
            self._plan = yaml.safe_load(f)
        self.get_logger().info(f"Loaded semantic plan from {self._mission_path}.")

    def _seed_queue(self):
        return [{"name": e.get("name", ""),
                 "description": e.get("description", ""),
                 "intent": e.get("intent", "")}
                for e in self._plan.get("environments", [])]

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
            self._narrative = {"done": nar.get("done", ""), "next": nar.get("next", "")}
            self._failed = bool(last.get("mission_failed", False))
            self._served = True   # resuming mid-mission
            self.get_logger().info(f"Resumed from {path} at v{self._version}.")
        else:
            self._queue = self._seed_queue()
            self._visited = []
            self._narrative = {"done": "", "next": ""}
            self._version = -1
            self._served = False
            self._failed = False

    def _append_snapshot(self, trigger, action):
        """Append a full snapshot — the versioned, git-like history (queue included)."""
        self._version += 1
        cur = self._current()
        rec = {
            "version": self._version,
            "ts": self._now(),
            "current_environment": cur["name"] if cur else "",
            "mission_complete": self._complete(),
            "mission_failed": self._failed,
            "action": action,
            "trigger": trigger,
            "queue": self._queue,       # remaining environments (order shows inserts)
            "visited": self._visited,   # completed environments (enriched descriptions)
            "narrative": {"done": self._narrative.get("done", ""),
                          "next": self._narrative.get("next", "")},
        }
        with open(self._narrative_path(), "a") as f:
            f.write(json.dumps(rec) + "\n")
        return rec

    def _append_log(self, event, result="", observation=""):
        rec = {"ts": self._now(), "event": event, "result": result,
               "observation": observation}
        with open(self._log_path(), "a") as f:
            f.write(json.dumps(rec) + "\n")
        return rec

    def _narrative_path(self):
        return self._p("narrative_path") or self._sibling(".narrative.jsonl")

    def _log_path(self):
        return self._p("log_path") or self._sibling(".log.jsonl")

    def _sibling(self, suffix):
        # Artifacts live next to the *real* mission file. os.path.realpath resolves
        # the --symlink-install symlink back to the source tree (editor-visible in a
        # dev workspace); on a plain copied install it is a no-op and they sit beside
        # the installed mission. Explicit narrative_path/log_path still override.
        base, _ = os.path.splitext(os.path.realpath(self._mission_path))
        return base + suffix

    # ------------------------------------------------------------------
    # Small utilities

    @staticmethod
    def _now():
        return datetime.now(timezone.utc).isoformat(timespec="seconds").replace(
            "+00:00", "Z")

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
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
