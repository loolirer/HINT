"""Semantic mission planner — the narrative director for HINT missions.

Consumes a Semantic Plan (a static prior: environments visited in order — hard
rails — each a brief intent) and drives navigation by maintaining a rolling
Narrative State that is recompiled every cycle by the reasoner. Exposes one
BT-facing action server:

- ``~/advance`` — report-and-advance. The caller reports the outcome of the move
                  it just executed (``success`` + the planner's VLM
                  ``observation``); the node folds it into the narrative via a
                  single reasoner call and returns the next instruction, or
                  ``mission_done``.

There are no discrete steps, statuses, retries or pass/fail judging: divergence
from the plan is normal material the narrative absorbs. All LLM work is one
``/reasoner_node/reason`` call per cycle (the ``compile.txt`` template); this
node holds no genai/API key.

Persistence (siblings of the mission YAML):
- ``*.narrative.jsonl`` — the Narrative State as an append-only, versioned
  history: one full snapshot per recompile, each embedding the outcome that
  triggered it. The tail is the current state; reading records 0..N reconstructs
  the whole belief evolution (git-like).
- ``*.log.jsonl`` — the raw action log (debug + the per-cycle increment).

The ``compile.txt`` prompt is filled by literal ``{token}`` substitution (NOT
``str.format`` — the template contains JSON braces).
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

NARRATIVE_SCHEMA = ('{"done": string, "trying": string, "next": string, '
                    '"current_environment": string, "environment_description": string, '
                    '"mission_complete": bool}')


class MissionPlannerNode(Node):
    def __init__(self):
        super().__init__("mission_planner_node")

        share = get_package_share_directory("mission_planner")
        self.declare_parameter(
            "mission_path", os.path.join(share, "missions", "apartment_tidy.yaml"))
        self.declare_parameter("brief_path", os.path.join(share, "config", "brief.md"))
        self.declare_parameter("templates_dir", os.path.join(share, "templates"))
        self.declare_parameter("narrative_path", "")   # empty -> <mission>.narrative.jsonl
        self.declare_parameter("log_path", "")          # empty -> <mission>.log.jsonl
        self.declare_parameter("reasoner_action", "/reasoner_node/reason")
        self.declare_parameter("reasoner_timeout", 30.0)

        self._lock = threading.Lock()
        self._plan = None        # the semantic plan dict (static prior)
        self._narrative = None   # current narrative dict (5 keys, flat)
        self._version = -1       # version of the last narrative snapshot written
        self._served = False     # has a prior instruction been served this session?

        self._brief = self._read(self._p("brief_path"))
        self._load_plan()
        self._load_or_seed_narrative()

        cbg = ReentrantCallbackGroup()
        self._reasoner = ActionClient(
            self, Reason, self._p("reasoner_action"), callback_group=cbg)
        self._advance_srv = ActionServer(
            self, MissionAdvance, "~/advance",
            execute_callback=self._advance_cb,
            cancel_callback=self._cancel_cb, callback_group=cbg)

        self.get_logger().info(
            f"Mission planner ready — '{self._plan.get('mission', '')}' "
            f"({len(self._plan.get('environments', []))} environments), "
            f"narrative v{self._version}.")

    # ------------------------------------------------------------------
    # Action server — report-and-advance (one narrative recompile per cycle)

    def _advance_cb(self, goal_handle):
        with self._lock:
            req = goal_handle.request
            self._feedback(goal_handle, MissionAdvance, "RUNNING")

            # Outcome of the previously-served instruction (none on the first call).
            trigger = None
            outcome_text = "(nothing yet — this is the first cycle)"
            if self._served:
                trigger = {"success": bool(req.success), "observation": req.observation}
                outcome_text = self._outcome_text(req)
                self._append_log("follow",
                                 result="success" if req.success else "failure",
                                 observation=req.observation)

            # One reasoner call: fold the outcome into the rolling narrative.
            new = self._compile(outcome_text, goal_handle)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                cancelled = MissionAdvance.Result()
                cancelled.message = "Cancelled."
                self.get_logger().info("Advance cancelled (BT halt).")
                return cancelled

            result = MissionAdvance.Result()
            if new is None:
                # Compile failed (reasoner down / timeout / unparseable). Keep the
                # previous narrative, append NO snapshot, and re-serve the current
                # instruction — so the loop retries instead of falsely completing.
                result.mission_done = False
                result.area = self._narrative.get("current_environment", "")
                result.description = self._narrative.get("next", "")
                result.message = "Compile failed — re-serving previous instruction."
                self._served = True
                goal_handle.succeed()
                self.get_logger().warn(result.message)
                return result

            self._narrative = new
            self._append_narrative(trigger)
            result.area = self._narrative.get("current_environment", "")

            if self._narrative.get("mission_complete"):
                result.mission_done = True
                result.message = self._narrative.get("done", "") or "Mission complete."
                goal_handle.succeed()
                self.get_logger().info(f"Mission complete (narrative v{self._version}).")
                return result

            result.mission_done = False
            result.description = self._narrative.get("next", "")
            result.message = self._narrative.get("trying", "")
            self._served = True
            goal_handle.succeed()
            self.get_logger().info(
                f"v{self._version} [{result.area}] next: {result.description}")
            return result

    # ------------------------------------------------------------------
    # Narrative recompile (the single reasoner call)

    def _compile(self, outcome_text, goal_handle=None):
        prompt = self._fill("compile.txt", {
            "brief": self._brief,
            "semantic_plan": self._plan_text(),
            "narrative": self._narrative_text(self._narrative),
            "outcome": outcome_text,
        })
        data = self._call_reasoner(prompt, NARRATIVE_SCHEMA, goal_handle)
        if not isinstance(data, dict):
            self.get_logger().warn("Narrative compile failed — keeping previous narrative.")
            return None
        current_env = str(data.get("current_environment",
                                   self._narrative.get("current_environment", "")))
        # Fold the enriched description back into the in-memory plan (the source
        # YAML is never touched); it is persisted in the narrative snapshot for
        # resume/reconstruction. A blank reply leaves the accumulated detail intact.
        new_desc = str(data.get("environment_description", "")).strip()
        if new_desc:
            self._set_env_description(current_env, new_desc)
        return {
            "done": str(data.get("done", "")),
            "trying": str(data.get("trying", "")),
            "next": str(data.get("next", "")),
            "current_environment": current_env,
            "mission_complete": bool(data.get("mission_complete", False)),
        }

    def _outcome_text(self, req):
        base = ("The last move executed successfully" if req.success
                else "The last move failed or was interrupted")
        if req.observation:
            return f"{base}. Planner reasoning: {req.observation}"
        return base + "."

    def _plan_text(self):
        envs = self._plan.get("environments", [])
        cur = self._narrative.get("current_environment", "") if self._narrative else ""
        cur_idx = next((i for i, e in enumerate(envs) if e.get("name") == cur), 0)
        lines = [f"Mission: {self._plan.get('mission', '')}", "Environments (in order):"]
        for i, env in enumerate(envs):
            name = env.get("name", "")
            if i < cur_idx:
                # Past environments are captured in the narrative's `done`; send
                # only the name to save tokens (their description/intent are moot).
                lines.append(f"  {i + 1}. {name} (done)")
            else:
                desc = env.get("description", "")
                head = f"  {i + 1}. {name}" + (f" — {desc}" if desc else "")
                lines.append(f"{head}: {env.get('intent', '')}")
        return "\n".join(lines)

    def _set_env_description(self, name, description):
        """Update an environment's (evolving) description in the in-memory plan."""
        for env in self._plan.get("environments", []):
            if env.get("name") == name:
                env["description"] = description
                return

    @staticmethod
    def _narrative_text(nar):
        # Only the accumulated `done` (+ current environment) carries forward into
        # the prompt. `trying`/`next` are regenerated each cycle and the result of
        # the last `next` is already in {outcome}, so echoing them back is redundant.
        return ("current_environment: {ce}\n"
                "done: {done}").format(
            ce=nar.get("current_environment", ""),
            done=nar.get("done", "") or "(nothing yet)")

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
        # Block the current server thread on a client future, polling so we can
        # bail out early on a BT-halt cancel. The MultiThreaded executor keeps
        # spinning the reasoner client's callbacks on other threads (reentrant
        # group), so the done-callback fires and sets the event. On timeout OR
        # cancel we return None without waiting further; on cancel we also cancel
        # the reasoner goal so it does not run orphaned server-side. (The in-flight
        # Gemini call still finishes in the reasoner's background thread; only its
        # result is dropped — cancellation is cooperative.)
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
        text = self._read(os.path.join(self._p("templates_dir"), template_name))
        # Drop the leading '#' comment header so it isn't sent to the model, then
        # substitute literal {name} tokens (NOT str.format — the body has JSON
        # braces). Token *values* (e.g. the markdown brief) are inserted after the
        # comment strip, so their own '#' headings survive.
        body = "\n".join(ln for ln in text.splitlines()
                         if not ln.lstrip().startswith("#"))
        for key, value in tokens.items():
            body = body.replace("{" + key + "}", value)
        return body.strip()

    # ------------------------------------------------------------------
    # Persistence / IO

    def _load_plan(self):
        with open(self._p("mission_path"), "r") as f:
            self._plan = yaml.safe_load(f)
        self.get_logger().info(f"Loaded semantic plan from {self._p('mission_path')}.")

    def _load_or_seed_narrative(self):
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
            nar = last.get("narrative", {})
            self._narrative = {
                "done": nar.get("done", ""),
                "trying": nar.get("trying", ""),
                "next": nar.get("next", ""),
                "current_environment": last.get("current_environment", ""),
                "mission_complete": bool(last.get("mission_complete", False)),
            }
            # Restore the enriched environment descriptions onto the in-memory plan.
            for name, desc in (last.get("environments") or {}).items():
                if desc:
                    self._set_env_description(name, desc)
            self._served = True   # resuming mid-mission; a prior instruction existed
            self.get_logger().info(f"Resumed narrative from {path} at v{self._version}.")
        else:
            envs = self._plan.get("environments", [])
            self._narrative = {
                "done": "", "trying": "", "next": "",
                "current_environment": envs[0].get("name", "") if envs else "",
                "mission_complete": False,
            }
            self._version = -1
            self._served = False

    def _append_narrative(self, trigger):
        """Append a full narrative snapshot — the versioned, git-like history."""
        self._version += 1
        rec = {
            "version": self._version,
            "ts": self._now(),
            "current_environment": self._narrative.get("current_environment", ""),
            "mission_complete": bool(self._narrative.get("mission_complete", False)),
            "trigger": trigger,
            # The evolving environment descriptions (the enriched belief) travel
            # with each snapshot so the history reconstructs and resumes fully.
            "environments": {e.get("name", ""): e.get("description", "")
                             for e in self._plan.get("environments", [])},
            "narrative": {
                "done": self._narrative.get("done", ""),
                "trying": self._narrative.get("trying", ""),
                "next": self._narrative.get("next", ""),
            },
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
        base, _ = os.path.splitext(self._p("mission_path"))
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
