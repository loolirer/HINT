"""Semantic mission planner — the state authority for HINT missions.

Consumes a hierarchical mission YAML (mission -> areas -> steps) and drives it
one step at a time through a single BT-facing action server:

- ``~/advance`` — a report-and-advance step. The caller reports the outcome of
                  the step it just executed (``success`` + the VLM
                  ``observation``); the node judges that step (updating status,
                  appending the log, compressing a finished area, aborting on a
                  failed step), then returns the *next* directive (the next
                  step's instruction, re-planned via the reasoner on a retry), or
                  ``mission_done`` when the mission is over. On the first call
                  nothing has executed yet, so it just hands out the first step.

All LLM work is delegated to the ``reasoner`` node (``/reasoner_node/reason``,
text-in / JSON-out); this node holds no genai/API key. State (status, attempts,
result, summary) is persisted after every transition to a sibling
``*.state.yaml`` and the log to ``*.log.jsonl``, so a restart resumes in place.
Prompts live in ``templates/*.txt`` (filled by literal ``{token}`` substitution,
not ``str.format`` — the templates contain JSON braces).
"""

import json
import os
import threading
from datetime import datetime, timezone

import yaml

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from hint_interfaces.action import MissionAdvance, Reason

# A step/area is "finished" (skipped by the sequencer) in either terminal state.
FINISHED = ("done", "failed")


class MissionPlannerNode(Node):
    def __init__(self):
        super().__init__("mission_planner_node")

        share = get_package_share_directory("mission_planner")
        self.declare_parameter(
            "mission_path", os.path.join(share, "missions", "apartment_tidy.yaml"))
        self.declare_parameter("brief_path", os.path.join(share, "config", "brief.md"))
        self.declare_parameter("templates_dir", os.path.join(share, "templates"))
        self.declare_parameter("state_path", "")   # empty -> <mission>.state.yaml
        self.declare_parameter("log_path", "")      # empty -> <mission>.log.jsonl
        self.declare_parameter("reasoner_action", "/reasoner_node/reason")
        self.declare_parameter("reasoner_timeout", 30.0)
        self.declare_parameter("max_attempts", 3)

        self._lock = threading.Lock()
        self._mission = None   # working mission dict (with live status)
        self._log = []         # in-memory mirror of the jsonl log

        self._brief = self._read(self._p("brief_path"))
        self._load_mission()

        cbg = ReentrantCallbackGroup()
        self._reasoner = ActionClient(
            self, Reason, self._p("reasoner_action"), callback_group=cbg)
        self._advance_srv = ActionServer(
            self, MissionAdvance, "~/advance",
            execute_callback=self._advance_cb, callback_group=cbg)

        self.get_logger().info(
            f"Mission planner ready — '{self._mission.get('mission', '')}' "
            f"({len(self._mission.get('areas', []))} areas).")

    # ------------------------------------------------------------------
    # Action server — report-and-advance (one step of the mission loop)

    def _advance_cb(self, goal_handle):
        with self._lock:
            req = goal_handle.request
            self._feedback(goal_handle, MissionAdvance, "RUNNING")
            result = MissionAdvance.Result()

            # 1. Judge the step we just executed (if any). On the first call no
            #    step is active, so this is skipped and we simply serve step one.
            area, step = self._active_step()
            if step is not None:
                self._judge_and_update(area, step, req)

            # 2. A failed step aborts the whole mission — never advance past it.
            fail_area, fail_step = self._failed_step()
            if fail_step is not None:
                result.mission_done = True
                result.mission_failed = True
                result.message = (
                    f"Mission failed at {fail_area['id']}/{fail_step['id']}: "
                    f"{fail_step.get('result', '')}")
                self._persist()
                goal_handle.succeed()
                self.get_logger().warn(result.message)
                return result

            # 3. Otherwise hand out the next actionable step, or finish.
            area, step = self._first_unfinished()
            if step is None:
                result.mission_done = True
                result.mission_failed = False
                result.message = self._mission_summary()
                self._persist()
                goal_handle.succeed()
                self.get_logger().info(result.message)
                return result

            # A step still active with attempts>0 is a retry after a failed judge:
            # revise its instruction via the reasoner before re-serving it.
            if step.get("status") == "active" and step.get("attempts", 0) > 0:
                revised = self._replan(area, step)
                if revised:
                    self._append_log(area["id"], step["id"], "replan",
                                     prompt=step.get("instruction", ""),
                                     result=revised, status="active")
                    step["instruction"] = revised

            step["status"] = "active"
            area["status"] = "active"
            self._persist()

            result.mission_done = False
            result.mission_failed = False
            result.description = step.get("instruction", "")
            result.area = area["id"]
            result.step_id = step["id"]
            result.message = (
                f"Serving {step['id']} (attempt {step.get('attempts', 0) + 1}).")
            goal_handle.succeed()
            self.get_logger().info(result.message)
            return result

    def _judge_and_update(self, area, step, req):
        """Judge the just-executed step from its reported outcome and update state."""
        # Record the execution outcome; the grounded VLM observation is logged
        # verbatim for the judge to reason over.
        self._append_log(area["id"], step["id"], "follow",
                         result="success" if req.success else "failure",
                         observation=req.observation, status="active")

        verdict = self._judge(area, step, req)
        completed = bool(verdict.get("completed")) if verdict else False
        reason = (verdict.get("reason") if verdict else None) or "judge unavailable"
        summary = (verdict.get("summary") if verdict else "") or ""

        if completed:
            step["status"] = "done"
            step["result"] = summary or reason
        else:
            step["attempts"] = step.get("attempts", 0) + 1
            step["result"] = reason
            if step["attempts"] >= int(self._p("max_attempts")):
                # A failed step fails its area and aborts the whole mission.
                step["status"] = "failed"
                area["status"] = "failed"
            # else stays "active" — the next advance re-serves it with a replan.

        self._append_log(area["id"], step["id"], "judge",
                         result=f"completed={completed}", observation=reason,
                         status=step["status"])

        # Area completion + compression only when ALL its steps completed
        # successfully (a failed step aborts the mission, so it is never compressed).
        if (area.get("status") not in FINISHED
                and all(s.get("status") == "done" for s in area.get("steps", []))):
            area["status"] = "done"
            area["summary"] = self._compress(area) or area.get("summary", "")
            self._append_log(area["id"], "", "compress", result="area complete",
                             observation=area["summary"], status="done")
        self.get_logger().info(f"{step['id']}: completed={completed} — {reason}")

    # ------------------------------------------------------------------
    # Mission-state helpers

    def _first_unfinished(self):
        """(area, step) of the first not-yet-finished step, in order; else (None, None)."""
        for area in self._mission.get("areas", []):
            for step in area.get("steps", []):
                if step.get("status") not in FINISHED:
                    return area, step
        return None, None

    def _active_step(self):
        """(area, step) of the step currently being executed (status active); else (None, None)."""
        for area in self._mission.get("areas", []):
            for step in area.get("steps", []):
                if step.get("status") == "active":
                    return area, step
        return None, None

    def _failed_step(self):
        """(area, step) of the first failed step (which aborts the mission); else (None, None)."""
        for area in self._mission.get("areas", []):
            for step in area.get("steps", []):
                if step.get("status") == "failed":
                    return area, step
        return None, None

    def _mission_summary(self):
        total = failed = 0
        for area in self._mission.get("areas", []):
            for step in area.get("steps", []):
                total += 1
                failed += step.get("status") == "failed"
        if failed:
            return (f"Mission complete — {total - failed}/{total} steps done, "
                    f"{failed} failed.")
        return f"Mission complete — all {total} steps done."

    # ------------------------------------------------------------------
    # Reasoner-backed steps (judge / replan / compress)

    def _judge(self, area, step, req):
        outcome = "execution succeeded" if req.success else "execution failed"
        if req.observation:
            outcome += f" | observation: {req.observation}"
        prompt = self._fill("judge.txt", {
            "brief": self._brief,
            "mission": self._mission.get("mission", ""),
            "area_goal": area.get("goal", ""),
            "step_instruction": step.get("instruction", ""),
            "step_verify": step.get("verify", "") or "(none)",
            "outcome": outcome,
            "log": self._log_text(step_id=step["id"]),
        })
        return self._call_reasoner(
            prompt, '{"completed": bool, "reason": string, "summary": string}')

    def _replan(self, area, step):
        prompt = self._fill("replan.txt", {
            "brief": self._brief,
            "mission": self._mission.get("mission", ""),
            "area_goal": area.get("goal", ""),
            "step_instruction": step.get("instruction", ""),
            "log": self._log_text(step_id=step["id"]),
        })
        data = self._call_reasoner(
            prompt, '{"revised_instruction": string, "reason": string}')
        if data and data.get("revised_instruction"):
            return data["revised_instruction"]
        return None

    def _compress(self, area):
        prompt = self._fill("compress.txt", {
            "brief": self._brief,
            "mission": self._mission.get("mission", ""),
            "area_goal": area.get("goal", ""),
            "log": self._log_text(area_id=area["id"]),
        })
        data = self._call_reasoner(prompt, '{"summary": string}')
        if data and data.get("summary"):
            return data["summary"]
        return None

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

    def _call_reasoner(self, prompt, schema):
        timeout = float(self._p("reasoner_timeout"))
        if not self._reasoner.wait_for_server(timeout_sec=timeout):
            self.get_logger().warn("Reasoner action server unavailable.")
            return None
        goal = Reason.Goal()
        goal.prompt = prompt
        goal.schema = schema
        handle = self._await(self._reasoner.send_goal_async(goal), timeout)
        if handle is None or not handle.accepted:
            self.get_logger().warn("Reasoner rejected the goal or timed out.")
            return None
        wrapped = self._await(handle.get_result_async(), timeout)
        if wrapped is None:
            self.get_logger().warn("Reasoner result timed out.")
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
    def _await(future, timeout):
        # Block the current server thread on a client future; the MultiThreaded
        # executor keeps spinning the reasoner client's callbacks on other
        # threads (reentrant group), so the done-callback fires and sets the event.
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            return None
        return future.result()

    # ------------------------------------------------------------------
    # Persistence / logging / IO

    def _load_mission(self):
        state = self._state_path()
        src = state if os.path.exists(state) else self._p("mission_path")
        with open(src, "r") as f:
            self._mission = yaml.safe_load(f)
        self._normalize()

        self._log = []
        log_path = self._log_path()
        if os.path.exists(log_path):
            with open(log_path, "r") as f:
                for line in f:
                    line = line.strip()
                    if line:
                        try:
                            self._log.append(json.loads(line))
                        except json.JSONDecodeError:
                            pass
        self.get_logger().info(f"Loaded mission from {src}.")

    def _normalize(self):
        """Fill runtime fields so a hand-authored mission can omit them."""
        for area in self._mission.get("areas", []):
            area.setdefault("status", "pending")
            area.setdefault("summary", "")
            for step in area.get("steps", []):
                step.setdefault("status", "pending")
                step.setdefault("attempts", 0)
                step.setdefault("result", "")
                step.setdefault("verify", "")
                # A step left "active" by a prior run didn't finish this session;
                # demote it so the first advance re-serves it cleanly instead of
                # judging an execution that never happened.
                if step["status"] == "active":
                    step["status"] = "pending"

    def _persist(self):
        with open(self._state_path(), "w") as f:
            yaml.safe_dump(self._mission, f, sort_keys=False,
                           default_flow_style=False, allow_unicode=True)

    def _append_log(self, area, step_id, event, prompt="", result="",
                    observation="", status=""):
        rec = {
            "ts": datetime.now(timezone.utc).isoformat(timespec="seconds").replace(
                "+00:00", "Z"),
            "area": area, "step_id": step_id, "event": event,
            "prompt": prompt, "result": result, "observation": observation,
            "status": status,
        }
        self._log.append(rec)
        with open(self._log_path(), "a") as f:
            f.write(json.dumps(rec) + "\n")
        return rec

    def _log_text(self, step_id=None, area_id=None):
        lines = [json.dumps(rec) for rec in self._log
                 if (step_id is None or rec.get("step_id") == step_id)
                 and (area_id is None or rec.get("area") == area_id)]
        return "\n".join(lines) if lines else "(no records)"

    def _state_path(self):
        return self._p("state_path") or self._sibling(".state.yaml")

    def _log_path(self):
        return self._p("log_path") or self._sibling(".log.jsonl")

    def _sibling(self, suffix):
        base, _ = os.path.splitext(self._p("mission_path"))
        return base + suffix

    # ------------------------------------------------------------------
    # Small utilities

    def _p(self, name):
        return self.get_parameter(name).value

    @staticmethod
    def _read(path):
        with open(path, "r") as f:
            return f.read()

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
