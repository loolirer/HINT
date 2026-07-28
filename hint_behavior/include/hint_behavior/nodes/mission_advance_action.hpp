#pragma once

#include <optional>
#include <string>
#include <vector>

#include <behaviortree_ros2/bt_action_node.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <hint_interfaces/action/mission_advance.hpp>

namespace hint_behavior
{

// One cycle of the mission loop. Reports whether the move just executed succeeded
// (`success`) to the mission_planner — the cognition node — which makes BOTH VLM
// calls (recompiles its narrative AND plans the path) and returns the *next move*
// as a ready-to-drive trajectory: `markers` + `turn_degrees` + `stamp`, written to
// the blackboard for FollowVisualPathAction + SpinAction. Returns SUCCESS while
// there is a move to run, FAILURE when the mission is over — complete OR failed
// (stuck past the cycle cap, or a cognition call failed past its retry budget).
// {mission_failed} distinguishes the two so run_mission maps a failure to overall
// FAILURE. On the first tick nothing has executed (success defaults true), so the
// node simply plans and hands out the first move.
class MissionAdvance
  : public BT::RosActionNode<hint_interfaces::action::MissionAdvance>
{
public:
  using RosActionNode::RosActionNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      // String (not bool) to match SetBlackboard, which writes "true"/"false"
      // as a string — avoids any blackboard type-lock mismatch at tree build.
      BT::InputPort<std::string>("success", "true", "did the move just executed succeed?"),
      BT::InputPort<std::string>("mission", "",
                                 "mission YAML to run; empty keeps the node's current/default"),
      // The next move to drive — the node plans it internally (no PlanVisualPath leaf).
      BT::OutputPort<std::vector<geometry_msgs::msg::Point>>(
        "markers", "next ground path (normalized image space) for FollowVisualPath"),
      BT::OutputPort<double>(
        "turn_degrees", "in-place turn to apply after the path (+left / -right, deg)"),
      BT::OutputPort<builtin_interfaces::msg::Time>(
        "stamp", "current-view frame stamp the path was planned on (grounds the follow)"),
      BT::OutputPort<std::string>("area", "the current environment (from the narrative)"),
      BT::OutputPort<bool>("mission_failed", "true when the mission ended stuck (past the cycle cap)"),
    });
  }

  bool setGoal(Goal & goal) override
  {
    goal.success      = (getInput<std::string>("success").value_or("true") != "false");
    goal.mission_path = getInput<std::string>("mission").value_or("");
    // Run-boundary signal: true on the FIRST advance of this run, false after.
    // The leaf instance is rebuilt per ExecuteTree goal (behavior_server composes
    // a fresh tree per goal), so this member re-initializes to true each run — an
    // explicit "new run, reset the mission" flag, NOT inferred from an empty
    // observation (which a mid-run move can legitimately produce).
    goal.first  = first_run_;
    first_run_  = false;
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & wr) override
  {
    setOutput("area", wr.result->area);
    setOutput("mission_failed", wr.result->mission_failed);
    // The next move to drive (planned by the cognition node this cycle).
    setOutput("markers", wr.result->markers);
    setOutput("turn_degrees", wr.result->turn_degrees);
    setOutput("stamp", wr.result->stamp);

    if (wr.result->mission_done) {
      if (wr.result->mission_failed) {
        RCLCPP_WARN(logger(), "Mission failed: %s", wr.result->message.c_str());
      } else {
        RCLCPP_INFO(logger(), "Mission complete: %s", wr.result->message.c_str());
      }
      return BT::NodeStatus::FAILURE;   // loop stop signal (complete or failed)
    }
    RCLCPP_INFO(logger(), "[%s] %zu wpt, turn %+.0f: %s",
                wr.result->area.c_str(), wr.result->markers.size(),
                wr.result->turn_degrees, wr.result->message.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  // An infra failure yields no directive — treat it as a mission failure so the
  // tree maps it to overall FAILURE, not a clean finish.
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error,
                           const std::optional<WrappedResult> &) override
  {
    setOutput("mission_failed", true);
    RCLCPP_WARN(logger(), "MissionAdvance could not run (%s)", BT::toStr(error));
    return BT::NodeStatus::FAILURE;
  }

private:
  // Latched true at construction (fresh instance per tree build = per run), flipped
  // false after the first goal is dispatched. See setGoal.
  bool first_run_{true};
};

}  // namespace hint_behavior
