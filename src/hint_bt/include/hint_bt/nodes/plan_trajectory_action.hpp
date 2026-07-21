#pragma once

#include <optional>
#include <string>
#include <vector>

#include <behaviortree_ros2/bt_action_node.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <hint_interfaces/action/plan_trajectory.hpp>

namespace hint_bt
{

// Plans a ground trajectory from a text description via trajectory_planner's
// PlanTrajectory action; outputs the ordered normalized waypoints (markers).
class PlanTrajectoryAction
  : public BT::RosActionNode<hint_interfaces::action::PlanTrajectory>
{
public:
  using RosActionNode::RosActionNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("description"),
      BT::OutputPort<std::vector<geometry_msgs::msg::Point>>("markers"),
      BT::OutputPort<double>(
        "turn_degrees", "in-place turn to apply after the path (+left / -right, deg)"),
      BT::OutputPort<builtin_interfaces::msg::Time>("stamp"),
      BT::OutputPort<std::string>(
        "message", "the VLM's explanation of the chosen path (or why planning failed)"),
    });
  }

  bool setGoal(Goal & goal) override
  {
    goal.description   = getInput<std::string>("description").value();
    goal.stamp.sec     = 0;
    goal.stamp.nanosec = 0;   // 0 → latest frame in the ring buffer
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & wr) override
  {
    // Surface the VLM's path reasoning on every path (success or planner-reported
    // failure) so the mission log captures it as grounded visual feedback.
    setOutput("message", wr.result->message);
    if (!wr.result->success) {
      RCLCPP_WARN(logger(), "Trajectory planning failed: %s", wr.result->message.c_str());
      return BT::NodeStatus::FAILURE;
    }
    setOutput("markers",      wr.result->markers);
    setOutput("turn_degrees", wr.result->turn_degrees);
    setOutput("stamp",        wr.result->stamp);
    return BT::NodeStatus::SUCCESS;
  }

  // Aborts carry the model's rationale — empty waypoints mean "no path visible"
  // or "already there", with the reason in the result message. Propagate it to
  // {message} so a consumer (the mission narrative) gets that grounding instead
  // of a stale value. Uses the two-arg overload because the abort path
  // (bt_action_node.hpp) passes the result there; the single-arg one drops it.
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error,
                           const std::optional<WrappedResult> & wr) override
  {
    if (wr) {
      setOutput("message", wr->result->message);
    }
    RCLCPP_WARN(logger(), "PlanTrajectory failed (%s)", BT::toStr(error));
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace hint_bt
