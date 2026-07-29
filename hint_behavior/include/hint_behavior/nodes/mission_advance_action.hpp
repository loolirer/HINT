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

class MissionAdvance
  : public BT::RosActionNode<hint_interfaces::action::MissionAdvance>
{
public:
  using RosActionNode::RosActionNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("success", "true", "did the move just executed succeed?"),
      BT::InputPort<std::string>("mission", "",
                                 "mission YAML to run; empty keeps the node's current/default"),
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
    goal.first  = first_run_;
    first_run_  = false;
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & wr) override
  {
    setOutput("area", wr.result->area);
    setOutput("mission_failed", wr.result->mission_failed);
    setOutput("markers", wr.result->markers);
    setOutput("turn_degrees", wr.result->turn_degrees);
    setOutput("stamp", wr.result->stamp);

    if (wr.result->mission_done) {
      if (wr.result->mission_failed) {
        RCLCPP_WARN(logger(), "Mission failed: %s", wr.result->message.c_str());
      } else {
        RCLCPP_INFO(logger(), "Mission complete: %s", wr.result->message.c_str());
      }
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(logger(), "[%s] %zu wpt, turn %+.0f: %s",
                wr.result->area.c_str(), wr.result->markers.size(),
                wr.result->turn_degrees, wr.result->message.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error,
                           const std::optional<WrappedResult> &) override
  {
    setOutput("mission_failed", true);
    RCLCPP_WARN(logger(), "MissionAdvance could not run (%s)", BT::toStr(error));
    return BT::NodeStatus::FAILURE;
  }

private:
  bool first_run_{true};
};

}  // namespace hint_behavior
