#pragma once

#include <vector>

#include <behaviortree_ros2/bt_action_node.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <hint_interfaces/action/follow_visual_path.hpp>

namespace hint_behavior
{

// Follows a ground path via hint_navigation's path_projector FollowVisualPath action,
// which grounds the normalized waypoints into an odom nav_msgs/Path and drives
// Nav2's follow_path (MPPI) until the goal is reached.
class FollowVisualPathAction
  : public BT::RosActionNode<hint_interfaces::action::FollowVisualPath>
{
public:
  using RosActionNode::RosActionNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::vector<geometry_msgs::msg::Point>>("waypoints"),
      BT::InputPort<builtin_interfaces::msg::Time>("stamp"),
    });
  }

  bool setGoal(Goal & goal) override
  {
    goal.waypoints = getInput<std::vector<geometry_msgs::msg::Point>>("waypoints").value();
    goal.stamp     = getInput<builtin_interfaces::msg::Time>("stamp").value();
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult &) override
  {
    // Only reached on a SUCCEEDED goal (aborts/cancels go to onFailure), so the
    // follow completed — no success bool to check.
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(logger(), "FollowVisualPath action error: %s", BT::toStr(error));
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace hint_behavior
