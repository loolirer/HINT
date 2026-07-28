#pragma once

#include <vector>

#include <behaviortree_ros2/bt_action_node.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <hint_interfaces/action/follow_path.hpp>

namespace hint_behavior
{

// Follows a ground path via visual_servoing's pursuit_servo FollowPath
// action (pure pursuit until every waypoint has passed under the robot).
class FollowPathAction
  : public BT::RosActionNode<hint_interfaces::action::FollowPath>
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

  BT::NodeStatus onResultReceived(const WrappedResult & wr) override
  {
    if (!wr.result->success) {
      RCLCPP_WARN(logger(), "Follow path failed: %s", wr.result->message.c_str());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(logger(), "FollowPath action error: %s", BT::toStr(error));
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace hint_behavior
