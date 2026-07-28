#pragma once

#include <cmath>
#include <cstdint>

#include <behaviortree_ros2/bt_action_node.hpp>

#include <nav2_msgs/action/spin.hpp>

namespace hint_behavior
{

// Rotates the robot in place via Nav2's Spin behavior (/spin) — the end-of-path /
// scan re-orientation appended after FollowPathAction. yaw_degrees is the VLM's
// turn: + = left (CCW), - = right (CW); 0 completes instantly (a no-op turn).
class SpinAction : public BT::RosActionNode<nav2_msgs::action::Spin>
{
public:
  using RosActionNode::RosActionNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<double>("yaw_degrees", "in-place turn in degrees (+left / -right)"),
      BT::InputPort<double>("time_allowance", 10.0, "seconds allowed to complete the turn"),
    });
  }

  bool setGoal(Goal & goal) override
  {
    const double deg = getInput<double>("yaw_degrees").value_or(0.0);
    goal.target_yaw = static_cast<float>(deg * M_PI / 180.0);  // Spin target is radians
    const double t = getInput<double>("time_allowance").value_or(10.0);
    goal.time_allowance.sec = static_cast<int32_t>(t);
    goal.time_allowance.nanosec =
      static_cast<uint32_t>((t - static_cast<double>(static_cast<int32_t>(t))) * 1e9);
    // Note: newer Nav2 adds goal.disable_collision_checks; this version lacks it, so
    // collision checking stays at the server default.
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & wr) override
  {
    if (wr.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_WARN(logger(), "Spin did not complete (result code %d)",
                  static_cast<int>(wr.code));
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(logger(), "Spin action error: %s", BT::toStr(error));
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace hint_behavior
