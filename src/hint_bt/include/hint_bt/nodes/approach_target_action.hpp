#pragma once

#include <behaviortree_ros2/bt_action_node.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>

#include <hint_interfaces/action/approach_target.hpp>

namespace hint_bt
{

// Drives the robot to a tracked ROI via visual_servoing's ApproachTarget action.
class ApproachTargetAction
  : public BT::RosActionNode<hint_interfaces::action::ApproachTarget>
{
public:
  using RosActionNode::RosActionNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<sensor_msgs::msg::RegionOfInterest>("roi"),
      BT::InputPort<builtin_interfaces::msg::Time>("stamp"),
      BT::InputPort<double>(
        "setpoint_offset", 0.0,
        "Normalized horizontal setpoint bias in [-1, 1]; 0.0 keeps the target centered"),
    });
  }

  bool setGoal(Goal & goal) override
  {
    goal.roi             = getInput<sensor_msgs::msg::RegionOfInterest>("roi").value();
    goal.stamp           = getInput<builtin_interfaces::msg::Time>("stamp").value();
    goal.setpoint_offset = static_cast<float>(getInput<double>("setpoint_offset").value());
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & wr) override
  {
    if (!wr.result->success) {
      RCLCPP_WARN(logger(), "Approach failed: %s", wr.result->message.c_str());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(logger(), "ApproachTarget action error: %s", BT::toStr(error));
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace hint_bt
