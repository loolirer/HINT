#pragma once

#include <behaviortree_ros2/bt_action_node.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>

#include <hint_interfaces/action/ground_description.hpp>

namespace hint_bt
{

// Grounds a text description to an ROI via description_detector's GroundDescription action.
class GroundDescriptionAction
  : public BT::RosActionNode<hint_interfaces::action::GroundDescription>
{
public:
  using RosActionNode::RosActionNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("description"),
      BT::OutputPort<sensor_msgs::msg::RegionOfInterest>("roi"),
      BT::OutputPort<builtin_interfaces::msg::Time>("stamp"),
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
    if (!wr.result->success) {
      RCLCPP_WARN(logger(), "Grounding failed: %s", wr.result->message.c_str());
      return BT::NodeStatus::FAILURE;
    }
    setOutput("roi",   wr.result->roi);
    setOutput("stamp", wr.result->stamp);
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(logger(), "GroundDescription action error: %s", BT::toStr(error));
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace hint_bt
