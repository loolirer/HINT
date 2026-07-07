#pragma once

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
      RCLCPP_WARN(logger(), "Trajectory planning failed: %s", wr.result->message.c_str());
      return BT::NodeStatus::FAILURE;
    }
    setOutput("markers", wr.result->markers);
    setOutput("stamp",   wr.result->stamp);
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(logger(), "PlanTrajectory action error: %s", BT::toStr(error));
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace hint_bt
