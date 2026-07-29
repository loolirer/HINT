#pragma once

#include <optional>
#include <string>

#include <behaviortree_ros2/bt_action_node.hpp>

#include <builtin_interfaces/msg/time.hpp>

#include <hint_interfaces/action/visual_reason.hpp>

namespace hint_behavior
{

class VisualReasonAction
  : public BT::RosActionNode<hint_interfaces::action::VisualReason>
{
public:
  using RosActionNode::RosActionNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("prompt", "text / context to reason over"),
      BT::InputPort<std::string>(
        "schema", "",
        "optional JSON shape the reply must match; empty = free-form text"),
      BT::OutputPort<std::string>(
        "response",
        "the model reply (canonical JSON when a schema was given), or the "
        "failure reason"),
      BT::OutputPort<builtin_interfaces::msg::Time>(
        "stamp",
        "stamp of the frame reasoned over (mirrors PlanVisualPath); zero for "
        "this text-only leaf, which sends no images"),
    });
  }

  bool setGoal(Goal & goal) override
  {
    goal.prompt = getInput<std::string>("prompt").value();
    goal.schema = getInput<std::string>("schema").value_or("");
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & wr) override
  {
    setOutput("response", wr.result->response);
    setOutput("stamp", wr.result->stamp);
    RCLCPP_INFO(logger(), "Visual Reasoner replied: %s", wr.result->response.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error,
                           const std::optional<WrappedResult> & wr) override
  {
    if (wr) {
      setOutput("response", wr->result->response);
    }
    RCLCPP_WARN(logger(), "VisualReason action could not run (%s)", BT::toStr(error));
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace hint_behavior
