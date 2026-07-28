#pragma once

#include <optional>
#include <string>

#include <behaviortree_ros2/bt_action_node.hpp>

#include <builtin_interfaces/msg/time.hpp>

#include <hint_interfaces/action/reason.hpp>

namespace hint_behavior
{

// Calls the reasoner's Reason action — generic text-in / JSON-out LLM
// reasoning with no camera involved. Feeds the assembled "prompt" (optionally
// constrained to a JSON "schema") to the model and writes the reply to the
// "response" output port. It is the generic reasoning primitive behind the
// mission planner (which uses it directly, not via this leaf, to recompile its
// narrative each cycle) and available to any tree that needs a text→JSON step.
// Prompts belong to the caller, so build the "prompt" string upstream (e.g. from
// a template) and bind it here.
//
// SUCCESS when the model returned a usable reply; FAILURE when the call could
// not run (empty prompt, timeout, API error, unparseable JSON) — the reasoner
// *aborts* those, so they surface through onFailure. Either way the reply (or,
// on failure, the reason) is written to "response" for the tree to read/log.
class ReasonAction
  : public BT::RosActionNode<hint_interfaces::action::Reason>
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
    // Only reached on a SUCCEEDED goal (aborts go to onFailure), so the reply is
    // always usable — no success bool to check.
    setOutput("response", wr.result->response);
    setOutput("stamp", wr.result->stamp);
    RCLCPP_INFO(logger(), "Reasoner replied: %s", wr.result->response.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  // A reasoning call that could not run (aborted, cancelled, server
  // unreachable, send timeout) never yields a reply — surface FAILURE and pass
  // through whatever reason the reasoner managed to send.
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error,
                           const std::optional<WrappedResult> & wr) override
  {
    if (wr) {
      setOutput("response", wr->result->response);
    }
    RCLCPP_WARN(logger(), "Reason action could not run (%s)", BT::toStr(error));
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace hint_behavior
