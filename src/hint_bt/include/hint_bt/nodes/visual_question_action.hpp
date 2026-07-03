#pragma once

#include <optional>
#include <string>

#include <behaviortree_ros2/bt_action_node.hpp>

#include <builtin_interfaces/msg/time.hpp>

#include <hint_interfaces/action/visual_question.hpp>

namespace hint_bt
{

// Asks visual_question's VisualQuestion action a yes/no question about a
// camera frame and maps the verdict onto the node status: SUCCESS on "yes",
// FAILURE on "no". The VLM call is long-running, so this is necessarily an
// async RosActionNode even though it behaves as a condition when placed inside
// a Sequence/Fallback (a BT::ConditionNode must return synchronously and
// cannot wrap an async ROS2 action).
//
// A sanity check that cannot run must not masquerade as a "no". Whenever the
// verdict is unavailable — the node answered answered=false (no frame, decode
// error, timeout, API error, unparseable reply), or the action itself
// aborted/was cancelled/the server was unreachable — the node *abstains* by
// returning the status named in the "on_unknown" port (default SUCCESS). In a
// Sequence guard, SUCCESS means "don't veto"; in a Fallback rescue, set it to
// FAILURE so it means "don't rescue". Either way the primary node's own result
// is left to stand.
class VisualQuestionAction
  : public BT::RosActionNode<hint_interfaces::action::VisualQuestion>
{
public:
  using RosActionNode::RosActionNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("question"),
      BT::InputPort<std::string>(
        "on_unknown", "SUCCESS",
        "status to return when the VLM could not answer: SUCCESS=abstain "
        "without vetoing (default), FAILURE=abstain without rescuing"),
      BT::OutputPort<std::string>("rationale"),
    });
  }

  bool setGoal(Goal & goal) override
  {
    goal.question      = getInput<std::string>("question").value();
    goal.stamp.sec     = 0;
    goal.stamp.nanosec = 0;   // 0 → latest frame in the ring buffer
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & wr) override
  {
    setOutput("rationale", wr.result->rationale);

    if (!wr.result->answered) {
      RCLCPP_WARN(logger(), "Visual question could not answer: %s",
                  wr.result->rationale.c_str());
      return onUnknown();
    }

    if (wr.result->affirmative) {
      RCLCPP_INFO(logger(), "Visual question answered YES: %s",
                  wr.result->rationale.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(logger(), "Visual question answered NO: %s",
                wr.result->rationale.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Infra failures (aborted, cancelled, server unreachable, send timeout) never
  // yield a verdict — treat them the same as answered=false and abstain.
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error,
                           const std::optional<WrappedResult> & wr) override
  {
    if (wr) {
      setOutput("rationale", wr->result->rationale);
    }
    RCLCPP_WARN(logger(), "VisualQuestion action could not run (%s) — abstaining",
                BT::toStr(error));
    return onUnknown();
  }

private:
  // Abstain: return the status named by the "on_unknown" port. Anything
  // starting with 'f'/'F' means FAILURE (don't rescue); otherwise SUCCESS
  // (don't veto) — the safe fail-open default for a guard.
  BT::NodeStatus onUnknown()
  {
    std::string policy = getInput<std::string>("on_unknown").value_or("SUCCESS");
    if (!policy.empty() && (policy.front() == 'f' || policy.front() == 'F')) {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace hint_bt
