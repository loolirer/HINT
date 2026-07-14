#pragma once

#include <optional>
#include <string>

#include <behaviortree_ros2/bt_action_node.hpp>

#include <hint_interfaces/action/mission_advance.hpp>

namespace hint_bt
{

// One cycle of the mission loop. Reports the outcome of the move just executed
// (`success` + the VLM `observation`, the planner's path reasoning) to the
// mission_planner, which folds it into its rolling narrative via one reasoner
// call and returns the *next* directive. Writes the next instruction and the
// current environment (`area`) to the blackboard. Returns SUCCESS while there is
// a move to run, FAILURE when the mission is complete (the loop's stop signal).
// On the first tick nothing has executed (success defaults true, observation
// empty), so the node simply hands out the first instruction.
class MissionAdvance
  : public BT::RosActionNode<hint_interfaces::action::MissionAdvance>
{
public:
  using RosActionNode::RosActionNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      // String (not bool) to match SetBlackboard, which writes "true"/"false"
      // as a string — avoids any blackboard type-lock mismatch at tree build.
      BT::InputPort<std::string>("success", "true", "did the move just executed succeed?"),
      BT::InputPort<std::string>("observation", "",
                                 "grounded VLM feedback from the execution (verbatim)"),
      BT::OutputPort<std::string>("description", "next instruction to feed trajectory_planner"),
      BT::OutputPort<std::string>("area", "the current environment (from the narrative)"),
    });
  }

  bool setGoal(Goal & goal) override
  {
    goal.success     = (getInput<std::string>("success").value_or("true") != "false");
    goal.observation = getInput<std::string>("observation").value_or("");
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & wr) override
  {
    setOutput("description", wr.result->description);
    setOutput("area", wr.result->area);

    if (wr.result->mission_done) {
      RCLCPP_INFO(logger(), "Mission complete: %s", wr.result->message.c_str());
      return BT::NodeStatus::FAILURE;   // loop stop signal
    }
    RCLCPP_INFO(logger(), "[%s] next: %s",
                wr.result->area.c_str(), wr.result->description.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error,
                           const std::optional<WrappedResult> &) override
  {
    RCLCPP_WARN(logger(), "MissionAdvance could not run (%s)", BT::toStr(error));
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace hint_bt
