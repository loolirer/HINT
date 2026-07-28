#pragma once

#include <optional>
#include <string>
#include <vector>

#include <behaviortree_ros2/bt_action_node.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <hint_interfaces/action/plan_path.hpp>

namespace hint_behavior
{

// Plans a ground path from a text description via path_planner's
// PlanPath action; outputs the ordered normalized waypoints (markers).
// The frames to plan over come in via {images} — the unified hint_narrative
// buffer, forwarded from MissionAdvance — not from a camera buffer in the node.
class PlanPathAction
  : public BT::RosActionNode<hint_interfaces::action::PlanPath>
{
public:
  using RosActionNode::RosActionNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("description"),
      BT::InputPort<std::vector<sensor_msgs::msg::CompressedImage>>(
        "images", "frames to plan over (unified narrative buffer, last = current view)"),
      BT::OutputPort<std::vector<geometry_msgs::msg::Point>>("markers"),
      BT::OutputPort<double>(
        "turn_degrees", "in-place turn to apply after the path (+left / -right, deg)"),
      BT::OutputPort<builtin_interfaces::msg::Time>("stamp"),
      BT::OutputPort<std::string>(
        "message", "the VLM's explanation of the chosen path (or why planning failed)"),
    });
  }

  bool setGoal(Goal & goal) override
  {
    goal.description = getInput<std::string>("description").value();
    // Frames to plan over ride in on the goal (the unified narrative buffer);
    // the node keeps no camera buffer of its own. Missing/empty is allowed here
    // (the server aborts on empty), so tolerate an unbound port.
    goal.images =
      getInput<std::vector<sensor_msgs::msg::CompressedImage>>("images").value_or(
        std::vector<sensor_msgs::msg::CompressedImage>{});
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & wr) override
  {
    // Only reached on a SUCCEEDED goal (aborts go to onFailure), so this is always
    // a valid plan — no success bool to check. Surface the VLM's path reasoning so
    // the mission log captures it as grounded visual feedback.
    setOutput("message",      wr.result->message);
    setOutput("markers",      wr.result->markers);
    setOutput("turn_degrees", wr.result->turn_degrees);
    setOutput("stamp",        wr.result->stamp);
    return BT::NodeStatus::SUCCESS;
  }

  // Aborts carry the model's rationale — empty waypoints mean "no path visible"
  // or "already there", with the reason in the result message. Propagate it to
  // {message} so a consumer (the mission narrative) gets that grounding instead
  // of a stale value. Uses the two-arg overload because the abort path
  // (bt_action_node.hpp) passes the result there; the single-arg one drops it.
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error,
                           const std::optional<WrappedResult> & wr) override
  {
    if (wr) {
      setOutput("message", wr->result->message);
    }
    RCLCPP_WARN(logger(), "PlanPath failed (%s)", BT::toStr(error));
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace hint_behavior
