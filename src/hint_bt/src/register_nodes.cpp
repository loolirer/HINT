#include "hint_bt/register_nodes.hpp"

#include <behaviortree_ros2/ros_node_params.hpp>

#include "hint_bt/nodes/approach_target_action.hpp"
#include "hint_bt/nodes/ground_description_action.hpp"
#include "hint_bt/nodes/visual_question_action.hpp"

namespace hint_bt
{

void registerHintNodes(BT::BehaviorTreeFactory & factory, std::shared_ptr<rclcpp::Node> node)
{
  // Each node type gets its own params so default_port_value can differ.
  BT::RosNodeParams ground_params(node, "/description_detector_node/ground_description");
  BT::RosNodeParams approach_params(node, "/visual_servoing_node/approach_target");
  BT::RosNodeParams question_params(node, "/visual_question_node/ask");

  factory.registerNodeType<GroundDescriptionAction>("GroundDescriptionAction", ground_params);
  factory.registerNodeType<ApproachTargetAction>("ApproachTargetAction", approach_params);
  factory.registerNodeType<VisualQuestionAction>("VisualQuestionAction", question_params);
}

}  // namespace hint_bt
