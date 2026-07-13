#include "hint_bt/register_nodes.hpp"

#include <behaviortree_ros2/ros_node_params.hpp>

#include "hint_bt/nodes/approach_target_action.hpp"
#include "hint_bt/nodes/follow_trajectory_action.hpp"
#include "hint_bt/nodes/ground_description_action.hpp"
#include "hint_bt/nodes/plan_trajectory_action.hpp"
#include "hint_bt/nodes/reason_action.hpp"
#include "hint_bt/nodes/visual_question_action.hpp"

namespace hint_bt
{

void registerHintNodes(BT::BehaviorTreeFactory & factory, std::shared_ptr<rclcpp::Node> node)
{
  // Each node type gets its own params so default_port_value can differ.
  BT::RosNodeParams ground_params(node, "/description_detector_node/ground_description");
  BT::RosNodeParams approach_params(node, "/visual_servoing_node/approach_target");
  BT::RosNodeParams plan_params(node, "/trajectory_planner_node/plan_trajectory");
  BT::RosNodeParams follow_params(node, "/pursuit_servo_node/follow_trajectory");
  BT::RosNodeParams question_params(node, "/visual_question_node/ask");
  BT::RosNodeParams reason_params(node, "/reasoner_node/reason");

  factory.registerNodeType<GroundDescriptionAction>("GroundDescriptionAction", ground_params);
  factory.registerNodeType<ApproachTargetAction>("ApproachTargetAction", approach_params);
  factory.registerNodeType<PlanTrajectoryAction>("PlanTrajectoryAction", plan_params);
  factory.registerNodeType<FollowTrajectoryAction>("FollowTrajectoryAction", follow_params);
  factory.registerNodeType<VisualQuestionAction>("VisualQuestionAction", question_params);
  factory.registerNodeType<ReasonAction>("ReasonAction", reason_params);
}

}  // namespace hint_bt
