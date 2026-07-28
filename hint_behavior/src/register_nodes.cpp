#include "hint_behavior/register_nodes.hpp"

#include <behaviortree_ros2/ros_node_params.hpp>

#include "hint_behavior/nodes/follow_visual_path_action.hpp"
#include "hint_behavior/nodes/mission_advance_action.hpp"
#include "hint_behavior/nodes/plan_visual_path_action.hpp"
#include "hint_behavior/nodes/visual_reason_action.hpp"
#include "hint_behavior/nodes/spin_action.hpp"

namespace hint_behavior
{

void registerHintNodes(BT::BehaviorTreeFactory & factory, std::shared_ptr<rclcpp::Node> node)
{
  // Each node type gets its own params so default_port_value can differ.
  BT::RosNodeParams plan_params(node, "/path_planner/plan_visual_path");
  BT::RosNodeParams follow_params(node, "/path_projector_node/follow_visual_path");
  BT::RosNodeParams spin_params(node, "/spin");
  BT::RosNodeParams visual_reason_params(node, "/visual_reasoner/visual_reason");
  BT::RosNodeParams mission_advance_params(node, "/narrative_navigation/advance");

  factory.registerNodeType<PlanVisualPathAction>("PlanVisualPathAction", plan_params);
  factory.registerNodeType<FollowVisualPathAction>("FollowVisualPathAction", follow_params);
  factory.registerNodeType<SpinAction>("SpinAction", spin_params);
  factory.registerNodeType<VisualReasonAction>("VisualReasonAction", visual_reason_params);
  factory.registerNodeType<MissionAdvance>("MissionAdvance", mission_advance_params);
}

}  // namespace hint_behavior
