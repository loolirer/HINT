#include "hint_behavior/register_nodes.hpp"

#include <behaviortree_ros2/ros_node_params.hpp>

#include "hint_behavior/nodes/follow_trajectory_action.hpp"
#include "hint_behavior/nodes/mission_advance_action.hpp"
#include "hint_behavior/nodes/plan_trajectory_action.hpp"
#include "hint_behavior/nodes/reason_action.hpp"
#include "hint_behavior/nodes/spin_action.hpp"

namespace hint_behavior
{

void registerHintNodes(BT::BehaviorTreeFactory & factory, std::shared_ptr<rclcpp::Node> node)
{
  // Each node type gets its own params so default_port_value can differ.
  BT::RosNodeParams plan_params(node, "/trajectory_generator/plan_trajectory");
  BT::RosNodeParams follow_params(node, "/trajectory_navigator_node/follow_trajectory");
  BT::RosNodeParams spin_params(node, "/spin");
  BT::RosNodeParams reason_params(node, "/visual_reasoner/reason");
  BT::RosNodeParams mission_advance_params(node, "/narrative_navigation/advance");

  factory.registerNodeType<PlanTrajectoryAction>("PlanTrajectoryAction", plan_params);
  factory.registerNodeType<FollowTrajectoryAction>("FollowTrajectoryAction", follow_params);
  factory.registerNodeType<SpinAction>("SpinAction", spin_params);
  factory.registerNodeType<ReasonAction>("ReasonAction", reason_params);
  factory.registerNodeType<MissionAdvance>("MissionAdvance", mission_advance_params);
}

}  // namespace hint_behavior
