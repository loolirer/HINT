#pragma once

#include <memory>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace hint_behavior
{

// Registers every HINT-specific BT leaf node type against its fixed action
// server name. Called once at executor startup, before any tree is loaded —
// tree XML only ever references these node type names, never redefines them.
void registerHintNodes(BT::BehaviorTreeFactory & factory, std::shared_ptr<rclcpp::Node> node);

}  // namespace hint_behavior
