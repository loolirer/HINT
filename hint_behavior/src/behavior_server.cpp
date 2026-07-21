#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <behaviortree_ros2/tree_execution_server.hpp>

#include "hint_behavior/register_nodes.hpp"

// Thin HINT-specific wrapper around behaviortree_ros2's TreeExecutionServer:
// registers the HINT leaf node types, and treats each goal's `payload` as
// the tree's raw XML text so a caller can send an arbitrary, freshly
// composed tree per goal instead of only invoking trees preloaded at
// startup from the `behavior_trees` ROS param.
class HintBtExecutorNode : public BT::TreeExecutionServer
{
public:
  explicit HintBtExecutorNode(const rclcpp::Node::SharedPtr & node)
    : BT::TreeExecutionServer(node)
  {
  }

protected:
  void registerNodesIntoFactory(BT::BehaviorTreeFactory & factory) override
  {
    hint_behavior::registerHintNodes(factory, node());
  }

  // `payload` is expected to be a full BTCPP_format="4" document containing
  // a <BehaviorTree ID="..."> that matches `tree_name`. Registering it here
  // makes it available to the base class's subsequent factory.createTree()
  // call. An empty payload falls back to a tree already preloaded from disk.
  bool onGoalReceived(const std::string & tree_name, const std::string & payload) override
  {
    if (payload.empty()) {
      return true;
    }
    try {
      factory().registerBehaviorTreeFromText(payload);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        node()->get_logger(), "Failed to register tree '%s': %s", tree_name.c_str(), e.what());
      return false;
    }
    return true;
  }

  // Narrates progress: name of the currently RUNNING leaf (action) node.
  std::optional<std::string> onLoopFeedback() override
  {
    std::string name;
    tree().applyVisitor([&name](const BT::TreeNode * n) {
      if (name.empty() && n->type() == BT::NodeType::ACTION &&
          n->status() == BT::NodeStatus::RUNNING)
      {
        name = n->name();
      }
    });
    return name.empty() ? std::nullopt : std::optional<std::string>(name);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto ros_node = std::make_shared<rclcpp::Node>("hint_behavior_server");
  auto executor_node = std::make_shared<HintBtExecutorNode>(ros_node);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(ros_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
