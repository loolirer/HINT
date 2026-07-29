#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <behaviortree_ros2/tree_execution_server.hpp>

#include "hint_behavior/register_nodes.hpp"

class HintBtExecutorNode : public BT::TreeExecutionServer
{
public:
  explicit HintBtExecutorNode(const rclcpp::Node::SharedPtr & node)
    : BT::TreeExecutionServer(node)
  {
    state_pub_ = node->create_publisher<std_msgs::msg::String>(
      "~/state", rclcpp::QoS(1).transient_local());
    publishState("IDLE");
  }

protected:
  void registerNodesIntoFactory(BT::BehaviorTreeFactory & factory) override
  {
    hint_behavior::registerHintNodes(factory, node());
  }

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
    if (!name.empty()) {
      publishState(name);
      return name;
    }
    return std::nullopt;
  }

  std::optional<std::string> onTreeExecutionCompleted(BT::NodeStatus, bool) override
  {
    publishState("IDLE");
    return std::nullopt;
  }

private:
  void publishState(const std::string & s)
  {
    std_msgs::msg::String msg;
    msg.data = s;
    state_pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
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
