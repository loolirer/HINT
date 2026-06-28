#include <chrono>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_ros2/bt_action_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>

#include <hint_interfaces/action/approach_target.hpp>
#include <hint_interfaces/action/ground_description.hpp>

using namespace BT;

// ── Leaf 1: ground a text description to an ROI ───────────────────────────

class GroundDescriptionAction
  : public RosActionNode<hint_interfaces::action::GroundDescription>
{
public:
  using RosActionNode::RosActionNode;

  static PortsList providedPorts()
  {
    return providedBasicPorts({
      InputPort<std::string>("description"),
      OutputPort<sensor_msgs::msg::RegionOfInterest>("roi"),
      OutputPort<builtin_interfaces::msg::Time>("stamp"),
    });
  }

  bool setGoal(Goal & goal) override
  {
    goal.description   = getInput<std::string>("description").value();
    goal.stamp.sec     = 0;
    goal.stamp.nanosec = 0;   // 0 → latest frame in the ring buffer
    return true;
  }

  NodeStatus onResultReceived(const WrappedResult & wr) override
  {
    if (!wr.result->success) {
      RCLCPP_WARN(logger(), "Grounding failed: %s", wr.result->message.c_str());
      return NodeStatus::FAILURE;
    }
    setOutput("roi",   wr.result->roi);
    setOutput("stamp", wr.result->stamp);
    return NodeStatus::SUCCESS;
  }

  NodeStatus onFailure(ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(logger(), "GroundDescription action error: %s", toStr(error));
    return NodeStatus::FAILURE;
  }

  void onHalt() override {}
};

// ── Leaf 2: drive the robot to the grounded ROI ───────────────────────────

class ApproachTargetAction
  : public RosActionNode<hint_interfaces::action::ApproachTarget>
{
public:
  using RosActionNode::RosActionNode;

  static PortsList providedPorts()
  {
    return providedBasicPorts({
      InputPort<sensor_msgs::msg::RegionOfInterest>("roi"),
      InputPort<builtin_interfaces::msg::Time>("stamp"),
    });
  }

  bool setGoal(Goal & goal) override
  {
    goal.roi   = getInput<sensor_msgs::msg::RegionOfInterest>("roi").value();
    goal.stamp = getInput<builtin_interfaces::msg::Time>("stamp").value();
    return true;
  }

  NodeStatus onResultReceived(const WrappedResult & wr) override
  {
    if (!wr.result->success) {
      RCLCPP_WARN(logger(), "Approach failed: %s", wr.result->message.c_str());
      return NodeStatus::FAILURE;
    }
    return NodeStatus::SUCCESS;
  }

  NodeStatus onFailure(ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(logger(), "ApproachTarget action error: %s", toStr(error));
    return NodeStatus::FAILURE;
  }

  void onHalt() override {}
};

// ── Main ──────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Usage: approach_described_target \"<description>\"\n";
    return 1;
  }

  auto node = std::make_shared<rclcpp::Node>("hint_bt_node");

  // Each node type gets its own params so default_port_value can differ.
  RosNodeParams ground_params(node, "/description_detector_node/ground_description");
  RosNodeParams approach_params(node, "/visual_servoing_node/approach_target");

  BehaviorTreeFactory factory;
  factory.registerNodeType<GroundDescriptionAction>("GroundDescriptionAction", ground_params);
  factory.registerNodeType<ApproachTargetAction>("ApproachTargetAction",       approach_params);

  const std::string xml_path =
    ament_index_cpp::get_package_share_directory("hint_bt") +
    "/config/approach_described_target.xml";
  auto tree = factory.createTreeFromFile(xml_path);

  // Inject the description before the first tick.
  tree.rootBlackboard()->set("description", std::string(argv[1]));

  // ROS2 must spin to deliver action result callbacks while the tick loop runs.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  NodeStatus status = NodeStatus::RUNNING;
  while (rclcpp::ok() && status == NodeStatus::RUNNING) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    status = tree.tickOnce();
  }

  RCLCPP_INFO(node->get_logger(), "Tree finished: %s",
    status == NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return status == NodeStatus::SUCCESS ? 0 : 1;
}
