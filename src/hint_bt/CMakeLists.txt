cmake_minimum_required(VERSION 3.8)
project(hint_bt)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(hint_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Registry of HINT-specific BT leaf node types (GroundDescriptionAction,
# ApproachTargetAction, ...), shared by every executable that needs to
# build/tick a tree referencing them.
add_library(hint_bt_nodes
  src/register_nodes.cpp
)
target_include_directories(hint_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(hint_bt_nodes
  rclcpp
  behaviortree_ros2
  hint_interfaces
  sensor_msgs
  geometry_msgs
  builtin_interfaces
)

# Long-running action server: a thin HintBtExecutorNode subclass of
# behaviortree_ros2's TreeExecutionServer. Registration of HINT leaf types
# happens via hint_bt_nodes; btcpp_ros2_interfaces (ExecuteTree action) and
# rclcpp_action come in transitively through behaviortree_ros2.
add_executable(bt_executor_node
  src/bt_executor_node.cpp
)
target_link_libraries(bt_executor_node hint_bt_nodes)
ament_target_dependencies(bt_executor_node
  rclcpp
  behaviortree_ros2
)

install(TARGETS bt_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS hint_bt_nodes
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY behaviors/
  DESTINATION share/${PROJECT_NAME}/behaviors
)

ament_package()
