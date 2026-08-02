cmake_minimum_required(VERSION 3.8)
project(hint_behavior)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hint_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

add_library(hint_behavior_nodes
  src/register_nodes.cpp
)
target_include_directories(hint_behavior_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(hint_behavior_nodes
  rclcpp
  behaviortree_ros2
  hint_interfaces
  nav2_msgs
  sensor_msgs
  geometry_msgs
  builtin_interfaces
)

add_executable(behavior_server
  src/behavior_server.cpp
)
target_link_libraries(behavior_server hint_behavior_nodes)
ament_target_dependencies(behavior_server
  rclcpp
  behaviortree_ros2
  std_msgs
)

install(TARGETS behavior_server
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS hint_behavior_nodes
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY behaviors/
  DESTINATION share/${PROJECT_NAME}/behaviors
)

ament_package()
