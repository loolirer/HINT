cmake_minimum_required(VERSION 3.8)
project(hint_bt)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(hint_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

add_executable(approach_described_target
  src/approach_described_target.cpp
)

ament_target_dependencies(approach_described_target
  rclcpp
  ament_index_cpp
  behaviortree_ros2
  hint_interfaces
  sensor_msgs
  builtin_interfaces
)

install(TARGETS approach_described_target
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
