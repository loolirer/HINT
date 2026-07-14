cmake_minimum_required(VERSION 3.8)
project(hint_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VisualWaypoints.msg"
  "srv/SetTarget.srv"
  "srv/SetWaypoints.srv"
  "srv/StopTracking.srv"
  "action/ApproachTarget.action"
  "action/FollowTrajectory.action"
  "action/GroundDescription.action"
  "action/VisualQuestion.action"
  "action/PlanTrajectory.action"
  "action/Reason.action"
  "action/MissionAdvance.action"
  DEPENDENCIES builtin_interfaces std_msgs sensor_msgs geometry_msgs
)

ament_package()
