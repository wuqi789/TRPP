cmake_minimum_required(VERSION 3.8)
project(obstacle_traversal_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VehicleState.msg"
  "msg/PushabilityAssessment.msg"
  "msg/TraversalStatus.msg"
  "msg/FilterAuthorization.msg"
  "msg/FilterState.msg"
  "msg/ApproachVelocity.msg"
  "msg/ObstacleDetection.msg"
  "msg/MechanicalAssessment.msg"
  "action/AssessPushability.action"
  "action/ApproachObstacle.action"
  "action/ProbePushability.action"
  "srv/ResetTraversal.srv"
  DEPENDENCIES action_msgs builtin_interfaces geometry_msgs nav_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
