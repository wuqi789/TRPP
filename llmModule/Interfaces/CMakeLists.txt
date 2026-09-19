cmake_minimum_required(VERSION 3.8)
project(semantic_navigation_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Constraint.msg"
  "msg/CheckResult.msg"
  "msg/NavigationIntent.msg"
  "msg/ResolutionResult.msg"
  "msg/VerificationResult.msg"
  "msg/NavigationTaskStatus.msg"
  "msg/SystemReadiness.msg"
  "msg/PathAdaptationStatus.msg"
  "msg/PushabilityObject.msg"
  "msg/PushabilityMap.msg"
  "action/ExecuteNavigation.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
