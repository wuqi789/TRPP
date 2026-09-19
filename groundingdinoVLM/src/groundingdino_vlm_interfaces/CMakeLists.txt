cmake_minimum_required(VERSION 3.8)
project(groundingdino_vlm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectionCandidate.msg"
  "msg/TargetVerification.msg"
  "action/VerifyTarget.action"
  DEPENDENCIES std_msgs sensor_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
