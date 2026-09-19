cmake_minimum_required(VERSION 3.8)
project(llm_module3)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(semantic_navigation_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VerificationResult.msg"
  "srv/VerifyNavigation.srv"
)

ament_python_install_package(verification_core)
ament_python_install_package(entity_checker)
ament_python_install_package(topology_checker)
ament_python_install_package(geometry_checker)
ament_python_install_package(dynamic_checker)
ament_python_install_package(module3_ros_interface)

install(PROGRAMS module3_ros_interface/verification_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME verification_node
)
install(PROGRAMS module3_ros_interface/map_normalizer_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME validation_map_normalizer
)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(module3_legacy_core_tests test/test_verification.py)
  ament_add_pytest_test(module3_pipeline_tests test/test_pipeline.py)
  ament_add_pytest_test(module3_runtime_contract_tests test/test_runtime_contract.py)
  ament_add_pytest_test(module3_map_normalizer_tests test/test_map_normalizer.py)
endif()

ament_export_dependencies(
  rosidl_default_runtime
  semantic_navigation_interfaces
  geometry_msgs
)
ament_package()
