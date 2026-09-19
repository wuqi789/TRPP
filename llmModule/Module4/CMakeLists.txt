cmake_minimum_required(VERSION 3.8)
project(llm_module4)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(semantic_navigation_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NavigationStatus.msg"
)

ament_python_install_package(execution_core)
ament_python_install_package(route_planning)
ament_python_install_package(module4_ros_interface)

install(PROGRAMS module4_ros_interface/navigation_executor_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME navigation_executor_node
)
install(PROGRAMS scripts/smoke_vlm_route.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME smoke_vlm_route
)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(module4_route_grid_tests test/test_route_grid.py)
  ament_add_pytest_test(module4_route_planner_tests test/test_route_planner.py)
  ament_add_pytest_test(module4_fifo_tests test/test_fifo.py)
  ament_add_pytest_test(module4_execution_helper_tests test/test_execution_helpers.py)
  ament_add_pytest_test(module4_execution_contract_tests test/test_execution_contract.py)
  ament_add_pytest_test(module4_semantic_launch_tests test/test_semantic_launch.py)
endif()

ament_export_dependencies(
  rosidl_default_runtime
  semantic_navigation_interfaces
  nav_msgs
)
ament_package()
