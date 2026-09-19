cmake_minimum_required(VERSION 3.8)
project(llm_module2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(semantic_navigation_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SemanticGoal.msg"
  "srv/QueryEntity.srv"
  "srv/QueryRelation.srv"
  "srv/GetPose.srv"
  "srv/QueryTopology.srv"
)

ament_python_install_package(semantic_graph)
ament_python_install_package(builder)
ament_python_install_package(query)
ament_python_install_package(providers)
ament_python_install_package(matching)
ament_python_install_package(resolution)
ament_python_install_package(ros_interface)

install(PROGRAMS ros_interface/semantic_graph_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME semantic_graph_node
)
install(DIRECTORY launch maps DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(module2_graph_tests test/test_graph.py)
  ament_add_pytest_test(module2_resolution_tests test/test_resolution.py)
  ament_add_pytest_test(module2_scout_map_tests test/test_scout_map.py)
endif()

ament_export_dependencies(
  rosidl_default_runtime
  semantic_navigation_interfaces
  geometry_msgs
)
ament_package()
