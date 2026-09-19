cmake_minimum_required(VERSION 3.8)
project(llm_module1)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(semantic_navigation_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NavigationIntent.msg"
)

ament_python_install_package(llm_agent)

install(PROGRAMS llm_agent/llm_agent_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME llm_agent_node
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(module1_parser_tests test/test_parser.py)
endif()

ament_export_dependencies(rosidl_default_runtime semantic_navigation_interfaces)
ament_package()
