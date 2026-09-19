cmake_minimum_required(VERSION 3.8)
project(obstacle_traversal)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/traversal_manager
  scripts/scan_filter
  scripts/velocity_gate
  scripts/approach_controller
  scripts/preflight
  scripts/acceptance_recorder
  scripts/acceptance_report
  scripts/system_diagnostics
  scripts/core_acceptance_driver
  scripts/publish_instruction
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(obstacle_traversal_core test/test_core.py)
  ament_add_pytest_test(obstacle_traversal_acceptance_report test/test_acceptance_report.py)
  ament_add_pytest_test(obstacle_traversal_four_terminal test/test_four_terminal_contract.py)
  ament_add_pytest_test(obstacle_traversal_system_diagnostics test/test_system_diagnostics.py)
endif()

ament_package()
