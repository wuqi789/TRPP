cmake_minimum_required(VERSION 3.8)
project(agx_arm_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install package resource directories
install(
  DIRECTORY
    launch
    urdf
    config
    rviz
    scripts
    meshes
    mujoco
  DESTINATION share/${PROJECT_NAME}/
)

# Install the agx_arm_urdf submodule (URDF and mesh files)
# Run before use: git submodule update --init --recursive
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/agx_arm_urdf")
  install(
    DIRECTORY agx_arm_urdf/
    DESTINATION share/${PROJECT_NAME}/agx_arm_urdf/
  )
  message(STATUS "[agx_arm_description] agx_arm_urdf submodule found; installing URDF resources.")
else()
  message(WARNING
    "[agx_arm_description] agx_arm_urdf submodule not found!\n"
    "Run: git submodule update --init --recursive"
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
