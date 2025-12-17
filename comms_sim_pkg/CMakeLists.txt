cmake_minimum_required(VERSION 3.8)
project(comms_sim_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =============================================================================
# Find dependencies
# =============================================================================
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# =============================================================================
# Generate custom messages
# =============================================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CommsQuality.msg"
  DEPENDENCIES std_msgs
)

# =============================================================================
# Install Python modules
# =============================================================================
ament_python_install_package(${PROJECT_NAME})

# =============================================================================
# Install Python executables
# =============================================================================
install(PROGRAMS
  comms_sim_pkg/comms_node.py
  comms_sim_pkg/ugv_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# =============================================================================
# Install launch files
# =============================================================================
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# =============================================================================
# Install resource files (world, config, etc.)
# =============================================================================
install(DIRECTORY
  resource
  DESTINATION share/${PROJECT_NAME}
)

# =============================================================================
# Testing
# =============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# =============================================================================
# Export and finalize
# =============================================================================
ament_export_dependencies(rosidl_default_runtime)
ament_package()
