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
find_package(comms_sim_msgs REQUIRED)

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
  comms_sim_pkg/link_controller_node.py
  comms_sim_pkg/sim_logger_node.py
  comms_sim_pkg/link_scheduling_strategy.py
  DESTINATION lib/${PROJECT_NAME}
)

# =============================================================================
# Install launch and resource files
# =============================================================================
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

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
ament_package()
