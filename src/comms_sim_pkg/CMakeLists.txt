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
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(comms_sim_msgs REQUIRED)

# Gazebo Harmonic (gz-sim8)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-math7 REQUIRED)

# C++ executable
add_executable(comms_node_cpp
  src/comms_node.cpp
  src/antenna_pattern_parser.cpp
  src/comms_calculator.cpp
)
target_include_directories(comms_node_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(comms_node_cpp
  rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs
  comms_sim_msgs yaml_cpp_vendor
)
target_compile_options(comms_node_cpp PRIVATE -O3)

install(TARGETS comms_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# =============================================================================
# Gazebo Plugins
# =============================================================================
add_library(TxControllerPlugin SHARED
  src/TxControllerPlugin.cc
  src/antenna_pattern_parser.cpp
  src/comms_calculator.cpp
)
target_include_directories(TxControllerPlugin PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(TxControllerPlugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-math7::gz-math7
)
ament_target_dependencies(TxControllerPlugin
  yaml_cpp_vendor
)
set_target_properties(TxControllerPlugin PROPERTIES PREFIX "")

install(TARGETS TxControllerPlugin
  DESTINATION lib/${PROJECT_NAME}/plugins
)

# =============================================================================
# Install Python modules
# =============================================================================
ament_python_install_package(${PROJECT_NAME})

# =============================================================================
# Install Python executables
# =============================================================================
install(PROGRAMS
  comms_sim_pkg/sim_ready_gate_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# =============================================================================
# Install launch and resource files
# =============================================================================
install(DIRECTORY
  launch
  config
  models
  worlds
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
