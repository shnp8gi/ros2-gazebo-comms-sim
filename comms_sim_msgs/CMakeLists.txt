cmake_minimum_required(VERSION 3.8)
project(comms_sim_msgs)

# =============================================================================
# Find dependencies
# =============================================================================
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# =============================================================================
# Generate custom messages
# =============================================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CommsQuality.msg"
  DEPENDENCIES std_msgs
)

# =============================================================================
# Export and finalize
# =============================================================================
ament_export_dependencies(rosidl_default_runtime)
ament_package()
