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
find_package(Protobuf REQUIRED)

# =============================================================================
# 制御プレーン⇔データプレーン間のカスタム protobuf メッセージ生成
# (C++ はプラグインへ、Python は comms_sim_proto パッケージとしてインストール)
# =============================================================================
protobuf_generate_cpp(COMMS_PROTO_SRCS COMMS_PROTO_HDRS proto/comms_sim_msgs.proto)

set(PY_PROTO_DIR ${CMAKE_CURRENT_BINARY_DIR}/comms_sim_proto)
add_custom_command(
  OUTPUT ${PY_PROTO_DIR}/comms_sim_msgs_pb2.py ${PY_PROTO_DIR}/__init__.py
  COMMAND ${CMAKE_COMMAND} -E make_directory ${PY_PROTO_DIR}
  COMMAND protobuf::protoc --python_out=${PY_PROTO_DIR}
          -I${CMAKE_CURRENT_SOURCE_DIR}/proto
          ${CMAKE_CURRENT_SOURCE_DIR}/proto/comms_sim_msgs.proto
  COMMAND ${CMAKE_COMMAND} -E touch ${PY_PROTO_DIR}/__init__.py
  DEPENDS proto/comms_sim_msgs.proto
  COMMENT "Generating Python protobuf module (comms_sim_proto)"
)
add_custom_target(comms_sim_msgs_py ALL
  DEPENDS ${PY_PROTO_DIR}/comms_sim_msgs_pb2.py ${PY_PROTO_DIR}/__init__.py)
install(FILES ${PY_PROTO_DIR}/comms_sim_msgs_pb2.py ${PY_PROTO_DIR}/__init__.py
  DESTINATION ${PYTHON_INSTALL_DIR}/comms_sim_proto)

# =============================================================================
# Gazebo Plugins
# =============================================================================
add_library(TxControllerPlugin SHARED
  src/TxControllerPlugin.cc
  src/antenna_pattern_parser.cpp
  src/comms_calculator.cpp
  ${COMMS_PROTO_SRCS}
)
target_include_directories(TxControllerPlugin PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(TxControllerPlugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-math7::gz-math7
  protobuf::libprotobuf
)
ament_target_dependencies(TxControllerPlugin
  yaml_cpp_vendor
)
set_target_properties(TxControllerPlugin PROPERTIES PREFIX "")

# 移動遮蔽体用の軽量waypoint追従プラグイン (通信機能なし)
add_library(WaypointMoverPlugin SHARED
  src/WaypointMoverPlugin.cc
)
target_include_directories(WaypointMoverPlugin PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(WaypointMoverPlugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-math7::gz-math7
)
set_target_properties(WaypointMoverPlugin PROPERTIES PREFIX "")

# 姿勢レコーダ (軌跡を記録し、通信計算を事後に回すため)
add_library(PoseRecorderPlugin SHARED
  src/PoseRecorderPlugin.cc
)
target_include_directories(PoseRecorderPlugin PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(PoseRecorderPlugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-math7::gz-math7
)
set_target_properties(PoseRecorderPlugin PROPERTIES PREFIX "")

# KKF C++リファレンスのゴールデンダンプ (Python kkf_core との数値一致検証用)
add_executable(kkf_golden_dump src/kkf_golden_dump.cc)
target_include_directories(kkf_golden_dump PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)
install(TARGETS kkf_golden_dump
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS TxControllerPlugin WaypointMoverPlugin PoseRecorderPlugin
  DESTINATION lib/${PROJECT_NAME}/plugins
)

# =============================================================================
# Install Python modules
# =============================================================================
ament_python_install_package(${PROJECT_NAME})

# =============================================================================
# Install Python executables
# =============================================================================
# 対象となるPythonノードスクリプトのリストを一元管理（DRY原則）
# 実行権限は git のファイルモード(100755)で管理する。ビルド時にソースツリーへ
# chmod する等の副作用は持たせないこと。
set(PYTHON_EXECUTABLE_NODES
  comms_sim_pkg/sim_ready_gate_node.py
  comms_sim_pkg/progress_logger_node.py
  comms_sim_pkg/mission_coordinator_node.py
  comms_sim_pkg/kkf_scheduler_node.py
)

install(PROGRAMS
  ${PYTHON_EXECUTABLE_NODES}
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
