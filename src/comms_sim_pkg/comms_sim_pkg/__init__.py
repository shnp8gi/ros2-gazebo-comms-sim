# =============================================================================
# comms_sim_pkg - ROS 2 Communication Simulation Package
# =============================================================================
"""
ROS 2 package for simulating wireless communication quality between
a mobile TX and fixed ground stations in Gazebo Harmonic.

通信計算・スケジューリングの実体は C++ (Gazebo プラグイン) に移行済み。
本パッケージは補助ノード群と、KKF制御プレーンの純粋数理層 (kkf_core) を提供する。
"""
