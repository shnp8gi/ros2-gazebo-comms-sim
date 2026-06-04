#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <comms_sim_msgs/msg/comms_quality.hpp>
#include <Eigen/Dense>
#include <optional>
#include <vector>
#include <string>

#include "comms_sim_pkg/antenna_pattern_parser.hpp"
#include "comms_sim_pkg/comms_calculator.hpp"

namespace comms_sim {

enum class LinkState { DISCONNECTED, ESTABLISHING, CONNECTED };

struct RxConfig {
  Eigen::Vector3d position;
  Eigen::Vector3d antenna_offset;
  Eigen::Vector3d rpy;
  Eigen::Vector3d antenna_relative_rpy;
  std::string name;
  Eigen::Matrix3d rotmat;
};

struct VehicleAntenna {
  std::string name;
  Eigen::Vector3d offset;
  Eigen::Vector3d relative_rpy;
};

class CommsSimulatorNode : public rclcpp::Node {
public:
  CommsSimulatorNode();
  ~CommsSimulatorNode();

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void on_rx_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_logging_start_topic(const std_msgs::msg::Bool::SharedPtr msg);
  void on_mission_complete(const std_msgs::msg::Bool::SharedPtr msg);
  void on_link_grant(const std_msgs::msg::Bool::SharedPtr msg);
  void on_all_ready(const std_msgs::msg::Bool::SharedPtr msg);

  void calculate_and_publish(const Eigen::Vector3d& pos, const Eigen::Vector3d& ori, double dt_step, double current_time);
  bool update_link_state(double rssi, double current_time, bool has_link_grant);
  bool get_current_segment_pose(double& best_px, double& best_py, double& best_yaw);

  static Eigen::Vector3d quat_to_rpy(double x, double y, double z, double w);

  // Components
  AntennaPatternParser antenna_parser_;
  std::unique_ptr<CommsCalculator> comms_calculator_;
  std::unique_ptr<PropagationModel> propagation_model_;

  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr bs_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr logging_start_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_complete_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr link_grant_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr all_ready_sub_;

  // Publishers
  rclcpp::Publisher<comms_sim_msgs::msg::CommsQuality>::SharedPtr quality_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr link_request_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;

  // Timers
  rclcpp::TimerBase::SharedPtr ready_timer_;

  // Parameters
  double sampling_rate_;
  double noise_variance_;
  std::string e_plane_path_;
  std::string h_plane_path_;
  double comm_data_limit_mb_;
  double tx_power_;
  std::string mcs_table_path_;
  double link_establishment_time_;
  std::string mission_complete_topic_;
  std::string odom_topic_;
  std::vector<double> tx_spawn_pose_;
  Eigen::Vector3d tx_antenna_relative_rpy_;
  std::string rx_pose_topic_;
  std::string vehicle_name_;
  std::string cmd_vel_topic_;
  std::string logging_trigger_;
  std::string logging_start_topic_;

  // State
  std::vector<RxConfig> rx_nodes_;
  std::vector<VehicleAntenna> vehicle_antennas_;
  std::vector<std::vector<double>> waypoints_;
  std::string scheduling_policy_;

  Eigen::Vector3d tx_antenna_offset_;

  std::optional<Eigen::Vector3d> tx_orientation_;
  std::optional<Eigen::Vector3d> tx_local_position_;
  
  bool odom_offset_set_;
  Eigen::Vector3d odom_offset_;

  double total_data_transmitted_;
  bool comm_active_;
  std::optional<rclcpp::Time> last_odom_time_;
  bool logging_ready_;
  LinkState link_state_;
  std::optional<double> link_establishment_start_time_;
  int establishment_step_count_ = 0;

  bool has_link_grant_;
  bool all_nodes_ready_ = false;

  double rssi_threshold_;
  
  std::optional<double> last_calc_sim_time_;
  std::optional<Eigen::Vector3d> last_tx_pos_;
  std::optional<Eigen::Vector3d> last_tx_orientation_;
  double last_rssi_;
  double publish_rate_;
  double last_publish_time_;
};

}  // namespace comms_sim
