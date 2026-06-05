#include "comms_sim_pkg/comms_node.hpp"
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <cmath>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <regex>
#include <sstream>
#include <iomanip>

namespace comms_sim {

CommsSimulatorNode::CommsSimulatorNode()
: Node("comms_simulator_node"),
  odom_offset_set_(false),
  odom_offset_(0, 0, 0),
  total_data_transmitted_(0.0),
  comm_active_(true),
  link_state_(LinkState::DISCONNECTED),
  has_link_grant_(false),
  last_rssi_(0.0),
  next_grid_time_(std::nullopt),
  last_odom_pos_(std::nullopt)
{
  this->declare_parameter("sampling_rate", 1.0);
  this->declare_parameter("noise_variance", 2.0);
  this->declare_parameter("e_plane_path", "");
  this->declare_parameter("h_plane_path", "");
  this->declare_parameter("comm_data_limit_mb", -1.0);
  this->declare_parameter("path_loss.c", 299792458.0);
  this->declare_parameter("path_loss.frequency", 6.0e10);
  this->declare_parameter("path_loss.exponent", 2.0);
  this->declare_parameter("path_loss.d0", 1.0);
  this->declare_parameter("path_loss.pl_d0", -1.0);
  this->declare_parameter("tx_power", -7.0);
  this->declare_parameter("rx_position", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("rx_antenna_offset", std::vector<double>{0.0, 0.0, 10.5});
  this->declare_parameter("tx_antenna_offset", std::vector<double>{0.0, 0.0, 1.3});
  this->declare_parameter("mcs_table_path", "");
  this->declare_parameter("link_establishment_time_ms", 2.0);
  this->declare_parameter("mission_complete_topic", "/mission_complete");
  this->declare_parameter("odom_topic", "/odom");
  this->declare_parameter("tx_spawn_pose", std::vector<double>{-20.0, 0.0, 0.0});
  this->declare_parameter("tx_antenna_relative_rpy", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("rx_pose_topic", "/rx/pose");
  this->declare_parameter("rx_rpy", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("rx_antenna_relative_rpy", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("rx_positions", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("rx_antenna_offsets", std::vector<double>{0.0, 0.0, 10.5});
  this->declare_parameter("rx_rpys", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("rx_antenna_relative_rpys", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("logging_start_trigger", "on_movement");
  this->declare_parameter("logging_start_topic", "/logging/start");
  this->declare_parameter("max_antenna_attenuation", 30.0);
  this->declare_parameter("vehicle_name", "");
  this->declare_parameter("cmd_vel_topic", "/cmd_vel");
  this->declare_parameter("config_file_path", "/workspace/src/comms_sim_pkg/config/sim_params.yaml");
  this->declare_parameter("publish_rate", 100.0);

  sampling_rate_ = this->get_parameter("sampling_rate").as_double();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  last_publish_time_ = -1.0;
  noise_variance_ = this->get_parameter("noise_variance").as_double();
  e_plane_path_ = this->get_parameter("e_plane_path").as_string();
  h_plane_path_ = this->get_parameter("h_plane_path").as_string();
  comm_data_limit_mb_ = this->get_parameter("comm_data_limit_mb").as_double();
  tx_power_ = this->get_parameter("tx_power").as_double();
  mcs_table_path_ = this->get_parameter("mcs_table_path").as_string();
  link_establishment_time_ = this->get_parameter("link_establishment_time_ms").as_double() / 1000.0;
  mission_complete_topic_ = this->get_parameter("mission_complete_topic").as_string();
  odom_topic_ = this->get_parameter("odom_topic").as_string();
  tx_spawn_pose_ = this->get_parameter("tx_spawn_pose").as_double_array();
  
  auto rel = this->get_parameter("tx_antenna_relative_rpy").as_double_array();
  if (rel.size() >= 3) {
      tx_antenna_relative_rpy_ = Eigen::Vector3d(rel[0], rel[1], rel[2]);
  } else {
      tx_antenna_relative_rpy_ = Eigen::Vector3d::Zero();
  }

  rx_pose_topic_ = this->get_parameter("rx_pose_topic").as_string();
  vehicle_name_ = this->get_parameter("vehicle_name").as_string();
  cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
  logging_trigger_ = this->get_parameter("logging_start_trigger").as_string();
  logging_start_topic_ = this->get_parameter("logging_start_topic").as_string();

  logging_ready_ = (logging_trigger_ == "immediate");

  double c = this->get_parameter("path_loss.c").as_double();
  double freq = this->get_parameter("path_loss.frequency").as_double();
  double exp = this->get_parameter("path_loss.exponent").as_double();
  double d0 = this->get_parameter("path_loss.d0").as_double();
  double pl_d0 = this->get_parameter("path_loss.pl_d0").as_double();

  auto propagation_model = std::make_unique<LogDistancePathLossModel>(c, freq, exp, d0, pl_d0);

  comms_calculator_ = std::make_unique<CommsCalculator>(
    std::move(propagation_model), tx_power_, noise_variance_, mcs_table_path_);

  rssi_threshold_ = comms_calculator_->rssi_min;

  double max_att = this->get_parameter("max_antenna_attenuation").as_double();
  antenna_parser_ = AntennaPatternParser(max_att);
  if (!e_plane_path_.empty()) antenna_parser_.load_e_plane(e_plane_path_);
  if (!h_plane_path_.empty()) antenna_parser_.load_h_plane(h_plane_path_);

  auto bs_positions = this->get_parameter("rx_positions").as_double_array();
  auto bs_offsets = this->get_parameter("rx_antenna_offsets").as_double_array();
  auto bs_rpys = this->get_parameter("rx_rpys").as_double_array();
  auto bs_rel_rpys = this->get_parameter("rx_antenna_relative_rpys").as_double_array();

  if (bs_positions.size() >= 3 && bs_positions.size() % 3 == 0) {
      size_t n_bs = bs_positions.size() / 3;
      for (size_t i = 0; i < n_bs; ++i) {
          RxConfig bs;
          bs.position = Eigen::Vector3d(bs_positions[i*3], bs_positions[i*3+1], bs_positions[i*3+2]);
          if (bs_offsets.size() >= (i+1)*3) {
              bs.antenna_offset = Eigen::Vector3d(bs_offsets[i*3], bs_offsets[i*3+1], bs_offsets[i*3+2]);
          } else {
              bs.antenna_offset = Eigen::Vector3d(0, 0, 3.0);
          }
          if (bs_rpys.size() >= (i+1)*3) {
              bs.rpy = Eigen::Vector3d(bs_rpys[i*3], bs_rpys[i*3+1], bs_rpys[i*3+2]);
          } else {
              bs.rpy = Eigen::Vector3d::Zero();
          }
          if (bs_rel_rpys.size() >= (i+1)*3) {
              bs.antenna_relative_rpy = Eigen::Vector3d(bs_rel_rpys[i*3], bs_rel_rpys[i*3+1], bs_rel_rpys[i*3+2]);
          } else {
              bs.antenna_relative_rpy = Eigen::Vector3d::Zero();
          }
          bs.name = "antenna_" + std::to_string(i);
          Eigen::Vector3d ant_rpy = bs.rpy + bs.antenna_relative_rpy;
          bs.rotmat = antenna_parser_.rpy_to_rotmat(ant_rpy.x(), ant_rpy.y(), ant_rpy.z());
          rx_nodes_.push_back(bs);
      }
  } else {
      RxConfig bs;
      auto pos = this->get_parameter("rx_position").as_double_array();
      auto off = this->get_parameter("rx_antenna_offset").as_double_array();
      auto rpy = this->get_parameter("rx_rpy").as_double_array();
      auto rel = this->get_parameter("rx_antenna_relative_rpy").as_double_array();
      if(pos.size()>=3) bs.position = Eigen::Vector3d(pos[0], pos[1], pos[2]); else bs.position = Eigen::Vector3d::Zero();
      if(off.size()>=3) bs.antenna_offset = Eigen::Vector3d(off[0], off[1], off[2]); else bs.antenna_offset = Eigen::Vector3d::Zero();
      if(rpy.size()>=3) bs.rpy = Eigen::Vector3d(rpy[0], rpy[1], rpy[2]); else bs.rpy = Eigen::Vector3d::Zero();
      if(rel.size()>=3) bs.antenna_relative_rpy = Eigen::Vector3d(rel[0], rel[1], rel[2]); else bs.antenna_relative_rpy = Eigen::Vector3d::Zero();
      bs.name = "antenna_0";
      Eigen::Vector3d ant_rpy = bs.rpy + bs.antenna_relative_rpy;
      bs.rotmat = antenna_parser_.rpy_to_rotmat(ant_rpy.x(), ant_rpy.y(), ant_rpy.z());
      rx_nodes_.push_back(bs);
  }

  auto tx_off = this->get_parameter("tx_antenna_offset").as_double_array();
  if (tx_off.size() >= 3) {
      tx_antenna_offset_ = Eigen::Vector3d(tx_off[0], tx_off[1], tx_off[2]);
  } else {
      tx_antenna_offset_ = Eigen::Vector3d::Zero();
  }

  // Load from YAML
  try {
      config_file_path_ = this->get_parameter("config_file_path").as_string();
      YAML::Node config = YAML::LoadFile(config_file_path_);
      if (config["simulation"]) {
          logging_level_ = config["simulation"]["logging_level"].as<int>(1);
      }
      if (config["vehicles"]) {
          for (auto v : config["vehicles"]) {
              bool is_match = false;
              if (v["name"] && v["name"].as<std::string>() == vehicle_name_) is_match = true;
              else if (v["antennas"]) {
                  for (auto a : v["antennas"]) {
                      if (a["name"] && a["name"].as<std::string>() == vehicle_name_) {
                          is_match = true; break;
                      }
                  }
              }
              if (is_match) {
                  if (v["waypoints"]) {
                      for (auto wp : v["waypoints"]) {
                          std::vector<double> w = wp.as<std::vector<double>>();
                          waypoints_.push_back(w);
                      }
                  }
                  if (v["antennas"]) {
                      for (auto a : v["antennas"]) {
                          VehicleAntenna va;
                          va.name = a["name"].as<std::string>();
                          std::vector<double> off = a["offset"].as<std::vector<double>>();
                          va.offset = Eigen::Vector3d(off[0], off[1], off[2]);
                          if (a["relative_rpy"]) {
                              std::vector<double> rpy = a["relative_rpy"].as<std::vector<double>>();
                              va.relative_rpy = Eigen::Vector3d(rpy[0], rpy[1], rpy[2]);
                          } else {
                              va.relative_rpy = Eigen::Vector3d::Zero();
                          }
                          vehicle_antennas_.push_back(va);
                      }
                  }
                  scheduling_policy_ = config["link_controller_node"]["ros__parameters"]["scheduling_policy"].as<std::string>("sequential");
                  break;
              }
          }
      }
  } catch(...) {
      RCLCPP_WARN(this->get_logger(), "Failed to load yaml config");
  }

  rclcpp::QoS sensor_qos(10);
  sensor_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", sensor_qos, std::bind(&CommsSimulatorNode::imu_callback, this, std::placeholders::_1));

  if (logging_trigger_ == "on_movement") {
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, sensor_qos, std::bind(&CommsSimulatorNode::on_cmd_vel, this, std::placeholders::_1));
  } else if (logging_trigger_ == "on_topic") {
    logging_start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      logging_start_topic_, 10, std::bind(&CommsSimulatorNode::on_logging_start_topic, this, std::placeholders::_1));
  }

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, sensor_qos, std::bind(&CommsSimulatorNode::odom_callback, this, std::placeholders::_1));

  mission_complete_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    mission_complete_topic_, 10, std::bind(&CommsSimulatorNode::on_mission_complete, this, std::placeholders::_1));

  if (!vehicle_name_.empty()) {
      link_grant_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/" + vehicle_name_ + "/link_grant", 10, std::bind(&CommsSimulatorNode::on_link_grant, this, std::placeholders::_1));
      link_request_pub_ = this->create_publisher<std_msgs::msg::Float64>("/" + vehicle_name_ + "/link_request", 10);
      quality_pub_ = this->create_publisher<comms_sim_msgs::msg::CommsQuality>("/" + vehicle_name_ + "/comms/quality", 1000);
  } else {
      quality_pub_ = this->create_publisher<comms_sim_msgs::msg::CommsQuality>("/comms/quality", 1000);
  }

  // 通信ハードウェアの内部クロックを模擬:
  // sampling_rate_ Hz の固定周期で通信品質を計算するタイマー
  double timer_period_s = 1.0 / sampling_rate_;
  RCLCPP_INFO(this->get_logger(), "Timer-driven comms calc enabled: %.1f Hz (%.4f ms period)",
      sampling_rate_, timer_period_s * 1000.0);

  // Ready シグナルの発行（全ノードの起動同期用）
  if (!vehicle_name_.empty()) {
      auto ready_qos = rclcpp::QoS(1).reliable().transient_local();
      ready_pub_ = this->create_publisher<std_msgs::msg::Bool>(
          "/" + vehicle_name_ + "/ready", ready_qos);
          
      auto all_ready_qos = rclcpp::QoS(1).reliable().durability(rclcpp::DurabilityPolicy::Volatile);
      all_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
          "/sim/all_ready", all_ready_qos, std::bind(&CommsSimulatorNode::on_all_ready, this, std::placeholders::_1));

      ready_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(500),
          [this]() {
              if (all_nodes_ready_) {
                  ready_timer_->cancel();
                  return;
              }
              auto ready_msg = std_msgs::msg::Bool();
              ready_msg.data = true;
              ready_pub_->publish(ready_msg);
          });
  }
}

CommsSimulatorNode::~CommsSimulatorNode() {
  save_log_to_csv();
}

Eigen::Vector3d CommsSimulatorNode::quat_to_rpy(double x, double y, double z, double w) {
  double sinr_cosp = 2.0 * (w * x + y * z);
  double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  double roll = std::atan2(sinr_cosp, cosr_cosp);

  double sinp = 2.0 * (w * y - z * x);
  double pitch;
  if (std::abs(sinp) >= 1.0)
    pitch = std::copysign(M_PI / 2.0, sinp);
  else
    pitch = std::asin(sinp);

  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);

  return Eigen::Vector3d(roll, pitch, yaw);
}

void CommsSimulatorNode::on_link_grant(const std_msgs::msg::Bool::SharedPtr msg) {
    has_link_grant_ = msg->data;
}

void CommsSimulatorNode::on_all_ready(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data && !all_nodes_ready_) {
        all_nodes_ready_ = true;
        RCLCPP_INFO(this->get_logger(), "Received all_ready signal.");
    }
}

void CommsSimulatorNode::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (!logging_ready_) {
    if (std::abs(msg->linear.x) > 1e-3 || std::abs(msg->linear.y) > 1e-3) {
      logging_ready_ = true;
    }
  }
}

void CommsSimulatorNode::on_logging_start_topic(const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data && !logging_ready_) {
    logging_ready_ = true;
  }
}

void CommsSimulatorNode::on_rx_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  (void)msg;
}

void CommsSimulatorNode::on_mission_complete(const std_msgs::msg::Bool::SharedPtr msg) {
  (void)msg;
}

bool CommsSimulatorNode::get_current_segment_pose(double& best_px, double& best_py, double& best_yaw) {
    if (waypoints_.empty() || !tx_local_position_.has_value() || tx_spawn_pose_.size() < 3) return false;
    double ux = tx_local_position_.value().x();
    double uy = tx_local_position_.value().y();

    best_px = ux; best_py = uy; best_yaw = 0.0;
    double min_dist_sq = 1e9;
    double ax = tx_spawn_pose_[0], ay = tx_spawn_pose_[1];

    for (const auto& wp : waypoints_) {
        double bx = wp[0], by = wp[1];
        double vx = bx - ax, vy = by - ay;
        double v_len_sq = vx*vx + vy*vy;

        if (v_len_sq < 1e-6) {
            double dist_sq = (ux - bx)*(ux - bx) + (uy - by)*(uy - by);
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                best_px = bx; best_py = by;
                best_yaw = std::atan2(vy, vx);
            }
        } else {
            double t = ((ux - ax)*vx + (uy - ay)*vy) / v_len_sq;
            t = std::max(0.0, std::min(1.0, t));
            double px = ax + t*vx, py = ay + t*vy;
            double dist_sq = (ux - px)*(ux - px) + (uy - py)*(uy - py);
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                best_px = px; best_py = py;
                best_yaw = std::atan2(vy, vx);
            }
        }
        ax = bx; ay = by;
    }
    return true;
}

void CommsSimulatorNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  tx_orientation_ = quat_to_rpy(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

  if (vehicle_name_.find("shinkansen") != std::string::npos) {
      double px, py, yaw;
      if (get_current_segment_pose(px, py, yaw)) {
          tx_orientation_ = Eigen::Vector3d(0.0, 0.0, yaw);
      }
  }
}

void CommsSimulatorNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  Eigen::Vector3d odom_pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  if (!odom_offset_set_) {
      if (tx_spawn_pose_.size() >= 3) {
          Eigen::Vector3d spawn(tx_spawn_pose_[0], tx_spawn_pose_[1], tx_spawn_pose_[2]);
          if ((odom_pos - spawn).norm() > 10.0) return;
          odom_offset_ = spawn - odom_pos;
      }
      odom_offset_set_ = true;
      last_odom_pos_ = odom_pos;
  } else {
      // Validate that position did not jump suddenly (DDS residual messages from previous runs)
      if (last_odom_pos_.has_value()) {
          double jump_dist = (odom_pos - last_odom_pos_.value()).norm();
          if (jump_dist > 20.0) {
              RCLCPP_WARN(this->get_logger(), 
                  "Ignored odom message due to large position jump (%.2f m) - likely stale DDS message", jump_dist);
              return;
          }
      }
      last_odom_pos_ = odom_pos;
  }
  tx_local_position_ = odom_pos + odom_offset_;

  if (vehicle_name_.find("shinkansen") != std::string::npos) {
      double px, py, yaw;
      if (get_current_segment_pose(px, py, yaw)) {
          tx_local_position_.value().x() = px;
          tx_local_position_.value().y() = py;
      }
  }

  if (!logging_ready_) {
      last_odom_time_ = msg->header.stamp;
      last_tx_pos_ = tx_local_position_;
      last_tx_orientation_ = tx_orientation_;
      return;
  }

  rclcpp::Time current_odom_time = msg->header.stamp;
  double current_time_sec = current_odom_time.seconds();

  if (!last_odom_time_.has_value()) {
      last_odom_time_ = current_odom_time;
      last_tx_pos_ = tx_local_position_;
      last_tx_orientation_ = tx_orientation_;
      
      double T_s = 1.0 / sampling_rate_;
      next_grid_time_ = std::ceil(current_time_sec / T_s) * T_s;
      step_count_ = 0;
      return;
  }

  double last_time_sec = rclcpp::Time(last_odom_time_.value()).seconds();
  if (current_time_sec < last_time_sec - 1.0) {
      RCLCPP_INFO(this->get_logger(), "Simulation time rewound (%.3f -> %.3f). Resetting grid and step count.", 
          last_time_sec, current_time_sec);
      last_odom_time_ = current_odom_time;
      last_tx_pos_ = tx_local_position_;
      last_tx_orientation_ = tx_orientation_;
      
      double T_s = 1.0 / sampling_rate_;
      next_grid_time_ = std::ceil(current_time_sec / T_s) * T_s;
      step_count_ = 0;
      last_odom_pos_ = std::nullopt;
      odom_offset_set_ = false;
      return;
  } else if (current_time_sec <= last_time_sec) {
      return;
  }

  double T_s = 1.0 / sampling_rate_;

  if (!next_grid_time_.has_value() || next_grid_time_.value() < last_time_sec) {
      next_grid_time_ = std::ceil(last_time_sec / T_s) * T_s;
      step_count_ = 0;
  }

  Eigen::Vector3d start_pos = last_tx_pos_.value_or(tx_local_position_.value());
  Eigen::Vector3d end_pos = tx_local_position_.value();
  Eigen::Vector3d start_ori = last_tx_orientation_.value_or(tx_orientation_.value());
  Eigen::Vector3d end_ori = tx_orientation_.value();

  double dt_odom = current_time_sec - last_time_sec;

  while (next_grid_time_.value() <= current_time_sec + 1e-9) {
      double t = next_grid_time_.value();
      double alpha = (t - last_time_sec) / dt_odom;
      alpha = std::max(0.0, std::min(1.0, alpha));

      Eigen::Vector3d pos = start_pos + alpha * (end_pos - start_pos);
      Eigen::Vector3d ori = start_ori + alpha * (end_ori - start_ori);

      calculate_and_publish(pos, ori, T_s, t);

      next_grid_time_ = next_grid_time_.value() + T_s;
  }

  last_odom_time_ = current_odom_time;
  last_tx_pos_ = tx_local_position_;
  last_tx_orientation_ = tx_orientation_;
}

bool CommsSimulatorNode::update_link_state(double rssi, double current_time, bool has_link_grant) {
  if (!has_link_grant || !comm_active_) {
      if (link_state_ != LinkState::DISCONNECTED) {
          link_state_ = LinkState::DISCONNECTED;
          link_establishment_start_time_.reset();
          establishment_step_count_ = 0;
      }
      return false;
  }

  // リンク確立に必要なステップ数（無線規格のフレーム単位を模擬）
  int required_steps = static_cast<int>(
      std::ceil(link_establishment_time_ * sampling_rate_));

  if (link_state_ == LinkState::DISCONNECTED) {
      if (rssi > rssi_threshold_) {
          link_state_ = LinkState::ESTABLISHING;
          link_establishment_start_time_ = current_time;
          establishment_step_count_ = 1;
          if (required_steps <= 1) {
              link_state_ = LinkState::CONNECTED;
              establishment_step_count_ = 0;
              return true;
          }
      }
      return false;
  } else if (link_state_ == LinkState::ESTABLISHING) {
      if (rssi <= rssi_threshold_) {
          link_state_ = LinkState::DISCONNECTED;
          link_establishment_start_time_.reset();
          establishment_step_count_ = 0;
          return false;
      }
      establishment_step_count_++;
      if (establishment_step_count_ >= required_steps) {
          link_state_ = LinkState::CONNECTED;
          establishment_step_count_ = 0;
          return true;
      }
      return false;
  } else if (link_state_ == LinkState::CONNECTED) {
      if (rssi <= rssi_threshold_) {
          link_state_ = LinkState::DISCONNECTED;
          link_establishment_start_time_.reset();
          establishment_step_count_ = 0;
          return false;
      }
      return true;
  }
  return false;
}

void CommsSimulatorNode::calculate_and_publish(const Eigen::Vector3d& pos, const Eigen::Vector3d& ori, double dt_step, double current_time) {
    if (rx_nodes_.empty()) return;

    Eigen::Vector3d tx_ant_rpy = ori + tx_antenna_relative_rpy_;
    Eigen::Matrix3d rx_rotmat = antenna_parser_.rpy_to_rotmat(tx_ant_rpy.x(), tx_ant_rpy.y(), tx_ant_rpy.z());
    Eigen::Matrix3d vehicle_rotmat = antenna_parser_.rpy_to_rotmat(ori.x(), ori.y(), ori.z());
    Eigen::Vector3d tx_antenna_pos = pos + vehicle_rotmat * tx_antenna_offset_;

    CommsMetrics best_metrics;
    int best_bs_idx = 0;
    double best_tx_total = 0.0;
    double best_rx_total = 0.0;
    Eigen::Vector3d best_bs_pos = rx_nodes_[0].position + rx_nodes_[0].antenna_offset;

    for (size_t i = 0; i < rx_nodes_.size(); ++i) {
        auto& bs = rx_nodes_[i];
        Eigen::Vector3d bs_antenna_pos = bs.position + bs.antenna_offset;
        Eigen::Vector3d bs_ant_rpy = bs.rpy + bs.antenna_relative_rpy;

        auto gain_res = antenna_parser_.get_tx_rx_gains(
            tx_antenna_pos, tx_ant_rpy, bs_antenna_pos, bs_ant_rpy,
            &rx_rotmat, &bs.rotmat);
        double tx_tot = gain_res.tx_total;
        double rx_tot = gain_res.rx_total;

        CommsMetrics metrics = comms_calculator_->calculate_all(tx_antenna_pos, bs_antenna_pos, tx_tot + rx_tot, true);
        if (i == 0 || metrics.rssi > best_metrics.rssi) {
            best_metrics = metrics;
            best_bs_idx = static_cast<int>(i);
            best_tx_total = tx_tot;
            best_rx_total = rx_tot;
            best_bs_pos = bs_antenna_pos;
        }
    }

    last_rssi_ = best_metrics.rssi;
    bool local_has_link_grant = has_link_grant_;

    if ((scheduling_policy_ == "feedforward_optimal" || scheduling_policy_ == "rssi_priority" || 
         scheduling_policy_ == "physical_score_priority" || scheduling_policy_ == "geometric_beam_priority" || 
         scheduling_policy_ == "geometric_weighted") && !vehicle_antennas_.empty()) 
    {
        double best_va_rssi_global = -1e9;
        std::string best_ant_name = "";

        for (const auto& va : vehicle_antennas_) {
            bool is_current_va = (va.name == vehicle_name_);
            double best_va_rssi = -1e9;

            if (is_current_va && noise_variance_ == 0.0) {
                best_va_rssi = best_metrics.rssi;
            } else {
                Eigen::Vector3d va_pos_world = pos + vehicle_rotmat * va.offset;
                Eigen::Vector3d va_ant_rpy = ori + va.relative_rpy;
                Eigen::Matrix3d va_rotmat = antenna_parser_.rpy_to_rotmat(va_ant_rpy.x(), va_ant_rpy.y(), va_ant_rpy.z());
                for (const auto& bs : rx_nodes_) {
                    Eigen::Vector3d bs_antenna_pos = bs.position + bs.antenna_offset;
                    auto gain_res2 = antenna_parser_.get_tx_rx_gains(
                        va_pos_world, va_ant_rpy, bs_antenna_pos, bs.rpy + bs.antenna_relative_rpy,
                        &va_rotmat, &bs.rotmat);
                    double tx_tot = gain_res2.tx_total;
                    double rx_tot = gain_res2.rx_total;
                    
                    CommsMetrics metrics_va = comms_calculator_->calculate_all(va_pos_world, bs_antenna_pos, tx_tot + rx_tot, false);
                    if (metrics_va.rssi > best_va_rssi) best_va_rssi = metrics_va.rssi;
                }
            }
            if (best_va_rssi > best_va_rssi_global) {
                best_va_rssi_global = best_va_rssi;
                best_ant_name = va.name;
            }
        }
        local_has_link_grant = (vehicle_name_ == best_ant_name);
    }
    LinkState old_link_state = link_state_;
    bool link_ready = update_link_state(best_metrics.rssi, current_time, local_has_link_grant);
    bool state_changed = (old_link_state != link_state_);

    double actual_throughput = 0.0;
    if (link_ready && comm_active_ && logging_ready_) {
        actual_throughput = best_metrics.throughput;
        if (actual_throughput > 0) {
            total_data_transmitted_ += actual_throughput * 1000.0 / 8.0 * dt_step;
            if (comm_data_limit_mb_ > 0 && total_data_transmitted_ >= comm_data_limit_mb_) {
                comm_active_ = false;
            }
        }
    }

    if (logging_ready_) {
        if (node_start_time_ < 0.0) {
            node_start_time_ = current_time;
        }
        if (vehicle_start_time_ < 0.0) {
            vehicle_start_time_ = current_time;
        }

        if (logging_level_ >= 3) {
            bool should_log = true;
            if (logging_level_ == 3 && link_state_ != LinkState::CONNECTED) {
                should_log = false;
            }

            if (should_log) {
                CommsLogRecord record;
                record.time_s = current_time - node_start_time_;
                record.vehicle_time_s = current_time - vehicle_start_time_;
                record.vehicle_name = vehicle_name_;
                record.has_link_grant = local_has_link_grant;
                record.distance_m = best_metrics.distance;
                record.rssi_dBm = best_metrics.rssi;
                record.throughput_Gbps = actual_throughput;
                record.total_data_MB = total_data_transmitted_;
                record.path_loss_dB = best_metrics.path_loss;
                record.e_gain_dB = best_tx_total;
                record.h_gain_dB = best_rx_total;
                record.comm_active = comm_active_;
                record.tx_x_m = pos.x();
                record.tx_y_m = pos.y();
                record.tx_z_m = pos.z();
                record.bs_x_m = rx_nodes_[best_bs_idx].position.x();
                record.bs_y_m = rx_nodes_[best_bs_idx].position.y();
                record.bs_z_m = best_bs_pos.z();
                record.link_state = link_state_ == LinkState::CONNECTED ? "CONNECTED" : (link_state_ == LinkState::ESTABLISHING ? "ESTABLISHING" : "DISCONNECTED");
                
                log_records_.push_back(record);
            }
        }
    }

    bool should_publish = false;
    int publish_interval_steps = std::max(1, static_cast<int>(std::round(sampling_rate_ / publish_rate_)));
    if (step_count_ == 0 || state_changed || (step_count_ % publish_interval_steps == 0)) 
    {
        should_publish = true;
        last_publish_time_ = current_time;
    }
    step_count_++;

    if (should_publish) {
        if (link_request_pub_) {
            std_msgs::msg::Float64 req_msg;
            req_msg.data = comm_active_ ? best_metrics.rssi : -1e9;
            link_request_pub_->publish(req_msg);
        }

        if (quality_pub_) {
            comms_sim_msgs::msg::CommsQuality msg;
            msg.header.stamp.sec = static_cast<int32_t>(current_time);
            msg.header.stamp.nanosec = static_cast<uint32_t>(std::round((current_time - msg.header.stamp.sec) * 1e9));
            msg.header.frame_id = "world";
            msg.distance = best_metrics.distance;
            msg.rssi = best_metrics.rssi;
            msg.throughput = actual_throughput;
            msg.total_data_transmitted = total_data_transmitted_;
            msg.tx_x = pos.x();
            msg.tx_y = pos.y();
            msg.tx_z = pos.z();
            msg.rx_x = rx_nodes_[best_bs_idx].position.x();
            msg.rx_y = rx_nodes_[best_bs_idx].position.y();
            msg.rx_z = best_bs_pos.z();
            msg.antenna_gain_e_plane = best_tx_total;
            msg.antenna_gain_h_plane = best_rx_total;
            msg.path_loss = best_metrics.path_loss;
            msg.comm_active = comm_active_;
            msg.link_state = link_state_ == LinkState::CONNECTED ? "CONNECTED" : (link_state_ == LinkState::ESTABLISHING ? "ESTABLISHING" : "DISCONNECTED");
            quality_pub_->publish(msg);
        }
    }
}

std::string CommsSimulatorNode::get_output_csv_path() {
    std::string summary_filename = "sweep_summary.csv";
    std::string output_subdir = "";
    std::string output_dir = "/workspace/sim_results/";
    double y_pos = 0.0;
    double angle = 0.0;

    try {
        YAML::Node config = YAML::LoadFile(config_file_path_);
        if (config["simulation"]) {
            summary_filename = config["simulation"]["summary_filename"].as<std::string>("sweep_summary.csv");
            output_subdir = config["simulation"]["output_subdir"].as<std::string>("");
            output_dir = config["simulation"]["output_dir"].as<std::string>("/workspace/sim_results/");
        }

        if (config["spawn_entities"]) {
            for (auto const& node : config["spawn_entities"]) {
                std::string key = node.first.as<std::string>();
                if (key.find("antenna") != std::string::npos || key.find("Antenna") != std::string::npos) {
                    auto antenna_cfg = node.second;
                    if (antenna_cfg["pose"]) {
                        auto pose = antenna_cfg["pose"].as<std::vector<double>>();
                        if (pose.size() >= 6) {
                            y_pos = pose[1];
                            double entity_yaw = pose[5];
                            double entity_yaw_deg = entity_yaw * 180.0 / M_PI;
                            double raw_yaw = 0.0;
                            if (antenna_cfg["antenna_relative_rpy"]) {
                                auto rel_rpy = antenna_cfg["antenna_relative_rpy"].as<std::vector<double>>();
                                if (rel_rpy.size() >= 3) {
                                    raw_yaw = rel_rpy[2];
                                }
                            }
                            double raw_yaw_deg = raw_yaw * 180.0 / M_PI;
                            angle = std::round(std::fmod(raw_yaw_deg + entity_yaw_deg + 180.0, 360.0) * 10.0) / 10.0;
                            if (angle < 0) angle += 360.0;
                        }
                    }
                    break;
                }
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "get_output_csv_path: Failed to parse YAML config: %s", e.what());
    }

    // Yポーズと角度の文字列表現
    std::ostringstream y_ss;
    y_ss << std::round(y_pos * 100.0) / 100.0;
    std::string y_str = y_ss.str();

    std::ostringstream a_ss;
    a_ss << angle;
    std::string angle_str = a_ss.str();

    // パス構築
    std::string run_dir = "";
    std::regex sweep_regex("sweep_summary_(\\d{8}_\\d{6})_run(\\d+)");
    std::smatch sweep_match;

    // パスの末尾のスラッシュ等調整
    if (!output_dir.empty() && output_dir.back() == '/') {
        output_dir.pop_back();
    }

    if (std::regex_search(summary_filename, sweep_match, sweep_regex)) {
        std::string sweep_timestamp = sweep_match[1].str();
        int run_idx = std::stoi(sweep_match[2].str());
        char run_name_buf[128];
        std::snprintf(run_name_buf, sizeof(run_name_buf), "run_%03d_y%s_a%s", run_idx, y_str.c_str(), angle_str.c_str());
        run_dir = output_dir + "/sweep_" + sweep_timestamp + "/runs/" + run_name_buf;
    } else {
        int run_idx = -1;
        std::regex run_regex("run(\\d+)");
        std::smatch run_match;
        if (std::regex_search(summary_filename, run_match, run_regex)) {
            run_idx = std::stoi(run_match[1].str());
        }

        char run_name_buf[128];
        if (run_idx >= 0) {
            std::snprintf(run_name_buf, sizeof(run_name_buf), "run_%03d_y%s_a%s", run_idx, y_str.c_str(), angle_str.c_str());
        } else {
            std::snprintf(run_name_buf, sizeof(run_name_buf), "run_fallback");
        }

        if (!output_subdir.empty()) {
            run_dir = output_dir + "/" + output_subdir + "/runs/" + run_name_buf;
        } else {
            run_dir = output_dir + "/" + run_name_buf;
        }
    }

    std::string suffix = (logging_level_ == 3) ? "_connected.csv" : "_full.csv";
    return run_dir + "/comms/" + vehicle_name_ + suffix;
}

void CommsSimulatorNode::save_log_to_csv() {
    if (logging_level_ < 3 || log_records_.empty()) {
        return;
    }

    std::string csv_path = get_output_csv_path();
    RCLCPP_INFO(this->get_logger(), "Saving %zu log records to CSV: %s", log_records_.size(), csv_path.c_str());

    try {
        std::filesystem::path p(csv_path);
        std::filesystem::create_directories(p.parent_path());

        std::ofstream file(csv_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file for writing: %s", csv_path.c_str());
            return;
        }

        // CSV ヘッダー
        if (logging_level_ == 3) {
            file << "time_s,vehicle_name,distance_m,rssi_dBm,throughput_Gbps,total_data_MB,"
                 << "path_loss_dB,e_gain_dB,h_gain_dB,tx_x_m,tx_y_m,tx_z_m,bs_x_m,bs_y_m,bs_z_m\n";
        } else {
            file << "time_s,vehicle_time_s,vehicle_name,has_link_grant,distance_m,rssi_dBm,"
                 << "throughput_Gbps,total_data_MB,path_loss_dB,e_gain_dB,h_gain_dB,comm_active,"
                 << "tx_x_m,tx_y_m,tx_z_m,bs_x_m,bs_y_m,bs_z_m,link_state\n";
        }

        // データ書き出し
        file << std::fixed << std::setprecision(6);
        for (const auto& rec : log_records_) {
            if (logging_level_ == 3) {
                file << rec.time_s << ","
                     << rec.vehicle_name << ","
                     << rec.distance_m << ","
                     << rec.rssi_dBm << ","
                     << rec.throughput_Gbps << ","
                     << rec.total_data_MB << ","
                     << rec.path_loss_dB << ","
                     << rec.e_gain_dB << ","
                     << rec.h_gain_dB << ","
                     << rec.tx_x_m << ","
                     << rec.tx_y_m << ","
                     << rec.tx_z_m << ","
                     << rec.bs_x_m << ","
                     << rec.bs_y_m << ","
                     << rec.bs_z_m << "\n";
            } else {
                file << rec.time_s << ","
                     << rec.vehicle_time_s << ","
                     << rec.vehicle_name << ","
                     << (rec.has_link_grant ? "True" : "False") << ","
                     << rec.distance_m << ","
                     << rec.rssi_dBm << ","
                     << rec.throughput_Gbps << ","
                     << rec.total_data_MB << ","
                     << rec.path_loss_dB << ","
                     << rec.e_gain_dB << ","
                     << rec.h_gain_dB << ","
                     << (rec.comm_active ? "True" : "False") << ","
                     << rec.tx_x_m << ","
                     << rec.tx_y_m << ","
                     << rec.tx_z_m << ","
                     << rec.bs_x_m << ","
                     << rec.bs_y_m << ","
                     << rec.bs_z_m << ","
                     << rec.link_state << "\n";
            }
        }
        file.close();

        try {
            std::filesystem::permissions(csv_path, std::filesystem::perms::all);
        } catch(...) {}

        RCLCPP_INFO(this->get_logger(), "Successfully saved logs to %s", csv_path.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception while saving logs to CSV: %s", e.what());
    }
}

} // namespace comms_sim

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<comms_sim::CommsSimulatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
