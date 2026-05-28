#include "comms_sim_pkg/comms_node.hpp"
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <cmath>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace comms_sim {

CommsSimulatorNode::CommsSimulatorNode()
: Node("comms_simulator_node"),
  odom_offset_set_(false),
  odom_offset_(0, 0, 0),
  total_data_transmitted_(0.0),
  comm_active_(true),
  link_state_(LinkState::DISCONNECTED),
  has_link_grant_(false),
  last_rssi_(0.0)
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
  this->declare_parameter("base_station_position", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("base_station_antenna_offset", std::vector<double>{0.0, 0.0, 10.5});
  this->declare_parameter("ugv_antenna_offset", std::vector<double>{0.0, 0.0, 1.3});
  this->declare_parameter("mcs_table_path", "");
  this->declare_parameter("link_establishment_time_ms", 2.0);
  this->declare_parameter("mission_complete_topic", "/mission_complete");
  this->declare_parameter("odom_topic", "/odom");
  this->declare_parameter("ugv_spawn_pose", std::vector<double>{-20.0, 0.0, 0.0});
  this->declare_parameter("ugv_antenna_relative_rpy", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("base_station_pose_topic", "/base_station/pose");
  this->declare_parameter("base_station_rpy", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("base_station_antenna_relative_rpy", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("base_station_positions", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("base_station_antenna_offsets", std::vector<double>{0.0, 0.0, 10.5});
  this->declare_parameter("base_station_rpys", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("base_station_antenna_relative_rpys", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("logging_start_trigger", "on_movement");
  this->declare_parameter("logging_start_topic", "/logging/start");
  this->declare_parameter("max_antenna_attenuation", 30.0);
  this->declare_parameter("vehicle_name", "");
  this->declare_parameter("cmd_vel_topic", "/cmd_vel");

  sampling_rate_ = this->get_parameter("sampling_rate").as_double();
  noise_variance_ = this->get_parameter("noise_variance").as_double();
  e_plane_path_ = this->get_parameter("e_plane_path").as_string();
  h_plane_path_ = this->get_parameter("h_plane_path").as_string();
  comm_data_limit_mb_ = this->get_parameter("comm_data_limit_mb").as_double();
  tx_power_ = this->get_parameter("tx_power").as_double();
  mcs_table_path_ = this->get_parameter("mcs_table_path").as_string();
  link_establishment_time_ = this->get_parameter("link_establishment_time_ms").as_double() / 1000.0;
  mission_complete_topic_ = this->get_parameter("mission_complete_topic").as_string();
  odom_topic_ = this->get_parameter("odom_topic").as_string();
  ugv_spawn_pose_ = this->get_parameter("ugv_spawn_pose").as_double_array();
  
  auto rel = this->get_parameter("ugv_antenna_relative_rpy").as_double_array();
  if (rel.size() >= 3) {
      ugv_antenna_relative_rpy_ = Eigen::Vector3d(rel[0], rel[1], rel[2]);
  } else {
      ugv_antenna_relative_rpy_ = Eigen::Vector3d::Zero();
  }

  base_station_pose_topic_ = this->get_parameter("base_station_pose_topic").as_string();
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

  auto bs_positions = this->get_parameter("base_station_positions").as_double_array();
  auto bs_offsets = this->get_parameter("base_station_antenna_offsets").as_double_array();
  auto bs_rpys = this->get_parameter("base_station_rpys").as_double_array();
  auto bs_rel_rpys = this->get_parameter("base_station_antenna_relative_rpys").as_double_array();

  if (bs_positions.size() >= 3 && bs_positions.size() % 3 == 0) {
      size_t n_bs = bs_positions.size() / 3;
      for (size_t i = 0; i < n_bs; ++i) {
          BaseStationConfig bs;
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
          base_stations_.push_back(bs);
      }
  } else {
      BaseStationConfig bs;
      auto pos = this->get_parameter("base_station_position").as_double_array();
      auto off = this->get_parameter("base_station_antenna_offset").as_double_array();
      auto rpy = this->get_parameter("base_station_rpy").as_double_array();
      auto rel = this->get_parameter("base_station_antenna_relative_rpy").as_double_array();
      if(pos.size()>=3) bs.position = Eigen::Vector3d(pos[0], pos[1], pos[2]); else bs.position = Eigen::Vector3d::Zero();
      if(off.size()>=3) bs.antenna_offset = Eigen::Vector3d(off[0], off[1], off[2]); else bs.antenna_offset = Eigen::Vector3d::Zero();
      if(rpy.size()>=3) bs.rpy = Eigen::Vector3d(rpy[0], rpy[1], rpy[2]); else bs.rpy = Eigen::Vector3d::Zero();
      if(rel.size()>=3) bs.antenna_relative_rpy = Eigen::Vector3d(rel[0], rel[1], rel[2]); else bs.antenna_relative_rpy = Eigen::Vector3d::Zero();
      bs.name = "antenna_0";
      Eigen::Vector3d ant_rpy = bs.rpy + bs.antenna_relative_rpy;
      bs.rotmat = antenna_parser_.rpy_to_rotmat(ant_rpy.x(), ant_rpy.y(), ant_rpy.z());
      base_stations_.push_back(bs);
  }

  auto ugv_off = this->get_parameter("ugv_antenna_offset").as_double_array();
  if (ugv_off.size() >= 3) {
      ugv_antenna_offset_ = Eigen::Vector3d(ugv_off[0], ugv_off[1], ugv_off[2]);
  } else {
      ugv_antenna_offset_ = Eigen::Vector3d::Zero();
  }

  // Load from YAML
  try {
      std::string yaml_path = "/workspace/src/comms_sim_pkg/config/sim_params.yaml";
      YAML::Node config = YAML::LoadFile(yaml_path);
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
      quality_pub_ = this->create_publisher<comms_sim_msgs::msg::CommsQuality>("/" + vehicle_name_ + "/comms/quality", 10);
  } else {
      quality_pub_ = this->create_publisher<comms_sim_msgs::msg::CommsQuality>("/comms/quality", 10);
  }
}

CommsSimulatorNode::~CommsSimulatorNode() {}

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

void CommsSimulatorNode::on_base_station_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  (void)msg;
}

void CommsSimulatorNode::on_mission_complete(const std_msgs::msg::Bool::SharedPtr msg) {
  (void)msg;
}

bool CommsSimulatorNode::get_current_segment_pose(double& best_px, double& best_py, double& best_yaw) {
    if (waypoints_.empty() || !ugv_local_position_.has_value() || ugv_spawn_pose_.size() < 3) return false;
    double ux = ugv_local_position_.value().x();
    double uy = ugv_local_position_.value().y();

    best_px = ux; best_py = uy; best_yaw = 0.0;
    double min_dist_sq = 1e9;
    double ax = ugv_spawn_pose_[0], ay = ugv_spawn_pose_[1];

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
  ugv_orientation_ = quat_to_rpy(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

  if (vehicle_name_.find("shinkansen") != std::string::npos) {
      double px, py, yaw;
      if (get_current_segment_pose(px, py, yaw)) {
          ugv_orientation_ = Eigen::Vector3d(0.0, 0.0, yaw);
      }
  }
}

void CommsSimulatorNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  Eigen::Vector3d odom_pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  if (!odom_offset_set_) {
      if (ugv_spawn_pose_.size() >= 3) {
          Eigen::Vector3d spawn(ugv_spawn_pose_[0], ugv_spawn_pose_[1], ugv_spawn_pose_[2]);
          if ((odom_pos - spawn).norm() > 10.0) return;
          odom_offset_ = spawn - odom_pos;
      }
      odom_offset_set_ = true;
  }
  ugv_local_position_ = odom_pos + odom_offset_;

  if (vehicle_name_.find("shinkansen") != std::string::npos) {
      double px, py, yaw;
      if (get_current_segment_pose(px, py, yaw)) {
          ugv_local_position_.value().x() = px;
          ugv_local_position_.value().y() = py;
      }
  }

  if (!simulation_start_time_.has_value()) {
      simulation_start_time_ = this->get_clock()->now().nanoseconds() / 1e9;
  }

  calculate_and_publish();
}

bool CommsSimulatorNode::update_link_state(double rssi, double current_time, bool has_link_grant) {
  if (!has_link_grant || !comm_active_) {
      if (link_state_ != LinkState::DISCONNECTED) {
          link_state_ = LinkState::DISCONNECTED;
          link_establishment_start_time_.reset();
      }
      return false;
  }

  if (link_state_ == LinkState::DISCONNECTED) {
      if (rssi > rssi_threshold_) {
          link_state_ = LinkState::ESTABLISHING;
          link_establishment_start_time_ = current_time;
          if (link_establishment_time_ <= (1.0 / sampling_rate_)) {
              link_state_ = LinkState::CONNECTED;
              return true;
          }
      }
      return false;
  } else if (link_state_ == LinkState::ESTABLISHING) {
      if (rssi <= rssi_threshold_) {
          link_state_ = LinkState::DISCONNECTED;
          link_establishment_start_time_.reset();
          return false;
      }
      if (current_time - link_establishment_start_time_.value() >= link_establishment_time_) {
          link_state_ = LinkState::CONNECTED;
          return true;
      }
      return false;
  } else if (link_state_ == LinkState::CONNECTED) {
      if (rssi <= rssi_threshold_) {
          link_state_ = LinkState::DISCONNECTED;
          link_establishment_start_time_.reset();
          return false;
      }
      return true;
  }
  return false;
}

void CommsSimulatorNode::calculate_and_publish() {
    if (base_stations_.empty() || !ugv_local_position_.has_value() || !ugv_orientation_.has_value()) return;

    double current_time = this->get_clock()->now().nanoseconds() / 1e9;
    double dt_target = 1.0 / sampling_rate_;
    
    int num_steps = 1;
    double dt_actual = dt_target;

    if (!last_calc_sim_time_.has_value() || !last_ugv_pos_.has_value() || !last_ugv_orientation_.has_value()) {
        last_calc_sim_time_ = current_time;
        last_ugv_pos_ = ugv_local_position_.value();
        last_ugv_orientation_ = ugv_orientation_.value();
        num_steps = 1;
        dt_actual = dt_target;
    } else {
        double elapsed = current_time - last_calc_sim_time_.value();
        if (elapsed <= 0.0) {
            return;
        }
        num_steps = static_cast<int>(std::round(elapsed / dt_target));
        if (num_steps <= 0) {
            num_steps = 1;
        } else if (num_steps > 5000) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Large simulation time gap detected (%f s). Capping sub-steps to 5000.", elapsed);
            num_steps = 5000;
        }
        dt_actual = elapsed / num_steps;
    }

    CommsMetrics best_metrics;
    Eigen::Vector3d best_bs_pos = base_stations_[0].position + base_stations_[0].antenna_offset;
    double best_tx_total = 0, best_rx_total = 0;
    int best_bs_idx = 0;
    double actual_throughput = 0.0;

    for (int step = 1; step <= num_steps; ++step) {
        double frac = static_cast<double>(step) / num_steps;
        double t_sub = last_calc_sim_time_.value() + step * dt_actual;

        // Interpolate position and orientation (already projected)
        Eigen::Vector3d pos_sub = last_ugv_pos_.value() + frac * (ugv_local_position_.value() - last_ugv_pos_.value());
        Eigen::Vector3d ori_sub = last_ugv_orientation_.value() + frac * (ugv_orientation_.value() - last_ugv_orientation_.value());

        Eigen::Vector3d ugv_ant_rpy = ori_sub + ugv_antenna_relative_rpy_;
        Eigen::Matrix3d rx_rotmat = antenna_parser_.rpy_to_rotmat(ugv_ant_rpy.x(), ugv_ant_rpy.y(), ugv_ant_rpy.z());
        Eigen::Vector3d ugv_antenna_pos = pos_sub + rx_rotmat * ugv_antenna_offset_;

        CommsMetrics step_best_metrics;
        int step_best_bs_idx = 0;
        double step_best_tx_total = 0.0;
        double step_best_rx_total = 0.0;
        Eigen::Vector3d step_best_bs_pos = Eigen::Vector3d::Zero();

        for (size_t i = 0; i < base_stations_.size(); ++i) {
            auto& bs = base_stations_[i];
            Eigen::Vector3d bs_antenna_pos = bs.position + bs.antenna_offset;
            Eigen::Vector3d bs_ant_rpy = bs.rpy + bs.antenna_relative_rpy;

            auto gain_res = antenna_parser_.get_tx_rx_gains(
                bs_antenna_pos, bs_ant_rpy, ugv_antenna_pos, ugv_ant_rpy,
                &bs.rotmat, &rx_rotmat);
            double tx_tot = gain_res.tx_total;
            double rx_tot = gain_res.rx_total;

            CommsMetrics metrics = comms_calculator_->calculate_all(ugv_antenna_pos, bs_antenna_pos, tx_tot + rx_tot, true);
            if (i == 0 || metrics.rssi > step_best_metrics.rssi) {
                step_best_metrics = metrics;
                step_best_bs_idx = i;
                step_best_tx_total = tx_tot;
                step_best_rx_total = rx_tot;
                step_best_bs_pos = bs_antenna_pos;
            }
        }

        last_rssi_ = step_best_metrics.rssi;
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
                    best_va_rssi = step_best_metrics.rssi;
                } else {
                    Eigen::Vector3d va_pos_world = pos_sub + va.offset;
                    for (const auto& bs : base_stations_) {
                        Eigen::Vector3d bs_antenna_pos = bs.position + bs.antenna_offset;
                        auto gain_res2 = antenna_parser_.get_tx_rx_gains(
                            bs_antenna_pos, bs.rpy + bs.antenna_relative_rpy, va_pos_world, ugv_ant_rpy,
                            &bs.rotmat, &rx_rotmat);
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

        bool link_ready = update_link_state(step_best_metrics.rssi, t_sub, local_has_link_grant);
        actual_throughput = 0.0;
        if (link_ready && comm_active_ && logging_ready_) {
            actual_throughput = step_best_metrics.throughput;
            if (actual_throughput > 0) {
                total_data_transmitted_ += actual_throughput * 1000.0 / 8.0 * dt_actual;
                if (comm_data_limit_mb_ > 0 && total_data_transmitted_ >= comm_data_limit_mb_) {
                    comm_active_ = false;
                }
            }
        }

        if (step == num_steps) {
            best_metrics = step_best_metrics;
            best_bs_idx = step_best_bs_idx;
            best_tx_total = step_best_tx_total;
            best_rx_total = step_best_rx_total;
            best_bs_pos = step_best_bs_pos;
        }
    }

    last_calc_sim_time_ = current_time;
    last_ugv_pos_ = ugv_local_position_;
    last_ugv_orientation_ = ugv_orientation_;

    if (link_request_pub_) {
        std_msgs::msg::Float64 req_msg;
        req_msg.data = comm_active_ ? best_metrics.rssi : -1e9;
        link_request_pub_->publish(req_msg);
    }

    if (quality_pub_) {
        comms_sim_msgs::msg::CommsQuality msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "world";
        msg.distance = best_metrics.distance;
        msg.rssi = best_metrics.rssi;
        msg.throughput = actual_throughput;
        msg.total_data_transmitted = total_data_transmitted_;
        msg.ugv_x = ugv_local_position_.value().x();
        msg.ugv_y = ugv_local_position_.value().y();
        msg.ugv_z = ugv_local_position_.value().z();
        msg.base_station_x = base_stations_[best_bs_idx].position.x();
        msg.base_station_y = base_stations_[best_bs_idx].position.y();
        msg.base_station_z = best_bs_pos.z();
        msg.antenna_gain_e_plane = best_tx_total;
        msg.antenna_gain_h_plane = best_rx_total;
        msg.path_loss = best_metrics.path_loss;
        msg.comm_active = comm_active_;
        msg.link_state = link_state_ == LinkState::CONNECTED ? "CONNECTED" : (link_state_ == LinkState::ESTABLISHING ? "ESTABLISHING" : "DISCONNECTED");
        quality_pub_->publish(msg);
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
