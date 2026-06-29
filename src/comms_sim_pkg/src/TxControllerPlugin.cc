#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>
#include <vector>
#include <algorithm>
#include <numeric>
#include <string>
#include <sstream>
#include <cmath>
#include <iostream>
#include <chrono>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <regex>
#include <iomanip>
#include "comms_sim_pkg/comms_calculator.hpp"
#include "comms_sim_pkg/antenna_pattern_parser.hpp"

namespace tx_controller
{
    struct Waypoint {
        double x, y, z, v;
    };

    inline std::string resolve_path(const std::string& raw_path) {
        if (raw_path.empty()) return raw_path;
        if (std::filesystem::exists(raw_path)) return raw_path;
        
        std::string path = raw_path;
        size_t pos = path.find("/workspace/config/");
        if (pos != std::string::npos) {
            std::string resolved = path;
            resolved.replace(pos, 18, "/workspace/src/comms_sim_pkg/config/");
            if (std::filesystem::exists(resolved)) {
                return resolved;
            }
            resolved = path;
            resolved.replace(pos, 18, "/workspace/install/comms_sim_pkg/share/comms_sim_pkg/config/");
            if (std::filesystem::exists(resolved)) {
                return resolved;
            }
        }
        
        pos = path.find("workspace/config/");
        if (pos != std::string::npos) {
            std::string resolved = path;
            resolved.replace(pos, 17, "workspace/src/comms_sim_pkg/config/");
            if (std::filesystem::exists(resolved)) {
                return resolved;
            }
            resolved = path;
            resolved.replace(pos, 17, "workspace/install/comms_sim_pkg/share/comms_sim_pkg/config/");
            if (std::filesystem::exists(resolved)) {
                return resolved;
            }
        }
        
        return raw_path;
    }

    struct TrajectorySample {
        Eigen::Vector3d pos;
        double yaw;
    };

    inline std::vector<TrajectorySample> sample_trajectory(const std::vector<Eigen::Vector3d>& polyline_points, double resolution) {
        std::vector<TrajectorySample> samples;
        if (polyline_points.empty()) return samples;
        
        double dist_to_next = 0.0;
        for (size_t i = 0; i < polyline_points.size() - 1; ++i) {
            Eigen::Vector3d pt_a = polyline_points[i];
            Eigen::Vector3d pt_b = polyline_points[i+1];
            
            Eigen::Vector3d dir_vec = pt_b - pt_a;
            double length = dir_vec.norm();
            if (length < 1e-6) continue;
            
            Eigen::Vector3d unit_dir = dir_vec / length;
            double yaw = std::atan2(dir_vec.y(), dir_vec.x());
            
            double t = dist_to_next;
            while (t <= length) {
                Eigen::Vector3d pt = pt_a + t * unit_dir;
                samples.push_back({pt, yaw});
                t += resolution;
            }
            dist_to_next = t - length;
        }
        return samples;
    }

    struct LutPair {
        std::string tx_antenna;  // 車載アンテナ名
        std::string rx_antenna;  // 基地局アンテナ名
        double rssi = -999.0;
    };

    struct LutEntry {
        double x, y, z;
        std::vector<LutPair> pairs;  // N個のペア (RSSIの高い順)
    };

    struct AntennaInfo {
        std::string name;
        Eigen::Vector3d offset;
        Eigen::Vector3d relative_rpy;
        
        double total_data_transmitted = 0.0;
        bool comm_active = true;
        std::string link_state = "DISCONNECTED";
        double link_establishment_start_time = -1.0;
        int establishment_step_count = 0;
        double last_rssi = -999.0;
        int assigned_bs_idx = -1;  // マルチペア: 割り当てられた基地局インデックス (-1 = 未割当)

        struct LogRecord {
            double time_s;
            double vehicle_time_s;
            std::string vehicle_name;
            bool has_link_grant;
            double distance_m;
            double rssi_dBm;
            double throughput_Gbps;
            double total_data_MB;
            double path_loss_dB;
            double e_gain_dB;
            double h_gain_dB;
            bool comm_active;
            double tx_x_m, tx_y_m, tx_z_m;
            double bs_x_m, bs_y_m, bs_z_m;
            std::string link_state;
            bool in_main_lobe;
            double off_boresight_e_deg;
            double off_boresight_h_deg;
        };
        std::vector<LogRecord> log_records;
    };

    struct BaseStationInfo {
        std::string name;
        Eigen::Vector3d position;
        Eigen::Vector3d antenna_offset;
        Eigen::Vector3d rpy;
        Eigen::Vector3d antenna_relative_rpy;
        Eigen::Matrix3d rotmat;
    };

    struct EventRecord {
        double time_s;
        std::string event_type;
        std::string active_antenna;
        std::string prev_antenna;
        std::string details;
    };

    struct ControlRecord {
        double time_s;
        double vehicle_x;
        double vehicle_y;
        double vehicle_yaw;
        std::string nominal_antenna;
        std::string active_antenna;
        bool switching_active;
        double last_switch_time_s;
    };

    class TxControllerPlugin :
        public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
    {
    public:
        TxControllerPlugin() = default;
        ~TxControllerPlugin() override {
            if (this->comms_initialized && !this->logs_saved) {
                this->SaveLogs();
                this->logs_saved = true;
            }
        }

        void Configure(const gz::sim::Entity &_entity,
                       const std::shared_ptr<const sdf::Element> &_sdf,
                       gz::sim::EntityComponentManager &_ecm,
                       gz::sim::EventManager &_eventMgr) override
        {
            (void)_eventMgr;
            this->model = gz::sim::Model(_entity);
            if (!this->model.Valid(_ecm)) {
                gzerr << "[TxControllerPlugin] Plugin should be attached to a model entity." << std::endl;
                return;
            }

            // Read params
            if (_sdf->HasElement("waypoints")) {
                std::string wp_str = _sdf->Get<std::string>("waypoints");
                
                // Replace commas with spaces
                for (char &c : wp_str) {
                    if (c == ',') c = ' ';
                }
                
                std::stringstream ss(wp_str);
                double val;
                std::vector<double> vals;
                while (ss >> val) {
                    vals.push_back(val);
                }
                for (size_t i = 0; i + 3 < vals.size(); i += 4) {
                    this->waypoints.push_back({vals[i], vals[i+1], vals[i+2], vals[i+3]});
                }
            }

            if (_sdf->HasElement("waypoint_tolerance"))
                this->waypoint_tolerance = _sdf->Get<double>("waypoint_tolerance");
            if (_sdf->HasElement("heading_gain"))
                this->heading_gain = _sdf->Get<double>("heading_gain");
            if (_sdf->HasElement("max_acceleration"))
                this->max_acceleration = _sdf->Get<double>("max_acceleration");
            if (_sdf->HasElement("max_angular_velocity"))
                this->max_angular_velocity = _sdf->Get<double>("max_angular_velocity");
            if (_sdf->HasElement("is_shinkansen"))
                this->is_shinkansen = _sdf->Get<bool>("is_shinkansen");

            // Topic names
            std::string mission_topic = "/mission_complete";
            if (_sdf->HasElement("mission_complete_topic"))
                mission_topic = _sdf->Get<std::string>("mission_complete_topic");
            
            std::string ready_pub_topic = "/tx/ready";
            if (_sdf->HasElement("ready_pub_topic"))
                ready_pub_topic = _sdf->Get<std::string>("ready_pub_topic");
                
            std::string all_ready_topic = "/sim/all_ready";
            if (_sdf->HasElement("all_ready_topic"))
                all_ready_topic = _sdf->Get<std::string>("all_ready_topic");

            if (_sdf->HasElement("config_file_path"))
                this->config_file_path = _sdf->Get<std::string>("config_file_path");

            // Setup publishers
            this->mission_complete_pub = this->node.Advertise<gz::msgs::Boolean>(mission_topic);
            this->ready_pub = this->node.Advertise<gz::msgs::Boolean>(ready_pub_topic);
            this->mission_progress_pub = this->node.Advertise<gz::msgs::Double>("/" + this->model.Name(_ecm) + "/mission_progress");

            this->total_path_distance = 0.0;
            for (size_t i = 1; i < this->waypoints.size(); ++i) {
                double dx = this->waypoints[i].x - this->waypoints[i-1].x;
                double dy = this->waypoints[i].y - this->waypoints[i-1].y;
                this->total_path_distance += std::sqrt(dx*dx + dy*dy);
            }

            // Setup subscriber
            this->node.Subscribe(all_ready_topic, &TxControllerPlugin::OnAllReady, this);

            // Initialize Comms Sim
            if (!this->config_file_path.empty()) {
                try {
                    this->InitializeComms(this->config_file_path, _ecm);
                } catch (const std::exception& e) {
                    gzerr << "[TxControllerPlugin] Exception during comms init: " << e.what() << std::endl;
                }
            }

            gzmsg << "[TxControllerPlugin] Initialized on model [" << this->model.Name(_ecm) 
                  << "] with " << this->waypoints.size() << " waypoints." << std::endl;
        }

        void OnAllReady(const gz::msgs::Boolean &_msg) {
            if (_msg.data() && !this->all_ready) {
                this->all_ready = true;
                gzmsg << "[TxControllerPlugin] Received all_ready signal on model [" << this->model_name << "]." << std::endl;
            }
        }

        void PreUpdate(const gz::sim::UpdateInfo &_info,
                       gz::sim::EntityComponentManager &_ecm) override
        {
            if (_info.paused) return;
            
            if (this->model_name.empty()) {
                this->model_name = this->model.Name(_ecm);
            }

            // Update comms simulation and scheduling
            this->UpdateComms(_info, _ecm);

            // 1. Wait for all_ready
            if (!this->all_ready) {
                // Periodically publish our ready state
                if (_info.simTime.count() - this->last_ready_pub_time > 500000000) { // 0.5s
                    gz::msgs::Boolean msg;
                    msg.set_data(true);
                    this->ready_pub.Publish(msg);
                    this->last_ready_pub_time = _info.simTime.count();
                }
                return;
            }

            // 2. Mission complete logic
            if (this->mission_complete) {
                if (this->comms_initialized && !this->logs_saved) {
                    this->SaveLogs();
                    this->logs_saved = true;
                }
                // Periodically re-publish mission complete
                if (_info.simTime.count() - this->last_complete_pub_time > 1000000000) { // 1.0s
                    gz::msgs::Boolean msg;
                    msg.set_data(true);
                    this->mission_complete_pub.Publish(msg);
                    this->last_complete_pub_time = _info.simTime.count();
                }
                this->SetVelocity(_ecm, 0.0, 0.0);
                return;
            }

            if (this->current_waypoint_idx >= this->waypoints.size() || this->waypoints.empty()) {
                this->mission_complete = true;
                if (this->comms_initialized && !this->logs_saved) {
                    this->SaveLogs();
                    this->logs_saved = true;
                }
                this->SetVelocity(_ecm, 0.0, 0.0);
                gzmsg << "[TxControllerPlugin] Mission Complete for [" << this->model_name << "]!" << std::endl;
                
                gz::msgs::Boolean msg;
                msg.set_data(true);
                this->mission_complete_pub.Publish(msg);
                this->last_complete_pub_time = _info.simTime.count();
                return;
            }

            // 3. Get current pose
            auto poseComp = _ecm.Component<gz::sim::components::Pose>(this->model.Entity());
            if (!poseComp) return;
            gz::math::Pose3d pose = poseComp->Data();

            double current_x = pose.Pos().X();
            double current_y = pose.Pos().Y();
            double current_yaw = pose.Rot().Yaw();

            // 4. Compute tracking
            auto target = this->waypoints[this->current_waypoint_idx];
            double dx = target.x - current_x;
            double dy = target.y - current_y;
            double distance = std::sqrt(dx*dx + dy*dy);
            double target_heading = std::atan2(dy, dx);

            // 5. Check waypoint arrival
            if (distance < this->waypoint_tolerance) {
                gzmsg << "[TxControllerPlugin] Reached Waypoint " << this->current_waypoint_idx 
                      << " for [" << this->model_name << "]" << std::endl;
                this->current_waypoint_idx++;
                return;
            }

            // 6. Compute velocity
            double heading_error = target_heading - current_yaw;
            while (heading_error > M_PI) heading_error -= 2 * M_PI;
            while (heading_error < -M_PI) heading_error += 2 * M_PI;

            if (this->is_shinkansen) heading_error = 0.0;

            double w = this->heading_gain * heading_error;
            w = std::max(-this->max_angular_velocity, std::min(this->max_angular_velocity, w));
            if (this->is_shinkansen) w = 0.0;

            double turn_factor = 1.0 - std::min(1.0, std::abs(heading_error) / (M_PI / 2.0));
            double target_v = target.v * std::max(0.3, turn_factor);

            double dt = std::chrono::duration<double>(_info.dt).count();
            if (dt > 0) {
                double accel_step = this->max_acceleration * dt;
                if (target_v > this->current_v) {
                    this->current_v = std::min(this->current_v + accel_step, target_v);
                } else {
                    this->current_v = std::max(this->current_v - accel_step, target_v);
                }
            }

            this->SetVelocity(_ecm, this->current_v, w);

            this->PublishProgress(_info, current_x, current_y);
        }

        void PublishProgress(const gz::sim::UpdateInfo &_info, double current_x, double current_y) {
            double covered_dist = 0.0;
            for (size_t i = 1; i < this->current_waypoint_idx && i < this->waypoints.size(); ++i) {
                double dx = this->waypoints[i].x - this->waypoints[i-1].x;
                double dy = this->waypoints[i].y - this->waypoints[i-1].y;
                covered_dist += std::sqrt(dx*dx + dy*dy);
            }
            if (this->current_waypoint_idx < this->waypoints.size() && this->current_waypoint_idx > 0) {
                double px = this->waypoints[this->current_waypoint_idx - 1].x;
                double py = this->waypoints[this->current_waypoint_idx - 1].y;
                double cx = this->waypoints[this->current_waypoint_idx].x;
                double cy = this->waypoints[this->current_waypoint_idx].y;
                double segment_dist = std::sqrt((cx-px)*(cx-px) + (cy-py)*(cy-py));
                
                double dist_to_target = std::sqrt((cx - current_x)*(cx - current_x) + (cy - current_y)*(cy - current_y));
                double segment_progress = segment_dist - dist_to_target;
                if (segment_progress < 0) segment_progress = 0;
                covered_dist += segment_progress;
            }
            
            double progress = 1.0;
            if (this->total_path_distance > 0.001) {
                progress = std::min(1.0, std::max(0.0, covered_dist / this->total_path_distance));
            } else if (!this->mission_complete && this->waypoints.size() <= 1) {
                progress = 0.0; 
            }
            if (this->mission_complete) progress = 1.0;
            
            if (_info.simTime.count() - this->last_progress_pub_time > 500000000) { // 0.5s
                gz::msgs::Double msg;
                msg.set_data(progress);
                this->mission_progress_pub.Publish(msg);
                this->last_progress_pub_time = _info.simTime.count();
            }
        }

    private:
        void InitializeComms(const std::string &config_path, gz::sim::EntityComponentManager &_ecm) {
            YAML::Node config = YAML::LoadFile(config_path);
            
            // 1. Read simulation configurations
            bool direct_params_found = false;
            if (config["simulation"]) {
                this->logging_level = config["simulation"]["logging_level"].as<int>(1);
                this->config_summary_filename = config["simulation"]["summary_filename"].as<std::string>("sweep_summary.csv");
                this->config_output_subdir = config["simulation"]["output_subdir"].as<std::string>("");
                this->config_output_dir = config["simulation"]["output_dir"].as<std::string>("/workspace/sim_results/");
                if (config["simulation"]["y_position"] && config["simulation"]["angle_deg"]) {
                    this->config_y_pos = config["simulation"]["y_position"].as<double>();
                    this->config_angle = config["simulation"]["angle_deg"].as<double>();
                    direct_params_found = true;
                }
            }
            
            if (!direct_params_found && config["spawn_entities"]) {
                for (auto const& node : config["spawn_entities"]) {
                    std::string key = node.first.as<std::string>();
                    auto antenna_cfg = node.second;
                    if (antenna_cfg["pose"]) {
                        auto pose = antenna_cfg["pose"].as<std::vector<double>>();
                        if (pose.size() >= 6) {
                            this->config_y_pos = pose[1];
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
                            double angle_val = std::round(std::fmod(raw_yaw_deg + entity_yaw_deg + 180.0, 360.0) * 10.0) / 10.0;
                            if (angle_val < 0) angle_val += 360.0;
                            this->config_angle = angle_val;
                        }
                    }
                    break; // Just use the first one as fallback
                }
            }
            
            // 2. Read comms parameters
            auto comms_params = config["comms_simulator_node"]["ros__parameters"];
            double comm_data_limit = comms_params["comm_data_limit_mb"].as<double>(-1.0);
            this->comm_data_limit_mb = comm_data_limit;
            this->link_establishment_time_ms = comms_params["link_establishment_time_ms"].as<double>(2.0);
            double tx_power = comms_params["tx_power"].as<double>(-7.0);
            double noise_variance = comms_params["noise_variance"].as<double>(0.0);
            std::string mcs_table_path = resolve_path(comms_params["mcs_table_path"].as<std::string>(""));
            
            double max_antenna_attenuation = comms_params["max_antenna_attenuation"].as<double>(30.0);
            double mainlobe_angle_margin_deg = comms_params["mainlobe_angle_margin_deg"].as<double>(5.0);
            double mainlobe_e_half_angle_deg = comms_params["mainlobe_e_half_angle_deg"].as<double>(-1.0);
            double mainlobe_h_half_angle_deg = comms_params["mainlobe_h_half_angle_deg"].as<double>(-1.0);

            // 3. Setup propagation model
            auto pl = comms_params["path_loss"];
            double c = pl["c"].as<double>(299792458.0);
            double frequency = pl["frequency"].as<double>(6e10);
            double exponent = pl["exponent"].as<double>(2.0);
            double d0 = pl["d0"].as<double>(1.0);
            double pl_d0 = pl["pl_d0"].as<double>(-1.0);

            auto propagation_model = std::make_unique<comms_sim::LogDistancePathLossModel>(frequency, c, exponent, d0, pl_d0);
            this->comms_calculator = std::make_unique<comms_sim::CommsCalculator>(
                std::move(propagation_model), tx_power, noise_variance, mcs_table_path);

            this->antenna_parser = std::make_unique<comms_sim::AntennaPatternParser>(
                max_antenna_attenuation, mainlobe_angle_margin_deg, mainlobe_e_half_angle_deg, mainlobe_h_half_angle_deg);
            
            std::string e_plane_path = resolve_path(comms_params["e_plane_path"].as<std::string>(""));
            std::string h_plane_path = resolve_path(comms_params["h_plane_path"].as<std::string>(""));
            if (!e_plane_path.empty()) this->antenna_parser->load_e_plane(e_plane_path);
            if (!h_plane_path.empty()) this->antenna_parser->load_h_plane(h_plane_path);

            // 4. Link controller parameters
            auto link_ctrl_params = config["link_controller_node"]["ros__parameters"];
            this->scheduling_policy = link_ctrl_params["scheduling_policy"].as<std::string>("sequential");
            this->filter_main_lobe = link_ctrl_params["filter_main_lobe"].as<bool>(true);
            this->beam_gain_threshold = link_ctrl_params["beam_gain_threshold"].as<double>(5.0);
            this->min_hold_time_s = link_ctrl_params["min_hold_time_s"].as<double>(1.0);
            this->switch_margin_db = link_ctrl_params["switch_margin_db"].as<double>(2.0);
            this->proactive_grace_period_s = link_ctrl_params["proactive_grace_period_s"].as<double>(0.5);
            this->proactive_handover_score_threshold = link_ctrl_params["proactive_handover_score_threshold"].as<double>(-80.0);
            this->time_slot_duration_s = link_ctrl_params["time_slot_duration_s"].as<double>(10.0);
            this->weight_distance = link_ctrl_params["weight_distance"].as<double>(0.7);
            this->weight_angle = link_ctrl_params["weight_angle"].as<double>(0.3);
            this->heatmap_resolution_m = link_ctrl_params["heatmap_resolution_m"].as<double>(0.2);
            this->ff_max_pairs = link_ctrl_params["ff_max_pairs"].as<int>(-1);

            // 5. Load antennas configuration for this vehicle model
            std::string current_model_name = this->model.Name(_ecm);
            
            if (config["vehicles"]) {
                for (auto const &v : config["vehicles"]) {
                    std::string v_name = v["name"].as<std::string>();
                    if (v_name == current_model_name) {
                        if (v["antennas"]) {
                            for (auto const &ant_cfg : v["antennas"]) {
                                AntennaInfo ant;
                                ant.name = ant_cfg["name"].as<std::string>();
                                auto offset_vec = ant_cfg["offset"].as<std::vector<double>>();
                                ant.offset = Eigen::Vector3d(offset_vec[0], offset_vec[1], offset_vec[2]);
                                auto rpy_vec = ant_cfg["relative_rpy"].as<std::vector<double>>();
                                ant.relative_rpy = Eigen::Vector3d(rpy_vec[0], rpy_vec[1], rpy_vec[2]);
                                ant.total_data_transmitted = 0.0;
                                ant.comm_active = true;
                                ant.link_state = "DISCONNECTED";
                                ant.link_establishment_start_time = -1.0;
                                ant.establishment_step_count = 0;
                                ant.last_rssi = -999.0;
                                this->vehicle_antennas.push_back(ant);
                            }
                        }
                    }
                }
            }

            // 6. Load base station configs from spawn_entities
            if (config["spawn_entities"]) {
                for (auto const &node : config["spawn_entities"]) {
                    std::string key = node.first.as<std::string>();
                    auto bs_cfg = node.second;
                    BaseStationInfo bs;
                    bs.name = key;
                    auto pose_vec = bs_cfg["pose"].as<std::vector<double>>();
                    bs.position = Eigen::Vector3d(pose_vec[0], pose_vec[1], pose_vec[2]);
                    bs.rpy = Eigen::Vector3d(pose_vec[3], pose_vec[4], pose_vec[5]);
                    
                    auto offset_vec = bs_cfg["antenna_offset"].as<std::vector<double>>();
                    bs.antenna_offset = Eigen::Vector3d(offset_vec[0], offset_vec[1], offset_vec[2]);
                    
                    auto rel_rpy_vec = bs_cfg["antenna_relative_rpy"].as<std::vector<double>>();
                    bs.antenna_relative_rpy = Eigen::Vector3d(rel_rpy_vec[0], rel_rpy_vec[1], rel_rpy_vec[2]);
                    this->base_stations_cfg.push_back(bs);
                }
            }

            // Determine center_antenna_name
            if (!this->vehicle_antennas.empty()) {
                this->center_antenna_name = this->vehicle_antennas[0].name;
                for (const auto &ant : this->vehicle_antennas) {
                    if (ant.name.find("mid") != std::string::npos || ant.name.find("Mid") != std::string::npos) {
                        this->center_antenna_name = ant.name;
                        break;
                    }
                }
            }

            // Precalculate LUT for feedforward_optimal
            if (this->scheduling_policy == "feedforward_optimal") {
                this->PrecalculateLUT();
            }

            this->comms_initialized = true;
            gzmsg << "[TxControllerPlugin] Loaded " << this->vehicle_antennas.size() << " antennas for UGV." << std::endl;
            gzmsg << "[TxControllerPlugin] Comms simulator initialized successfully!" << std::endl;
        }

        void PrecalculateLUT() {
            std::vector<Eigen::Vector3d> polyline_points;
            for (const auto &wp : this->waypoints) {
                polyline_points.push_back(Eigen::Vector3d(wp.x, wp.y, wp.z));
            }

            auto samples = sample_trajectory(polyline_points, this->heatmap_resolution_m);
            if (samples.empty()) return;

            this->lut.clear();

            size_t num_tx = this->vehicle_antennas.size();
            size_t num_rx = this->base_stations_cfg.size();
            size_t num_pairs = std::min(num_tx, num_rx);
            if (this->ff_max_pairs > 0) {
                num_pairs = std::min(num_pairs, static_cast<size_t>(this->ff_max_pairs));
            }

            for (const auto &sample : samples) {
                Eigen::Matrix3d vehicle_rotmat = this->antenna_parser->rpy_to_rotmat(0.0, 0.0, sample.yaw);

                // Step 1: 全 (TX, RX) 組み合わせのRSSIを計算
                std::vector<std::vector<double>> rssi_matrix(num_tx, std::vector<double>(num_rx, -999.0));

                for (size_t tx_idx = 0; tx_idx < num_tx; ++tx_idx) {
                    const auto &ant = this->vehicle_antennas[tx_idx];
                    Eigen::Vector3d ant_pos_world = sample.pos + vehicle_rotmat * ant.offset;
                    Eigen::Vector3d ant_rpy = Eigen::Vector3d(0.0, 0.0, sample.yaw) + ant.relative_rpy;
                    Eigen::Matrix3d ant_rotmat = this->antenna_parser->rpy_to_rotmat(ant_rpy.x(), ant_rpy.y(), ant_rpy.z());

                    for (size_t rx_idx = 0; rx_idx < num_rx; ++rx_idx) {
                        const auto &bs = this->base_stations_cfg[rx_idx];
                        Eigen::Vector3d bs_antenna_pos = bs.position + bs.antenna_offset;
                        Eigen::Vector3d bs_ant_rpy = bs.rpy + bs.antenna_relative_rpy;
                        Eigen::Matrix3d bs_rotmat = this->antenna_parser->rpy_to_rotmat(bs_ant_rpy.x(), bs_ant_rpy.y(), bs_ant_rpy.z());

                        auto gain_res = this->antenna_parser->get_tx_rx_gains(
                            ant_pos_world, ant_rpy, bs_antenna_pos, bs_ant_rpy,
                            &ant_rotmat, &bs_rotmat);

                        auto [el, az] = this->antenna_parser->calculate_antenna_frame_angles(
                            ant_pos_world, bs_antenna_pos, ant_rpy, &ant_rotmat);
                        double off_e_deg = std::abs(el * 180.0 / M_PI);
                        double off_h_deg = std::abs(az * 180.0 / M_PI);
                        bool tx_in = this->antenna_parser->is_in_main_lobe(off_e_deg, off_h_deg);

                        auto [el_rx, az_rx] = this->antenna_parser->calculate_antenna_frame_angles(
                            bs_antenna_pos, ant_pos_world, bs_ant_rpy, &bs_rotmat);
                        double off_e_rx_deg = std::abs(el_rx * 180.0 / M_PI);
                        double off_h_rx_deg = std::abs(az_rx * 180.0 / M_PI);
                        bool rx_in = this->antenna_parser->is_in_main_lobe(off_e_rx_deg, off_h_rx_deg);

                        bool in_main = tx_in && rx_in;

                        if (!this->filter_main_lobe || in_main) {
                            auto metrics = this->comms_calculator->calculate_all(ant_pos_world, bs_antenna_pos, gain_res.tx_total + gain_res.rx_total, false);
                            rssi_matrix[tx_idx][rx_idx] = metrics.rssi;
                        }
                    }
                }

                // Step 2: 全列挙で最適N個ペアを決定
                std::vector<int> rx_indices(num_rx);
                std::iota(rx_indices.begin(), rx_indices.end(), 0);

                double best_total_rssi = -1e9;
                std::vector<LutPair> best_pairs;

                do {
                    double total_rssi = 0.0;
                    std::vector<LutPair> candidate_pairs;
                    for (size_t p = 0; p < num_pairs; ++p) {
                        size_t tx_idx = p;
                        size_t rx_idx = static_cast<size_t>(rx_indices[p]);
                        double rssi = rssi_matrix[tx_idx][rx_idx];
                        total_rssi += rssi;
                        candidate_pairs.push_back({
                            this->vehicle_antennas[tx_idx].name,
                            this->base_stations_cfg[rx_idx].name,
                            rssi
                        });
                    }
                    if (total_rssi > best_total_rssi) {
                        best_total_rssi = total_rssi;
                        best_pairs = candidate_pairs;
                    }
                } while (std::next_permutation(rx_indices.begin(), rx_indices.end()));

                // RSSIの高い順にソート
                std::sort(best_pairs.begin(), best_pairs.end(), [](const LutPair &a, const LutPair &b) {
                    return a.rssi > b.rssi;
                });

                LutEntry entry;
                entry.x = sample.pos.x();
                entry.y = sample.pos.y();
                entry.z = sample.pos.z();
                entry.pairs = best_pairs;
                this->lut.push_back(entry);
            }

            gzmsg << "[TxControllerPlugin] Precalculated multi-pair feedforward LUT with " << this->lut.size() 
                  << " entries, " << num_pairs << " pairs per entry." << std::endl;
        }

        void UpdateComms(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) {
            if (!this->comms_initialized) return;

            // Wait until base stations are located in Gazebo
            if (!this->base_stations_located) {
                this->base_stations.clear();
                _ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
                    [&](const gz::sim::Entity &_ent,
                        const gz::sim::components::Model *,
                        const gz::sim::components::Name *_name) -> bool
                    {
                        std::string name = _name->Data();
                        for (auto &bs : this->base_stations_cfg) {
                            if (bs.name == name) {
                                BaseStationInfo bs_info = bs;
                                auto poseComp = _ecm.Component<gz::sim::components::Pose>(_ent);
                                if (poseComp) {
                                    gz::math::Pose3d p = poseComp->Data();
                                    bs_info.position = Eigen::Vector3d(p.Pos().X(), p.Pos().Y(), p.Pos().Z());
                                    bs_info.rpy = Eigen::Vector3d(p.Rot().Roll(), p.Rot().Pitch(), p.Rot().Yaw());
                                    Eigen::Vector3d ant_rpy_updated = bs_info.rpy + bs_info.antenna_relative_rpy;
                                    bs_info.rotmat = this->antenna_parser->rpy_to_rotmat(ant_rpy_updated.x(), ant_rpy_updated.y(), ant_rpy_updated.z());
                                }
                                this->base_stations.push_back(bs_info);
                            }
                        }
                        return true;
                    });
                if (this->base_stations.size() >= this->base_stations_cfg.size()) {
                    this->base_stations_located = true;
                    gzmsg << "[TxControllerPlugin] Located all base stations in Gazebo!" << std::endl;
                } else {
                    return;
                }
            }

            double current_time = std::chrono::duration<double>(_info.simTime).count();
            double dt = std::chrono::duration<double>(_info.dt).count();
            if (dt <= 0.0) return;

            if (this->start_time_sec < 0.0) {
                this->start_time_sec = current_time;
            }

            // 1. Get vehicle pose
            auto poseComp = _ecm.Component<gz::sim::components::Pose>(this->model.Entity());
            if (!poseComp) return;
            gz::math::Pose3d vehicle_pose = poseComp->Data();
            Eigen::Vector3d pos(vehicle_pose.Pos().X(), vehicle_pose.Pos().Y(), vehicle_pose.Pos().Z());
            Eigen::Vector3d ori(vehicle_pose.Rot().Roll(), vehicle_pose.Rot().Pitch(), vehicle_pose.Rot().Yaw());
            Eigen::Matrix3d vehicle_rotmat = this->antenna_parser->rpy_to_rotmat(ori.x(), ori.y(), ori.z());

            // 2. Compute comms metrics for all antennas
            struct AntennaMetrics {
                double best_rssi = -999.0;
                double distance = 0.0;
                double path_loss = 0.0;
                double throughput = 0.0;
                double e_gain = 0.0;
                double h_gain = 0.0;
                Eigen::Vector3d bs_pos;
                bool in_main_lobe = false;
                double off_boresight_e = 0.0;
                double off_boresight_h = 0.0;
                int bs_idx = -1;
            };
            std::vector<AntennaMetrics> ant_metrics_list(this->vehicle_antennas.size());
            std::vector<std::vector<AntennaMetrics>> all_ant_bs_metrics(this->vehicle_antennas.size());

            for (size_t a_idx = 0; a_idx < this->vehicle_antennas.size(); ++a_idx) {
                auto &ant = this->vehicle_antennas[a_idx];
                Eigen::Vector3d ant_pos_world = pos + vehicle_rotmat * ant.offset;
                Eigen::Vector3d ant_rpy = ori + ant.relative_rpy;
                Eigen::Matrix3d ant_rotmat = this->antenna_parser->rpy_to_rotmat(ant_rpy.x(), ant_rpy.y(), ant_rpy.z());

                double best_rssi = -999.0;
                int best_bs_idx = 0;
                std::vector<AntennaMetrics> bs_metrics(this->base_stations.size());
                
                for (size_t bs_idx = 0; bs_idx < this->base_stations.size(); ++bs_idx) {
                    const auto &bs = this->base_stations[bs_idx];
                    Eigen::Vector3d bs_antenna_pos = bs.position + bs.antenna_offset;
                    Eigen::Vector3d bs_ant_rpy = bs.rpy + bs.antenna_relative_rpy;

                    auto gain_res = this->antenna_parser->get_tx_rx_gains(
                        ant_pos_world, ant_rpy, bs_antenna_pos, bs_ant_rpy,
                        &ant_rotmat, &bs.rotmat);

                    double tx_tot = gain_res.tx_total;
                    double rx_tot = gain_res.rx_total;

                    auto metrics = this->comms_calculator->calculate_all(ant_pos_world, bs_antenna_pos, tx_tot + rx_tot, false);
                    
                    auto [el, az] = this->antenna_parser->calculate_antenna_frame_angles(
                        ant_pos_world, bs_antenna_pos, ant_rpy, &ant_rotmat);
                    double off_boresight_e_deg = std::abs(el * 180.0 / M_PI);
                    double off_boresight_h_deg = std::abs(az * 180.0 / M_PI);
                    // TX && RX 双方向でメインローブ内か判定 (PrecalculateLUT と同一ロジック)
                    bool tx_in_main = this->antenna_parser->is_in_main_lobe(off_boresight_e_deg, off_boresight_h_deg);
                    auto [el_rx, az_rx] = this->antenna_parser->calculate_antenna_frame_angles(
                        bs_antenna_pos, ant_pos_world, bs_ant_rpy, &bs.rotmat);
                    double off_e_rx_deg = std::abs(el_rx * 180.0 / M_PI);
                    double off_h_rx_deg = std::abs(az_rx * 180.0 / M_PI);
                    bool rx_in_main = this->antenna_parser->is_in_main_lobe(off_e_rx_deg, off_h_rx_deg);
                    bool in_main = tx_in_main && rx_in_main;

                    AntennaMetrics &m = bs_metrics[bs_idx];
                    m.best_rssi = metrics.rssi;
                    m.distance = metrics.distance;
                    m.path_loss = metrics.path_loss;
                    m.throughput = metrics.throughput;
                    m.e_gain = gain_res.tx_e;
                    m.h_gain = gain_res.tx_h;
                    m.bs_pos = bs_antenna_pos;
                    m.in_main_lobe = in_main;
                    m.off_boresight_e = off_boresight_e_deg;
                    m.off_boresight_h = off_boresight_h_deg;
                    m.bs_idx = static_cast<int>(bs_idx);

                    if (metrics.rssi > best_rssi) {
                        best_rssi = metrics.rssi;
                        best_bs_idx = bs_idx;
                    }
                }
                all_ant_bs_metrics[a_idx] = bs_metrics;
                ant.last_rssi = bs_metrics[best_bs_idx].best_rssi;
                ant_metrics_list[a_idx] = bs_metrics[best_bs_idx];
            }

            // 3. Scheduling Policy
            int new_active_idx = this->active_antenna_idx;
            std::string switch_details = "";

            if (this->scheduling_policy == "feedforward_optimal") {
                if (!this->lut.empty()) {
                    double min_dist_sq = 1e9;
                    int best_lut_idx = this->last_lut_idx;
                    int n_points = this->lut.size();
                    
                    int start_idx = std::max(0, this->last_lut_idx - 100);
                    int end_idx = std::min(n_points, this->last_lut_idx + 100);
                    for (int i = start_idx; i < end_idx; ++i) {
                        const auto &pt = this->lut[i];
                        double dx = pos.x() - pt.x;
                        double dy = pos.y() - pt.y;
                        double dz = pos.z() - pt.z;
                        double dist_sq = dx*dx + dy*dy + dz*dz;
                        if (dist_sq < min_dist_sq) {
                            min_dist_sq = dist_sq;
                            best_lut_idx = i;
                        }
                    }

                    if (best_lut_idx == start_idx || best_lut_idx == end_idx - 1 || this->last_lut_idx == 0) {
                        for (int i = 0; i < n_points; ++i) {
                            const auto &pt = this->lut[i];
                            double dx = pos.x() - pt.x;
                            double dy = pos.y() - pt.y;
                            double dz = pos.z() - pt.z;
                            double dist_sq = dx*dx + dy*dy + dz*dz;
                            if (dist_sq < min_dist_sq) {
                                min_dist_sq = dist_sq;
                                best_lut_idx = i;
                            }
                        }
                    }
                    this->last_lut_idx = best_lut_idx;

                    // マルチペア: LUTの各ペアに基づきアンテナにBSを割り当て
                    const auto &lut_entry = this->lut[best_lut_idx];
                    
                    // まず全アンテナの割り当てをリセット
                    for (auto &ant : this->vehicle_antennas) {
                        ant.assigned_bs_idx = -1;
                    }

                    for (const auto &pair : lut_entry.pairs) {
                        // TX アンテナのインデックスを検索
                        for (size_t ai = 0; ai < this->vehicle_antennas.size(); ++ai) {
                            if (this->vehicle_antennas[ai].name == pair.tx_antenna) {
                                // RX 基地局のインデックスを検索
                                for (size_t bi = 0; bi < this->base_stations.size(); ++bi) {
                                    if (this->base_stations[bi].name == pair.rx_antenna) {
                                        this->vehicle_antennas[ai].assigned_bs_idx = static_cast<int>(bi);
                                        break;
                                    }
                                }
                                break;
                            }
                        }
                    }

                    // 後方互換: active_antenna_idx は最良ペア(pairs[0])のTXを設定
                    if (!lut_entry.pairs.empty()) {
                        for (size_t i = 0; i < this->vehicle_antennas.size(); ++i) {
                            if (this->vehicle_antennas[i].name == lut_entry.pairs[0].tx_antenna) {
                                new_active_idx = static_cast<int>(i);
                                break;
                            }
                        }
                    }
                    
                    if (new_active_idx != this->active_antenna_idx) {
                        std::ostringstream ss;
                        ss << "Multi-pair feedforward switch at x=" << std::fixed << std::setprecision(2) << pos.x()
                           << " (" << lut_entry.pairs.size() << " pairs)";
                        switch_details = ss.str();
                    }

                    // For feedforward_optimal, assign metrics based on assigned_bs_idx
                    for (size_t a_idx = 0; a_idx < this->vehicle_antennas.size(); ++a_idx) {
                        auto &ant = this->vehicle_antennas[a_idx];
                        if (ant.assigned_bs_idx >= 0) {
                            ant_metrics_list[a_idx] = all_ant_bs_metrics[a_idx][ant.assigned_bs_idx];
                            ant.last_rssi = ant_metrics_list[a_idx].best_rssi;
                        }
                    }
                }
            } else if (this->scheduling_policy == "rssi_priority") {
                double max_rssi = -1e9;
                int best_ant = new_active_idx;
                for (size_t i = 0; i < this->vehicle_antennas.size(); ++i) {
                    if (ant_metrics_list[i].best_rssi > max_rssi) {
                        max_rssi = ant_metrics_list[i].best_rssi;
                        best_ant = i;
                    }
                }
                bool hold_passed = (current_time - this->last_grant_change_time) >= this->min_hold_time_s;
                if (best_ant != this->active_antenna_idx && hold_passed && (max_rssi > ant_metrics_list[this->active_antenna_idx].best_rssi + this->switch_margin_db)) {
                    new_active_idx = best_ant;
                    std::ostringstream ss;
                    ss << "RSSI-based switch (RSSI: " << std::fixed << std::setprecision(2) << max_rssi << " dBm)";
                    switch_details = ss.str();
                }
            } else if (this->scheduling_policy == "simple_no_handover") {
                std::vector<bool> bs_in_use(this->base_stations.size(), false);
                
                // 1. Maintain connected antennas
                for (size_t i = 0; i < this->vehicle_antennas.size(); ++i) {
                    auto &ant = this->vehicle_antennas[i];
                    if (ant.assigned_bs_idx >= 0 && ant.assigned_bs_idx < static_cast<int>(this->base_stations.size())) {
                        double current_rssi = all_ant_bs_metrics[i][ant.assigned_bs_idx].best_rssi;
                        if (current_rssi >= this->comms_calculator->rssi_min) {
                            bs_in_use[ant.assigned_bs_idx] = true;
                            ant_metrics_list[i] = all_ant_bs_metrics[i][ant.assigned_bs_idx];
                            ant.last_rssi = current_rssi;
                        } else {
                            ant.assigned_bs_idx = -1; // Disconnect
                        }
                    }
                }
                
                // 2. Connect disconnected antennas to best available BS
                for (size_t i = 0; i < this->vehicle_antennas.size(); ++i) {
                    auto &ant = this->vehicle_antennas[i];
                    if (ant.assigned_bs_idx < 0) {
                        double best_rssi = -999.0;
                        int best_bs = -1;
                        for (size_t bs_idx = 0; bs_idx < this->base_stations.size(); ++bs_idx) {
                            if (bs_in_use[bs_idx]) continue;
                            double rssi = all_ant_bs_metrics[i][bs_idx].best_rssi;
                            if (rssi > best_rssi) {
                                best_rssi = rssi;
                                best_bs = static_cast<int>(bs_idx);
                            }
                        }
                        if (best_bs >= 0 && best_rssi >= this->comms_calculator->rssi_min) {
                            ant.assigned_bs_idx = best_bs;
                            bs_in_use[best_bs] = true;
                            ant_metrics_list[i] = all_ant_bs_metrics[i][best_bs];
                            ant.last_rssi = best_rssi;
                        }
                    }
                }
            } else if (this->scheduling_policy == "physical_score_priority") {
                double max_score = -1e9;
                int best_ant = new_active_idx;
                for (size_t i = 0; i < this->vehicle_antennas.size(); ++i) {
                    double score = ant_metrics_list[i].e_gain + ant_metrics_list[i].h_gain - ant_metrics_list[i].path_loss;
                    if (score > max_score) {
                        max_score = score;
                        best_ant = i;
                    }
                }
                bool hold_passed = (current_time - this->last_grant_change_time) >= this->min_hold_time_s;
                double current_score = ant_metrics_list[this->active_antenna_idx].e_gain + ant_metrics_list[this->active_antenna_idx].h_gain - ant_metrics_list[this->active_antenna_idx].path_loss;
                if (best_ant != this->active_antenna_idx && hold_passed && (max_score > current_score + this->switch_margin_db)) {
                    new_active_idx = best_ant;
                    std::ostringstream ss;
                    ss << "Physical score-based switch (Score: " << std::fixed << std::setprecision(2) << max_score << " dB)";
                    switch_details = ss.str();
                }
            } else if (this->scheduling_policy == "geometric_beam_priority") {
                double max_align = -1e9;
                int best_ant = new_active_idx;
                for (size_t i = 0; i < this->vehicle_antennas.size(); ++i) {
                    double align = std::min(ant_metrics_list[i].e_gain, ant_metrics_list[i].h_gain);
                    if (align > max_align) {
                        max_align = align;
                        best_ant = i;
                    }
                }
                bool hold_passed = (current_time - this->last_grant_change_time) >= this->min_hold_time_s;
                double current_align = std::min(ant_metrics_list[this->active_antenna_idx].e_gain, ant_metrics_list[this->active_antenna_idx].h_gain);
                if (best_ant != this->active_antenna_idx && hold_passed && (max_align > current_align + this->switch_margin_db)) {
                    new_active_idx = best_ant;
                    std::ostringstream ss;
                    ss << "Geometric beam-based switch (Align: " << std::fixed << std::setprecision(2) << max_align << " dB)";
                    switch_details = ss.str();
                }
            }

            // Proactive handover check
            if (this->scheduling_policy != "feedforward_optimal" && this->scheduling_policy != "simple_no_handover") {
                bool grace_passed = (current_time - this->last_grant_change_time) > this->proactive_grace_period_s;
                bool hold_passed = (current_time - this->last_grant_change_time) >= this->min_hold_time_s;
                std::string active_state = this->vehicle_antennas[this->active_antenna_idx].link_state;
                
                if (active_state == "DISCONNECTED" && grace_passed && hold_passed) {
                    double best_score = -1e9;
                    int best_idx = -1;
                    for (size_t i = 0; i < this->vehicle_antennas.size(); ++i) {
                        if (static_cast<int>(i) == this->active_antenna_idx) continue;
                        if (!this->vehicle_antennas[i].comm_active) continue;
                        if (this->filter_main_lobe && !ant_metrics_list[i].in_main_lobe) continue;

                        double score = ant_metrics_list[i].e_gain + ant_metrics_list[i].h_gain - ant_metrics_list[i].path_loss;
                        if (score > best_score) {
                            best_score = score;
                            best_idx = i;
                        }
                    }
                    
                    if (best_idx != -1 && best_score >= this->proactive_handover_score_threshold) {
                        new_active_idx = best_idx;
                        std::ostringstream ss;
                        ss << "Proactive handover switch (Score: " << std::fixed << std::setprecision(2) << best_score << " dB)";
                        switch_details = ss.str();
                    }
                }
            }

            if (new_active_idx != this->active_antenna_idx) {
                int old_idx = this->active_antenna_idx;
                this->active_antenna_idx = new_active_idx;
                this->last_grant_change_time = current_time;

                if (this->logging_level >= 2) {
                    EventRecord ev;
                    ev.time_s = current_time;
                    ev.event_type = "SWITCH";
                    ev.active_antenna = this->vehicle_antennas[new_active_idx].name;
                    ev.prev_antenna = this->vehicle_antennas[old_idx].name;
                    ev.details = switch_details;
                    this->event_logs.push_back(ev);
                }
            }

            if (this->logging_level >= 5 && this->scheduling_policy == "feedforward_optimal") {
                ControlRecord ctrl;
                ctrl.time_s = current_time;
                ctrl.vehicle_x = pos.x();
                ctrl.vehicle_y = pos.y();
                ctrl.vehicle_yaw = ori.z();
                // nominal_antenna: LUTの最良ペアのTXアンテナ名
                if (!this->lut.empty() && this->last_lut_idx >= 0 && this->last_lut_idx < static_cast<int>(this->lut.size()) && !this->lut[this->last_lut_idx].pairs.empty()) {
                    ctrl.nominal_antenna = this->lut[this->last_lut_idx].pairs[0].tx_antenna;
                } else {
                    ctrl.nominal_antenna = "";
                }
                
                if (this->active_antenna_idx >= 0 && this->active_antenna_idx < static_cast<int>(this->vehicle_antennas.size())) {
                    ctrl.active_antenna = this->vehicle_antennas[this->active_antenna_idx].name;
                } else {
                    ctrl.active_antenna = "";
                }
                ctrl.switching_active = (new_active_idx != this->active_antenna_idx);
                ctrl.last_switch_time_s = this->last_grant_change_time;
                this->control_logs.push_back(ctrl);
            }

            // 4. Update Link States and Data Accumulation
            for (size_t i = 0; i < this->vehicle_antennas.size(); ++i) {
                auto &ant = this->vehicle_antennas[i];
                // feedforward_optimal と simple_no_handover: assigned_bs_idx >= 0 ならグラント有り（マルチペア）
                // 他のポリシー: active_antenna_idx と一致すればグラント有り（シングルペア）
                bool has_grant;
                if (this->scheduling_policy == "feedforward_optimal" || this->scheduling_policy == "simple_no_handover") {
                    has_grant = (ant.assigned_bs_idx >= 0);
                } else {
                    has_grant = (static_cast<int>(i) == this->active_antenna_idx);
                }
                
                bool link_ready = false;
                if (!has_grant || !ant.comm_active) {
                    if (ant.link_state != "DISCONNECTED") {
                        ant.link_state = "DISCONNECTED";
                        ant.link_establishment_start_time = -1.0;
                        ant.establishment_step_count = 0;
                    }
                } else {
                    int required_steps = static_cast<int>(std::ceil(this->link_establishment_time_ms / 1000.0 * 1000.0));
                    
                    if (ant.link_state == "DISCONNECTED") {
                        if (ant.last_rssi > this->comms_calculator->rssi_min) {
                            ant.link_state = "ESTABLISHING";
                            ant.link_establishment_start_time = current_time;
                            ant.establishment_step_count = 1;
                            if (required_steps <= 1) {
                                ant.link_state = "CONNECTED";
                                ant.establishment_step_count = 0;
                                link_ready = true;
                            }
                        }
                    } else if (ant.link_state == "ESTABLISHING") {
                        if (ant.last_rssi <= this->comms_calculator->rssi_min) {
                            ant.link_state = "DISCONNECTED";
                            ant.link_establishment_start_time = -1.0;
                            ant.establishment_step_count = 0;
                        } else {
                            ant.establishment_step_count++;
                            if (ant.establishment_step_count >= required_steps) {
                                ant.link_state = "CONNECTED";
                                ant.establishment_step_count = 0;
                                link_ready = true;
                            }
                        }
                    } else if (ant.link_state == "CONNECTED") {
                        if (ant.last_rssi <= this->comms_calculator->rssi_min) {
                            ant.link_state = "DISCONNECTED";
                            ant.link_establishment_start_time = -1.0;
                            ant.establishment_step_count = 0;
                        } else {
                            link_ready = true;
                        }
                    }
                }

                double actual_throughput = 0.0;
                if (link_ready && ant.comm_active && this->all_ready) {
                    actual_throughput = ant_metrics_list[i].throughput;
                    if (actual_throughput > 0.0) {
                        ant.total_data_transmitted += actual_throughput * 1000.0 / 8.0 * dt;
                        if (this->comm_data_limit_mb > 0 && ant.total_data_transmitted >= this->comm_data_limit_mb) {
                            ant.comm_active = false;
                        }
                    }
                }

                // Log Record
                if (this->logging_level >= 3) {
                    bool should_log = true;
                    if (this->logging_level == 3 && ant.link_state != "CONNECTED") {
                        should_log = false;
                    }

                    if (should_log) {
                        Eigen::Vector3d ant_pos_world = pos + vehicle_rotmat * ant.offset;
                        typename AntennaInfo::LogRecord rec;
                        rec.time_s = current_time;
                        rec.vehicle_time_s = current_time - this->start_time_sec;
                        rec.vehicle_name = ant.name;
                        rec.has_link_grant = has_grant;
                        rec.distance_m = ant_metrics_list[i].distance;
                        rec.rssi_dBm = ant_metrics_list[i].best_rssi;
                        rec.throughput_Gbps = actual_throughput;
                        rec.total_data_MB = ant.total_data_transmitted;
                        rec.path_loss_dB = ant_metrics_list[i].path_loss;
                        rec.e_gain_dB = ant_metrics_list[i].e_gain;
                        rec.h_gain_dB = ant_metrics_list[i].h_gain;
                        rec.comm_active = ant.comm_active;
                        rec.tx_x_m = ant_pos_world.x();
                        rec.tx_y_m = ant_pos_world.y();
                        rec.tx_z_m = ant_pos_world.z();
                        rec.bs_x_m = ant_metrics_list[i].bs_pos.x();
                        rec.bs_y_m = ant_metrics_list[i].bs_pos.y();
                        rec.bs_z_m = ant_metrics_list[i].bs_pos.z();
                        rec.link_state = ant.link_state;
                        rec.in_main_lobe = ant_metrics_list[i].in_main_lobe;
                        rec.off_boresight_e_deg = ant_metrics_list[i].off_boresight_e;
                        rec.off_boresight_h_deg = ant_metrics_list[i].off_boresight_h;
                        ant.log_records.push_back(rec);
                    }
                }
            }
        }

        std::string GetAntennaCSVPath(const std::string &ant_name) {
            std::string summary_filename = this->config_summary_filename;
            std::string output_subdir = this->config_output_subdir;
            std::string output_dir = this->config_output_dir;
            double y_pos = this->config_y_pos;
            double angle = this->config_angle;

            std::ostringstream y_ss;
            y_ss << std::round(y_pos * 100.0) / 100.0;
            std::string y_str = y_ss.str();

            std::ostringstream a_ss;
            a_ss << angle;
            std::string angle_str = a_ss.str();

            std::string run_dir = "";
            std::regex sweep_regex("sweep_summary_(\\d{8}_\\d{6})_run(\\d+)");
            std::smatch sweep_match;

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

            std::string suffix = (this->logging_level == 3) ? "_connected.csv" : "_full.csv";
            return run_dir + "/comms/" + ant_name + suffix;
        }

        void SaveLogs() {
            if (this->vehicle_antennas.empty()) return;
            std::string sample_csv_path = this->GetAntennaCSVPath(this->vehicle_antennas[0].name);
            std::filesystem::path run_dir_path = std::filesystem::path(sample_csv_path).parent_path().parent_path();
            gzmsg << "[TxControllerPlugin] Saving logs to directory: " << run_dir_path.string() << " (antennas: " << this->vehicle_antennas.size() << ")" << std::endl;
            
            for (const auto &ant : this->vehicle_antennas) {
                std::string csv_path = this->GetAntennaCSVPath(ant.name);
                try {
                    std::filesystem::create_directories(std::filesystem::path(csv_path).parent_path());
                    std::ofstream file(csv_path);
                    if (!file.is_open()) {
                        std::cerr << "[TxControllerPlugin] Failed to open CSV: " << csv_path << std::endl;
                        continue;
                    }
                    if (this->logging_level == 3) {
                        file << "time_s,vehicle_name,distance_m,rssi_dBm,throughput_Gbps,total_data_MB,"
                             << "path_loss_dB,e_gain_dB,h_gain_dB,tx_x_m,tx_y_m,tx_z_m,bs_x_m,bs_y_m,bs_z_m\n";
                    } else {
                        file << "time_s,vehicle_time_s,vehicle_name,has_link_grant,distance_m,rssi_dBm,"
                             << "throughput_Gbps,total_data_MB,path_loss_dB,e_gain_dB,h_gain_dB,comm_active,"
                             << "tx_x_m,tx_y_m,tx_z_m,bs_x_m,bs_y_m,bs_z_m,link_state,"
                             << "in_main_lobe,off_boresight_e_deg,off_boresight_h_deg\n";
                    }
                    file << std::fixed << std::setprecision(6);
                    for (const auto &rec : ant.log_records) {
                        if (this->logging_level == 3) {
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
                                 << rec.link_state << ","
                                 << (rec.in_main_lobe ? "True" : "False") << ","
                                 << rec.off_boresight_e_deg << ","
                                 << rec.off_boresight_h_deg << "\n";
                        }
                    }
                    file.close();
                    try {
                        std::filesystem::permissions(csv_path, std::filesystem::perms::all);
                    } catch(...) {}
                } catch (const std::exception &e) {
                    std::cerr << "[TxControllerPlugin] Error saving CSV: " << e.what() << std::endl;
                }
            }

            if (this->logging_level >= 2 && !this->event_logs.empty()) {
                std::string ev_path = (run_dir_path / "control" / "events.csv").string();
                try {
                    std::filesystem::create_directories(std::filesystem::path(ev_path).parent_path());
                    std::ofstream file(ev_path);
                    if (file.is_open()) {
                        file << "time_s,event_type,active_antenna,prev_antenna,details\n";
                        file << std::fixed << std::setprecision(6);
                        for (const auto &ev : this->event_logs) {
                            file << ev.time_s << ","
                                 << ev.event_type << ","
                                 << ev.active_antenna << ","
                                 << ev.prev_antenna << ","
                                 << ev.details << "\n";
                        }
                        file.close();
                        try {
                            std::filesystem::permissions(ev_path, std::filesystem::perms::all);
                        } catch(...) {}
                    }
                } catch(...) {}
            }

            if (this->logging_level >= 5 && !this->control_logs.empty()) {
                std::string ctrl_path = (run_dir_path / "control" / "feedforward_log.csv").string();
                try {
                    std::filesystem::create_directories(std::filesystem::path(ctrl_path).parent_path());
                    std::ofstream file(ctrl_path);
                    if (file.is_open()) {
                        file << "time_s,vehicle_x,vehicle_y,vehicle_yaw,nominal_antenna,active_antenna,switching_active,last_switch_time_s\n";
                        file << std::fixed << std::setprecision(6);
                        for (const auto &ctrl : this->control_logs) {
                            file << ctrl.time_s << ","
                                 << ctrl.vehicle_x << ","
                                 << ctrl.vehicle_y << ","
                                 << ctrl.vehicle_yaw << ","
                                 << ctrl.nominal_antenna << ","
                                 << ctrl.active_antenna << ","
                                 << (ctrl.switching_active ? "True" : "False") << ","
                                 << ctrl.last_switch_time_s << "\n";
                        }
                        file.close();
                        try {
                            std::filesystem::permissions(ctrl_path, std::filesystem::perms::all);
                        } catch(...) {}
                    }
                } catch(...) {}
            }

            // Copy config file to result directory if it's a single launch
            if (run_dir_path.string().find("/sweep_") == std::string::npos) {
                if (!this->config_file_path.empty() && std::filesystem::exists(this->config_file_path)) {
                    std::string backup_path = (run_dir_path / "scenario_config_backup.yaml").string();
                    try {
                        std::filesystem::copy_file(this->config_file_path, backup_path, std::filesystem::copy_options::overwrite_existing);
                        std::filesystem::permissions(backup_path, std::filesystem::perms::all);
                    } catch(const std::exception& e) {
                        std::cerr << "[TxControllerPlugin] Failed to copy scenario config: " << e.what() << std::endl;
                    }
                }
            }
        }

        void SetVelocity(gz::sim::EntityComponentManager &_ecm, double v, double w) {
            auto linComp = _ecm.Component<gz::sim::components::LinearVelocityCmd>(this->model.Entity());
            if (!linComp) {
                _ecm.CreateComponent(this->model.Entity(), gz::sim::components::LinearVelocityCmd({v, 0, 0}));
            } else {
                linComp->Data() = {v, 0, 0};
            }

            auto angComp = _ecm.Component<gz::sim::components::AngularVelocityCmd>(this->model.Entity());
            if (!angComp) {
                _ecm.CreateComponent(this->model.Entity(), gz::sim::components::AngularVelocityCmd({0, 0, w}));
            } else {
                angComp->Data() = {0, 0, w};
            }
        }

        gz::sim::Model model;
        std::string model_name;
        
        std::vector<Waypoint> waypoints;
        size_t current_waypoint_idx = 0;
        double waypoint_tolerance = 2.0;
        double heading_gain = 1.5;
        double max_acceleration = 0.5;
        double max_angular_velocity = 1.0;
        bool is_shinkansen = false;

        double current_v = 0.0;
        
        bool all_ready = false;
        bool mission_complete = false;
        int64_t last_ready_pub_time = 0;
        int64_t last_complete_pub_time = 0;

        gz::transport::Node node;
        gz::transport::Node::Publisher mission_complete_pub;
        gz::transport::Node::Publisher ready_pub;
        gz::transport::Node::Publisher mission_progress_pub;
        
        double total_path_distance = 0.0;
        int64_t last_progress_pub_time = 0;

        // Comms configurations and state variables
        bool comms_initialized = false;
        bool logs_saved = false;
        int logging_level = 1;
        std::string config_file_path = "";
        std::string scheduling_policy = "sequential";
        bool filter_main_lobe = true;
        double beam_gain_threshold = 5.0;
        double min_hold_time_s = 1.0;
        double switch_margin_db = 2.0;
        double proactive_grace_period_s = 0.5;
        double proactive_handover_score_threshold = -80.0;
        double time_slot_duration_s = 10.0;
        double weight_distance = 0.7;
        double weight_angle = 0.3;
        double heatmap_resolution_m = 0.2;
        int ff_max_pairs = -1;
        std::string center_antenna_name = "shinkansen_mid";

        double comm_data_limit_mb = -1.0;
        double link_establishment_time_ms = 2.0;

        std::vector<AntennaInfo> vehicle_antennas;
        std::vector<BaseStationInfo> base_stations_cfg;
        std::vector<BaseStationInfo> base_stations;
        bool base_stations_located = false;

        std::unique_ptr<comms_sim::AntennaPatternParser> antenna_parser;
        std::unique_ptr<comms_sim::CommsCalculator> comms_calculator;

        int active_antenna_idx = 0;
        double last_grant_change_time = 0.0;
        double start_time_sec = -1.0;
        std::vector<LutEntry> lut;
        int last_lut_idx = 0;

        std::vector<EventRecord> event_logs;
        std::vector<ControlRecord> control_logs;

        // Cached run details
        double config_y_pos = 0.0;
        double config_angle = 0.0;
        std::string config_summary_filename = "sweep_summary.csv";
        std::string config_output_subdir = "";
        std::string config_output_dir = "/workspace/sim_results/";
    };
}

GZ_ADD_PLUGIN(
    tx_controller::TxControllerPlugin,
    gz::sim::System,
    tx_controller::TxControllerPlugin::ISystemConfigure,
    tx_controller::TxControllerPlugin::ISystemPreUpdate
)
GZ_ADD_PLUGIN_ALIAS(tx_controller::TxControllerPlugin, "tx_controller::TxControllerPlugin")
