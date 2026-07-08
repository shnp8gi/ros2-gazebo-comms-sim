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
#include <atomic>
#include <mutex>
#include "comms_sim_pkg/comms_calculator.hpp"
#include "comms_sim_pkg/antenna_pattern_parser.hpp"

#include "comms_sim_pkg/DataTypes.hpp"
#include "comms_sim_pkg/Utils.hpp"

#include "comms_sim_pkg/VehicleMotionController.hpp"
#include "comms_sim_pkg/SimulationLogger.hpp"
#include "comms_sim_pkg/CommsEnvironment.hpp"
#include "comms_sim_pkg/BlockageEnvironment.hpp"
#include "comms_sim_pkg/HandoverScheduler.hpp"
#include "comms_sim_pkg/ExternalScheduleStrategy.hpp"
#include "comms_sim_pkg/kkf/KkfConfig.hpp"
#include "comms_sim_pkg/kkf/KkfPredictiveStrategy.hpp"
#include "comms_sim_msgs.pb.h"

namespace tx_controller
{

    class TxControllerPlugin :
        public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
    {
    public:
        TxControllerPlugin() = default;
        ~TxControllerPlugin() override {
            std::lock_guard<std::mutex> lock(this->logs_mutex);
            if (this->comms_initialized && !this->logs_saved) {
                this->logger.SaveLogs(this->model_name, this->vehicle_antennas, this->scheduling_policy);
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

            std::vector<Waypoint> local_waypoints;
            double local_waypoint_tolerance = 1.0;
            double local_max_angular_velocity = 1.0;
            double local_heading_gain = 1.0;
            double local_max_acceleration = 1.0;
            bool local_is_shinkansen = false;

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
                    local_waypoints.push_back({vals[i], vals[i+1], vals[i+2], vals[i+3]});
                }
            }

            if (_sdf->HasElement("waypoint_tolerance"))
                local_waypoint_tolerance = _sdf->Get<double>("waypoint_tolerance");
            if (_sdf->HasElement("heading_gain"))
                local_heading_gain = _sdf->Get<double>("heading_gain");
            if (_sdf->HasElement("max_acceleration"))
                local_max_acceleration = _sdf->Get<double>("max_acceleration");
            if (_sdf->HasElement("max_angular_velocity"))
                local_max_angular_velocity = _sdf->Get<double>("max_angular_velocity");
            if (_sdf->HasElement("is_shinkansen"))
                local_is_shinkansen = _sdf->Get<bool>("is_shinkansen");

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

            if (_sdf->HasElement("config_file_path")) {
                this->config_file_path = _sdf->Get<std::string>("config_file_path");
            }

            // Setup publishers
            this->mission_complete_pub = this->node.Advertise<gz::msgs::Boolean>(mission_topic);
            this->ready_pub = this->node.Advertise<gz::msgs::Boolean>(ready_pub_topic);
            this->mission_progress_pub = this->node.Advertise<gz::msgs::Double>("/" + this->model.Name(_ecm) + "/mission_progress");

            // Setup subscriber
            this->node.Subscribe(all_ready_topic, &TxControllerPlugin::OnAllReady, this);

            // Initialize Comms Sim
            if (!this->config_file_path.empty()) {
                try {
                    this->InitializeComms(this->config_file_path, _ecm, local_waypoints);
                } catch (const std::exception& e) {
                    gzerr << "[TxControllerPlugin] Error initializing comms: " << e.what() << std::endl;
                }
            }

            this->motion_controller.Configure(
                local_waypoints,
                local_waypoint_tolerance,
                local_max_angular_velocity,
                local_heading_gain,
                local_max_acceleration,
                local_is_shinkansen
            );

            gzmsg << "[TxControllerPlugin] Initialized on model [" << this->model.Name(_ecm) 
                  << "] with " << local_waypoints.size() << " waypoints." << std::endl;
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

            // 3. Get current pose
            auto poseComp = _ecm.Component<gz::sim::components::Pose>(this->model.Entity());
            if (!poseComp) return;
            gz::math::Pose3d pose = poseComp->Data();

            MotionState current_state;
            current_state.x = pose.Pos().X();
            current_state.y = pose.Pos().Y();
            current_state.yaw = pose.Rot().Yaw();

            if (this->motion_controller.IsMissionComplete()) {
                if (!this->mission_complete) {
                    this->mission_complete = true;
                    if (this->comms_initialized && !this->logs_saved) {
                        this->logger.SaveLogs(this->model_name, this->vehicle_antennas, this->scheduling_policy);
                        this->logs_saved = true;
                    }
                    gzmsg << "[TxControllerPlugin] Mission Complete for [" << this->model_name << "]!" << std::endl;
                }
                
                // Periodically re-publish mission complete
                if (_info.simTime.count() - this->last_complete_pub_time > 1000000000) { // 1.0s
                    gz::msgs::Boolean msg;
                    msg.set_data(true);
                    this->mission_complete_pub.Publish(msg);
                    this->last_complete_pub_time = _info.simTime.count();
                }
            }

            double dt = std::chrono::duration<double>(_info.dt).count();

            // 4. Compute tracking and velocity using the new controller
            MotionCommand cmd = this->motion_controller.CalculateCommand(current_state, dt);

            // 5. Apply velocity
            this->SetVelocity(_ecm, cmd.linear_velocity, cmd.angular_velocity);

            this->PublishProgress(_info, current_state);
        }

        void PublishProgress(const gz::sim::UpdateInfo &_info, const MotionState &current_state) {
            double progress = this->motion_controller.GetMissionProgress(current_state);
            
            if (_info.simTime.count() - this->last_progress_pub_time > 500000000) { // 0.5s
                gz::msgs::Double msg;
                msg.set_data(progress);
                this->mission_progress_pub.Publish(msg);
                this->last_progress_pub_time = _info.simTime.count();
            }
        }

    private:
        void InitializeComms(const std::string &config_path, gz::sim::EntityComponentManager &_ecm, std::vector<Waypoint>& local_waypoints) {
            YAML::Node config = YAML::LoadFile(config_path);
            
            // 1. Read simulation configurations
            LoggerConfig logger_config;
            logger_config.config_file_path = this->config_file_path;
            if (config["simulation"]) {
                if (config["simulation"]["logging_level"]) {
                    this->logging_level = config["simulation"]["logging_level"].as<int>(1);
                }
                if (config["simulation"]["output_subdir"]) {
                    logger_config.output_subdir = config["simulation"]["output_subdir"].as<std::string>();
                }
                if (config["simulation"]["summary_filename"]) {
                    logger_config.summary_filename = config["simulation"]["summary_filename"].as<std::string>();
                }
                if (config["simulation"]["logging_angle_unit"]) {
                    logger_config.angle_unit = config["simulation"]["logging_angle_unit"].as<std::string>();
                }
            }
            this->logger.Configure(logger_config);
            
            // 2. Read comms parameters
            auto comms_params = config["comms_simulator_node"]["ros__parameters"];
            double comm_data_limit = comms_params["comm_data_limit_mb"].as<double>(-1.0);
            this->comm_data_limit_mb = comm_data_limit;
            this->link_establishment_time_ms = comms_params["link_establishment_time_ms"].as<double>(2.0);

            // 4. Link controller parameters
            auto link_ctrl_params = config["link_controller_node"]["ros__parameters"];
            this->scheduling_policy = link_ctrl_params["scheduling_policy"].as<std::string>("sequential");
            this->filter_main_lobe = link_ctrl_params["filter_main_lobe"].as<bool>(true);
            this->min_hold_time_s = link_ctrl_params["min_hold_time_s"].as<double>(1.0);
            this->switch_margin_db = link_ctrl_params["switch_margin_db"].as<double>(2.0);
            this->proactive_grace_period_s = link_ctrl_params["proactive_grace_period_s"].as<double>(0.5);
            this->proactive_handover_score_threshold = link_ctrl_params["proactive_handover_score_threshold"].as<double>(-80.0);
            this->heatmap_resolution_m = link_ctrl_params["heatmap_resolution_m"].as<double>(0.2);
            this->min_hold_distance_m = link_ctrl_params["min_hold_distance_m"].as<double>(0.0);
            this->ff_max_pairs = link_ctrl_params["ff_max_pairs"].as<int>(-1);

            this->comms_env.Configure(this->config_file_path);
            this->blockage_env.Configure(config);
            this->scheduler.Configure(this->scheduling_policy, this->filter_main_lobe, this->min_hold_time_s,
                                      this->switch_margin_db, this->proactive_grace_period_s,
                                      this->proactive_handover_score_threshold, this->comm_data_limit_mb,
                                      this->link_establishment_time_ms);

            // 5. Load antennas configuration for this vehicle model
            std::string current_model_name = this->model.Name(_ecm);
            
            if (config["vehicles"]) {
                for (auto const &v : config["vehicles"]) {
                    std::string v_name = v["name"].as<std::string>();
                    if (v_name == current_model_name) {
                        if (v["waypoints"]) {
                            local_waypoints.clear();
                            for (auto const &wp : v["waypoints"]) {
                                Waypoint w;
                                w.x = wp[0].as<double>();
                                w.y = wp[1].as<double>();
                                w.z = wp[2].as<double>();
                                w.v = wp[3].as<double>();
                                local_waypoints.push_back(w);
                            }
                        }
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

            // 6. Load base station configs
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
                    Eigen::Vector3d ant_rpy_updated = bs.rpy + bs.antenna_relative_rpy;
                    bs.rotmat = utils::rpy_to_rotmat(ant_rpy_updated.x(), ant_rpy_updated.y(), ant_rpy_updated.z());
                    this->base_stations_cfg.push_back(bs);
                }
            }

            // Precalculate LUT for feedforward_optimal
            if (this->scheduling_policy == "feedforward_optimal") {
                this->PrecalculateLUT(local_waypoints);
            }

            // 測定レポート出力 (制御プレーンへの観測レポート、設計書§7)
            if (comms_params["measurement_report"]) {
                auto rep = comms_params["measurement_report"];
                this->report_enabled = rep["enabled"].as<bool>(this->report_enabled);
                this->report_period_s = rep["period_s"].as<double>(this->report_period_s);
                this->report_noise_std_db = rep["noise_std_db"].as<double>(this->report_noise_std_db);
                this->report_topic = rep["topic"].as<std::string>(this->report_topic);
                this->report_observe_all_pairs =
                    rep["observe_all_pairs"].as<bool>(this->report_observe_all_pairs);
                this->report_rng.seed(rep["seed"].as<unsigned>(123));
            }
            if (this->report_enabled) {
                this->report_pub = this->node.Advertise<comms_sim::msgs::MeasurementReport>(this->report_topic);
            }

            // 外部スケジュール実行戦略 (制御プレーンが生成したスケジュールに追従)
            if (this->scheduling_policy == "external_schedule") {
                std::string schedule_topic =
                    link_ctrl_params["schedule_topic"].as<std::string>("/comms/ho_schedule");
                this->external_strategy = std::make_shared<ExternalScheduleStrategy>();
                this->scheduler.RegisterStrategy(this->external_strategy);
                this->node.Subscribe(schedule_topic, &TxControllerPlugin::OnHoSchedule, this);
                gzmsg << "[TxControllerPlugin] External schedule strategy registered (topic: "
                      << schedule_topic << ")" << std::endl;
            }

            // Register KKF predictive strategy (第1層+第3層: 学習地図 + ビタビDP)
            if (this->scheduling_policy == "kkf_predictive") {
                std::vector<Eigen::Vector3d> track_points;
                for (const auto& wp : local_waypoints) {
                    track_points.push_back(Eigen::Vector3d(wp.x, wp.y, wp.z));
                }
                std::vector<Eigen::Vector3d> bs_antenna_positions;
                for (const auto& bs : this->base_stations_cfg) {
                    bs_antenna_positions.push_back(bs.position + bs.rotmat * bs.antenna_offset);
                }
                auto kkf_config = kkf::KkfConfig::FromYaml(link_ctrl_params);
                this->scheduler.RegisterStrategy(std::make_shared<kkf::KkfPredictiveStrategy>(
                    kkf_config, kkf::RoadCoordinate(track_points), bs_antenna_positions));
                gzmsg << "[TxControllerPlugin] KKF predictive strategy registered ("
                      << bs_antenna_positions.size() << " RSU maps, observe_all_pairs="
                      << (kkf_config.observe_all_pairs ? "true" : "false") << ")" << std::endl;
            }

            this->comms_initialized = true;
            this->scheduler.SetLUT(this->lut);
            gzmsg << "[TxControllerPlugin] Precalculated LUT with " << this->lut.size() << " trajectory points." << std::endl;
            gzmsg << "[TxControllerPlugin] Comms simulator initialized successfully!" << std::endl;
        }

        void PrecalculateLUT(const std::vector<Waypoint>& waypoints_to_use) {
            std::vector<Eigen::Vector3d> polyline_points;
            for (const auto &wp : waypoints_to_use) {
                polyline_points.push_back(Eigen::Vector3d(wp.x, wp.y, wp.z));
            }

            auto samples = utils::sample_trajectory(polyline_points, this->heatmap_resolution_m);
            if (samples.empty()) return;

            this->lut.clear();

            size_t num_tx = this->vehicle_antennas.size();
            size_t num_rx = this->base_stations_cfg.size();
            size_t num_pairs = std::min(num_tx, num_rx);
            if (this->ff_max_pairs > 0) {
                num_pairs = std::min(num_pairs, static_cast<size_t>(this->ff_max_pairs));
            }

            std::vector<LutPair> current_active_pairs;
            double current_active_rssi_sum = -1e9;
            double current_distance_m = 0.0;
            double last_switch_distance_m = 0.0;

            for (const auto &sample : samples) {
                Eigen::Matrix3d vehicle_rotmat = utils::rpy_to_rotmat(0.0, 0.0, sample.yaw);

                auto all_metrics = this->comms_env.CalculateMetrics(
                    this->vehicle_antennas, this->base_stations_cfg, sample.pos, vehicle_rotmat);

                // Step 1: 全 (TX, RX) 組み合わせのRSSIを計算
                std::vector<std::vector<double>> rssi_matrix(num_tx, std::vector<double>(num_rx, -999.0));
                for (size_t tx_idx = 0; tx_idx < num_tx; ++tx_idx) {
                    for (size_t rx_idx = 0; rx_idx < num_rx; ++rx_idx) {
                        const auto& m = all_metrics[tx_idx][rx_idx];
                        if (!this->filter_main_lobe || m.in_main_lobe) {
                            rssi_matrix[tx_idx][rx_idx] = m.best_rssi;
                        }
                    }
                }

                // Step 2: 全列挙で現在の最適N個ペアを決定
                std::vector<int> rx_indices(num_rx);
                std::iota(rx_indices.begin(), rx_indices.end(), 0);

                double best_total_rssi = -1e9;
                std::vector<LutPair> candidate_pairs;

                do {
                    double total_rssi = 0.0;
                    std::vector<LutPair> temp_pairs;
                    for (size_t p = 0; p < num_pairs; ++p) {
                        size_t tx_idx = p;
                        size_t rx_idx = static_cast<size_t>(rx_indices[p]);
                        double rssi = rssi_matrix[tx_idx][rx_idx];
                        total_rssi += rssi;
                        temp_pairs.push_back({
                            this->vehicle_antennas[tx_idx].name,
                            this->base_stations_cfg[rx_idx].name,
                            rssi
                        });
                    }
                    if (total_rssi > best_total_rssi) {
                        best_total_rssi = total_rssi;
                        candidate_pairs = temp_pairs;
                    }
                } while (std::next_permutation(rx_indices.begin(), rx_indices.end()));

                std::sort(candidate_pairs.begin(), candidate_pairs.end(), [](const LutPair &a, const LutPair &b) {
                    return a.rssi > b.rssi;
                });

                // Step 3: 現在のペア (current_active_pairs) が維持された場合のスコア計算
                double current_total_rssi = -1e9;
                if (!current_active_pairs.empty()) {
                    current_total_rssi = 0.0;
                    for (auto &pair : current_active_pairs) {
                        size_t tx_idx = 0, rx_idx = 0;
                        for (size_t i = 0; i < num_tx; ++i) if (this->vehicle_antennas[i].name == pair.tx_antenna) tx_idx = i;
                        for (size_t i = 0; i < num_rx; ++i) if (this->base_stations_cfg[i].name == pair.rx_antenna) rx_idx = i;
                        
                        double rssi = rssi_matrix[tx_idx][rx_idx];
                        pair.rssi = rssi; // 現在の位置でのRSSIに更新
                        current_total_rssi += rssi;
                    }
                }

                // Step 4: ヒステリシスと保持距離に基づく切り替え判定
                double distance_since_switch = current_distance_m - last_switch_distance_m;
                bool can_switch = (distance_since_switch >= this->min_hold_distance_m);

                if (current_active_pairs.empty() || 
                    (can_switch && (best_total_rssi > current_total_rssi + this->switch_margin_db))) 
                {
                    current_active_pairs = candidate_pairs;
                    current_active_rssi_sum = best_total_rssi;
                    last_switch_distance_m = current_distance_m;
                } else {
                    current_active_rssi_sum = current_total_rssi;
                }

                // LUT登録
                LutEntry entry;
                entry.x = sample.pos.x();
                entry.y = sample.pos.y();
                entry.z = sample.pos.z();
                entry.pairs = current_active_pairs;
                this->lut.push_back(entry);

                current_distance_m += this->heatmap_resolution_m;
            }

            gzmsg << "[TxControllerPlugin] Precalculated multi-pair feedforward LUT with " << this->lut.size() 
                  << " entries, " << num_pairs << " pairs per entry. (Hysteresis margin: " 
                  << this->switch_margin_db << " dB, Min Hold Dist: " << this->min_hold_distance_m << " m)" << std::endl;
        }

        /// 制御プレーンからのスケジュール受信 (gz-transport受信スレッド)
        void OnHoSchedule(const comms_sim::msgs::HoSchedule &msg) {
            if (!this->external_strategy) return;
            ExternalScheduleStrategy::Schedule schedule;
            schedule.valid_until = msg.valid_until();
            for (const auto &entry : msg.plan()) {
                schedule.plan.push_back({entry.t_start(), entry.ant(), entry.bs(),
                                         entry.mode() == comms_sim::msgs::MEASURE});
            }
            this->external_strategy->SetSchedule(schedule);
        }

        void UpdateComms(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) {
            std::lock_guard<std::mutex> lock(this->logs_mutex);
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
                            if (bs.name == name &&
                                std::none_of(this->base_stations.begin(), this->base_stations.end(),
                                    [&name](const BaseStationInfo& existing) { return existing.name == name; }))
                            {
                                BaseStationInfo bs_info = bs;
                                auto poseComp = _ecm.Component<gz::sim::components::Pose>(_ent);
                                if (poseComp) {
                                    gz::math::Pose3d p = poseComp->Data();
                                    bs_info.position = Eigen::Vector3d(p.Pos().X(), p.Pos().Y(), p.Pos().Z());
                                    bs_info.rpy = Eigen::Vector3d(p.Rot().Roll(), p.Rot().Pitch(), p.Rot().Yaw());
                                    Eigen::Vector3d ant_rpy_updated = bs_info.rpy + bs_info.antenna_relative_rpy;
                                    bs_info.rotmat = utils::rpy_to_rotmat(ant_rpy_updated.x(), ant_rpy_updated.y(), ant_rpy_updated.z());
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

            double current_time_s = std::chrono::duration<double>(_info.simTime).count();
            double dt = std::chrono::duration<double>(_info.dt).count();
            if (dt <= 0.0) return;

            if (this->start_time_sec < 0.0) {
                this->start_time_sec = current_time_s;
            }

            // 1. Get vehicle pose
            auto poseComp = _ecm.Component<gz::sim::components::Pose>(this->model.Entity());
            if (!poseComp) return;
            gz::math::Pose3d vehicle_pose = poseComp->Data();
            Eigen::Vector3d pos(vehicle_pose.Pos().X(), vehicle_pose.Pos().Y(), vehicle_pose.Pos().Z());
            Eigen::Vector3d ori(vehicle_pose.Rot().Roll(), vehicle_pose.Rot().Pitch(), vehicle_pose.Rot().Yaw());
            Eigen::Matrix3d vehicle_rotmat = utils::rpy_to_rotmat(ori.x(), ori.y(), ori.z());

            // 2. Compute comms metrics for all antennas (動的チャネル: 遮蔽・シャドウ・フェージング)
            this->blockage_env.Refresh(_ecm);
            std::vector<std::vector<AntennaMetrics>> all_ant_bs_metrics = this->comms_env.CalculateMetrics(
                this->vehicle_antennas, this->base_stations, pos, vehicle_rotmat,
                current_time_s, &this->blockage_env.Obstacles());

            // 3. Scheduling Policy
            auto sched_res = this->scheduler.UpdateLinks(
                current_time_s, pos, vehicle_rotmat, this->vehicle_antennas, this->base_stations,
                all_ant_bs_metrics, this->active_antenna_idx, this->last_switch_time_s,
                this->comms_env.GetRssiMin());

            this->active_antenna_idx = sched_res.new_active_idx;
            this->active_antenna_name = sched_res.active_antenna_name;
            this->switching_active = sched_res.switching_active;
            this->last_switch_time_s = sched_res.last_switch_time_s;

            for (const auto& ev : sched_res.events) {
                this->logger.AddEventRecord(ev);
            }

            // Extract best metrics list for data calculation
            std::vector<AntennaMetrics> ant_metrics_list(this->vehicle_antennas.size());
            for (size_t i = 0; i < this->vehicle_antennas.size(); ++i) {
                int bs_idx = this->vehicle_antennas[i].assigned_bs_idx;
                if (bs_idx >= 0 && bs_idx < static_cast<int>(this->base_stations.size())) {
                    ant_metrics_list[i] = all_ant_bs_metrics[i][bs_idx];
                } else {
                    double max_rssi = -999.0;
                    int best_bs = 0;
                    for (size_t j = 0; j < this->base_stations.size(); ++j) {
                        if (all_ant_bs_metrics[i][j].best_rssi > max_rssi) {
                            max_rssi = all_ant_bs_metrics[i][j].best_rssi;
                            best_bs = j;
                        }
                    }
                    ant_metrics_list[i] = all_ant_bs_metrics[i][best_bs];
                }
            }

            if (this->logging_level >= 5) {
                ControlRecord cr;
                cr.time_s = current_time_s;
                cr.vehicle_x = pos.x();
                cr.vehicle_y = pos.y();
                cr.vehicle_yaw = ori.z();
                cr.nominal_antenna = "auto";
                cr.active_antenna = this->active_antenna_name;
                cr.switching_active = this->switching_active;
                cr.last_switch_time_s = this->last_switch_time_s;
                this->logger.AddControlRecord(cr);
            }

            // 4. Update Link States and Data Accumulation
            bool do_report = this->report_enabled && current_time_s >= this->next_report_time;
            comms_sim::msgs::MeasurementReport report_msg;
            std::vector<bool> grant_flags(this->vehicle_antennas.size(), false);

            for (size_t i = 0; i < this->vehicle_antennas.size(); ++i) {
                auto &ant = this->vehicle_antennas[i];
                bool has_grant;
                if (this->scheduling_policy == "feedforward_optimal" ||
                    this->scheduling_policy == "simple_no_handover") {
                    has_grant = ant.assigned_bs_idx >= 0;
                } else if (this->scheduling_policy == "external_schedule" ||
                           this->scheduling_policy == "kkf_predictive") {
                    // 単一ペアネット方式: 割当BSが未確定 (スケジュール未着等) の間は
                    // grantを与えない (フェイルセーフ = リンクなし)
                    has_grant = (static_cast<int>(i) == this->active_antenna_idx) &&
                                ant.assigned_bs_idx >= 0;
                } else {
                    has_grant = static_cast<int>(i) == this->active_antenna_idx;
                }
                
                bool link_ready = false;
                if (!has_grant || !ant.comm_active) {
                    if (ant.link_state != "DISCONNECTED") {
                        ant.link_state = "DISCONNECTED";
                        ant.link_establishment_start_time = -1.0;
                    }
                } else {
                    int required_steps = static_cast<int>(std::ceil((this->link_establishment_time_ms / 1000.0) / dt));
                    if (ant.link_state == "DISCONNECTED" && ant.last_rssi > this->comms_env.GetRssiMin()) {
                        ant.link_state = "ESTABLISHING";
                        ant.establishment_step_count = 1;
                    } else if (ant.link_state == "ESTABLISHING") {
                        if (ant.last_rssi <= this->comms_env.GetRssiMin()) ant.link_state = "DISCONNECTED";
                        else if (++ant.establishment_step_count >= required_steps) { ant.link_state = "CONNECTED"; link_ready = true; }
                    } else if (ant.link_state == "CONNECTED") {
                        if (ant.last_rssi <= this->comms_env.GetRssiMin()) ant.link_state = "DISCONNECTED";
                        else link_ready = true;
                    }
                }

                // measure_only (測定専用ペアネット) はリンク確立してもデータ会計を行わない
                if (link_ready && ant.comm_active && this->all_ready && !ant.measure_only) {
                    ant.total_data_transmitted += ant_metrics_list[i].throughput * 1000.0 / 8.0 * dt;
                    if (this->comm_data_limit_mb > 0 && ant.total_data_transmitted >= this->comm_data_limit_mb) ant.comm_active = false;
                }

                grant_flags[i] = has_grant;

                if (this->logging_level >= 3) {
                    typename AntennaInfo::LogRecord rec;
                    rec.time_s = current_time_s;
                    rec.vehicle_time_s = current_time_s - this->start_time_sec;
                    rec.vehicle_name = ant.name;
                    rec.has_link_grant = has_grant;
                    rec.distance_m = ant_metrics_list[i].distance;
                    rec.rssi_dBm = ant_metrics_list[i].best_rssi;
                    rec.throughput_Gbps = ant_metrics_list[i].throughput;
                    rec.total_data_MB = ant.total_data_transmitted;
                    rec.path_loss_dB = ant_metrics_list[i].path_loss;
                    rec.e_gain_dB = ant_metrics_list[i].e_gain;
                    rec.h_gain_dB = ant_metrics_list[i].h_gain;
                    rec.comm_active = ant.comm_active;
                    rec.link_state = ant.link_state;
                    rec.in_main_lobe = ant_metrics_list[i].in_main_lobe;
                    rec.off_boresight_e_deg = ant_metrics_list[i].off_boresight_e;
                    rec.off_boresight_h_deg = ant_metrics_list[i].off_boresight_h;
                    rec.link_los = ant_metrics_list[i].is_los;
                    rec.blockage_loss_dB = ant_metrics_list[i].blockage_loss_db;
                    rec.shadow_dB = ant_metrics_list[i].shadow_db;
                    rec.fading_dB = ant_metrics_list[i].fading_loss_db;
                    
                    rec.tx_x_m = pos.x();
                    rec.tx_y_m = pos.y();
                    rec.tx_z_m = pos.z();
                    
                    int bs_idx = ant.assigned_bs_idx;
                    if (bs_idx >= 0 && bs_idx < static_cast<int>(this->base_stations.size())) {
                        rec.bs_x_m = this->base_stations[bs_idx].position.x();
                        rec.bs_y_m = this->base_stations[bs_idx].position.y();
                        rec.bs_z_m = this->base_stations[bs_idx].position.z();
                    } else {
                        // find best BS for logging disconnected state
                        double max_rssi = -999.0;
                        int best_bs = 0;
                        for (size_t j = 0; j < this->base_stations.size(); ++j) {
                            if (all_ant_bs_metrics[i][j].best_rssi > max_rssi) {
                                max_rssi = all_ant_bs_metrics[i][j].best_rssi;
                                best_bs = j;
                            }
                        }
                        if (best_bs < static_cast<int>(this->base_stations.size())) {
                            rec.bs_x_m = this->base_stations[best_bs].position.x();
                            rec.bs_y_m = this->base_stations[best_bs].position.y();
                            rec.bs_z_m = this->base_stations[best_bs].position.z();
                        }
                    }

                    ant.log_records.push_back(rec);
                }
            }

            // 測定レポートの発行 (20Hz sim 目安、physics step 毎の発行は禁止)
            // P2P制約: grant中の割当ペアのみ (observe_all_pairs=true なら全ペア=理想観測の上限評価)
            if (do_report) {
                std::normal_distribution<double> meas_noise(0.0, this->report_noise_std_db);
                for (size_t i = 0; i < this->vehicle_antennas.size(); ++i) {
                    const auto &ant = this->vehicle_antennas[i];
                    for (size_t b = 0; b < this->base_stations.size(); ++b) {
                        bool granted_pair = grant_flags[i] &&
                                            ant.assigned_bs_idx == static_cast<int>(b);
                        if (!this->report_observe_all_pairs && !granted_pair) continue;
                        auto *entry = report_msg.add_reports();
                        entry->set_ant(static_cast<int>(i));
                        entry->set_bs(static_cast<int>(b));
                        entry->set_rssi_dbm(all_ant_bs_metrics[i][b].best_rssi +
                                            meas_noise(this->report_rng));
                        entry->set_link_state(granted_pair ? ant.link_state : "DISCONNECTED");
                        entry->set_mode((granted_pair && !ant.measure_only)
                                            ? comms_sim::msgs::DATA
                                            : comms_sim::msgs::MEASURE);
                    }
                }
                report_msg.set_t_sim(current_time_s);
                auto *vp = report_msg.mutable_vehicle_pos();
                vp->set_x(pos.x());
                vp->set_y(pos.y());
                vp->set_z(pos.z());
                this->report_pub.Publish(report_msg);
                this->next_report_time = current_time_s + this->report_period_s;
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
        
        VehicleMotionController motion_controller;
        SimulationLogger logger;

        std::atomic<bool> all_ready{false};
        bool mission_complete = false;
        int64_t last_ready_pub_time = 0;
        int64_t last_complete_pub_time = 0;

        gz::transport::Node node;
        gz::transport::Node::Publisher mission_complete_pub;
        gz::transport::Node::Publisher ready_pub;
        gz::transport::Node::Publisher mission_progress_pub;
        
        int64_t last_progress_pub_time = 0;

        bool comms_initialized = false;
        bool logs_saved = false;
        int logging_level = 1;
        std::string config_file_path = "";
        std::string scheduling_policy = "sequential";
        bool filter_main_lobe = true;
        double min_hold_time_s = 1.0;
        double switch_margin_db = 2.0;
        double proactive_grace_period_s = 0.5;
        double proactive_handover_score_threshold = -80.0;
        double heatmap_resolution_m = 0.2;
        double min_hold_distance_m = 0.0;
        int ff_max_pairs = -1;

        double comm_data_limit_mb = -1.0;
        double link_establishment_time_ms = 2.0;

        std::vector<AntennaInfo> vehicle_antennas;
        std::vector<BaseStationInfo> base_stations_cfg;
        std::vector<BaseStationInfo> base_stations;
        bool base_stations_located = false;

        CommsEnvironment comms_env;
        HandoverScheduler scheduler;

        int active_antenna_idx = 0;
        std::string active_antenna_name;
        double last_switch_time_s = 0.0;
        bool switching_active = false;
        double start_time_sec = -1.0;
        std::vector<LutEntry> lut;

        std::vector<EventRecord> event_logs;
        std::vector<ControlRecord> control_logs;

        // Cached run details
        double config_y_pos = 0.0;
        double config_angle = 0.0;
        std::string config_summary_filename = "sweep_summary.csv";
        std::string config_output_subdir = "";
        std::string config_output_dir = "/workspace/sim_results/";

        std::mutex logs_mutex;
        BlockageEnvironment blockage_env;

        // 測定レポート出力 (データプレーンI/O)
        bool report_enabled = true;
        bool report_observe_all_pairs = false;  // false = P2P制約に忠実 (grantペアのみ)
        double report_period_s = 0.05;
        double report_noise_std_db = 2.0;
        std::string report_topic = "/comms/measurement_report";
        double next_report_time = 0.0;
        std::mt19937 report_rng{123};
        gz::transport::Node::Publisher report_pub;

        // 外部スケジュール実行戦略 (scheduling_policy == "external_schedule" 時のみ)
        std::shared_ptr<ExternalScheduleStrategy> external_strategy;
    };
} // namespace tx_controller

GZ_ADD_PLUGIN(
    tx_controller::TxControllerPlugin,
    gz::sim::System,
    tx_controller::TxControllerPlugin::ISystemConfigure,
    tx_controller::TxControllerPlugin::ISystemPreUpdate
)
GZ_ADD_PLUGIN_ALIAS(tx_controller::TxControllerPlugin, "tx_controller::TxControllerPlugin")
