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
#include <map>
#include <condition_variable>
#include <cstdlib>
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

    /**
     * BsOccupancyRegistry
     * -------------------
     * 複数Tx車両のプラグインインスタンス間で共有するBS占有台帳 (プロセス内 singleton)。
     * P2P (802.15.3e ペアネット) では1つのBSデバイスは同時に1つのペアネットにしか
     * 参加できないため、他車が占有中のBSへの grant は物理的に成立しない。
     * MEASURE (測定専用) ペアネットも同様にBSを占有する。
     */
    class BsOccupancyRegistry {
    public:
        static BsOccupancyRegistry& Instance() {
            static BsOccupancyRegistry inst;
            return inst;
        }

        /// bs が owner にとって利用可能か (空き or 自分が占有中) を副作用なく確認する。
        /// greedy_fcfs が「空きBSへのフォールバック」判定に使う (Sync は占有を伴い
        /// 探索途中で自分の既得BSを解放してしまうため、選択前の照会には使えない)。
        bool Available(int bs, const std::string& owner) {
            std::lock_guard<std::mutex> lock(this->mtx);
            auto it = this->owner_by_bs.find(bs);
            return it == this->owner_by_bs.end() || it->second == owner;
        }

        /// owner の占有を desired_bs のみに同期する (他の占有は解放)。
        /// desired_bs<0 は全解放。占有に成功(既得含む)したら true。
        bool Sync(const std::string& owner, int desired_bs) {
            std::lock_guard<std::mutex> lock(this->mtx);
            for (auto it = this->owner_by_bs.begin(); it != this->owner_by_bs.end();) {
                if (it->second == owner && it->first != desired_bs) it = this->owner_by_bs.erase(it);
                else ++it;
            }
            if (desired_bs < 0) return false;
            auto it = this->owner_by_bs.find(desired_bs);
            if (it == this->owner_by_bs.end()) {
                this->owner_by_bs[desired_bs] = owner;
                return true;
            }
            return it->second == owner;
        }

    private:
        std::mutex mtx;
        std::map<int, std::string> owner_by_bs;
    };

    /**
     * GeoAssignmentRegistry
     * ---------------------
     * geo_optimal arm の中央調停 (プロセス内 singleton)。
     *
     * 各車のプラグインが「理論RSSI (距離減衰 + 両端指向性のみ。遮蔽・シャドウを
     * 見ない) の効用ベクトル」を毎 tick 投函し、P2P排他制約下で総効用最大の
     * 割当を返す。事前計算 LUT は使わない — 車速域では位置が毎 tick 得られ、
     * 理論RSSI は閉形式で即計算できるため、オンライン計算で足りる。
     *
     * 最適化: RSU 数は小さい (4) ので、RSU 集合のビットマスク DP で
     * **厳密な最大重みマッチング**を解く。O(車数 × 2^RSU数 × RSU数)。
     * ハンガリアン法より実装が単純で、同じ最適解を与える。
     *
     * 時間軸: 各 tick を独立に解く瞬時最適 (先読みなし)。T_est (2ms) が RSU 通過
     * 時間 (~0.6s) より十分小さいため、切替コストを織り込んだ系列最適との差は小さい。
     *
     * gz-sim は各物理ステップで全プラグインを単一スレッド逐次実行するため、
     * 解は「エポックが進んだ最初の問い合わせ」で一度だけ計算され、同 tick の
     * 残りの車はその解を読む。投函値は最大 1 tick (5ms = 8cm 相当) 古いが、
     * 全車が同じスナップショットを見るため割当の整合性は保たれる。
     */
    class GeoAssignmentRegistry {
    public:
        static GeoAssignmentRegistry& Instance() {
            static GeoAssignmentRegistry inst;
            return inst;
        }

        /// 効用を投函し、自車に割り当てられた BS を返す (-1 = 割当なし)。
        /// utilities[b] = BS b の理論RSSI [dBm]。圏外/不適格は -1e9 未満にすること。
        int Resolve(const std::string& owner, double t,
                    const std::vector<double>& utilities) {
            std::lock_guard<std::mutex> lock(this->mtx);
            this->posts[owner] = {t, utilities};

            if (t > this->epoch_t + 1e-9) {   // 新しい tick: 解き直す
                this->epoch_t = t;
                this->Solve(t);
            }
            auto it = this->assignment.find(owner);
            return it == this->assignment.end() ? -1 : it->second;
        }

    private:
        struct Post {
            double t = -1.0;
            std::vector<double> utilities;
        };

        /// ビットマスク DP による厳密な最大重みマッチング。
        /// 投函が古い車 (コリドーを出た等) は候補から外す。
        void Solve(double now) {
            const double kStale = 0.5;      // これ以上古い投函は無効 [s]
            const double kMinUtil = -1e8;   // これ未満は割当不可

            std::vector<const std::string*> names;
            std::vector<const std::vector<double>*> utils;
            std::size_t n_bs = 0;
            for (const auto& kv : this->posts) {     // std::map = 名前順 = 決定論
                if (now - kv.second.t > kStale) continue;
                names.push_back(&kv.first);
                utils.push_back(&kv.second.utilities);
                n_bs = std::max(n_bs, kv.second.utilities.size());
            }
            this->assignment.clear();
            if (names.empty() || n_bs == 0) return;

            const int B = static_cast<int>(n_bs);
            const int full = 1 << B;
            const int V = static_cast<int>(names.size());
            const double kNeg = -1e18;

            // dp[k][mask] = 先頭 k 台までで mask の RSU を使ったときの最大総効用
            std::vector<std::vector<double>> dp(V + 1, std::vector<double>(full, kNeg));
            std::vector<std::vector<int>> choice(V, std::vector<int>(full, -2));
            dp[0][0] = 0.0;
            for (int k = 0; k < V; ++k) {
                const auto& u = *utils[k];
                for (int mask = 0; mask < full; ++mask) {
                    if (dp[k][mask] <= kNeg) continue;
                    // この車を割り当てない
                    if (dp[k][mask] > dp[k + 1][mask]) {
                        dp[k + 1][mask] = dp[k][mask];
                        choice[k][mask] = -1;
                    }
                    // BS b に割り当てる
                    for (int b = 0; b < B && b < static_cast<int>(u.size()); ++b) {
                        if (mask & (1 << b)) continue;
                        if (u[b] < kMinUtil) continue;
                        int nm = mask | (1 << b);
                        double val = dp[k][mask] + u[b];
                        if (val > dp[k + 1][nm]) {
                            dp[k + 1][nm] = val;
                            choice[k][nm] = b;
                        }
                    }
                }
            }
            int best_mask = 0;
            double best_val = kNeg;
            for (int mask = 0; mask < full; ++mask) {
                if (dp[V][mask] > best_val) { best_val = dp[V][mask]; best_mask = mask; }
            }
            // 復元
            int mask = best_mask;
            for (int k = V - 1; k >= 0; --k) {
                int c = choice[k][mask];
                if (c >= 0) {
                    this->assignment[*names[k]] = c;
                    mask &= ~(1 << c);
                }
            }
        }

        std::mutex mtx;
        std::map<std::string, Post> posts;
        std::map<std::string, int> assignment;
        double epoch_t = -1.0;
    };

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
                    this->config_output_subdir = logger_config.output_subdir;
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
            // 通信計算の間引き周期 [s] (0 = 物理ステップ毎 = 従来動作)。物理は
            // 1kHz でもチャネル評価 (指向性補間 + 遮蔽OBB) はその精度を要さず、
            // ここが多車両×多遮蔽体で RTF のボトルネックになる。間引き時は
            // 経過sim時間を実効dtとしてデータ会計・リンク確立に用いるため、
            // report_period_s 以下に保てば観測レポート周期は変わらない。
            this->comms_update_period_s = comms_params["comms_update_period_s"].as<double>(0.0);
            // コリドーゲーティング (0 = 無効 = 全ペア評価 = 後方互換)
            this->link_eval_radius_m = comms_params["link_eval_radius_m"].as<double>(0.0);

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
            // assoc_hold (802.15.3e 準拠の受動接続): リンク監視の回復待機時間 [s]。
            // 保持中アソシの RSSI が閾値を割ってからこの時間内に回復すれば同じ RSU で
            // 再開 (再アソシ無し)、超えたら断を宣言し次の圏内 RSU へ再アソシ。
            // 掃引可能 (単一値でも複数条件でも sweep が上書きする)
            this->assoc_recover_timeout_s = link_ctrl_params["assoc_recover_timeout_s"].as<double>(1.0);

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

            // 決定論RSSIプロファイルの出力 (制御プレーンの事前地図 = 事前測量相当。
            // LUTと同じ決定論チャネルで全ペアのRSSI(経路位置)を書き出し、
            // KKF はこれを平均関数として偏差のみを学習する)
            if (link_ctrl_params["export_rssi_profile"].as<bool>(false)) {
                this->ExportRssiProfile(local_waypoints, current_model_name);
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

            // 全ペア RSSI の記録 (通信計算を事後に回すため)。
            // 評価されるリンクの集合と順序は幾何だけで決まり、割当の判断には
            // 依存しない。したがって RSSI は一度記録すれば手法をまたいで
            // 使い回せる。フェージングは状態を持つ RNG なので Python で
            // 再現するのは非現実的だが、記録してしまえば再現の必要がない
            if (comms_params && comms_params["record_pairs_dir"]) {
                const std::string dir = comms_params["record_pairs_dir"].as<std::string>("");
                if (!dir.empty()) {
                    std::filesystem::create_directories(dir);
                    this->pairs_path = dir + "/" + this->model.Name(_ecm) + "_pairs.csv";
                }
            }

            // 制御プレーンとのシム時刻同期 (lockstep)。実時間非依存にする
            if (link_ctrl_params && link_ctrl_params["kkf_lockstep"]) {
                this->lockstep = link_ctrl_params["kkf_lockstep"].as<bool>(false);
            }
            if (link_ctrl_params && link_ctrl_params["kkf_replan_period_s"]) {
                this->lockstep_epoch_s =
                    link_ctrl_params["kkf_replan_period_s"].as<double>(this->lockstep_epoch_s);
            }
            if (link_ctrl_params && link_ctrl_params["kkf_lockstep_timeout_s"]) {
                this->lockstep_timeout_s =
                    link_ctrl_params["kkf_lockstep_timeout_s"].as<double>(this->lockstep_timeout_s);
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

        /**
         * 決定論チャネル (距離減衰+アンテナ利得のみ) での全ペアRSSIを経路に沿って
         * サンプリングし CSV 出力する。制御プレーンが事前地図 (平均関数) として読む。
         * 事前測量 (サイトサーベイ) に相当し、遮蔽体・シャドウイング・フェージングは
         * 含まない (LUT の事前知識と同一 = 公平比較)。
         */
        void ExportRssiProfile(const std::vector<Waypoint>& waypoints_to_use,
                               const std::string& model_name_str) {
            std::vector<Eigen::Vector3d> polyline_points;
            for (const auto &wp : waypoints_to_use) {
                polyline_points.push_back(Eigen::Vector3d(wp.x, wp.y, wp.z));
            }
            double res = std::max(0.5, this->heatmap_resolution_m);
            auto samples = utils::sample_trajectory(polyline_points, res);
            if (samples.empty()) return;

            std::string subdir = this->config_output_subdir.empty()
                                     ? std::string("default") : this->config_output_subdir;
            std::string dir = "/workspace/sim_results/" + subdir + "/rssi_profiles";
            try {
                std::filesystem::create_directories(dir);
            } catch (const std::exception& e) {
                std::cerr << "[TxControllerPlugin] rssi_profiles dir error: " << e.what() << std::endl;
                return;
            }
            std::string path = dir + "/" + model_name_str + "_profile.csv";
            std::ofstream ofs(path);
            if (!ofs.is_open()) return;

            size_t num_tx = this->vehicle_antennas.size();
            size_t num_rx = this->base_stations_cfg.size();
            ofs << "px,py,pz";
            for (size_t a = 0; a < num_tx; ++a)
                for (size_t b = 0; b < num_rx; ++b)
                    ofs << ",rssi_" << a << "_" << b;
            ofs << "\n";

            for (const auto &sample : samples) {
                Eigen::Matrix3d rot = utils::rpy_to_rotmat(0.0, 0.0, sample.yaw);
                auto all_metrics = this->comms_env.CalculateMetrics(
                    this->vehicle_antennas, this->base_stations_cfg, sample.pos, rot);
                ofs << std::fixed << std::setprecision(3)
                    << sample.pos.x() << "," << sample.pos.y() << "," << sample.pos.z();
                for (size_t a = 0; a < num_tx; ++a)
                    for (size_t b = 0; b < num_rx; ++b)
                        ofs << "," << all_metrics[a][b].best_rssi;
                ofs << "\n";
            }
            ofs.close();
            gzmsg << "[TxControllerPlugin] Exported deterministic RSSI profile ("
                  << samples.size() << " pts) to " << path << std::endl;
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
            {   // lockstep: 「いつの状態から作られた計画か」を記録して待機側を起こす
                std::lock_guard<std::mutex> lk(this->lockstep_mutex);
                this->latest_sched_issued_t = std::max(this->latest_sched_issued_t,
                                                       msg.t_issued());
            }
            this->lockstep_cv.notify_all();

            // 計画が1件も無いメッセージは「今エポックは言うことがない」という
            // 合図として扱い、現在の割当には触れない。lockstep ではプラグインが
            // 毎エポック応答を待つため、空エポックでも配信が要る。
            // 「あなたには割り当てない」(他車向けの計画は入っている) 場合は
            // 従来どおり自車の計画を消す — 両者は意味が違う
            if (msg.plan_size() == 0) return;

            ExternalScheduleStrategy::Schedule schedule;
            schedule.valid_until = msg.valid_until();
            for (const auto &entry : msg.plan()) {
                // 複数Tx車両: vehicle が指定されたエントリは該当車両のみが実行する
                // (空文字は全車両向け = 単一車両構成の後方互換)
                if (!entry.vehicle().empty() && entry.vehicle() != this->model_name) continue;
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

            // 通信計算の間引き (comms_update_period_s > 0)。実効 dt = 前回comms
            // 更新からの経過sim時間とし、データ会計 (throughput·dt) とリンク確立
            // (T_est/dt ステップ) の総量が物理ステップ毎と一致するようにする。
            //
            // 判定はグローバルなグリッド (k·周期) で行う。前回時刻からの経過で
            // 判定すると、各車の初回更新時刻が起点になって位相が車ごと・走行ごと
            // にずれる。実測で、同一シードの2走行で車両の通信更新時刻が 1-3ms
            // ずれ、時刻を鍵にした突き合わせが全く成立しなかった。
            // 全車が同一グリッドに乗れば、記録した RSSI を手法をまたいで
            // 再利用できる
            if (this->comms_update_period_s > 0.0) {
                const double period = this->comms_update_period_s;
                const double grid_idx = std::floor(current_time_s / period + 1e-9);
                if (grid_idx <= this->last_comms_grid_idx) return;
                if (this->last_comms_time_s >= 0.0) {
                    dt = current_time_s - this->last_comms_time_s;
                }
                this->last_comms_grid_idx = grid_idx;
                this->last_comms_time_s = current_time_s;
            }

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

            // 1b. コリドーゲーティング (車両単位の早期打ち切り)。
            // 全 RSU が評価半径の外なら、この tick の通信処理をまるごと省く。
            // ペア単位のスキップだけでは BlockageEnvironment::Refresh (遮蔽体ごとの
            // ECM pose 取得 + 回転行列) が「車両数 × 遮蔽体数」で残り、連続交通流では
            // ここが支配的になるため、車両単位で先に落とす。
            //
            // 安全性: 半径は接続閾値の到達距離より十分大きく取る規約なので、grant を
            // 保持したまま半径外に出ることは起こらない。念のため未割当のときだけ省く。
            if (this->link_eval_radius_m > 0.0) {
                bool any_in_range = false;
                for (const auto& bs : this->base_stations) {
                    Eigen::Vector3d bs_ant = bs.position + bs.rotmat * bs.antenna_offset;
                    if ((pos - bs_ant).squaredNorm()
                            <= this->link_eval_radius_m * this->link_eval_radius_m) {
                        any_in_range = true;
                        break;
                    }
                }
                // 記録モードでは grant を見ない。grant 保持の有無で評価する
                // ステップ数が変わると、フェージング (状態を持つ共有 RNG) の
                // 消費もずれ、RSSI そのものが手法によって変わってしまう
                // (実測: 同一シードの2手法で最大 20 dB 差、9万行が片側のみ)。
                // 記録は「幾何だけで決まる正準なチャネル実現」を作るのが目的
                bool holds_grant = false;
                if (this->pairs_path.empty()) {
                    for (const auto& ant : this->vehicle_antennas) {
                        if (ant.assigned_bs_idx >= 0) { holds_grant = true; break; }
                    }
                }
                if (!any_in_range && !holds_grant) return;
            }

            // 2. Compute comms metrics for all antennas (動的チャネル: 遮蔽・シャドウ・フェージング)
            // 自車も遮蔽体として登録され得る (role: tx + blockage 属性) ため、
            // 自分の OBB は障害物リストから除外する。除外しないと自分の箱で
            // 自分のリンクが常時 NLOS になる
            this->blockage_env.Refresh(_ecm, this->model_name);
            std::vector<std::vector<AntennaMetrics>> all_ant_bs_metrics = this->comms_env.CalculateMetrics(
                this->vehicle_antennas, this->base_stations, pos, vehicle_rotmat,
                current_time_s, &this->blockage_env.Obstacles(),
                this->link_eval_radius_m);

            this->RecordPairs(current_time_s, all_ant_bs_metrics);

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

            // greedy_fcfs (先着順・空きBS貪欲): LUTも予測も持たない真の下限。
            // レジストリ上で「空いている(他車未占有)」BSのうち現在RSSIが最良の
            // ものを掴む。現BSがまだ閾値以上かつ保持中なら維持し (反応的、
            // proactive HOなし)、閾値を割ったら別の空きBSへ繋ぎ直す。
            // 車間調停なし・実測RSSIのみ = trend(調停あり)/lut(静的最適)の下に位置する。
            // gz-sim は各物理ステップで全プラグインを単一スレッド逐次実行するため、
            // 「先着」= エンティティ実行順で決定的に定まる。
            if (this->scheduling_policy == "greedy_fcfs") {
                double rmin = this->comms_env.GetRssiMin();
                auto& reg = BsOccupancyRegistry::Instance();
                int n_bs = static_cast<int>(this->base_stations.size());
                for (size_t i = 0; i < this->vehicle_antennas.size(); ++i) {
                    auto& ant = this->vehicle_antennas[i];
                    int cur = ant.assigned_bs_idx;
                    bool keep = (cur >= 0 && cur < n_bs
                                 && all_ant_bs_metrics[i][cur].best_rssi >= rmin
                                 && reg.Available(cur, this->model_name));
                    if (!keep) {
                        int best_bs = -1;
                        double best_rssi = -1e9;
                        for (int b = 0; b < n_bs; ++b) {
                            if (!reg.Available(b, this->model_name)) continue;  // 他車占有
                            double r = all_ant_bs_metrics[i][b].best_rssi;
                            if (r >= rmin && r > best_rssi) { best_rssi = r; best_bs = b; }
                        }
                        ant.assigned_bs_idx = best_bs;  // -1 = 全て占有/圏外なら未接続
                    }
                    if (ant.assigned_bs_idx >= 0) this->active_antenna_idx = static_cast<int>(i);
                }
            }

            // assoc_hold (802.15.3e 準拠の受動接続、HOなし): 最初に接続閾値を超えた
            // 空き RSU にアソシし、閾値以上の間は保持 (より良い RSU へ能動的に移らない)。
            // 閾値割れ (遮蔽等) は即断とせず回復待機に入り、assoc_recover_timeout_s 以内に
            // 回復すれば同じ RSU で再開、超えたら断を宣言し次の圏内 RSU へ再アソシする。
            // greedy_fcfs との違いは「保持中アソシを能動的に手放さない」点で、遮蔽区間を
            // 回避せずアウテージとして食う (=HOしないことの代償が指標に出る)。
            if (this->scheduling_policy == "assoc_hold") {
                double rmin = this->comms_env.GetRssiMin();
                auto& reg = BsOccupancyRegistry::Instance();
                int n_bs = static_cast<int>(this->base_stations.size());
                for (size_t i = 0; i < this->vehicle_antennas.size(); ++i) {
                    auto& ant = this->vehicle_antennas[i];
                    int cur = ant.assigned_bs_idx;
                    if (cur >= 0 && cur < n_bs) {
                        if (all_ant_bs_metrics[i][cur].best_rssi >= rmin) {
                            ant.outage_start_time = -1.0;   // 健全: 保持、回復待機解除
                        } else {
                            // 閾値割れ: 回復待機。T_recover 以内は同じ RSU を保持
                            if (ant.outage_start_time < 0.0)
                                ant.outage_start_time = current_time_s;
                            if (current_time_s - ant.outage_start_time
                                    > this->assoc_recover_timeout_s) {
                                ant.assigned_bs_idx = -1;   // タイムアウト: 断→再アソシ
                                ant.outage_start_time = -1.0;
                            }
                            // else: cur を据え置き (アウテージ計上)
                        }
                    }
                    // 未接続 (初回 or タイムアウト後): 最初に閾値を超えた空き RSU に受動アソシ。
                    // 最良ではなく先頭 (index 順) から採るため能動選択にならない
                    if (ant.assigned_bs_idx < 0) {
                        for (int b = 0; b < n_bs; ++b) {
                            if (!reg.Available(b, this->model_name)) continue;
                            if (all_ant_bs_metrics[i][b].best_rssi >= rmin) {
                                ant.assigned_bs_idx = b;
                                ant.outage_start_time = -1.0;
                                break;
                            }
                        }
                    }
                    if (ant.assigned_bs_idx >= 0) this->active_antenna_idx = static_cast<int>(i);
                }
            }

            // geo_optimal (幾何最適・オンライン): 遮蔽を考慮しない理論RSSI
            // (距離減衰 + 両端指向性のみ) を毎 tick 計算し、中央調停が P2P排他下で
            // 総効用最大の割当を返す。情報制約 = 「全車の位置・姿勢とアンテナ
            // パターンは既知、伝搬の確率成分 (遮蔽・シャドウ) は未知」。
            // 事前計算 LUT は使わない (車速域ではオンライン計算で足りる)。
            if (this->scheduling_policy == "geo_optimal") {
                double rmin = this->comms_env.GetRssiMin();
                int n_bs = static_cast<int>(this->base_stations.size());
                // sim_time < 0 = 決定論モード (遮蔽・シャドウ・フェージングを評価しない)
                auto geo = this->comms_env.CalculateMetrics(
                    this->vehicle_antennas, this->base_stations, pos, vehicle_rotmat,
                    -1.0, nullptr, this->link_eval_radius_m);

                // BS ごとに最良アンテナを代表に取り、効用ベクトルを組む。
                // 効用は「接続閾値からのマージン (rssi - rmin) [dB]」= 常に非負。
                // 生の RSSI (dBm, 負値) を使うと総和最大化が「誰も割り当てない
                // (合計 0)」を選ぶ縮退解に陥るため。マージンなら接続数を最大化し、
                // 同数の中では RSSI 順で最良の組合せになる (順序は RSSI と同一)。
                // レート基準にしたい場合は RateGbps に差し替えるだけでよい。
                std::vector<double> util(n_bs, -1e9);
                std::vector<int> util_ant(n_bs, 0);
                for (int b = 0; b < n_bs; ++b) {
                    for (size_t i = 0; i < this->vehicle_antennas.size(); ++i) {
                        double r = geo[i][b].best_rssi;
                        if (r >= rmin && (r - rmin) > util[b]) {
                            util[b] = r - rmin;
                            util_ant[b] = static_cast<int>(i);
                        }
                    }
                }
                int bs = GeoAssignmentRegistry::Instance().Resolve(
                    this->model_name, current_time_s, util);

                for (auto& ant : this->vehicle_antennas) ant.assigned_bs_idx = -1;
                if (bs >= 0 && bs < n_bs) {
                    int ai = util_ant[bs];
                    this->vehicle_antennas[ai].assigned_bs_idx = bs;
                    this->active_antenna_idx = ai;
                }
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
            // lockstep ではこの直後にエポック境界で停止する。停止前に必ず観測を
            // 出しておかないと、シム時刻が凍結した状態で制御プレーンが新しい
            // 入力を得られず、待ち合わせが永久に成立しない
            const bool at_epoch = this->lockstep && this->external_strategy
                                  && this->lockstep_epoch_s > 0.0
                                  && current_time_s + 1e-9 >= this->next_lockstep_epoch;
            bool do_report = this->report_enabled
                             && (current_time_s >= this->next_report_time || at_epoch);
            comms_sim::msgs::MeasurementReport report_msg;
            std::vector<bool> grant_flags(this->vehicle_antennas.size(), false);

            bool any_grant_synced = false;
            for (size_t i = 0; i < this->vehicle_antennas.size(); ++i) {
                auto &ant = this->vehicle_antennas[i];
                bool has_grant;
                if (this->scheduling_policy == "feedforward_optimal" ||
                    this->scheduling_policy == "simple_no_handover" ||
                    this->scheduling_policy == "greedy_fcfs" ||
                    this->scheduling_policy == "assoc_hold" ||
                    this->scheduling_policy == "geo_optimal") {
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

                // BS排他 (複数Tx車両): 他車が占有中のBSへの grant は成立しない。
                // 車両あたり1ペアネット前提 (本評価の全ポリシー) のため、複数grant時は
                // 最後の割当が優先される。
                if (has_grant && ant.assigned_bs_idx >= 0) {
                    has_grant = BsOccupancyRegistry::Instance().Sync(
                        this->model_name, ant.assigned_bs_idx);
                    any_grant_synced = true;
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

            // grant が1つも無いステップでは自車のBS占有を解放する
            if (!any_grant_synced) {
                BsOccupancyRegistry::Instance().Sync(this->model_name, -1);
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
                report_msg.set_vehicle(this->model_name);
                auto *vp = report_msg.mutable_vehicle_pos();
                vp->set_x(pos.x());
                vp->set_y(pos.y());
                vp->set_z(pos.z());
                this->report_pub.Publish(report_msg);
                this->last_report_msg = report_msg;
                this->has_last_report = true;
                // グリッドで決める (累積加算だと浮動小数の誤差が溜まり、
                // lockstep でエポック境界と観測時刻がずれる。ずれた回では
                // シム時刻を止めたまま新しい観測が出ず、制御プレーンが計画を
                // 作れないので待ち合わせが成立しない = タイムアウトまで硬直)
                this->next_report_time =
                    (std::floor(current_time_s / this->report_period_s) + 1.0)
                    * this->report_period_s;
            }

            this->WaitForControlEpoch(current_time_s);
        }

        /// \brief 制御プレーンの計画をシム時刻で待ち合わせる (lockstep)。
        ///
        /// 非同期のままだと、計画が「シム時刻の何時点で効き始めるか」が計算機の
        /// 混み具合で変わる。外部プロセスで動く手法だけが負荷で不利になり、
        /// 手法間比較が成立しない (実測: KKF ノード4個同時で grant -4.6%、
        /// プラグイン内で完結する assoc_hold は +2.6%)。
        ///
        /// エポック t_k = k·T ごとに、「t_{k-1} の状態から作られた計画」が届く
        /// まで PreUpdate を止める。シム時刻は進まないので、実時間がいくら
        /// かかっても計画が効き始めるシム時刻は必ず同じになる。
        /// 1エポック遅らせるのは、t_k の観測を全車が出し終える前に待つと
        /// デッドロックするため (待つ側が先に止まると後続が観測を出せない)。
        void WaitForControlEpoch(double current_time_s) {
            if (!this->lockstep || !this->external_strategy) return;
            if (this->lockstep_epoch_s <= 0.0) return;

            while (current_time_s + 1e-9 >= this->next_lockstep_epoch) {
                const double need = this->next_lockstep_epoch - this->lockstep_epoch_s;
                if (need >= 0.0) {
                    // シム時刻を止めている間は「待てば次の観測が来る」が成り立た
                    // ない。観測が1通でも取りこぼされると (108台×20Hz = 毎秒
                    // 2160通) 制御プレーンは永久に計画を出せず、回復不能な
                    // デッドロックになる。待機中は直近の観測を再送して復旧させる
                    bool got = false;
                    const double retry_s = 0.5;
                    double waited = 0.0;
                    while (waited < this->lockstep_timeout_s) {
                        {
                            std::unique_lock<std::mutex> lk(this->lockstep_mutex);
                            got = this->lockstep_cv.wait_for(
                                lk, std::chrono::duration<double>(retry_s),
                                [&]{ return this->latest_sched_issued_t >= need - 1e-9; });
                        }
                        if (got) break;
                        waited += retry_s;
                        if (this->has_last_report) {
                            this->report_pub.Publish(this->last_report_msg);
                            ++this->lockstep_resends;
                        }
                    }
                    if (!got) {
                        gzerr << "[TxControllerPlugin] lockstep timeout: t_sim="
                              << current_time_s << " epoch=" << need
                              << " latest_issued=" << this->latest_sched_issued_t
                              << " — 制御プレーンが応答しません。"
                              << "非決定的なデータを出さないため異常終了します。"
                              << std::endl;
                        std::exit(1);
                    }
                }
                this->next_lockstep_epoch += this->lockstep_epoch_s;
            }
        }

        /// \brief 全 (アンテナ, BS) ペアの RSSI を逐次書き出す。
        /// 1走行で数百万行になるためメモリには溜めない。
        void RecordPairs(double t,
                         const std::vector<std::vector<AntennaMetrics>>& metrics) {
            if (this->pairs_path.empty()) return;
            if (!this->pairs_ofs.is_open()) {
                this->pairs_ofs.open(this->pairs_path);
                if (!this->pairs_ofs) { this->pairs_path.clear(); return; }
                this->pairs_ofs << "t_s,ant,bs,rssi_dBm,los\n";
                this->pairs_ofs << std::fixed << std::setprecision(4);
            }
            for (size_t i = 0; i < metrics.size(); ++i) {
                for (size_t b = 0; b < metrics[i].size(); ++b) {
                    const auto& m = metrics[i][b];
                    this->pairs_ofs << t << ',' << i << ',' << b << ','
                                    << m.best_rssi << ',' << (m.is_los ? 1 : 0)
                                    << '\n';
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

        double last_comms_grid_idx = -1.0;   // 全車共通のグリッド番号
        // 通信計算の間引き (0 = 物理ステップ毎)。last_comms_time_s は実効dt算出用
        double comms_update_period_s = 0.0;
        double link_eval_radius_m = 0.0;
        double last_comms_time_s = -1.0;
        double assoc_recover_timeout_s = 1.0;  // assoc_hold: リンク監視の回復待機時間 [s]

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

        // 全ペア RSSI の逐次記録
        std::string pairs_path;
        std::ofstream pairs_ofs;

        // 制御プレーンとのシム時刻同期 (lockstep)
        bool lockstep = false;
        double lockstep_epoch_s = 0.2;
        double lockstep_timeout_s = 300.0;
        double next_lockstep_epoch = 0.0;
        double latest_sched_issued_t = -1e18;
        comms_sim::msgs::MeasurementReport last_report_msg;
        bool has_last_report = false;
        long lockstep_resends = 0;      // 再送回数 (取りこぼしの実測)
        std::mutex lockstep_mutex;
        std::condition_variable lockstep_cv;
    };
} // namespace tx_controller

GZ_ADD_PLUGIN(
    tx_controller::TxControllerPlugin,
    gz::sim::System,
    tx_controller::TxControllerPlugin::ISystemConfigure,
    tx_controller::TxControllerPlugin::ISystemPreUpdate
)
GZ_ADD_PLUGIN_ALIAS(tx_controller::TxControllerPlugin, "tx_controller::TxControllerPlugin")
