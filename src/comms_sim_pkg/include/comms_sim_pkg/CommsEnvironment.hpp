#pragma once

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "comms_sim_pkg/DataTypes.hpp"
#include "comms_sim_pkg/antenna_pattern_parser.hpp"
#include "comms_sim_pkg/comms_calculator.hpp"
#include "comms_sim_pkg/channel/ChannelModel.hpp"
#include "comms_sim_pkg/Utils.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>

namespace tx_controller
{
    class CommsEnvironment {
    public:
        CommsEnvironment() {}

        void Configure(const std::string& config_file_path) {
            YAML::Node config = YAML::LoadFile(config_file_path);
            auto comms_params = config["comms_simulator_node"]["ros__parameters"];
            
            double tx_power = comms_params["tx_power"].as<double>(-7.0);
            double noise_variance = comms_params["noise_variance"].as<double>(0.0);
            std::string mcs_table_path = tx_controller::utils::resolve_path(comms_params["mcs_table_path"].as<std::string>(""));
            
            double max_antenna_attenuation = comms_params["max_antenna_attenuation"].as<double>(30.0);
            double mainlobe_angle_margin_deg = comms_params["mainlobe_angle_margin_deg"].as<double>(5.0);
            double mainlobe_e_half_angle_deg = comms_params["mainlobe_e_half_angle_deg"].as<double>(-1.0);
            double mainlobe_h_half_angle_deg = comms_params["mainlobe_h_half_angle_deg"].as<double>(-1.0);
            
            std::string e_plane_path = tx_controller::utils::resolve_path(comms_params["e_plane_path"].as<std::string>(""));
            std::string h_plane_path = tx_controller::utils::resolve_path(comms_params["h_plane_path"].as<std::string>(""));

            auto pl = comms_params["path_loss"];

            auto prop_model = comms_sim::PropagationModelFactory::Create(pl);

            // レート写像 (RSSI→Mbps)。rate_model: 未指定なら MCS テーブル (旧互換)。
            auto rate_model = comms_sim::RateModelFactory::Create(
                comms_params["rate_model"], mcs_table_path);
            this->comms_calculator = std::make_unique<comms_sim::CommsCalculator>(
                std::move(prop_model), tx_power, noise_variance, std::move(rate_model));

            // 合成チャネル(遮蔽・シャドウイング・フェージング)。channel: 未指定なら
            // 距離減衰のみと等価に振る舞う(後方互換)。
            this->channel_model = comms_sim::ChannelModelFactory::Create(comms_params);

            this->antenna_parser = std::make_unique<comms_sim::AntennaPatternParser>(
                max_antenna_attenuation, mainlobe_angle_margin_deg,
                mainlobe_e_half_angle_deg, mainlobe_h_half_angle_deg);
                
            if (!e_plane_path.empty()) this->antenna_parser->load_e_plane(e_plane_path);
            if (!h_plane_path.empty()) this->antenna_parser->load_h_plane(h_plane_path);
        }

        /**
         * 全 (車載アンテナ × 基地局) ペアの通信メトリクスを計算する。
         *
         * @param sim_time  シミュレーション時刻 [s]。負値なら動的チャネル
         *                  (遮蔽・シャドウイング・フェージング) を評価しない
         *                  「決定論モード」(LUT事前計算=神託が使用)。
         * @param obstacles 遮蔽体リスト (nullptr 可)。
         * @param eval_radius_m  この距離を超えるペアは評価をスキップし、
         *   AntennaMetrics の既定値 (best_rssi = -999) を返す。0 以下 = 無効
         *   (= 全ペア評価 = 後方互換)。
         *
         *   連続交通流では待機中の車も含めて全 tick で全 RSU 分を評価するため、
         *   台数 × RSU 数に比例して計算量が増える (都市部2車線 over 条件で
         *   対象車 40 台 × RSU 4 台 = 160 リンク/tick)。RTF のボトルネックは
         *   物理ステップごとの全リンク チャネル評価 (指向性補間 + 遮蔽 OBB) だと
         *   判明しているので、閾値に届き得ない遠方ペアを先に落とす。
         *
         *   半径は「その距離では確実に接続閾値を割る」値に取ること。
         *   observe_all_pairs (oracle) ではスキップしたペアのレポートが
         *   -999 になるため、半径が近すぎると情報上界が壊れる。
         */
        std::vector<std::vector<AntennaMetrics>> CalculateMetrics(
            const std::vector<AntennaInfo>& vehicle_antennas,
            const std::vector<BaseStationInfo>& base_stations,
            const Eigen::Vector3d& vehicle_pos,
            const Eigen::Matrix3d& vehicle_rotmat,
            double sim_time = -1.0,
            const std::vector<comms_sim::ObstacleBox>* obstacles = nullptr,
            double eval_radius_m = 0.0)
        {
            std::vector<std::vector<AntennaMetrics>> all_metrics(vehicle_antennas.size());

            for (size_t i = 0; i < vehicle_antennas.size(); ++i) {
                const auto &ant = vehicle_antennas[i];
                Eigen::Vector3d ant_pos_world = vehicle_pos + vehicle_rotmat * ant.offset;
                Eigen::Vector3d ant_rpy = ant.relative_rpy;

                std::vector<AntennaMetrics> bs_metrics(base_stations.size());

                for (size_t bs_idx = 0; bs_idx < base_stations.size(); ++bs_idx) {
                    const auto &bs = base_stations[bs_idx];
                    Eigen::Vector3d bs_antenna_pos = bs.position + bs.rotmat * bs.antenna_offset;
                    Eigen::Vector3d bs_ant_rpy = bs.antenna_relative_rpy;

                    // コリドーゲーティング: 圏外確定のペアは指向性補間も
                    // チャネル評価もせず既定値 (best_rssi = -999) のまま残す
                    if (eval_radius_m > 0.0 &&
                        (ant_pos_world - bs_antenna_pos).squaredNorm()
                            > eval_radius_m * eval_radius_m) {
                        bs_metrics[bs_idx].bs_pos = bs_antenna_pos;
                        bs_metrics[bs_idx].distance =
                            (ant_pos_world - bs_antenna_pos).norm();
                        continue;
                    }

                    Eigen::Matrix3d ant_rotmat = vehicle_rotmat * tx_controller::utils::rpy_to_rotmat(ant_rpy.x(), ant_rpy.y(), ant_rpy.z());
                    // bs.rotmat は生成時点で pose_rpy + antenna_relative_rpy を合成済み（TxControllerPlugin参照）。
                    // ここで relative_rpy を再度掛けると二重適用になるため、そのまま使う。
                    Eigen::Matrix3d bs_full_rotmat = bs.rotmat;

                    auto gain_res = this->antenna_parser->get_tx_rx_gains(
                        ant_pos_world, ant_rpy, bs_antenna_pos, bs_ant_rpy,
                        &ant_rotmat, &bs_full_rotmat);

                    double total_gain = gain_res.tx_total + gain_res.rx_total;
                    comms_sim::CommsMetrics metrics_calc;
                    comms_sim::ChannelSample channel_sample;
                    bool dynamic_channel = (sim_time >= 0.0);
                    if (dynamic_channel) {
                        int link_id = static_cast<int>(i * base_stations.size() + bs_idx);
                        static const std::vector<comms_sim::ObstacleBox> kNoObstacles;
                        channel_sample = this->channel_model->Evaluate(
                            link_id, sim_time, ant_pos_world, bs_antenna_pos,
                            obstacles ? *obstacles : kNoObstacles);
                        metrics_calc = this->comms_calculator->calculate_from_loss(
                            (ant_pos_world - bs_antenna_pos).norm(),
                            channel_sample.TotalLossDb(), total_gain, false);
                    } else {
                        // 決定論モード: 距離減衰+アンテナ利得のみ (LUT事前計算用)
                        metrics_calc = this->comms_calculator->calculate_all(
                            ant_pos_world, bs_antenna_pos, total_gain, false);
                    }

                    auto [el, az] = this->antenna_parser->calculate_antenna_frame_angles(
                        ant_pos_world, bs_antenna_pos, ant_rpy, &ant_rotmat);
                    
                    double off_boresight_e_deg = std::abs(el * 180.0 / M_PI);
                    double off_boresight_h_deg = std::abs(az * 180.0 / M_PI);
                    bool in_main_lobe = this->antenna_parser->is_in_main_lobe(off_boresight_e_deg, off_boresight_h_deg);
                    
                    AntennaMetrics &m = bs_metrics[bs_idx];
                    m.distance = metrics_calc.distance;
                    m.best_rssi = metrics_calc.rssi;
                    m.throughput = metrics_calc.throughput;
                    m.path_loss = metrics_calc.path_loss;
                    if (dynamic_channel) {
                        m.is_los = channel_sample.is_los;
                        m.blockage_loss_db = channel_sample.blockage_loss_db;
                        m.shadow_db = channel_sample.shadow_db;
                        m.fading_loss_db = channel_sample.fading_loss_db;
                    }
                    m.e_gain = gain_res.tx_total;
                    m.h_gain = gain_res.rx_total;
                    m.in_main_lobe = in_main_lobe;
                    m.off_boresight_e = off_boresight_e_deg;
                    m.off_boresight_h = off_boresight_h_deg;
                    m.bs_pos = bs_antenna_pos;
                }
                all_metrics[i] = bs_metrics;
            }

            return all_metrics;
        }

        double GetRssiMin() const {
            if (this->comms_calculator) return this->comms_calculator->rssi_min;
            return -100.0;
        }

    private:
        std::unique_ptr<comms_sim::AntennaPatternParser> antenna_parser;
        std::unique_ptr<comms_sim::CommsCalculator> comms_calculator;
        std::unique_ptr<comms_sim::ChannelModel> channel_model;
    };
} // namespace tx_controller
