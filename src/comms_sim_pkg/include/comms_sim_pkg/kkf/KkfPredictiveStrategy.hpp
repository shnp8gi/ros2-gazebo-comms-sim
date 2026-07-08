#pragma once

#include <memory>
#include <sstream>
#include <iomanip>
#include <vector>
#include <Eigen/Dense>
#include "comms_sim_pkg/ISchedulingStrategy.hpp"
#include "comms_sim_pkg/SchedulingSupport.hpp"
#include "comms_sim_pkg/kkf/KkfConfig.hpp"
#include "comms_sim_pkg/kkf/RoadCoordinate.hpp"
#include "comms_sim_pkg/kkf/RadioEnvironmentMap.hpp"
#include "comms_sim_pkg/kkf/KkfMeasurementModel.hpp"
#include "comms_sim_pkg/kkf/HandoverPlanner.hpp"

namespace tx_controller
{
    namespace kkf
    {
        /**
         * KkfPredictiveStrategy
         * ---------------------
         * 責務: KKF予測駆動型ハンドオーバー方式(文書全体)の「編成」のみ。
         *  - 測定周期ごとに KkfMeasurementModel → RadioEnvironmentMap へ観測を流す(第1層)
         *  - 再計画周期ごとに予測系列のLCBを構成し HandoverPlanner で計画(第3層, MPC運用 5.3節)
         *  - 計画の先頭決定のみを実行し、SchedulingResult として返す
         * 統計・幾何・最適化の実体は全て注入/保持する専用モジュールに委譲しており、
         * 本クラス自身は数式を持たない。
         *
         * 真値メトリクス (all_ant_bs_metrics) は KkfMeasurementModel への入力と、
         * 物理リンク状態機械用の last_rssi 設定にのみ使用し、計画には使用しない。
         */
        class KkfPredictiveStrategy : public ISchedulingStrategy {
        public:
            KkfPredictiveStrategy(const KkfConfig& cfg,
                                  const RoadCoordinate& road,
                                  const std::vector<Eigen::Vector3d>& bs_antenna_positions)
                : cfg(cfg),
                  road(road),
                  map(road, bs_antenna_positions, cfg.kkf),
                  meas_model(cfg.meas_noise_std_db, cfg.observe_all_pairs, cfg.seed) {}

            std::string Name() const override { return "kkf_predictive"; }

            SchedulingResult UpdateLinks(
                double t,
                const Eigen::Vector3d& vehicle_pos,
                const Eigen::Matrix3d& vehicle_rotmat,
                std::vector<AntennaInfo>& vehicle_antennas,
                const std::vector<BaseStationInfo>& base_stations,
                const std::vector<std::vector<AntennaMetrics>>& all_ant_bs_metrics,
                int current_active_idx,
                double current_last_switch_time,
                double /*rssi_min*/) override
            {
                const int num_ant = static_cast<int>(vehicle_antennas.size());
                const int num_bs = static_cast<int>(base_stations.size());
                if (this->current_ant < 0) this->current_ant = current_active_idx;

                // 各車載アンテナの弧長座標(車体姿勢を反映した実位置の経路射影)
                std::vector<double> antenna_s(num_ant);
                for (int a = 0; a < num_ant; ++a) {
                    antenna_s[a] = this->road.Project(
                        vehicle_pos + vehicle_rotmat * vehicle_antennas[a].offset);
                }
                double s_vehicle = this->road.Project(vehicle_pos);

                SchedulingResult result;
                result.last_switch_time_s = current_last_switch_time;

                // --- 1. 測定レポート (第1層の観測更新) ---
                if (t >= this->next_meas_t) {
                    this->UpdateVelocityEstimate(t, s_vehicle);
                    auto per_bs_obs = this->meas_model.Sample(
                        t, all_ant_bs_metrics, antenna_s, this->current_ant, this->current_bs);
                    for (int b = 0; b < num_bs; ++b) {
                        this->map.Ingest(b, t, per_bs_obs[b]);
                    }
                    this->next_meas_t = t + this->cfg.measurement_period_s;
                }

                // --- 2. 再計画 (第3層, MPC: 毎周期引き直し先頭のみ実行) ---
                if (t >= this->next_plan_t) {
                    this->Replan(t, antenna_s, num_ant, num_bs, vehicle_antennas, result);
                    this->next_plan_t = t + this->cfg.replan_period_s;
                }

                // --- 3. 現在の割り当ての適用 (共通ヘルパ) ---
                scheduling::ApplyPairAssignment(vehicle_antennas, all_ant_bs_metrics,
                                                this->current_ant, this->current_bs);

                result.new_active_idx = this->current_ant;
                result.active_antenna_name = vehicle_antennas[this->current_ant].name;
                if (result.switching_active) {
                    result.last_switch_time_s = t;
                }
                return result;
            }

        private:
            /// 弧長速度のEMA推定(真の速度パラメータには依存しない)
            void UpdateVelocityEstimate(double t, double s_vehicle) {
                if (this->prev_meas_t >= 0.0 && t > this->prev_meas_t) {
                    double v_inst = (s_vehicle - this->prev_meas_s) / (t - this->prev_meas_t);
                    this->v_hat = this->has_velocity
                        ? this->cfg.velocity_ema_beta * this->v_hat +
                              (1.0 - this->cfg.velocity_ema_beta) * v_inst
                        : v_inst;
                    this->has_velocity = true;
                }
                this->prev_meas_t = t;
                this->prev_meas_s = s_vehicle;
            }

            /// LCB系列の構成(式(12)) → ビタビDP(式(13)) → 先頭決定の実行
            void Replan(double t, const std::vector<double>& antenna_s,
                        int num_ant, int num_bs,
                        std::vector<AntennaInfo>& vehicle_antennas,
                        SchedulingResult& result)
            {
                const int num_states = num_ant * num_bs;
                const int K = std::max(1, static_cast<int>(std::round(
                    this->cfg.horizon_s / this->cfg.plan_dt_s)));

                std::vector<std::vector<double>> lcb(num_states, std::vector<double>(K));
                for (int a = 0; a < num_ant; ++a) {
                    for (int b = 0; b < num_bs; ++b) {
                        for (int k = 0; k < K; ++k) {
                            double s_future = antenna_s[a] + this->v_hat * k * this->cfg.plan_dt_s;
                            double t_future = t + k * this->cfg.plan_dt_s;
                            auto pred = this->map.Predict(b, s_future, t_future);
                            lcb[a * num_bs + b][k] =
                                pred.mean - this->cfg.kappa * std::sqrt(pred.variance);
                        }
                    }
                }

                int initial_state = (this->current_bs >= 0)
                                        ? this->current_ant * num_bs + this->current_bs
                                        : -1;
                auto plan = HandoverPlanner::Solve(
                    lcb, this->cfg.plan_dt_s, this->cfg.switch_cost, initial_state);
                if (!plan.valid) return;

                int decided = plan.assignments.front();
                int new_ant = decided / num_bs;
                int new_bs = decided % num_bs;

                if (new_ant != this->current_ant || new_bs != this->current_bs) {
                    EventRecord ev;
                    ev.time_s = t;
                    ev.event_type = (new_ant != this->current_ant) ? "KKF_HANDOVER" : "KKF_BS_SWITCH";
                    ev.active_antenna = vehicle_antennas[new_ant].name;
                    ev.prev_antenna = (this->current_ant >= 0)
                                          ? vehicle_antennas[this->current_ant].name : "none";
                    std::ostringstream oss;
                    oss << "bs " << this->current_bs << " -> " << new_bs
                        << ", LCB " << std::fixed << std::setprecision(1)
                        << lcb[decided][0] << " dBm";
                    ev.details = oss.str();
                    result.events.push_back(ev);

                    // 同一アンテナでのBS切替はペアネット再確立を要する(802.15.3e P2P)ため
                    // リンク状態を明示的に落とし、確立遅延を物理状態機械に課す
                    if (new_ant == this->current_ant) {
                        vehicle_antennas[new_ant].link_state = "DISCONNECTED";
                    }
                    this->current_ant = new_ant;
                    this->current_bs = new_bs;
                    result.switching_active = true;
                }
            }

            KkfConfig cfg;
            RoadCoordinate road;
            RadioEnvironmentMap map;
            KkfMeasurementModel meas_model;

            int current_ant = -1;
            int current_bs = -1;
            double next_meas_t = -1e18;
            double next_plan_t = -1e18;
            double prev_meas_t = -1.0;
            double prev_meas_s = 0.0;
            double v_hat = 0.0;
            bool has_velocity = false;
        };
    } // namespace kkf
} // namespace tx_controller
