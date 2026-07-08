#pragma once

#include <random>
#include <vector>
#include "comms_sim_pkg/DataTypes.hpp"
#include "comms_sim_pkg/kkf/KrigedKalmanFilter.hpp"

namespace tx_controller
{
    namespace kkf
    {
        /**
         * KkfMeasurementModel
         * -------------------
         * 責務: シミュレータの真値メトリクスから「車両が実際に得られる測定レポート」を
         * 合成することのみ。KKF側で真値に触れてよいのはこのモジュールだけであり、
         * 予測・計画モジュールへの神託(ground truth)混入を構造的に遮断する境界である。
         *
         * P2P制約 (802.15.3e ペアネット、文書 6.1節):
         *  - observe_all_pairs = false : grant中の (アクティブアンテナ, 接続BS) ペアのみ観測(規格忠実)
         *  - observe_all_pairs = true  : 全ペア観測(理想観測の上限性能評価用)
         */
        class KkfMeasurementModel {
        public:
            KkfMeasurementModel(double noise_std_db, bool observe_all_pairs, unsigned seed)
                : noise_std_db(noise_std_db), observe_all_pairs(observe_all_pairs), rng(seed) {}

            /**
             * @param t                 現在時刻 [s]
             * @param true_metrics      真値メトリクス [antenna][bs]
             * @param antenna_s         各車載アンテナの弧長座標 s [m]
             * @param active_ant_idx    アクティブアンテナ (P2P制約時の観測元)
             * @param active_bs_idx     接続中BS (-1 = 未接続 → P2P制約時は観測なし)
             * @return                  BSごとの観測リスト [bs] -> observations
             */
            std::vector<std::vector<Observation>> Sample(
                double t,
                const std::vector<std::vector<AntennaMetrics>>& true_metrics,
                const std::vector<double>& antenna_s,
                int active_ant_idx,
                int active_bs_idx)
            {
                const size_t num_ant = true_metrics.size();
                const size_t num_bs = num_ant > 0 ? true_metrics[0].size() : 0;
                std::vector<std::vector<Observation>> per_bs(num_bs);
                std::normal_distribution<double> noise(0.0, this->noise_std_db);
                const double noise_var = this->noise_std_db * this->noise_std_db;

                for (size_t a = 0; a < num_ant; ++a) {
                    for (size_t b = 0; b < num_bs; ++b) {
                        if (!this->observe_all_pairs) {
                            if (static_cast<int>(a) != active_ant_idx ||
                                static_cast<int>(b) != active_bs_idx) continue;
                        }
                        Observation obs;
                        obs.s = antenna_s[a];
                        obs.t = t;
                        obs.z = true_metrics[a][b].best_rssi + noise(this->rng);
                        obs.noise_var = noise_var;
                        per_bs[b].push_back(obs);
                    }
                }
                return per_bs;
            }

        private:
            double noise_std_db;
            bool observe_all_pairs;
            std::mt19937 rng;
        };
    } // namespace kkf
} // namespace tx_controller
