#pragma once

#include <vector>
#include <algorithm>
#include "comms_sim_pkg/DataTypes.hpp"

namespace tx_controller
{
    namespace scheduling
    {
        /**
         * 単一ペア割当 (active_ant, active_bs) を antenna 配列へ反映する共通処理。
         * - 全アンテナの assigned_bs_idx をクリアし、アクティブペアのみ設定
         * - last_rssi を「割当ペア(未割当なら最良BS)の真値リンク品質」で更新
         *   (物理リンク状態機械の入力であり、スケジューラの知識とは無関係)
         * KkfPredictiveStrategy / ExternalScheduleStrategy が共用する純粋関数。
         */
        inline void ApplyPairAssignment(
            std::vector<AntennaInfo>& antennas,
            const std::vector<std::vector<AntennaMetrics>>& all_ant_bs_metrics,
            int active_ant, int active_bs)
        {
            const int num_ant = static_cast<int>(antennas.size());
            const int num_bs = all_ant_bs_metrics.empty()
                                   ? 0 : static_cast<int>(all_ant_bs_metrics[0].size());

            for (auto& ant : antennas) ant.assigned_bs_idx = -1;
            if (active_ant >= 0 && active_ant < num_ant &&
                active_bs >= 0 && active_bs < num_bs) {
                antennas[active_ant].assigned_bs_idx = active_bs;
            }

            for (int a = 0; a < num_ant; ++a) {
                int b = antennas[a].assigned_bs_idx;
                if (b < 0) {
                    double best = -999.0;
                    for (int j = 0; j < num_bs; ++j) {
                        best = std::max(best, all_ant_bs_metrics[a][j].best_rssi);
                    }
                    antennas[a].last_rssi = best;
                } else {
                    antennas[a].last_rssi = all_ant_bs_metrics[a][b].best_rssi;
                }
            }
        }
    } // namespace scheduling
} // namespace tx_controller
