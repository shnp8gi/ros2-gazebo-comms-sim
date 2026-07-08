#pragma once

#include <vector>
#include <limits>

namespace tx_controller
{
    namespace kkf
    {
        /**
         * HandoverPlanner
         * ---------------
         * 責務: リスクを織り込んだ品質系列に対するハンドオーバー計画の厳密解のみ(文書 5章)。
         *
         *   max Σ_k γ̃[a(k)][k]·Δt − λ Σ_k 1[a(k) ≠ a(k−1)]   (式(13))
         *
         * を状態数 A(候補ペア数)、計画長 K のビタビDPで O(K·A²) で解く。
         * γ̃ は下側信頼限界 γ̂ − κσ (式(12)) を呼び出し側が構成して渡す。
         * 入力は数値行列のみで、地図・車両・RSUの知識を一切持たない純粋モジュール。
         */
        class HandoverPlanner {
        public:
            struct Plan {
                std::vector<int> assignments;  // 各ステージの選択状態 a(0..K-1)
                double total_value = 0.0;
                bool valid = false;
            };

            /**
             * @param lcb_series      lcb_series[state][stage] = γ̃ [dB相当]
             * @param stage_dt_s      1ステージの時間幅 Δt [s]
             * @param switch_cost     切替1回あたりのコスト λ [dB·s相当]
             * @param initial_state   計画開始時点で接続中の状態 (-1 = 未接続/任意)
             */
            static Plan Solve(const std::vector<std::vector<double>>& lcb_series,
                              double stage_dt_s,
                              double switch_cost,
                              int initial_state)
            {
                Plan plan;
                const int A = static_cast<int>(lcb_series.size());
                if (A == 0 || lcb_series[0].empty()) return plan;
                const int K = static_cast<int>(lcb_series[0].size());
                constexpr double NEG_INF = -std::numeric_limits<double>::infinity();

                // value[a] = ステージ k までの最大累積価値、backptr で経路復元
                std::vector<std::vector<int>> backptr(K, std::vector<int>(A, -1));
                std::vector<double> value(A);
                for (int a = 0; a < A; ++a) {
                    double sw = (initial_state >= 0 && a != initial_state) ? switch_cost : 0.0;
                    value[a] = lcb_series[a][0] * stage_dt_s - sw;
                }

                for (int k = 1; k < K; ++k) {
                    std::vector<double> next_value(A, NEG_INF);
                    for (int a = 0; a < A; ++a) {
                        for (int prev = 0; prev < A; ++prev) {
                            double sw = (a != prev) ? switch_cost : 0.0;
                            double v = value[prev] - sw + lcb_series[a][k] * stage_dt_s;
                            if (v > next_value[a]) {
                                next_value[a] = v;
                                backptr[k][a] = prev;
                            }
                        }
                    }
                    value = next_value;
                }

                // 終端最大値から経路復元
                int best_last = 0;
                for (int a = 1; a < A; ++a) {
                    if (value[a] > value[best_last]) best_last = a;
                }
                plan.assignments.assign(K, best_last);
                for (int k = K - 1; k > 0; --k) {
                    plan.assignments[k - 1] = backptr[k][plan.assignments[k]];
                }
                plan.total_value = value[best_last];
                plan.valid = true;
                return plan;
            }
        };
    } // namespace kkf
} // namespace tx_controller
