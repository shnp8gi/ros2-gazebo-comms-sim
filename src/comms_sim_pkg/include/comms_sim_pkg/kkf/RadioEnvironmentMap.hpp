#pragma once

#include <memory>
#include <vector>
#include <Eigen/Dense>
#include "comms_sim_pkg/kkf/KrigedKalmanFilter.hpp"
#include "comms_sim_pkg/kkf/RoadCoordinate.hpp"

namespace tx_controller
{
    namespace kkf
    {
        /**
         * RadioEnvironmentMap
         * -------------------
         * 責務: 第1層(電波環境地図層)のファサード。
         * 「候補RSUごとに独立したKKFインスタンスを保持」(文書 2.1節)し、
         * 観測レポートのRSU別振り分けと予測クエリの窓口のみを担う。
         * 統計計算そのものは KrigedKalmanFilter に委譲する。
         */
        class RadioEnvironmentMap {
        public:
            RadioEnvironmentMap(const RoadCoordinate& road,
                                const std::vector<Eigen::Vector3d>& rsu_positions,
                                const KkfParams& params)
            {
                for (const auto& rsu_pos : rsu_positions) {
                    auto basis = std::make_shared<LogDistanceBasis>(road, rsu_pos);
                    this->filters.emplace_back(std::make_unique<KrigedKalmanFilter>(basis, params));
                }
            }

            int NumRsu() const { return static_cast<int>(this->filters.size()); }

            /// RSU bs_idx への観測レポート投入(観測が空でも時間更新は進める)
            void Ingest(int bs_idx, double t, const std::vector<Observation>& observations) {
                if (bs_idx < 0 || bs_idx >= this->NumRsu()) return;
                this->filters[bs_idx]->Update(t, observations);
            }

            /// RSU bs_idx の (s,t) における予測 (平均・分散)
            KrigedKalmanFilter::Prediction Predict(int bs_idx, double s, double t) const {
                if (bs_idx < 0 || bs_idx >= this->NumRsu()) return {};
                return this->filters[bs_idx]->PredictAt(s, t);
            }

        private:
            std::vector<std::unique_ptr<KrigedKalmanFilter>> filters;
        };
    } // namespace kkf
} // namespace tx_controller
