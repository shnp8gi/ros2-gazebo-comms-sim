#pragma once

#include <string>
#include <vector>
#include <Eigen/Dense>
#include "comms_sim_pkg/DataTypes.hpp"

namespace tx_controller
{
    /// スケジューリング1ステップの決定結果(全ポリシー共通の値オブジェクト)
    struct SchedulingResult {
        int new_active_idx = 0;
        std::string active_antenna_name;
        bool switching_active = false;
        double last_switch_time_s = 0.0;
        std::vector<EventRecord> events;
    };

    /**
     * ISchedulingStrategy
     * -------------------
     * 責務: ハンドオーバースケジューリング方式の抽象化のみ。
     * HandoverScheduler の if-else 分岐に方式を追記する代わりに、本インターフェースの
     * 実装を登録することで方式をプラグインのように着脱可能にする(OCP)。
     * 実装は vehicle_antennas の assigned_bs_idx / last_rssi / link_state を
     * 自らの方式に従って設定する責任を持つ。
     */
    class ISchedulingStrategy {
    public:
        virtual ~ISchedulingStrategy() = default;

        /// この戦略が担当する scheduling_policy 名
        virtual std::string Name() const = 0;

        virtual SchedulingResult UpdateLinks(
            double current_time_s,
            const Eigen::Vector3d& vehicle_pos,
            const Eigen::Matrix3d& vehicle_rotmat,
            std::vector<AntennaInfo>& vehicle_antennas,
            const std::vector<BaseStationInfo>& base_stations,
            const std::vector<std::vector<AntennaMetrics>>& all_ant_bs_metrics,
            int current_active_idx,
            double current_last_switch_time,
            double rssi_min) = 0;
    };
} // namespace tx_controller
