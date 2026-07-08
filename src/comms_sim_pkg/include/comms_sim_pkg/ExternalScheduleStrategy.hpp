#pragma once

#include <mutex>
#include <sstream>
#include <string>
#include <vector>
#include "comms_sim_pkg/ISchedulingStrategy.hpp"
#include "comms_sim_pkg/SchedulingSupport.hpp"

namespace tx_controller
{
    /**
     * ExternalScheduleStrategy
     * ------------------------
     * 責務: 外部(制御プレーン)から与えられた時系列スケジュールの「実行」のみ。
     * スケジュールの生成・最適化の知識を一切持たない(MPC の実行側、設計書 §7)。
     *
     * - 現在時刻に該当するスケジュール区間 (t_start が現在時刻以下の最後のエントリ)
     *   の割当 (ant, bs, mode) を適用する
     * - フェイルセーフ: スケジュール未受信・期限切れ (t > valid_until) の場合は
     *   現在の割当を維持する
     * - mode=MEASURE の区間は measure_only を立て、リンクは確立するがデータ会計は
     *   行われない (プラグイン側のデータ加算が本フラグを参照)
     *
     * 本クラスは gz-transport / protobuf に依存しない。受信メッセージから
     * Schedule 構造体への変換は呼び出し側(プラグイン)の責務。
     * SetSchedule は受信スレッドから呼ばれるため mutex で保護する。
     */
    class ExternalScheduleStrategy : public ISchedulingStrategy {
    public:
        struct ScheduleEntry {
            double t_start = 0.0;
            int ant = 0;
            int bs = 0;
            bool measure_only = false;
        };

        struct Schedule {
            double valid_until = -1.0;
            std::vector<ScheduleEntry> plan;  // t_start 昇順を前提
        };

        std::string Name() const override { return "external_schedule"; }

        void SetSchedule(const Schedule& schedule_in) {
            std::lock_guard<std::mutex> lock(this->mutex_);
            this->schedule = schedule_in;
        }

        SchedulingResult UpdateLinks(
            double t,
            const Eigen::Vector3d& /*vehicle_pos*/,
            const Eigen::Matrix3d& /*vehicle_rotmat*/,
            std::vector<AntennaInfo>& vehicle_antennas,
            const std::vector<BaseStationInfo>& /*base_stations*/,
            const std::vector<std::vector<AntennaMetrics>>& all_ant_bs_metrics,
            int current_active_idx,
            double current_last_switch_time,
            double /*rssi_min*/) override
        {
            if (this->current_ant < 0) this->current_ant = current_active_idx;

            SchedulingResult result;
            result.last_switch_time_s = current_last_switch_time;

            // 1. 現在時刻に該当するスケジュール区間の解決 (無効時は現割当を維持)
            int target_ant = this->current_ant;
            int target_bs = this->current_bs;
            bool target_measure = this->current_measure;
            {
                std::lock_guard<std::mutex> lock(this->mutex_);
                if (!this->schedule.plan.empty() && t <= this->schedule.valid_until) {
                    const ScheduleEntry* active_entry = nullptr;
                    for (const auto& entry : this->schedule.plan) {
                        if (entry.t_start <= t) active_entry = &entry;
                        else break;
                    }
                    if (active_entry) {
                        target_ant = active_entry->ant;
                        target_bs = active_entry->bs;
                        target_measure = active_entry->measure_only;
                    }
                }
            }

            // 2. ペア変更の実行 (同一アンテナでのBS切替はペアネット再確立を課す)
            const int num_ant = static_cast<int>(vehicle_antennas.size());
            if (target_ant >= 0 && target_ant < num_ant &&
                (target_ant != this->current_ant || target_bs != this->current_bs)) {
                EventRecord ev;
                ev.time_s = t;
                ev.event_type = (target_ant != this->current_ant)
                                    ? "EXTERNAL_HANDOVER" : "EXTERNAL_BS_SWITCH";
                ev.active_antenna = vehicle_antennas[target_ant].name;
                ev.prev_antenna = (this->current_ant >= 0)
                                      ? vehicle_antennas[this->current_ant].name : "none";
                std::ostringstream oss;
                oss << "bs " << this->current_bs << " -> " << target_bs
                    << (target_measure ? " (measure)" : "");
                ev.details = oss.str();
                result.events.push_back(ev);

                if (target_ant == this->current_ant) {
                    vehicle_antennas[target_ant].link_state = "DISCONNECTED";
                }
                this->current_ant = target_ant;
                this->current_bs = target_bs;
                result.switching_active = true;
                result.last_switch_time_s = t;
            }
            this->current_measure = target_measure;

            // 3. 割当と measure_only フラグの適用
            scheduling::ApplyPairAssignment(vehicle_antennas, all_ant_bs_metrics,
                                            this->current_ant, this->current_bs);
            for (auto& ant : vehicle_antennas) ant.measure_only = false;
            if (this->current_ant >= 0 && this->current_ant < num_ant) {
                vehicle_antennas[this->current_ant].measure_only = this->current_measure;
            }

            result.new_active_idx = this->current_ant;
            result.active_antenna_name = vehicle_antennas[this->current_ant].name;
            return result;
        }

    private:
        std::mutex mutex_;
        Schedule schedule;
        int current_ant = -1;
        int current_bs = -1;
        bool current_measure = false;
    };
} // namespace tx_controller
