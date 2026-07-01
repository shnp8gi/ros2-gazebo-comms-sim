#pragma once

#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include "comms_sim_pkg/DataTypes.hpp"

namespace tx_controller
{
    class HandoverScheduler {
    public:
        HandoverScheduler() = default;

        void Configure(const std::string& policy,
                       bool filter_main,
                       double min_hold,
                       double margin,
                       double grace_period,
                       double score_threshold,
                       double data_limit,
                       double est_time)
        {
            this->scheduling_policy = policy;
            this->filter_main_lobe = filter_main;
            this->min_hold_time_s = min_hold;
            this->switch_margin_db = margin;
            this->proactive_grace_period_s = grace_period;
            this->proactive_handover_score_threshold = score_threshold;
            this->comm_data_limit_mb = data_limit;
            this->link_establishment_time_ms = est_time;
        }

        void SetLUT(const std::vector<LutEntry>& external_lut) {
            this->lut = external_lut;
        }

        struct SchedulingResult {
            int new_active_idx;
            std::string active_antenna_name;
            bool switching_active;
            double last_switch_time_s;
            std::vector<EventRecord> events;
        };

        SchedulingResult UpdateLinks(
            double current_time_s,
            const Eigen::Vector3d& pos,
            std::vector<AntennaInfo>& vehicle_antennas,
            const std::vector<BaseStationInfo>& base_stations,
            const std::vector<std::vector<AntennaMetrics>>& all_ant_bs_metrics,
            int current_active_idx,
            double current_last_switch_time,
            double rssi_min)
        {
            SchedulingResult result;
            result.new_active_idx = current_active_idx;
            result.switching_active = false;
            result.last_switch_time_s = current_last_switch_time;
            
            // Pick best metrics for each antenna
            std::vector<AntennaMetrics> best_metrics(vehicle_antennas.size());
            for (size_t i = 0; i < vehicle_antennas.size(); ++i) {
                double max_rssi = -999.0;
                int best_bs = -1;
                for (size_t j = 0; j < base_stations.size(); ++j) {
                    if (all_ant_bs_metrics[i][j].best_rssi > max_rssi) {
                        max_rssi = all_ant_bs_metrics[i][j].best_rssi;
                        best_bs = j;
                    }
                }
                if (best_bs >= 0) {
                    best_metrics[i] = all_ant_bs_metrics[i][best_bs];
                }
            }

            AntennaInfo* active_antenna = &vehicle_antennas[current_active_idx];

            if (this->scheduling_policy == "feedforward_optimal") {
                if (!this->lut.empty()) {
                    int best_lut_idx = this->last_lut_idx;
                    int n_points = this->lut.size();
                    double min_dist_sq = 1e9;
                    
                    for (int i = std::max(0, this->last_lut_idx - 100); i < std::min(n_points, this->last_lut_idx + 100); ++i) {
                        const auto &pt = this->lut[i];
                        double dist_sq = (pos.x()-pt.x)*(pos.x()-pt.x) + (pos.y()-pt.y)*(pos.y()-pt.y) + (pos.z()-pt.z)*(pos.z()-pt.z);
                        if (dist_sq < min_dist_sq) { min_dist_sq = dist_sq; best_lut_idx = i; }
                    }
                    this->last_lut_idx = best_lut_idx;

                    const auto &lut_entry = this->lut[best_lut_idx];
                    for (auto &ant : vehicle_antennas) ant.assigned_bs_idx = -1;

                    for (const auto &pair : lut_entry.pairs) {
                        for (size_t ai = 0; ai < vehicle_antennas.size(); ++ai) {
                            if (vehicle_antennas[ai].name == pair.tx_antenna) {
                                for (size_t bi = 0; bi < base_stations.size(); ++bi) {
                                    if (base_stations[bi].name == pair.rx_antenna) {
                                        vehicle_antennas[ai].assigned_bs_idx = static_cast<int>(bi);
                                    }
                                }
                            }
                        }
                    }

                    if (!lut_entry.pairs.empty()) {
                        for (size_t i = 0; i < vehicle_antennas.size(); ++i) {
                            if (vehicle_antennas[i].name == lut_entry.pairs[0].tx_antenna) {
                                result.new_active_idx = i;
                            }
                        }
                    }
                    
                    for (size_t i = 0; i < vehicle_antennas.size(); ++i) {
                        if (vehicle_antennas[i].assigned_bs_idx >= 0) {
                            best_metrics[i] = all_ant_bs_metrics[i][vehicle_antennas[i].assigned_bs_idx];
                        }
                    }
                }
            } else if (this->scheduling_policy == "sequential") {
                double max_rssi = -1e9;
                int best_ant = result.new_active_idx;
                for (size_t i = 0; i < vehicle_antennas.size(); ++i) {
                    if (best_metrics[i].best_rssi > max_rssi) { max_rssi = best_metrics[i].best_rssi; best_ant = i; }
                }
                if (best_ant != current_active_idx && (current_time_s - current_last_switch_time) >= this->min_hold_time_s && (max_rssi > best_metrics[current_active_idx].best_rssi + this->switch_margin_db)) {
                    result.new_active_idx = best_ant;
                    EventRecord ev;
                    ev.time_s = current_time_s;
                    ev.event_type = "REACTIVE_HANDOVER";
                    ev.active_antenna = vehicle_antennas[result.new_active_idx].name;
                    ev.prev_antenna = vehicle_antennas[current_active_idx].name;
                    ev.details = "Switched to better antenna by margin";
                    result.events.push_back(ev);
                }
            } else if (this->scheduling_policy == "simple_no_handover") {
                std::vector<bool> bs_in_use(base_stations.size(), false);
                for (size_t i = 0; i < vehicle_antennas.size(); ++i) {
                    auto &ant = vehicle_antennas[i];
                    if (ant.assigned_bs_idx >= 0 && ant.assigned_bs_idx < static_cast<int>(base_stations.size())) {
                        double current_rssi = all_ant_bs_metrics[i][ant.assigned_bs_idx].best_rssi;
                        if (current_rssi >= rssi_min) {
                            bs_in_use[ant.assigned_bs_idx] = true;
                            best_metrics[i] = all_ant_bs_metrics[i][ant.assigned_bs_idx];
                            ant.last_rssi = current_rssi;
                        } else {
                            ant.assigned_bs_idx = -1;
                        }
                    }
                }
                for (size_t i = 0; i < vehicle_antennas.size(); ++i) {
                    auto &ant = vehicle_antennas[i];
                    if (ant.assigned_bs_idx < 0) {
                        double max_rssi = -999.0;
                        int best_bs = -1;
                        for (size_t bs_idx = 0; bs_idx < base_stations.size(); ++bs_idx) {
                            if (bs_in_use[bs_idx]) continue;
                            double rssi = all_ant_bs_metrics[i][bs_idx].best_rssi;
                            if (rssi > max_rssi) {
                                max_rssi = rssi;
                                best_bs = static_cast<int>(bs_idx);
                            }
                        }
                        if (best_bs >= 0 && max_rssi >= rssi_min) {
                            ant.assigned_bs_idx = best_bs;
                            bs_in_use[best_bs] = true;
                            best_metrics[i] = all_ant_bs_metrics[i][best_bs];
                            ant.last_rssi = max_rssi;
                        }
                    }
                }
            } else if (this->scheduling_policy == "physical_score_priority") {
                double max_score = -1e9;
                int best_ant = result.new_active_idx;
                for (size_t i = 0; i < vehicle_antennas.size(); ++i) {
                    double score = best_metrics[i].e_gain + best_metrics[i].h_gain - best_metrics[i].path_loss;
                    if (score > max_score) { max_score = score; best_ant = i; }
                }
                double current_score = best_metrics[current_active_idx].e_gain + best_metrics[current_active_idx].h_gain - best_metrics[current_active_idx].path_loss;
                if (best_ant != current_active_idx && (current_time_s - current_last_switch_time) >= this->min_hold_time_s && (max_score > current_score + this->switch_margin_db)) {
                    result.new_active_idx = best_ant;
                    EventRecord ev;
                    ev.time_s = current_time_s;
                    ev.event_type = "REACTIVE_HANDOVER";
                    ev.active_antenna = vehicle_antennas[result.new_active_idx].name;
                    ev.prev_antenna = vehicle_antennas[current_active_idx].name;
                    std::ostringstream ss;
                    ss << "Physical score-based switch (Score: " << std::fixed << std::setprecision(2) << max_score << " dB)";
                    ev.details = ss.str();
                    result.events.push_back(ev);
                }
            }

            // Proactive Handover Logic
            if (this->scheduling_policy != "feedforward_optimal" && this->scheduling_policy != "simple_no_handover") {
                if (active_antenna->link_state == "DISCONNECTED" && (current_time_s - current_last_switch_time) > this->proactive_grace_period_s) {
                    double best_score = -1e9;
                    int best_idx = -1;
                    for (size_t i = 0; i < vehicle_antennas.size(); ++i) {
                        if (static_cast<int>(i) == current_active_idx || !vehicle_antennas[i].comm_active || (this->filter_main_lobe && !best_metrics[i].in_main_lobe)) continue;
                        double score = best_metrics[i].e_gain + best_metrics[i].h_gain - best_metrics[i].path_loss;
                        if (score > best_score) { best_score = score; best_idx = i; }
                    }
                    if (best_idx != -1 && best_score >= this->proactive_handover_score_threshold) {
                        EventRecord ev;
                        ev.time_s = current_time_s;
                        ev.event_type = "PROACTIVE_HANDOVER";
                        ev.active_antenna = vehicle_antennas[best_idx].name;
                        ev.prev_antenna = active_antenna->name;
                        std::ostringstream oss;
                        oss << "Score " << std::fixed << std::setprecision(1) << best_score << " >= " << this->proactive_handover_score_threshold;
                        ev.details = oss.str();
                        result.events.push_back(ev);
                        result.new_active_idx = best_idx;
                    }
                }
            }

            if (result.new_active_idx != current_active_idx) {
                result.last_switch_time_s = current_time_s;
                result.switching_active = true;
            }
            result.active_antenna_name = vehicle_antennas[result.new_active_idx].name;

            // Apply best_rssi to last_rssi
            for (size_t i = 0; i < vehicle_antennas.size(); ++i) {
                vehicle_antennas[i].last_rssi = best_metrics[i].best_rssi;
            }

            return result;
        }

    private:
        std::string scheduling_policy = "sequential";
        bool filter_main_lobe = true;
        double min_hold_time_s = 1.0;
        double switch_margin_db = 2.0;
        double proactive_grace_period_s = 0.5;
        double proactive_handover_score_threshold = 50.0;
        double comm_data_limit_mb = -1.0;
        double link_establishment_time_ms = 2.0;

        std::vector<LutEntry> lut;
        int last_lut_idx = 0;
    };
} // namespace tx_controller
