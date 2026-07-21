#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <iostream>
#include "comms_sim_pkg/DataTypes.hpp"
#include "comms_sim_pkg/Utils.hpp"
#include <algorithm>

namespace tx_controller
{
    // --- Configuration ---
    // ロガーの出力先・書式に関する設定を一括で受け渡すための値オブジェクト。
    struct LoggerConfig {
        std::string config_file_path;
        std::string output_subdir;
        std::string summary_filename;
        std::string angle_unit = "rad";  // "rad" | "deg"（"degree" も許容）
    };

    // --- Domain Model ---
    struct SummaryMetrics {
        std::string entity_name;
        std::string scheduling_policy;
        double total_data_MB = 0.0;
        double average_rssi_dBm = -999.0;
        double average_throughput_Gbps = 0.0;
        double connected_time_s = 0.0;
        int handover_count = 0;
    };

    // --- Interface ---
    class ISummaryFormatter {
    public:
        virtual ~ISummaryFormatter() = default;
        virtual std::string GetHeader() const = 0;
        virtual std::string Format(const SummaryMetrics& metrics) const = 0;
    };

    // --- Presentation Layer ---
    class CsvSummaryFormatter : public ISummaryFormatter {
    public:
        std::string GetHeader() const override {
            return "vehicle_name,scheduling_policy,total_data_MB,average_rssi_dBm,average_throughput_Gbps,connected_time_s,handover_count\n";
        }
        std::string Format(const SummaryMetrics& m) const override {
            std::ostringstream oss;
            oss << m.entity_name << "," << m.scheduling_policy << "," 
                << std::fixed << std::setprecision(6) << m.total_data_MB << ","
                << std::fixed << std::setprecision(6) << m.average_rssi_dBm << ","
                << std::fixed << std::setprecision(6) << m.average_throughput_Gbps << ","
                << std::fixed << std::setprecision(6) << m.connected_time_s << ","
                << m.handover_count << "\n";
            return oss.str();
        }
    };

    // --- Domain Logic ---
    class MetricsCalculator {
    public:
        static SummaryMetrics CalculateForAntenna(const std::string& antenna_name, const AntennaInfo& ant, const std::string& policy, int handover_count) {
            SummaryMetrics metrics;
            metrics.entity_name = antenna_name;
            metrics.scheduling_policy = policy;
            metrics.total_data_MB = ant.total_data_transmitted;
            metrics.handover_count = handover_count;

            double sum_rssi = 0.0;
            double sum_throughput = 0.0;
            int connected_steps = 0;

            if (ant.log_records.size() >= 2) {
                double dt = ant.log_records[1].time_s - ant.log_records[0].time_s;
                for (const auto& log : ant.log_records) {
                    if (log.link_state == "CONNECTED") {
                        sum_rssi += log.rssi_dBm;
                        sum_throughput += log.throughput_Gbps;
                        connected_steps++;
                        metrics.connected_time_s += dt;
                    }
                }
            }

            metrics.average_rssi_dBm = (connected_steps > 0) ? (sum_rssi / connected_steps) : -999.0;
            metrics.average_throughput_Gbps = (connected_steps > 0) ? (sum_throughput / connected_steps) : 0.0;
            return metrics;
        }

        static SummaryMetrics CalculateTotal(const std::string& model_name, const std::vector<SummaryMetrics>& antenna_metrics, const std::string& policy, int total_handovers) {
            SummaryMetrics total;
            total.entity_name = model_name + "_total";
            total.scheduling_policy = policy;
            total.handover_count = total_handovers;

            int valid_antenna_count = 0;
            double sum_avg_rssi = 0.0;
            double sum_avg_throughput = 0.0;

            for (const auto& m : antenna_metrics) {
                total.total_data_MB += m.total_data_MB;
                total.connected_time_s += m.connected_time_s;
                if (m.average_rssi_dBm > -999.0) {
                    sum_avg_rssi += m.average_rssi_dBm;
                    sum_avg_throughput += m.average_throughput_Gbps;
                    valid_antenna_count++;
                }
            }
            
            total.average_rssi_dBm = (valid_antenna_count > 0) ? (sum_avg_rssi / valid_antenna_count) : -999.0;
            total.average_throughput_Gbps = (valid_antenna_count > 0) ? (sum_avg_throughput / valid_antenna_count) : 0.0;
            return total;
        }
    };

    class SimulationLogger {
    public:
        SimulationLogger() = default;

        void Configure(const LoggerConfig& config_in) {
            this->config = config_in;
        }

        void AddEventRecord(const EventRecord& record) {
            this->event_records.push_back(record);
        }

        void AddControlRecord(const ControlRecord& record) {
            this->control_records.push_back(record);
        }

        void SaveLogs(const std::string& model_name, const std::vector<AntennaInfo>& vehicle_antennas, const std::string& scheduling_policy) {
            std::string workspace_dir = "/workspace";
            std::string results_dir = workspace_dir + "/sim_results/" + this->config.output_subdir;

            if (this->config.output_subdir.empty()) {
                // Retrieve timestamp
                std::string run_timestamp = "unknown_time";
                if (!this->config.config_file_path.empty()) {
                    std::filesystem::path config_path(this->config.config_file_path);
                    std::string filename = config_path.stem().string();
                    size_t pos = filename.find("sim_params_");
                    if (pos != std::string::npos) {
                        run_timestamp = filename.substr(pos + 11);
                    }
                }
                results_dir = workspace_dir + "/sim_results/" + run_timestamp;
            }

            std::string logs_dir = results_dir + "/detailed_logs";
            std::string events_dir = results_dir + "/events";
            std::string controls_dir = results_dir + "/controls";
            std::string summaries_dir = results_dir + "/summaries";
            
            try {
                std::filesystem::create_directories(logs_dir);
                std::filesystem::create_directories(events_dir);
                std::filesystem::create_directories(controls_dir);
                std::filesystem::create_directories(summaries_dir);
            } catch (const std::exception& e) {
                std::cerr << "[SimulationLogger] Error creating directories: " << e.what() << std::endl;
                return;
            }

            std::string file_prefix = model_name;
            if (!this->config.summary_filename.empty()) {
                file_prefix = this->config.summary_filename;
                if (file_prefix.size() >= 4 && file_prefix.substr(file_prefix.size() - 4) == ".csv") {
                    file_prefix = file_prefix.substr(0, file_prefix.size() - 4);
                }
            }

            // 1. Save detailed logs per antenna
            for (const auto& ant : vehicle_antennas) {
                std::string filename = logs_dir + "/" + file_prefix + "_" + ant.name + "_log.csv";
                std::ofstream ofs(filename);
                if (ofs.is_open()) {
                    ofs << "time_s,vehicle_time_s,vehicle_name,has_link_grant,distance_m,rssi_dBm,"
                        << "throughput_Gbps,total_data_MB,path_loss_dB,tx_antenna_gain_dB,rx_antenna_gain_dB,"
                        << "comm_active,tx_x,tx_y,tx_z,bs_x,bs_y,bs_z,link_state,"
                        << "in_main_lobe,off_boresight_e_deg,off_boresight_h_deg,"
                        << "link_los,blockage_loss_dB,shadow_dB,fading_dB\n";
                    
                    for (const auto& log : ant.log_records) {
                        ofs << std::fixed << std::setprecision(6) << log.time_s << ","
                            << log.vehicle_time_s << ","
                            << log.vehicle_name << ","
                            << (log.has_link_grant ? 1 : 0) << ","
                            << std::setprecision(2) << log.distance_m << ","
                            << log.rssi_dBm << ","
                            << log.throughput_Gbps << ","
                            << log.total_data_MB << ","
                            << log.path_loss_dB << ","
                            << log.e_gain_dB << ","
                            << log.h_gain_dB << ","
                            << (log.comm_active ? 1 : 0) << ","
                            << log.tx_x_m << "," << log.tx_y_m << "," << log.tx_z_m << ","
                            << log.bs_x_m << "," << log.bs_y_m << "," << log.bs_z_m << ","
                            << log.link_state << ","
                            << (log.in_main_lobe ? 1 : 0) << ","
                            << log.off_boresight_e_deg << ","
                            << log.off_boresight_h_deg << ","
                            << (log.link_los ? 1 : 0) << ","
                            << log.blockage_loss_dB << ","
                            << log.shadow_dB << ","
                            << log.fading_dB << "\n";
                    }
                    ofs.close();
                    std::cout << "[SimulationLogger] Saved detailed log for [" << ant.name << "] to " << filename << std::endl;
                }
            }

            // 2. Save event log (複数Tx車両で衝突しないようモデル名を含める)
            if (!this->event_records.empty()) {
                std::string event_filename = events_dir + "/" + file_prefix + "_" + model_name + "_handover_events.csv";
                std::ofstream ofs_event(event_filename);
                if (ofs_event.is_open()) {
                    ofs_event << "time_s,event_type,active_antenna,prev_antenna,details\n";
                    for (const auto& ev : this->event_records) {
                        ofs_event << std::fixed << std::setprecision(6) << ev.time_s << ","
                                  << ev.event_type << ","
                                  << ev.active_antenna << ","
                                  << ev.prev_antenna << ",\""
                                  << ev.details << "\"\n";
                    }
                    ofs_event.close();
                    std::cout << "[SimulationLogger] Saved event log to " << event_filename << std::endl;
                }
            }

            // 3. Save control log (複数Tx車両で衝突しないようモデル名を含める)
            if (!this->control_records.empty()) {
                std::string control_filename = controls_dir + "/" + file_prefix + "_" + model_name + "_control_log.csv";
                std::ofstream ofs_ctrl(control_filename);
                if (ofs_ctrl.is_open()) {
                    ofs_ctrl << "time_s,vehicle_x,vehicle_y,vehicle_yaw_" << utils::angle_unit_label(this->config.angle_unit)
                             << ",nominal_antenna,active_antenna,switching_active,last_switch_time_s\n";
                    for (const auto& cr : this->control_records) {
                        ofs_ctrl << std::fixed << std::setprecision(6) << cr.time_s << ","
                                 << cr.vehicle_x << "," << cr.vehicle_y << ","
                                 << utils::convert_angle(cr.vehicle_yaw, this->config.angle_unit) << ","
                                 << cr.nominal_antenna << "," << cr.active_antenna << ","
                                 << (cr.switching_active ? 1 : 0) << "," << cr.last_switch_time_s << "\n";
                    }
                    ofs_ctrl.close();
                    std::cout << "[SimulationLogger] Saved control log to " << control_filename << std::endl;
                }
            }

            // 4. Save summary
            // 複数Tx車両では各車のプラグインインスタンスが同一サマリファイルへ書くため、
            // プロセス内 mutex で直列化して追記する (SaveLogs はインスタンスごとに1回のみ)。
            static std::mutex summary_file_mutex;
            std::lock_guard<std::mutex> sum_lock(summary_file_mutex);
            std::string sum_filename = summaries_dir + "/" + (this->config.summary_filename.empty() ? (model_name + "_summary.csv") : this->config.summary_filename);
            bool fresh_file = !std::filesystem::exists(sum_filename) ||
                              std::filesystem::file_size(sum_filename) == 0;
            std::ofstream ofs_sum(sum_filename, std::ios::app);
            if (ofs_sum.is_open()) {
                CsvSummaryFormatter formatter;
                if (fresh_file) ofs_sum << formatter.GetHeader();

                int handover_count = 0;
                for (const auto& ev : this->event_records) {
                    if (ev.event_type.find("HANDOVER") != std::string::npos) {
                        handover_count++;
                    }
                }

                std::vector<SummaryMetrics> antenna_metrics;
                for (const auto& ant : vehicle_antennas) {
                    auto metrics = MetricsCalculator::CalculateForAntenna(ant.name, ant, scheduling_policy, handover_count);
                    antenna_metrics.push_back(metrics);
                    ofs_sum << formatter.Format(metrics);
                }

                if (vehicle_antennas.size() >= 2) {
                    auto total_metrics = MetricsCalculator::CalculateTotal(model_name, antenna_metrics, scheduling_policy, handover_count);
                    ofs_sum << formatter.Format(total_metrics);
                } else if (vehicle_antennas.empty()) {
                    // Fallback if no antennas defined
                    SummaryMetrics m;
                    m.entity_name = model_name;
                    m.scheduling_policy = scheduling_policy;
                    ofs_sum << formatter.Format(m);
                }

                ofs_sum.close();
                std::cout << "[SimulationLogger] Saved summary to " << sum_filename << std::endl;
            }
        }

    private:
        LoggerConfig config;
        std::vector<EventRecord> event_records;
        std::vector<ControlRecord> control_records;
    };
} // namespace tx_controller
