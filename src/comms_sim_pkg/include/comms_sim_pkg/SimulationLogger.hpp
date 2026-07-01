#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <iostream>
#include "comms_sim_pkg/DataTypes.hpp"

namespace tx_controller
{
    class SimulationLogger {
    public:
        SimulationLogger() = default;

        void Configure(const std::string& config_file_path_in) {
            this->config_file_path = config_file_path_in;
        }

        void AddEventRecord(const EventRecord& record) {
            this->event_records.push_back(record);
        }

        void AddControlRecord(const ControlRecord& record) {
            this->control_records.push_back(record);
        }

        void SaveLogs(const std::string& model_name, const std::vector<AntennaInfo>& vehicle_antennas, const std::string& scheduling_policy) {
            // Retrieve timestamp
            std::string run_timestamp = "unknown_time";
            if (!this->config_file_path.empty()) {
                std::filesystem::path config_path(this->config_file_path);
                std::string filename = config_path.stem().string();
                size_t pos = filename.find("sim_params_");
                if (pos != std::string::npos) {
                    run_timestamp = filename.substr(pos + 11);
                }
            }

            std::string workspace_dir = "/workspace";
            std::string results_dir = workspace_dir + "/sim_results/" + run_timestamp;
            
            try {
                std::filesystem::create_directories(results_dir);
            } catch (const std::exception& e) {
                std::cerr << "[SimulationLogger] Error creating directories: " << e.what() << std::endl;
                return;
            }

            // 1. Save detailed logs per antenna
            for (const auto& ant : vehicle_antennas) {
                std::string filename = results_dir + "/" + model_name + "_" + ant.name + "_log.csv";
                std::ofstream ofs(filename);
                if (ofs.is_open()) {
                    ofs << "time_s,vehicle_time_s,vehicle_name,has_link_grant,distance_m,rssi_dBm,"
                        << "throughput_Gbps,total_data_MB,path_loss_dB,e_gain_dB,h_gain_dB,"
                        << "comm_active,tx_x,tx_y,tx_z,bs_x,bs_y,bs_z,link_state,"
                        << "in_main_lobe,off_boresight_e_deg,off_boresight_h_deg\n";
                    
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
                            << log.off_boresight_h_deg << "\n";
                    }
                    ofs.close();
                    std::cout << "[SimulationLogger] Saved detailed log for [" << ant.name << "] to " << filename << std::endl;
                }
            }

            // 2. Save event log
            if (!this->event_records.empty()) {
                std::string event_filename = results_dir + "/" + model_name + "_handover_events.csv";
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

            // 3. Save control log
            if (!this->control_records.empty()) {
                std::string control_filename = results_dir + "/" + model_name + "_control_log.csv";
                std::ofstream ofs_ctrl(control_filename);
                if (ofs_ctrl.is_open()) {
                    ofs_ctrl << "time_s,vehicle_x,vehicle_y,vehicle_yaw,nominal_antenna,active_antenna,switching_active,last_switch_time_s\n";
                    for (const auto& cr : this->control_records) {
                        ofs_ctrl << std::fixed << std::setprecision(6) << cr.time_s << ","
                                 << cr.vehicle_x << "," << cr.vehicle_y << "," << cr.vehicle_yaw << ","
                                 << cr.nominal_antenna << "," << cr.active_antenna << ","
                                 << (cr.switching_active ? 1 : 0) << "," << cr.last_switch_time_s << "\n";
                    }
                    ofs_ctrl.close();
                    std::cout << "[SimulationLogger] Saved control log to " << control_filename << std::endl;
                }
            }

            // 4. Save summary
            std::string summary_filename = results_dir + "/" + model_name + "_summary.csv";
            std::ofstream ofs_sum(summary_filename);
            if (ofs_sum.is_open()) {
                ofs_sum << "model_name,scheduling_policy,total_data_MB\n";
                double total_data = 0.0;
                for (const auto& ant : vehicle_antennas) {
                    total_data += ant.total_data_transmitted;
                }
                ofs_sum << model_name << "," << scheduling_policy << "," << std::fixed << std::setprecision(6) << total_data << "\n";
                ofs_sum.close();
                std::cout << "[SimulationLogger] Saved summary to " << summary_filename << std::endl;
            }
        }

    private:
        std::string config_file_path;
        std::vector<EventRecord> event_records;
        std::vector<ControlRecord> control_records;
    };
} // namespace tx_controller
