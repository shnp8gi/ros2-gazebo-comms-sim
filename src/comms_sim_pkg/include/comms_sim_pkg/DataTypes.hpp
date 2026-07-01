#pragma once

#include <string>
#include <vector>
#include <Eigen/Dense>

namespace tx_controller
{
    struct Waypoint {
        double x, y, z, v;
    };

    struct TrajectorySample {
        Eigen::Vector3d pos;
        double yaw;
    };

    struct LutPair {
        std::string tx_antenna;  // 車載アンテナ名
        std::string rx_antenna;  // 基地局アンテナ名
        double rssi = -999.0;
    };

    struct LutEntry {
        double x, y, z;
        std::vector<LutPair> pairs;  // N個のペア (RSSIの高い順)
    };

    struct AntennaInfo {
        std::string name;
        Eigen::Vector3d offset;
        Eigen::Vector3d relative_rpy;
        
        double total_data_transmitted = 0.0;
        bool comm_active = true;
        std::string link_state = "DISCONNECTED";
        double link_establishment_start_time = -1.0;
        int establishment_step_count = 0;
        double last_rssi = -999.0;
        int assigned_bs_idx = -1;  // マルチペア: 割り当てられた基地局インデックス (-1 = 未割当)

        struct LogRecord {
            double time_s;
            double vehicle_time_s;
            std::string vehicle_name;
            bool has_link_grant;
            double distance_m;
            double rssi_dBm;
            double throughput_Gbps;
            double total_data_MB;
            double path_loss_dB;
            double e_gain_dB;
            double h_gain_dB;
            bool comm_active;
            double tx_x_m, tx_y_m, tx_z_m;
            double bs_x_m, bs_y_m, bs_z_m;
            std::string link_state;
            bool in_main_lobe;
            double off_boresight_e_deg;
            double off_boresight_h_deg;
        };
        std::vector<LogRecord> log_records;
    };

    struct BaseStationInfo {
        std::string name;
        Eigen::Vector3d position;
        Eigen::Vector3d antenna_offset;
        Eigen::Vector3d rpy;
        Eigen::Vector3d antenna_relative_rpy;
        Eigen::Matrix3d rotmat;
    };

    struct EventRecord {
        double time_s;
        std::string event_type;
        std::string active_antenna;
        std::string prev_antenna;
        std::string details;
    };

    struct ControlRecord {
        double time_s;
        double vehicle_x;
        double vehicle_y;
        double vehicle_yaw;
        std::string nominal_antenna;
        std::string active_antenna;
        bool switching_active;
        double last_switch_time_s;
    };

    struct AntennaMetrics {
        double best_rssi = -999.0;
        double throughput = 0.0;
        double distance = 0.0;
        Eigen::Vector3d bs_pos;
        double path_loss = 0.0;
        double e_gain = 0.0;
        double h_gain = 0.0;
        bool in_main_lobe = false;
        double off_boresight_e = 0.0;
        double off_boresight_h = 0.0;
    };
} // namespace tx_controller

