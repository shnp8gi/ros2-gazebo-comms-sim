#pragma once

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "comms_sim_pkg/DataTypes.hpp"
#include "comms_sim_pkg/antenna_pattern_parser.hpp"
#include "comms_sim_pkg/comms_calculator.hpp"

namespace tx_controller
{
    class CommsEnvironment {
    public:
        CommsEnvironment() {
            this->antenna_parser = std::make_unique<comms_sim::AntennaPatternParser>();
            this->comms_calculator = std::make_unique<comms_sim::CommsCalculator>();
        }

        void Configure(const std::string& config_file_path) {
            this->comms_calculator->load_config(config_file_path);
            this->antenna_parser->load_config(config_file_path);
        }

        std::vector<std::vector<AntennaMetrics>> CalculateMetrics(
            const std::vector<AntennaInfo>& vehicle_antennas,
            const std::vector<BaseStationInfo>& base_stations,
            const Eigen::Vector3d& vehicle_pos,
            const Eigen::Matrix3d& vehicle_rotmat) 
        {
            std::vector<std::vector<AntennaMetrics>> all_metrics(vehicle_antennas.size());

            for (size_t i = 0; i < vehicle_antennas.size(); ++i) {
                const auto &ant = vehicle_antennas[i];
                Eigen::Vector3d ant_pos_world = vehicle_pos + vehicle_rotmat * ant.offset;
                Eigen::Vector3d ant_rpy = ant.relative_rpy;
                
                std::vector<AntennaMetrics> bs_metrics(base_stations.size());
                
                for (size_t bs_idx = 0; bs_idx < base_stations.size(); ++bs_idx) {
                    const auto &bs = base_stations[bs_idx];
                    Eigen::Vector3d bs_antenna_pos = bs.position + bs.rotmat * bs.antenna_offset;
                    Eigen::Vector3d bs_ant_rpy = bs.antenna_relative_rpy;
                    
                    Eigen::Matrix3d ant_rotmat;
                    auto gain_res = this->antenna_parser->calculate_gain(
                        ant_pos_world, ant_rpy, bs_antenna_pos, bs_ant_rpy,
                        &ant_rotmat, nullptr);

                    auto metrics_calc = this->comms_calculator->calculate_all(ant_pos_world, bs_antenna_pos, gain_res.tx_total + gain_res.rx_total, false);
                    
                    auto [el, az] = this->antenna_parser->calculate_antenna_frame_angles(
                        ant_pos_world, bs_antenna_pos, ant_rpy, &ant_rotmat);
                    
                    double off_boresight_e_deg = std::abs(el * 180.0 / M_PI);
                    double off_boresight_h_deg = std::abs(az * 180.0 / M_PI);
                    bool in_main_lobe = this->antenna_parser->is_in_main_lobe(off_boresight_e_deg, off_boresight_h_deg);
                    
                    AntennaMetrics &m = bs_metrics[bs_idx];
                    m.distance = metrics_calc.distance;
                    m.best_rssi = metrics_calc.rssi;
                    m.throughput = metrics_calc.throughput_gbps;
                    m.path_loss = metrics_calc.path_loss;
                    m.e_gain = gain_res.tx_total;
                    m.h_gain = gain_res.rx_total;
                    m.in_main_lobe = in_main_lobe;
                    m.off_boresight_e = off_boresight_e_deg;
                    m.off_boresight_h = off_boresight_h_deg;
                    m.bs_pos = bs_antenna_pos;
                }
                all_metrics[i] = bs_metrics;
            }

            return all_metrics;
        }

        double GetRssiMin() const {
            return this->comms_calculator->rssi_min;
        }

    private:
        std::unique_ptr<comms_sim::AntennaPatternParser> antenna_parser;
        std::unique_ptr<comms_sim::CommsCalculator> comms_calculator;
    };
} // namespace tx_controller
