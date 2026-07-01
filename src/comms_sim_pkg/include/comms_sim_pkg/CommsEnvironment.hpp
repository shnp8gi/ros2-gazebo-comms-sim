#pragma once

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "comms_sim_pkg/DataTypes.hpp"
#include "comms_sim_pkg/antenna_pattern_parser.hpp"
#include "comms_sim_pkg/comms_calculator.hpp"
#include "comms_sim_pkg/Utils.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>

namespace tx_controller
{
    class CommsEnvironment {
    public:
        CommsEnvironment() {}

        void Configure(const std::string& config_file_path) {
            YAML::Node config = YAML::LoadFile(config_file_path);
            auto comms_params = config["comms_simulator_node"]["ros__parameters"];
            
            double tx_power = comms_params["tx_power"].as<double>(-7.0);
            double noise_variance = comms_params["noise_variance"].as<double>(0.0);
            std::string mcs_table_path = tx_controller::utils::resolve_path(comms_params["mcs_table_path"].as<std::string>(""));
            
            double max_antenna_attenuation = comms_params["max_antenna_attenuation"].as<double>(30.0);
            double mainlobe_angle_margin_deg = comms_params["mainlobe_angle_margin_deg"].as<double>(5.0);
            double mainlobe_e_half_angle_deg = comms_params["mainlobe_e_half_angle_deg"].as<double>(-1.0);
            double mainlobe_h_half_angle_deg = comms_params["mainlobe_h_half_angle_deg"].as<double>(-1.0);
            
            std::string e_plane_path = tx_controller::utils::resolve_path(comms_params["e_plane_path"].as<std::string>(""));
            std::string h_plane_path = tx_controller::utils::resolve_path(comms_params["h_plane_path"].as<std::string>(""));

            auto pl = comms_params["path_loss"];
            double c = pl["c"].as<double>(299792458.0);

            auto prop_model = std::make_unique<comms_sim::LogDistancePathLossModel>(6.0e10, c);
            
            this->comms_calculator = std::make_unique<comms_sim::CommsCalculator>(
                std::move(prop_model), tx_power, noise_variance, mcs_table_path);
            
            this->antenna_parser = std::make_unique<comms_sim::AntennaPatternParser>(
                max_antenna_attenuation, mainlobe_angle_margin_deg,
                mainlobe_e_half_angle_deg, mainlobe_h_half_angle_deg);
                
            if (!e_plane_path.empty()) this->antenna_parser->load_e_plane(e_plane_path);
            if (!h_plane_path.empty()) this->antenna_parser->load_h_plane(h_plane_path);
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
                    auto gain_res = this->antenna_parser->get_tx_rx_gains(
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
                    m.throughput = metrics_calc.throughput;
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
            if (this->comms_calculator) return this->comms_calculator->rssi_min;
            return -100.0;
        }

    private:
        std::unique_ptr<comms_sim::AntennaPatternParser> antenna_parser;
        std::unique_ptr<comms_sim::CommsCalculator> comms_calculator;
    };
} // namespace tx_controller
