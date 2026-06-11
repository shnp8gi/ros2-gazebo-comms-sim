#include "comms_sim_pkg/comms_calculator.hpp"
#include <cmath>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iostream>

namespace comms_sim {

LogDistancePathLossModel::LogDistancePathLossModel(double frequency, double c, double exponent, double d0, double pl_d0)
    : d0(d0), pl_d0(pl_d0), exponent(exponent) {
  if (pl_d0 < 0) {
    double wavelength = c / frequency;
    this->pl_d0 = 20.0 * std::log10(4.0 * M_PI * d0 / wavelength);
  }
}

double LogDistancePathLossModel::calculate_path_loss(double distance) const {
  if (distance <= d0) return pl_d0;
  return pl_d0 + 10.0 * exponent * std::log10(distance / d0);
}

TwoRayGroundModel::TwoRayGroundModel(double tx_height, double rx_height, double tx_gain_db, double rx_gain_db)
    : tx_height(tx_height), rx_height(rx_height) {
  tx_gain_linear = std::pow(10.0, tx_gain_db / 10.0);
  rx_gain_linear = std::pow(10.0, rx_gain_db / 10.0);
}

double TwoRayGroundModel::calculate_path_loss(double distance) const {
  if (distance <= 0) return 0.0;
  double frequency = 6.0e10;
  double c = 299792458.0;
  double wavelength = c / frequency;
  double crossover = (4.0 * M_PI * tx_height * rx_height) / wavelength;

  if (distance < crossover) {
    return 20.0 * std::log10(4.0 * M_PI * distance / wavelength);
  }

  double gain_term = 10.0 * std::log10(tx_gain_linear * rx_gain_linear * tx_height * tx_height * rx_height * rx_height);
  double path_loss = 40.0 * std::log10(distance) - gain_term;
  return std::max(path_loss, 0.0);
}

CommsCalculator::CommsCalculator(std::unique_ptr<PropagationModel> model,
                                 double tx_power_dbm,
                                 double noise_variance,
                                 const std::string& mcs_table_path)
    : noise_variance(noise_variance), tx_power_dbm_(tx_power_dbm), rng_(std::random_device{}()) {
  if (model) {
    model_ = std::move(model);
  } else {
    model_ = std::make_unique<LogDistancePathLossModel>();
  }

  if (!mcs_table_path.empty()) {
    load_mcs_table(mcs_table_path);
  } else {
    init_default_mcs_table();
  }

  if (!mcs_rssi_.empty()) {
    rssi_min = mcs_rssi_.front();
    rssi_max = mcs_rssi_.back();
  }
}

void CommsCalculator::load_mcs_table(const std::string& path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    std::cerr << "Failed to open MCS table: " << path << std::endl;
    init_default_mcs_table();
    return;
  }

  std::vector<std::pair<double, double>> data;
  std::string line;
  while (std::getline(file, line)) {
    line.erase(0, line.find_first_not_of(" \r\n\t"));
    if (line.empty() || line[0] == '#' || (line.size() >= 2 && line[0] == '/' && line[1] == '/')) {
      continue;
    }
    std::stringstream ss(line);
    std::string t1, t2;
    if (std::getline(ss, t1, ',') && std::getline(ss, t2, ',')) {
      try {
        data.emplace_back(std::stod(t1), std::stod(t2));
      } catch (...) {
        // ignore
      }
    }
  }

  if (data.empty()) {
    init_default_mcs_table();
    return;
  }

  std::sort(data.begin(), data.end());
  mcs_rssi_.clear();
  mcs_throughput_.clear();
  for (const auto& p : data) {
    mcs_rssi_.push_back(p.first);
    mcs_throughput_.push_back(p.second);
  }
}

void CommsCalculator::init_default_mcs_table() {
  mcs_rssi_ = {-61, -58, -55, -51, -45, -39};
  mcs_throughput_ = {2.5813, 3.2853, 5.1627, 6.5707, 9.856, 13.1413};
}

double CommsCalculator::calculate_distance(const Eigen::Vector3d& tx_pos, const Eigen::Vector3d& bs_pos) const {
  return (tx_pos - bs_pos).norm();
}

std::pair<double, double> CommsCalculator::calculate_rssi(double distance, double antenna_gain_db, bool add_noise) {
  double path_loss = model_->calculate_path_loss(distance);
  double rssi = tx_power_dbm_ - path_loss + antenna_gain_db;

  if (add_noise && noise_variance > 0) {
    std::normal_distribution<double> dist(0.0, std::sqrt(noise_variance));
    rssi -= std::abs(dist(rng_));
  }
  return {rssi, path_loss};
}

double CommsCalculator::calculate_throughput(double rssi) const {
  if (rssi <= rssi_min) return 0.0;
  if (rssi >= rssi_max) return mcs_throughput_.back();

  auto it = std::upper_bound(mcs_rssi_.begin(), mcs_rssi_.end(), rssi);
  size_t idx = std::distance(mcs_rssi_.begin(), it) - 1;
  return mcs_throughput_[idx];
}

CommsMetrics CommsCalculator::calculate_all(const Eigen::Vector3d& tx_pos,
                                            const Eigen::Vector3d& bs_pos,
                                            double antenna_gain_db,
                                            bool add_noise) {
  double distance = calculate_distance(tx_pos, bs_pos);
  auto [rssi, path_loss] = calculate_rssi(distance, antenna_gain_db, add_noise);
  double throughput = calculate_throughput(rssi);

  return {distance, rssi, path_loss, throughput, model_->model_name()};
}

} // namespace comms_sim
