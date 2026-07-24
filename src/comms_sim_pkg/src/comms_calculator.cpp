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

TwoRayGroundModel::TwoRayGroundModel(double tx_height, double rx_height, double frequency, double c)
    : tx_height(tx_height), rx_height(rx_height), frequency(frequency), c(c) {
}

double TwoRayGroundModel::calculate_path_loss(double distance) const {
  if (distance <= 0) return 0.0;
  double wavelength = this->c / this->frequency;
  double crossover = (4.0 * M_PI * tx_height * rx_height) / wavelength;

  if (distance < crossover) {
    return 20.0 * std::log10(4.0 * M_PI * distance / wavelength);
  }

  // NOTE: Antenna gains are handled in calculate_rssi, so we don't apply them here
  // to avoid double counting.
  double gain_term = 10.0 * std::log10(tx_height * tx_height * rx_height * rx_height);
  double path_loss = 40.0 * std::log10(distance) - gain_term;
  return std::max(path_loss, 0.0);
}

CommsCalculator::CommsCalculator(std::unique_ptr<PropagationModel> model,
                                 double tx_power_dbm,
                                 double noise_variance,
                                 const std::string& mcs_table_path)
    : CommsCalculator(std::move(model), tx_power_dbm, noise_variance,
                      std::make_unique<McsTableRateModel>(mcs_table_path)) {}

CommsCalculator::CommsCalculator(std::unique_ptr<PropagationModel> model,
                                 double tx_power_dbm,
                                 double noise_variance,
                                 std::unique_ptr<IRateModel> rate_model)
    : noise_variance(noise_variance), tx_power_dbm_(tx_power_dbm), rng_(std::random_device{}()) {
  if (model) {
    model_ = std::move(model);
  } else {
    model_ = std::make_unique<LogDistancePathLossModel>();
  }
  rate_model_ = rate_model ? std::move(rate_model)
                           : std::make_unique<McsTableRateModel>("");
  rssi_min = rate_model_->MinRssiDbm();
  rssi_max = rate_model_->MaxRssiDbm();
}

double CommsCalculator::calculate_distance(const Eigen::Vector3d& tx_pos, const Eigen::Vector3d& bs_pos) const {
  return (tx_pos - bs_pos).norm();
}

std::pair<double, double> CommsCalculator::calculate_rssi(double distance, double antenna_gain_db, bool add_noise) {
  double path_loss = model_->calculate_path_loss(distance);
  double rssi = tx_power_dbm_ - path_loss + antenna_gain_db;

  if (add_noise && noise_variance > 0) {
    std::normal_distribution<double> dist(0.0, std::sqrt(noise_variance));
    rssi += dist(rng_);
  }
  return {rssi, path_loss};
}

double CommsCalculator::calculate_throughput(double rssi) const {
  return rate_model_->RateGbps(rssi);
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

CommsMetrics CommsCalculator::calculate_from_loss(double distance,
                                                  double total_loss_db,
                                                  double antenna_gain_db,
                                                  bool add_noise) {
  double rssi = tx_power_dbm_ - total_loss_db + antenna_gain_db;
  if (add_noise && noise_variance > 0) {
    std::normal_distribution<double> dist(0.0, std::sqrt(noise_variance));
    rssi += dist(rng_);
  }
  double throughput = calculate_throughput(rssi);
  return {distance, rssi, total_loss_db, throughput, model_->model_name()};
}

} // namespace comms_sim
