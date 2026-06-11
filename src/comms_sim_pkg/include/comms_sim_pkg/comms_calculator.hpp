#pragma once
#include <string>
#include <vector>
#include <random>
#include <memory>
#include <Eigen/Dense>

namespace comms_sim {

class PropagationModel {
public:
  virtual ~PropagationModel() = default;
  virtual double calculate_path_loss(double distance) const = 0;
  virtual std::string model_name() const = 0;
};

class LogDistancePathLossModel : public PropagationModel {
public:
  LogDistancePathLossModel(double frequency = 6.0e10,
                           double c = 299792458.0,
                           double exponent = 2.0,
                           double d0 = 1.0,
                           double pl_d0 = -1.0);
  double calculate_path_loss(double distance) const override;
  std::string model_name() const override { return "Log-Distance Path Loss Model"; }

  double d0, pl_d0, exponent;
};

class TwoRayGroundModel : public PropagationModel {
public:
  TwoRayGroundModel(double tx_height = 10.5,
                    double rx_height = 1.3,
                    double tx_gain_db = 0.0,
                    double rx_gain_db = 0.0);
  double calculate_path_loss(double distance) const override;
  std::string model_name() const override { return "Two-Ray Ground Reflection Model"; }

  double tx_height, rx_height, tx_gain_linear, rx_gain_linear;
};

struct CommsMetrics {
  double distance;
  double rssi;
  double path_loss;
  double throughput;
  std::string model_name;
};

class CommsCalculator {
public:
  CommsCalculator(std::unique_ptr<PropagationModel> model = nullptr,
                  double tx_power_dbm = 20.0,
                  double noise_variance = 2.0,
                  const std::string& mcs_table_path = "");

  CommsMetrics calculate_all(const Eigen::Vector3d& tx_pos,
                             const Eigen::Vector3d& bs_pos,
                             double antenna_gain_db = 0.0,
                             bool add_noise = true);

  double calculate_distance(const Eigen::Vector3d& tx_pos, const Eigen::Vector3d& bs_pos) const;
  std::pair<double, double> calculate_rssi(double distance, double antenna_gain_db = 0.0, bool add_noise = true);

  double rssi_min, rssi_max;
  double noise_variance;
  double tx_power_dbm_;

private:
  std::unique_ptr<PropagationModel> model_;
  std::mt19937 rng_;
  std::vector<double> mcs_rssi_, mcs_throughput_;

  void load_mcs_table(const std::string& path);
  void init_default_mcs_table();
  double calculate_throughput(double rssi) const;
};

}  // namespace comms_sim
