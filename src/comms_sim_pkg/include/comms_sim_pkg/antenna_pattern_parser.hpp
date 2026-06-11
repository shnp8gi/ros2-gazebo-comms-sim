#pragma once
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <optional>
#include <utility>

namespace comms_sim {

class AntennaPatternParser {
public:
  explicit AntennaPatternParser(double max_antenna_attenuation = 30.0,
                                double mainlobe_angle_margin_deg = 5.0,
                                double mainlobe_e_half_angle_override_deg = -1.0,
                                double mainlobe_h_half_angle_override_deg = -1.0);

  void load_e_plane(const std::string& filepath);
  void load_h_plane(const std::string& filepath);

  double get_e_plane_gain(double angle_deg) const;
  double get_h_plane_gain(double angle_deg) const;

  static Eigen::Matrix3d rpy_to_rotmat(double roll, double pitch, double yaw);

  std::pair<double, double> calculate_antenna_frame_angles(
      const Eigen::Vector3d& antenna_pos,
      const Eigen::Vector3d& target_pos,
      const Eigen::Vector3d& antenna_rpy,
      const Eigen::Matrix3d* rotmat = nullptr) const;

  struct GainResult { double e_gain; double h_gain; double total; };
  GainResult get_gain_from_angles(double elevation_rad, double azimuth_rad) const;

  struct TxRxGainResult { double tx_e; double tx_h; double tx_total; double rx_e; double rx_h; double rx_total; };
  TxRxGainResult get_tx_rx_gains(
      const Eigen::Vector3d& tx_pos, const Eigen::Vector3d& tx_rpy,
      const Eigen::Vector3d& rx_pos, const Eigen::Vector3d& rx_rpy,
      const Eigen::Matrix3d* tx_rotmat = nullptr,
      const Eigen::Matrix3d* rx_rotmat = nullptr) const;

  double e_plane_peak() const { return e_plane_peak_; }
  double h_plane_peak() const { return h_plane_peak_; }

  bool is_in_main_lobe(double elevation_deg, double azimuth_deg) const;
  double detect_first_null_angle(const std::vector<double>& angles,
                                 const std::vector<double>& gains) const;

  double e_mainlobe_half_angle() const { return e_mainlobe_half_angle_; }
  double h_mainlobe_half_angle() const { return h_mainlobe_half_angle_; }

private:
  std::vector<double> e_angles_, e_gains_;
  std::vector<double> h_angles_, h_gains_;
  double e_plane_peak_ = 0.0;
  double h_plane_peak_ = 0.0;
  double max_attenuation_;
  double mainlobe_angle_margin_deg_ = 5.0;
  double mainlobe_e_half_angle_override_deg_ = -1.0;
  double mainlobe_h_half_angle_override_deg_ = -1.0;
  double e_mainlobe_half_angle_ = 0.0;
  double h_mainlobe_half_angle_ = 0.0;

  static double wrap_pi(double angle_rad);
  static double linear_interp(const std::vector<double>& xs,
                              const std::vector<double>& ys,
                              double x, double fill = -1000.0);
  void load_pattern(const std::string& filepath,
                    std::vector<double>& angles,
                    std::vector<double>& gains,
                    double& peak);

};

}  // namespace comms_sim
