#include "comms_sim_pkg/antenna_pattern_parser.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <iostream>

namespace comms_sim {

AntennaPatternParser::AntennaPatternParser(double max_antenna_attenuation,
                                           double mainlobe_angle_margin_deg,
                                           double mainlobe_e_half_angle_override_deg,
                                           double mainlobe_h_half_angle_override_deg)
    : max_attenuation_(max_antenna_attenuation),
      mainlobe_angle_margin_deg_(mainlobe_angle_margin_deg),
      mainlobe_e_half_angle_override_deg_(mainlobe_e_half_angle_override_deg),
      mainlobe_h_half_angle_override_deg_(mainlobe_h_half_angle_override_deg) {}

void AntennaPatternParser::load_e_plane(const std::string& filepath) {
  load_pattern(filepath, e_angles_, e_gains_, e_plane_peak_);
  if (mainlobe_e_half_angle_override_deg_ > 0.0) {
    e_mainlobe_half_angle_ = mainlobe_e_half_angle_override_deg_;
  } else {
    double null_ang = detect_first_null_angle(e_angles_, e_gains_);
    e_mainlobe_half_angle_ = std::max(0.0, null_ang - mainlobe_angle_margin_deg_);
  }
}

void AntennaPatternParser::load_h_plane(const std::string& filepath) {
  load_pattern(filepath, h_angles_, h_gains_, h_plane_peak_);
  if (mainlobe_h_half_angle_override_deg_ > 0.0) {
    h_mainlobe_half_angle_ = mainlobe_h_half_angle_override_deg_;
  } else {
    double null_ang = detect_first_null_angle(h_angles_, h_gains_);
    h_mainlobe_half_angle_ = std::max(0.0, null_ang - mainlobe_angle_margin_deg_);
  }
}

void AntennaPatternParser::load_pattern(const std::string& filepath,
                                        std::vector<double>& angles,
                                        std::vector<double>& gains,
                                        double& peak) {
  std::ifstream file(filepath);
  if (!file.is_open()) {
    std::cerr << "Failed to open file: " << filepath << std::endl;
    return;
  }

  std::string line;
  bool header_found = false;
  angles.clear();
  gains.clear();
  peak = -1000.0;

  while (std::getline(file, line)) {
    // Trim string
    line.erase(0, line.find_first_not_of(" \r\n\t"));
    line.erase(line.find_last_not_of(" \r\n\t") + 1);

    if (line.empty() || line[0] == '#') continue;

    if (!header_found) {
      std::string lower_line = line;
      std::transform(lower_line.begin(), lower_line.end(), lower_line.begin(), ::tolower);
      if (lower_line.find("angle") != std::string::npos) {
        header_found = true;
        continue;
      }
    }

    std::stringstream ss(line);
    std::string token;
    if (std::getline(ss, token, ',')) {
      try {
        double angle = std::stod(token);
        if (std::getline(ss, token, ',')) {
          // Handle '−' full-width minus
          std::string gain_str = token;
          size_t pos;
          while ((pos = gain_str.find("−")) != std::string::npos) {
            gain_str.replace(pos, 3, "-"); // "−" is 3 bytes in UTF-8
          }
          double gain = std::stod(gain_str);
          angles.push_back(angle);
          gains.push_back(gain);
          if (gain > peak) peak = gain;
        }
      } catch (...) {
        continue;
      }
    }
  }

  // Sort by angle
  std::vector<std::pair<double, double>> data;
  data.reserve(angles.size());
  for (size_t i = 0; i < angles.size(); ++i) {
    data.emplace_back(angles[i], gains[i]);
  }
  std::sort(data.begin(), data.end());
  for (size_t i = 0; i < data.size(); ++i) {
    angles[i] = data[i].first;
    gains[i] = data[i].second;
  }
}

double AntennaPatternParser::linear_interp(const std::vector<double>& xs,
                                           const std::vector<double>& ys,
                                           double x, double fill) {
  if (xs.empty()) return fill;
  auto it = std::lower_bound(xs.begin(), xs.end(), x);
  if (it == xs.begin()) {
    if (x == xs.front()) return ys.front();
    return fill;
  }
  if (it == xs.end()) {
    if (x == xs.back()) return ys.back();
    return fill;
  }
  size_t idx = std::distance(xs.begin(), it);
  double x1 = xs[idx - 1];
  double y1 = ys[idx - 1];
  double x2 = xs[idx];
  double y2 = ys[idx];
  if (std::abs(x2 - x1) < 1e-12) return y1;
  return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
}

double AntennaPatternParser::get_e_plane_gain(double angle_deg) const {
  return linear_interp(e_angles_, e_gains_, angle_deg);
}

double AntennaPatternParser::get_h_plane_gain(double angle_deg) const {
  return linear_interp(h_angles_, h_gains_, angle_deg);
}

Eigen::Matrix3d AntennaPatternParser::rpy_to_rotmat(double roll, double pitch, double yaw) {
  double cr = std::cos(roll), sr = std::sin(roll);
  double cp = std::cos(pitch), sp = std::sin(pitch);
  double cy = std::cos(yaw), sy = std::sin(yaw);

  Eigen::Matrix3d rx;
  rx << 1.0, 0.0, 0.0,
        0.0, cr, -sr,
        0.0, sr, cr;

  Eigen::Matrix3d ry;
  ry << cp, 0.0, sp,
        0.0, 1.0, 0.0,
       -sp, 0.0, cp;

  Eigen::Matrix3d rz;
  rz << cy, -sy, 0.0,
        sy, cy, 0.0,
        0.0, 0.0, 1.0;

  return rz * ry * rx;
}

double AntennaPatternParser::wrap_pi(double angle_rad) {
  double res = std::fmod(angle_rad + M_PI, 2.0 * M_PI);
  if (res < 0) res += 2.0 * M_PI;
  return res - M_PI;
}

std::pair<double, double> AntennaPatternParser::calculate_antenna_frame_angles(
    const Eigen::Vector3d& antenna_pos,
    const Eigen::Vector3d& target_pos,
    const Eigen::Vector3d& antenna_rpy,
    const Eigen::Matrix3d* rotmat) const {
  Eigen::Vector3d v_world = target_pos - antenna_pos;
  double norm = v_world.norm();
  if (norm <= 1e-12) return {0.0, 0.0};
  v_world.normalize();

  Eigen::Matrix3d r;
  if (rotmat) {
    r = *rotmat;
  } else {
    r = rpy_to_rotmat(antenna_rpy.x(), antenna_rpy.y(), antenna_rpy.z());
  }

  Eigen::Vector3d v_ant = r.transpose() * v_world;

  double az = std::atan2(v_ant.y(), v_ant.x());
  double clamped_z = std::clamp(v_ant.z(), -1.0, 1.0);
  double el = std::asin(clamped_z);

  return {el, wrap_pi(az)};
}

AntennaPatternParser::GainResult AntennaPatternParser::get_gain_from_angles(double elevation_rad, double azimuth_rad) const {
  double elevation_deg = elevation_rad * 180.0 / M_PI;
  double azimuth_deg = azimuth_rad * 180.0 / M_PI;

  double e_gain = get_e_plane_gain(elevation_deg);
  double h_gain = get_h_plane_gain(azimuth_deg);
  double g_peak = std::max(e_plane_peak_, h_plane_peak_);

  double e_atten = std::min(g_peak - e_gain, max_attenuation_);
  double h_atten = std::min(g_peak - h_gain, max_attenuation_);

  double total = g_peak - e_atten - h_atten;
  double min_gain = g_peak - max_attenuation_;
  total = std::max(total, min_gain);

  return {e_gain, h_gain, total};
}

AntennaPatternParser::TxRxGainResult AntennaPatternParser::get_tx_rx_gains(
    const Eigen::Vector3d& tx_pos, const Eigen::Vector3d& tx_rpy,
    const Eigen::Vector3d& rx_pos, const Eigen::Vector3d& rx_rpy,
    const Eigen::Matrix3d* tx_rotmat,
    const Eigen::Matrix3d* rx_rotmat) const {
  
  auto [tx_el, tx_az] = calculate_antenna_frame_angles(tx_pos, rx_pos, tx_rpy, tx_rotmat);
  auto [rx_el, rx_az] = calculate_antenna_frame_angles(rx_pos, tx_pos, rx_rpy, rx_rotmat);

  auto tx_res = get_gain_from_angles(tx_el, tx_az);
  auto rx_res = get_gain_from_angles(rx_el, rx_az);

  return {tx_res.e_gain, tx_res.h_gain, tx_res.total,
          rx_res.e_gain, rx_res.h_gain, rx_res.total};
}

bool AntennaPatternParser::is_in_main_lobe(double elevation_deg, double azimuth_deg) const {
  double abs_el = std::abs(elevation_deg);
  double abs_az = std::abs(azimuth_deg);
  return (abs_el <= e_mainlobe_half_angle_) && (abs_az <= h_mainlobe_half_angle_);
}

double AntennaPatternParser::detect_first_null_angle(const std::vector<double>& angles,
                                                     const std::vector<double>& gains) const {
  if (angles.empty() || gains.empty()) return 0.0;
  
  // Find index closest to boresight (0.0)
  size_t center_idx = 0;
  double min_diff = 1e9;
  for (size_t i = 0; i < angles.size(); ++i) {
    double diff = std::abs(angles[i]);
    if (diff < min_diff) {
      min_diff = diff;
      center_idx = i;
    }
  }

  double peak_gain = gains[center_idx];

  // Scan outward to find first index where gain drops below peak - 3dB
  size_t hpbw_idx = center_idx;
  for (size_t i = center_idx; i < angles.size(); ++i) {
    if (gains[i] <= peak_gain - 3.0) {
      hpbw_idx = i;
      break;
    }
  }

  // Scan further outward to find the first local minimum (null angle)
  size_t null_idx = hpbw_idx;
  for (size_t i = hpbw_idx + 1; i < angles.size() - 1; ++i) {
    if (gains[i] < gains[i - 1] && gains[i] < gains[i + 1]) {
      null_idx = i;
      break;
    }
  }

  return std::abs(angles[null_idx]);
}

} // namespace comms_sim
