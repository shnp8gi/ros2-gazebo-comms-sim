// 別紙用の受信電力走査。通信コアと同じRadioModelを使用する。
#include "comms_sim_pkg/drone_link.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>

int main(int argc, char **argv) {
  if (argc != 4) throw std::invalid_argument("config.yaml old|corrected 出力prefix");
  auto c = comms_sim::ScenarioConfig::Load(argv[1]);
  c.scenario = "cross"; c.antenna_mode = "fixed";
  c.a.speed_kmh = c.b.speed_kmh = 18.; c.encounter_s = 6.; c.duration_s = 18.;
  if (std::string(argv[2]) == "old") {
    c.a.fixed_azimuth_offset_deg.reset(); c.b.fixed_azimuth_offset_deg.reset();
  }
  c.Validate(); comms_sim::RadioModel radio(c);
  std::ofstream curve(std::string(argv[3]) + "_curve.csv");
  curve << std::setprecision(15)
        << "time_s,remaining_path_m,distance_m,p_dbm,q_dbm,gain_a_dbi,gain_b_dbi,off_axis_a_deg,off_axis_b_deg\n";
  double maxq = -std::numeric_limits<double>::infinity(), maxt = 0, minangle = 180.;
  for (int i = 0; i <= 180000; ++i) {
    double t = i * .0001; auto r = radio.Evaluate(t);
    if (r.rssi_ab > maxq) { maxq = r.rssi_ab; maxt = t; }
    minangle = std::min(minangle, r.off_axis_a_deg);
    if (i % 100 == 0)
      curve << t << ',' << 30 - 5*t << ',' << r.distance << ',' << r.rssi_ba << ',' << r.rssi_ab
            << ',' << r.gain_a << ',' << r.gain_b << ',' << r.off_axis_a_deg << ',' << r.off_axis_b_deg << '\n';
  }
  auto r = radio.Evaluate(maxt);
  std::ofstream out(std::string(argv[3]) + "_maximum.csv");
  out << std::setprecision(15)
      << "time_s,remaining_path_m,distance_m,p_dbm,q_dbm,gain_a_dbi,gain_b_dbi,off_axis_a_deg,off_axis_b_deg,min_off_axis_deg\n"
      << maxt << ',' << 30 - 5*maxt << ',' << r.distance << ',' << r.rssi_ba << ',' << r.rssi_ab
      << ',' << r.gain_a << ',' << r.gain_b << ',' << r.off_axis_a_deg << ',' << r.off_axis_b_deg << ',' << minangle << '\n';
  if (!curve || !out) throw std::runtime_error("CSV書込み失敗");
}
