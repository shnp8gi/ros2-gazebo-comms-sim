#include "comms_sim_pkg/drone_link.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

// Gazeboを起動せず、シミュレータと同じ通信コアで正対時の電力を出力する。
int main(int argc, char **argv) {
  try {
    if (argc != 2) throw std::invalid_argument("使い方: link_budget_report 設定.yaml");
    auto config = comms_sim::ScenarioConfig::Load(argv[1]);
    config.scenario = "cross"; config.antenna_mode = "tracking";
    std::cout << "distance_m,direction,frequency_hz,tx_output_dbm,tx_after_loss_dbm,tx_gain_dbi,eirp_dbm,path_loss_db,isotropic_rx_dbm,rx_gain_dbi,antenna_rx_dbm,after_lna_dbm,after_mixer_dbm,receiver_input_dbm,sensitivity_dbm,margin_db,thermal_noise_dbm,input_referred_noise_dbm,receiver_noise_dbm,snr_db,required_snr_db,reachable\n";
    std::cout << std::fixed << std::setprecision(9);
    for (double distance : {1., 4., 8., 10., 12., std::sqrt(200.), 16.}) {
      config.b.altitude_m = config.a.altitude_m + distance;
      const auto sample = comms_sim::RadioModel(config).Evaluate(config.encounter_s);
      for (bool ab : {true, false}) {
        const auto &b = ab ? sample.budget_ab : sample.budget_ba;
        const auto &r = ab ? config.ab : config.ba;
        std::cout << distance << ',' << (ab ? "A_TO_B" : "B_TO_A") << ',' << r.frequency_hz
          << ',' << b.tx_output_dbm << ',' << b.tx_after_loss_dbm << ',' << b.tx_gain_dbi
          << ',' << b.eirp_dbm << ',' << b.path_loss_db << ',' << b.isotropic_rx_dbm
          << ',' << b.rx_gain_dbi << ',' << b.antenna_rx_dbm << ',' << b.after_lna_dbm
          << ',' << b.after_mixer_dbm << ',' << b.receiver_input_dbm << ',' << r.sensitivity_dbm
          << ',' << b.receiver_input_dbm-r.sensitivity_dbm << ',' << b.thermal_noise_dbm
          << ',' << b.input_referred_noise_dbm << ',' << b.receiver_noise_dbm << ',' << b.snr_db
          << ',' << r.required_snr_db.value_or(std::numeric_limits<double>::quiet_NaN())
          << ',' << (ab ? sample.ab_ok : sample.ba_ok) << '\n';
      }
    }
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
