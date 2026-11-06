// MAC送信の有無と独立に、同じ仮想フレームのP/Q受信可能性を比較する。
#include "comms_sim_pkg/drone_link.hpp"
#include "comms_sim_pkg/hrcp_mac.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

int main(int argc, char **argv) {
  try {
    if (argc != 3) throw std::invalid_argument("使い方: asymmetric_link_profile config.yaml new_output.csv");
    if (std::filesystem::exists(argv[2])) throw std::invalid_argument("既存結果は上書きしません");
    const auto c = comms_sim::ScenarioConfig::Load(argv[1]);
    if (!c.protocol.hrcp.theoretical.enabled) throw std::invalid_argument("理論PHY設定が必要です");
    const comms_sim::RadioModel radio(c);
    const comms_sim::TjxTheoreticalPhy phy(c.protocol.hrcp.theoretical);
    const auto control_bytes = comms_sim::hrcp_beacon_encode(comms_sim::HrcpBeacon{}).size();
    const auto data_bytes = comms_sim::hrcp_subframe(0, std::vector<uint8_t>(c.protocol.hrcp.data_bytes)).size();
    std::ofstream out(argv[2]);
    if (!out) throw std::runtime_error("出力先を開けません");
    out << std::setprecision(17)
        << "time_s,remaining_path_m,distance_m,p_snr_db,q_snr_db,p_antenna_dbm,q_antenna_dbm,p_tjx_dbm,q_tjx_dbm,p_tx_gain_dbi,p_rx_gain_dbi,q_tx_gain_dbi,q_rx_gain_dbi,off_axis_a_deg,off_axis_b_deg,control_bytes,p_control_per,q_control_per,p_control_lut_outside,q_control_lut_outside,p_data_mcs3_per,q_data_mcs3_per,p_data_mcs8_per,q_data_mcs8_per\n";
    for (uint64_t i=0;;++i) {
      const double t=std::min(i*c.log_interval_s,c.duration_s);
      const auto r=radio.Evaluate(t);
      const auto &p=r.budget_ba, &q=r.budget_ab;
      const auto pc=phy.Evaluate(3,control_bytes,p.snr_db,c.ba.noise_bandwidth_hz);
      const auto qc=phy.Evaluate(3,control_bytes,q.snr_db,c.ab.noise_bandwidth_hz);
      out << t << ',' << (c.encounter_s-t)*c.a.speed_kmh/3.6 << ',' << r.distance
          << ',' << p.snr_db << ',' << q.snr_db << ',' << p.antenna_rx_dbm << ',' << q.antenna_rx_dbm
          << ',' << p.receiver_input_dbm << ',' << q.receiver_input_dbm
          << ',' << p.tx_gain_dbi << ',' << p.rx_gain_dbi << ',' << q.tx_gain_dbi << ',' << q.rx_gain_dbi
          << ',' << r.off_axis_a_deg << ',' << r.off_axis_b_deg << ',' << control_bytes
          << ',' << pc.per << ',' << qc.per << ',' << pc.outside_range << ',' << qc.outside_range;
      for (unsigned mcs : {3u,8u})
        out << ',' << phy.Evaluate(mcs,data_bytes,p.snr_db,c.ba.noise_bandwidth_hz).per
            << ',' << phy.Evaluate(mcs,data_bytes,q.snr_db,c.ab.noise_bandwidth_hz).per;
      out << '\n';
      if (t>=c.duration_s) break;
    }
    if (!out) throw std::runtime_error("RF診断結果の書込みに失敗しました");
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
