#include "comms_sim_pkg/drone_link.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace comms_sim;
void near(double actual, double expected, const std::string &label) {
  if (!std::isfinite(actual) || std::abs(actual-expected) > 1e-8)
    throw std::runtime_error(label + ": actual=" + std::to_string(actual)
                             + ", expected=" + std::to_string(expected));
}
int main(int argc, char **argv) {
  try {
    if (argc != 3) throw std::invalid_argument("設定ディレクトリとExcel検証値が必要です");
    const auto defaults = ScenarioConfig::Load(std::string(argv[1])+"/drone_params.yaml");
    near(defaults.ab.frequency_hz, 272e9, "設定AB周波数");
    near(defaults.ba.frequency_hz, 61e9, "設定BA周波数");
    near(10*std::log10(defaults.ab.power_mw), -1, "設定AB電力");
    near(10*std::log10(defaults.ba.power_mw), -8, "設定BA電力");
    near(defaults.ab.sensitivity_dbm, -68.5, "設定AB感度維持");
    near(defaults.ba.sensitivity_dbm, -68.5, "設定BA感度維持");
    auto fixture = YAML::LoadFile(argv[2]);
    // Excelはc=3e8。シミュレータの厳密なcによりPLが増える分だけ受信電力が下がる。
    const double correction = 20*std::log10(3e8/299792458.0);
    for (const auto &sheet : fixture["sheets"]) {
      const auto cells = sheet.second;
      auto value = [&](const std::string &cell) { return cells[cell]["value"].as<double>(); };
      auto c = defaults;
      c.scenario = "cross"; c.antenna_mode = "tracking";
      c.b.altitude_m = c.a.altitude_m + value("B3");
      c.ab.power_mw = std::pow(10., value("B11")/10);
      c.ba.power_mw = std::pow(10., value("W11")/10);
      auto sample = RadioModel(c).Evaluate(c.encounter_s);
      for (bool ab : {true, false}) {
        std::string col = ab ? "B" : "W";
        auto &b = ab ? sample.budget_ab : sample.budget_ba;
        auto expected = [&](int row) { return value(col+std::to_string(row)); };
        near(b.tx_output_dbm, expected(11), "送信電力");
        near(b.tx_after_loss_dbm, expected(11)+expected(12), "送信損失の符号");
        near(b.tx_gain_dbi, expected(16), "送信アンテナ利得");
        near(b.rx_gain_dbi, expected(23), "受信アンテナ利得");
        near(b.eirp_dbm, expected(17), "EIRP");
        near(b.path_loss_db, expected(8)+correction, "FSPL");
        near(b.isotropic_rx_dbm, expected(19)-correction, "等方受信電力");
        near(b.antenna_rx_dbm, expected(24)-correction, "アンテナ端受信電力");
        if (ab) {
          near(b.after_lna_dbm, expected(26)-correction, "LNA後");
          near(b.after_mixer_dbm, expected(28)-correction, "Mixer後");
        }
        near(b.receiver_input_dbm, expected(ab ? 30 : 26)-correction, "TJX入力");
        near(ab ? sample.rssi_ab : sample.rssi_ba, b.receiver_input_dbm, "受信判定に使う電力");
        near(b.thermal_noise_dbm, expected(9), "Excel kTB");
        if (ab) near(b.input_referred_noise_dbm, value("B32"), "Excel kTB+NF");
        near(b.snr_db, b.antenna_rx_dbm-b.input_referred_noise_dbm, "同一基準点のSNR");
      }
    }
    // 異なる送受信利得と向きで、逆方向のTx/Rx取り違え・二重加算を検出する。
    auto c = defaults;
    c.scenario = "cross"; c.antenna_mode = "fixed";
    c.ba.tx_antenna_gain_dbi = 17.; c.ba.rx_antenna_gain_dbi = 29.;
    AntennaPatternParser pattern(c.attenuation_limit_db);
    pattern.load_e_plane(c.e_plane); pattern.load_h_plane(c.h_plane);
    const double boresight = pattern.get_gain_from_angles(0,0).total;
    auto sample = RadioModel(c).Evaluate(19.7);
    near(sample.budget_ba.tx_gain_dbi, 17+sample.gain_b-boresight, "BA送信はB側の指向性");
    near(sample.budget_ba.rx_gain_dbi, 29+sample.gain_a-boresight, "BA受信はA側の指向性");
    near(sample.rssi_ab, sample.budget_ab.receiver_input_dbm, "軸外AB電力");
    near(sample.rssi_ba, sample.budget_ba.receiver_input_dbm, "軸外BA電力");
    auto original = defaults.ab;
    const auto reference = calculate_power_budget(original, 100, 24, 24);
    original.rx_lna_gain_db += 10;
    auto amplified = calculate_power_budget(original, 100, 24, 24);
    near(amplified.receiver_input_dbm-reference.receiver_input_dbm, 10, "信号増幅");
    near(amplified.receiver_noise_dbm-reference.receiver_noise_dbm, 10, "雑音も増幅");
    near(amplified.snr_db, reference.snr_db, "等価NF一定では増幅でSNRが改善しない");
    original.noise_bandwidth_hz *= 2;
    auto wide = calculate_power_budget(original, 100, 24, 24);
    near(wide.snr_db-amplified.snr_db, -10*std::log10(2.), "帯域倍増のSNR低下");
    original.receiver_noise_figure_db += 3;
    near(calculate_power_budget(original,100,24,24).snr_db-wide.snr_db, -3, "NF増加のSNR低下");
    c = defaults; c.antenna_mode = "tracking";
    auto monitored = RadioModel(c).Evaluate(20);
    c.ab.required_snr_db = monitored.budget_ab.snr_db + 1;
    auto blocked = RadioModel(c).Evaluate(20);
    if (blocked.ab_ok || blocked.snr_ab_ok) throw std::runtime_error("SNR不足を拒否できていません");
    near(blocked.rssi_ab, monitored.rssi_ab, "雑音電力を信号RSSIへ混入しない");
    bool snr_failure = false;
    LinkProtocol noisy(c.protocol, [blocked](double){return blocked;}, [&](const LinkEvent &e) {
      if (e.type=="RX_FAILED" && e.detail=="SNR_BELOW_REQUIRED") snr_failure = true;
    });
    noisy.Advance(.01);
    if (!snr_failure) throw std::runtime_error("SNR不足の失敗理由がありません");
    c.ab.required_snr_db = monitored.budget_ab.snr_db;
    if (!RadioModel(c).Evaluate(20).ab_ok) throw std::runtime_error("SNR閾値ちょうどを許可しません");
    c.ab.rx_insertion_loss_db = -3;
    bool rejected = false;
    try { c.Validate(); } catch (const std::invalid_argument &) { rejected = true; }
    if (!rejected) throw std::runtime_error("負値の損失を拒否できていません");
    std::cout << "Excel全4距離の各段電力、方向別利得、既定値、損失符号の検証成功\n";
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
