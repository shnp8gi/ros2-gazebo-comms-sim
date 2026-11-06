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
    near(defaults.attenuation_limit_db,30,"既存設定の減衰上限を維持");
    AntennaPatternParser raw_pattern(std::numeric_limits<double>::infinity());
    raw_pattern.load_e_plane(defaults.e_plane);raw_pattern.load_h_plane(defaults.h_plane);
    const double peak=std::max(raw_pattern.e_plane_peak(),raw_pattern.h_plane_peak());
    const auto off_axis_raw=raw_pattern.get_gain_from_angles(0,M_PI/2);
    near(off_axis_raw.total,raw_pattern.get_e_plane_gain(0)+raw_pattern.get_h_plane_gain(90)-peak,"CSV合成値を30dBで打ち切らない");
    if(off_axis_raw.total>=peak-30)throw std::runtime_error("打切りなしの低利得を検証できていません");
    auto symmetric=defaults;symmetric.attenuation_limit_db=std::numeric_limits<double>::infinity();
    symmetric.ab.tx_antenna_gain_dbi.reset();symmetric.ab.rx_antenna_gain_dbi.reset();
    symmetric.ab.frequency_hz=60e9;symmetric.ab.power_mw=std::pow(10.,-.7);
    symmetric.ab.tx_insertion_loss_db=3;symmetric.ab.rx_lna_gain_db=0;
    symmetric.ab.rx_conversion_gain_db=3;symmetric.ab.rx_insertion_loss_db=3;
    symmetric.ba=symmetric.ab;symmetric.scenario="cross";symmetric.antenna_mode="fixed";
    symmetric.Validate();
    for(double t:{0.,19.,20.,21.,40.}){
      const auto sample=RadioModel(symmetric).Evaluate(t);
      near(sample.rssi_ab,sample.rssi_ba,"両機60GHz・同一回路での双方向電力一致");
      near(sample.budget_ab.tx_gain_dbi,sample.gain_a,"A送信はCSV合成利得そのもの");
      near(sample.budget_ba.rx_gain_dbi,sample.gain_a,"A受信はCSV合成利得そのもの");
    }
    near(defaults.ab.frequency_hz, 61e9, "PRDEV→PRC周波数");
    near(defaults.ba.frequency_hz, 272e9, "PRC→PRDEV周波数");
    near(10*std::log10(defaults.ab.power_mw), -8, "PRDEV送信電力");
    near(10*std::log10(defaults.ba.power_mw), -1, "PRC送信電力");
    near(defaults.ba.rx_lna_gain_db, 25, "PRDEV受信LNA");
    near(defaults.ab.rx_lna_gain_db, 0, "PRC受信LNAなし");
    near(defaults.ab.sensitivity_dbm, -68.5, "設定AB感度維持");
    near(defaults.ba.sensitivity_dbm, -68.5, "設定BA感度維持");
    if (defaults.ab.receiver_model != "rf_chain" || defaults.ba.receiver_model != "rf_chain")
      throw std::runtime_error("旧回路設定の参照面が変わっています");
    // 新構成の検証: 0 dBm - 3 dB + 20 dBi - 100 dB + 24 dBi = -59 dBm。
    RadioConfig nf;
    near(nf.noise_temperature_k, 290, "kTBの既定温度は290K");
    near(defaults.ab.noise_temperature_k, 290, "AB設定の雑音温度");
    near(defaults.ba.noise_temperature_k, 290, "BA設定の雑音温度");
    nf.receiver_model = "equivalent_nf";
    nf.power_mw = 1; nf.tx_insertion_loss_db = 3;
    nf.noise_bandwidth_hz = 2.1648e9;
    nf.receiver_noise_figure_db = 10;
    const auto nf_budget = calculate_power_budget(nf, 100, 20, 24);
    near(nf_budget.eirp_dbm, 17, "送信機損失を送信アンテナ前に適用");
    near(nf_budget.receiver_input_dbm, -59, "等価NFモデルのアンテナ端信号");
    near(nf_budget.thermal_noise_dbm, -80.6210094016926, "290K・2.1648GHzのkTB");
    near(nf_budget.receiver_noise_dbm, -70.6210094016926, "等価NFモデルの入力換算雑音");
    near(nf_budget.snr_db, 11.6210094016926, "等価NFモデルのSNR");
    auto warm = nf; warm.noise_temperature_k = 300;
    const auto warm_budget = calculate_power_budget(warm, 100, 20, 24);
    near(nf_budget.receiver_input_dbm, warm_budget.receiver_input_dbm, "温度変更で信号電力は変わらない");
    near(nf_budget.snr_db - warm_budget.snr_db, 0.147232568207064, "300Kから290KへのSNR改善");
    if (!std::isnan(nf_budget.after_lna_dbm) || !std::isnan(nf_budget.after_mixer_dbm))
      throw std::runtime_error("存在しない受信回路段に電力を記録しています");
    // 送信電力の参照位置を移し、損失を0にすれば同じリンクを再現できる。
    auto at_terminal = nf;
    at_terminal.power_mw = std::pow(10., -3./10); at_terminal.tx_insertion_loss_db = 0;
    const auto terminal_budget = calculate_power_budget(at_terminal, 100, 20, 24);
    near(terminal_budget.receiver_input_dbm, nf_budget.receiver_input_dbm, "給電端電力と送信損失0の等価性");
    near(terminal_budget.snr_db, nf_budget.snr_db, "送信電力基準の変更でSNRを維持");
    auto less_loss = nf; less_loss.tx_insertion_loss_db = 0;
    const auto less_loss_budget = calculate_power_budget(less_loss, 100, 20, 24);
    near(less_loss_budget.snr_db - nf_budget.snr_db, 3, "送信損失3dBを外すとSNRが3dB上昇");
    near(less_loss_budget.receiver_noise_dbm, nf_budget.receiver_noise_dbm, "送信損失は受信雑音に含めない");
    nf.receiver_noise_figure_db += 3;
    const auto noisy_budget = calculate_power_budget(nf, 100, 20, 24);
    near(noisy_budget.receiver_input_dbm, nf_budget.receiver_input_dbm, "NFは信号を減衰させない");
    near(noisy_budget.snr_db - nf_budget.snr_db, -3, "NF増加はSNRに一度だけ反映");
    // 現行60GHz設定は旧総受信利得0dB。閾値・MCS・曲線の参照電力を維持する。
    for (const std::string model : {"threshold", "statistical", "theoretical"}) {
      const auto current = ScenarioConfig::Load(std::string(argv[1])+"/scenarios/drone_dual60_cross_fixed_"+model+".yaml");
      if (current.ab.receiver_model != "equivalent_nf" || current.ba.receiver_model != "equivalent_nf")
        throw std::runtime_error("双方向60GHzに等価NFモデルが適用されていません");
      auto legacy = current;
      for (auto *r : {&legacy.ab, &legacy.ba}) {
        r->receiver_model = "rf_chain"; r->rx_conversion_gain_db = 3; r->rx_insertion_loss_db = 3;
      }
      const RadioModel current_radio(current), legacy_radio(legacy);
      for (double t : {0., 19., 22.928932188, 30., 40.}) {
        const auto actual = current_radio.Evaluate(t), previous = legacy_radio.Evaluate(t);
        near(actual.rssi_ab, previous.rssi_ab, "60GHz AB参照電力の維持");
        near(actual.rssi_ba, previous.rssi_ba, "60GHz BA参照電力の維持");
        near(actual.budget_ab.snr_db, previous.budget_ab.snr_db, "60GHz AB SNRの維持");
        near(actual.budget_ba.snr_db, previous.budget_ba.snr_db, "60GHz BA SNRの維持");
        if (current.protocol.hrcp.theoretical.enabled) {
          if (!std::isnan(actual.rate_ab_gbps) || !std::isnan(actual.rate_ba_gbps))
            throw std::runtime_error("理論PHYが旧MCS表のレートを参照しています");
        } else {
          near(actual.rate_ab_gbps, previous.rate_ab_gbps, "60GHz AB MCS表参照の維持");
          near(actual.rate_ba_gbps, previous.rate_ba_gbps, "60GHz BA MCS表参照の維持");
        }
        if (actual.ab_ok != previous.ab_ok || actual.ba_ok != previous.ba_ok)
          throw std::runtime_error("60GHzの閾値判定が変わっています");
      }
    }
    auto fixture = YAML::LoadFile(argv[2]);
    // Excelはc=3e8。シミュレータの厳密なcによりPLが増える分だけ受信電力が下がる。
    const double correction = 20*std::log10(3e8/299792458.0);
    for (const auto &sheet : fixture["sheets"]) {
      const auto cells = sheet.second;
      auto value = [&](const std::string &cell) { return cells[cell]["value"].as<double>(); };
      auto c = defaults;
      // 原資料の300K条件を明示して照合する。現行の既定温度は290K。
      c.ab.noise_temperature_k = c.ba.noise_temperature_k = 300;
      c.scenario = "cross"; c.antenna_mode = "tracking";
      c.b.altitude_m = c.a.altitude_m + value("B3");
      c.ba.power_mw = std::pow(10., value("B11")/10);
      c.ab.power_mw = std::pow(10., value("W11")/10);
      auto sample = RadioModel(c).Evaluate(c.encounter_s);
      for (bool ab : {true, false}) {
        std::string col = ab ? "W" : "B";
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
        if (!ab) {
          near(b.after_lna_dbm, expected(26)-correction, "LNA後");
          near(b.after_mixer_dbm, expected(28)-correction, "Mixer後");
        }
        near(b.receiver_input_dbm, expected(ab ? 26 : 30)-correction, "TJX入力");
        near(ab ? sample.rssi_ab : sample.rssi_ba, b.receiver_input_dbm, "受信判定に使う電力");
        near(b.thermal_noise_dbm, expected(9), "Excel kTB");
        if (!ab) near(b.input_referred_noise_dbm, value("B32"), "Excel kTB+NF");
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
    auto original = defaults.ba;
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
    std::cout << "等価NF・送信機損失・60GHz互換性、Excel全4距離の各段電力、方向別利得、既定値、損失符号の検証成功\n";
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
