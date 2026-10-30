#include "comms_sim_pkg/drone_link.hpp"
#include "comms_sim_pkg/hrcp_signal_monitor.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>

namespace comms_sim {
namespace {
constexpr double pi = 3.14159265358979323846;
double number(const YAML::Node &node, const char *key, double fallback) {
  return node && node[key] ? node[key].as<double>() : fallback;
}
void positive(double value, const char *name) {
  if (!std::isfinite(value) || value <= 0)
    throw std::invalid_argument(std::string(name) + " は正の有限値が必要です");
}
}

ScenarioConfig ScenarioConfig::Load(const std::string &path) {
  auto y = YAML::LoadFile(path);
  ScenarioConfig c;
  c.preview_mode=y["preview_mode"]&&y["preview_mode"].as<bool>();
  c.monitor_only=y["monitor_only"]&&y["monitor_only"].as<bool>();
  c.scenario = y["scenario"].as<std::string>();
  c.antenna_mode = y["antenna_mode"].as<std::string>();
  auto drone = [&](const char *key, DroneConfig &d) {
    d.speed_kmh = number(y[key], "speed_kmh", d.speed_kmh);
    d.altitude_m = number(y[key], "altitude_m", d.altitude_m);
    d.fixed_elevation_deg = number(y[key], "fixed_elevation_deg", d.fixed_elevation_deg);
  };
  drone("drone_a", c.a); drone("drone_b", c.b);
  auto radio = [&](const char *key, RadioConfig &r) {
    r.frequency_hz = number(y[key], "frequency_hz", r.frequency_hz);
    r.power_mw = number(y[key], "power_mw", r.power_mw);
    r.sensitivity_dbm = number(y[key], "sensitivity_dbm", r.sensitivity_dbm);
    auto node = y[key];
    if (node["tx_antenna_gain_dbi"]) r.tx_antenna_gain_dbi = node["tx_antenna_gain_dbi"].as<double>();
    if (node["rx_antenna_gain_dbi"]) r.rx_antenna_gain_dbi = node["rx_antenna_gain_dbi"].as<double>();
    r.tx_insertion_loss_db = number(node, "tx_insertion_loss_db", 0);
    r.rx_insertion_loss_db = number(node, "rx_insertion_loss_db", 0);
    r.rx_lna_gain_db = number(node, "rx_lna_gain_db", 0);
    r.rx_conversion_gain_db = number(node, "rx_conversion_gain_db", 0);
    r.noise_bandwidth_hz = number(node, "noise_bandwidth_hz", 0);
    r.noise_temperature_k = number(node, "noise_temperature_k", 300);
    r.receiver_noise_figure_db = number(node, "receiver_noise_figure_db", 0);
    if (node["required_snr_db"] && !node["required_snr_db"].IsNull())
      r.required_snr_db = node["required_snr_db"].as<double>();
  };
  radio("radio_ab", c.ab); radio("radio_ba", c.ba);
  c.encounter_s = number(y, "encounter_time_s", c.encounter_s);
  c.duration_s = number(y, "duration_s", c.duration_s);
  c.lateral_separation_m = number(y, "lateral_separation_m", c.lateral_separation_m);
  c.path_loss_exponent = number(y["propagation"], "exponent", c.path_loss_exponent);
  // 明示的なnullは減衰の打切りなし。省略時は従来の30 dBを維持する。
  const auto attenuation_limit = y["antenna"]["max_attenuation_db"];
  c.attenuation_limit_db = attenuation_limit && attenuation_limit.IsNull()
      ? std::numeric_limits<double>::infinity()
      : number(y["antenna"], "max_attenuation_db", c.attenuation_limit_db);
  auto sim = y["simulation"];
  c.allow_pre_encounter_end = sim["allow_pre_encounter_end"] && sim["allow_pre_encounter_end"].as<bool>();
  c.physics_step_s = number(sim, "physics_step_s", c.physics_step_s);
  c.real_time_factor = number(sim, "real_time_factor", c.real_time_factor);
  c.log_interval_s = number(sim, "log_interval_s", c.log_interval_s);
  auto p = y["protocol"];
  c.protocol.data_from_prc = p["data_from_prc"] && p["data_from_prc"].as<bool>();
  c.protocol.hrcp.data_from_prc = c.protocol.data_from_prc;
  c.protocol.beacon_interval_s = number(p, "beacon_interval_ms", 0.14) / 1000;
  c.protocol.control_frame_s = number(p, "control_frame_us", 20) / 1e6;
  c.protocol.ifs_s = number(p, "ifs_us", 10) / 1e6;
  c.protocol.data_frame_s = number(p, "data_frame_us", 100) / 1e6;
  c.protocol.response_timeout_s = number(p, "response_timeout_ms", 500) / 1000;
  c.protocol.ack_timeout_s = number(p, "ack_timeout_ms", 500) / 1000;
  c.protocol.inactivity_timeout_s = number(p, "inactivity_timeout_ms", 500) / 1000;
  c.protocol.channel_sample_s = c.physics_step_s;
  auto awgn = y["awgn"];
  if (awgn) {
    c.protocol.awgn.enabled = awgn["enabled"] ? awgn["enabled"].as<bool>() : false;
    if (awgn["seed"]) c.protocol.awgn.seed = awgn["seed"].as<uint64_t>();
    c.protocol.awgn.bit_rate_bps = number(awgn, "bit_rate_bps", 3.52e9);
    if (awgn["control_frame_bytes"]) {
      auto bytes = awgn["control_frame_bytes"].as<uint64_t>();
      if (bytes == 0 || bytes > 125000000000ULL)
        throw std::invalid_argument("制御フレーム長が範囲外です");
      c.protocol.awgn.control_frame_bits = bytes * 8;
    }
  }
  auto lab = y["lab_phy"];
  if (lab) {
    c.protocol.lab_phy.enabled = lab["enabled"] && lab["enabled"].as<bool>();
    c.protocol.lab_phy.seed = c.protocol.awgn.seed;
    if (lab["data_bytes"]) c.protocol.lab_phy.data_bytes = lab["data_bytes"].as<unsigned>();
    if (lab["chunk_samples"]) c.protocol.lab_phy.chunk_samples = lab["chunk_samples"].as<unsigned>();
    if (lab["payload_fec"] && lab["payload_fec"].as<std::string>() != "none")
      throw std::invalid_argument("規定LDPC行列が未設定です。仮PHYはpayload_fec: noneのみ使用できます");
  }
  auto hrcp = y["hrcp"];
  if (hrcp && hrcp["enabled"] && hrcp["enabled"].as<bool>()) {
    auto &h = c.protocol.hrcp; h.enabled = true; h.seed = c.protocol.awgn.seed;
    h.noise_enabled=c.protocol.awgn.enabled;h.monitor_only=c.monitor_only;
    h.data_bytes = hrcp["data_bytes"] ? hrcp["data_bytes"].as<unsigned>() : 2048;
    h.atp_ms = hrcp["atp_ms"] ? hrcp["atp_ms"].as<unsigned>() : 500;
    h.setup_timeout_ms = hrcp["setup_timeout_ms"] ? hrcp["setup_timeout_ms"].as<unsigned>() : 0;
    h.requested_atp_ms = hrcp["requested_atp_ms"] ? hrcp["requested_atp_ms"].as<unsigned>() : h.atp_ms;
    h.reference_distance_m = number(hrcp, "association_reference_distance_m", 30);
    if (hrcp["expected_rssi_admission_enabled"])
      h.expected_rssi_admission_enabled = hrcp["expected_rssi_admission_enabled"].as<bool>();
    if (h.expected_rssi_admission_enabled) {
      positive(h.reference_distance_m, "association_reference_distance_m");
      if (!c.ba.tx_antenna_gain_dbi) throw std::invalid_argument("Expected RSSI計算にはBA送信正面利得が必要です");
      const double pl0 = 20*std::log10(4*pi*c.ba.frequency_hz/299792458.);
      h.expected_rssi_dbm = 10*std::log10(c.ba.power_mw)-c.ba.tx_insertion_loss_db
        + *c.ba.tx_antenna_gain_dbi-pl0-10*c.path_loss_exponent*std::log10(h.reference_distance_m);
    }
    h.bandwidth_ab_hz=c.ab.noise_bandwidth_hz;h.bandwidth_ba_hz=c.ba.noise_bandwidth_hz;
    h.channel_sample_s=c.physics_step_s;
  }
  c.protocol.awgn.bandwidth_ab_hz = c.ab.noise_bandwidth_hz;
  c.protocol.awgn.bandwidth_ba_hz = c.ba.noise_bandwidth_hz;
  auto resolve = [&](const std::string &name) {
    auto target = std::filesystem::path(name);
    if (target.is_relative()) target = std::filesystem::path(path).parent_path() / target;
    if (!std::filesystem::is_regular_file(target))
      throw std::invalid_argument("参照ファイルがありません: " + target.string());
    return std::filesystem::absolute(target).string();
  };
  c.e_plane = resolve(y["antenna"]["e_plane"].as<std::string>());
  c.h_plane = resolve(y["antenna"]["h_plane"].as<std::string>());
  c.mcs_table = resolve(y["mcs_table"].as<std::string>());
  auto empirical=y["empirical_phy"];
  if(empirical && empirical["enabled"] && empirical["enabled"].as<bool>()){
    auto &e=c.protocol.hrcp.empirical;e.enabled=true;
    e.table=resolve(empirical["table"].as<std::string>());
    e.metric=empirical["metric"].as<std::string>();
    if(empirical["per_length_policy"])e.per_length_policy=empirical["per_length_policy"].as<std::string>();
    if(empirical["seed"])e.seed=empirical["seed"].as<uint64_t>();
    if(empirical["control_mcs"])e.control_mcs=empirical["control_mcs"].as<unsigned>();
    e.offset_ab_db=number(empirical,"rssi_offset_ab_db",0);
    e.offset_ba_db=number(empirical,"rssi_offset_ba_db",0);
    if(empirical["mcs_by_rate_gbps"]){e.mcs_by_rate.clear();
      for(const auto &entry:empirical["mcs_by_rate_gbps"])e.mcs_by_rate.emplace(entry.first.as<double>(),entry.second.as<unsigned>());}
  }
  c.output_dir = y["output_dir"] ? y["output_dir"].as<std::string>() : "";
  if (hrcp && hrcp["monitor_enabled"]) c.protocol.hrcp.monitor_enabled=hrcp["monitor_enabled"].as<bool>();
  if (hrcp && hrcp["monitor_waveforms"]) c.protocol.hrcp.monitor_waveforms=hrcp["monitor_waveforms"].as<bool>();
  if (hrcp && hrcp["noise_threads"]) c.protocol.hrcp.noise_threads=hrcp["noise_threads"].as<unsigned>();
  if (hrcp && hrcp["decode_threads"]) c.protocol.hrcp.decode_threads=hrcp["decode_threads"].as<unsigned>();
  if (hrcp && hrcp["monitor_rssi_interval_s"]) c.protocol.hrcp.monitor_rssi_interval_s=hrcp["monitor_rssi_interval_s"].as<double>();
  if (hrcp && hrcp["monitor_rssi_window_s"]) c.protocol.hrcp.monitor_rssi_window_s=hrcp["monitor_rssi_window_s"].as<double>();
  if (c.protocol.hrcp.monitor_enabled) c.protocol.hrcp.monitor_path=(std::filesystem::path(c.output_dir)/"live_signals.json").string();
  c.Validate();
  return c;
}

void ScenarioConfig::Validate() const {
  if (scenario != "parallel" && scenario != "cross")
    throw std::invalid_argument("scenario は parallel または cross です");
  if (antenna_mode != "fixed" && antenna_mode != "tracking")
    throw std::invalid_argument("antenna_mode は fixed または tracking です");
  positive(a.speed_kmh, "drone_a.speed_kmh"); positive(b.speed_kmh, "drone_b.speed_kmh");
  positive(a.altitude_m, "drone_a.altitude_m"); positive(b.altitude_m, "drone_b.altitude_m");
  positive(duration_s, "duration_s"); positive(encounter_s, "encounter_time_s");
  if (encounter_s >= duration_s && !allow_pre_encounter_end)
    throw std::invalid_argument("すれ違い時刻は実行時間の内側に設定してください");
  if (a.altitude_m >= b.altitude_m) throw std::invalid_argument("Aを低い機体、Bを高い機体に設定してください");
  positive(lateral_separation_m, "lateral_separation_m");
  for (const auto &r : {ab, ba}) {
    positive(r.frequency_hz, "frequency_hz"); positive(r.power_mw, "power_mw");
    if (!std::isfinite(r.sensitivity_dbm)) throw std::invalid_argument("受信感度が不正です");
    for (auto gain : {r.tx_antenna_gain_dbi, r.rx_antenna_gain_dbi})
      if (gain && !std::isfinite(*gain)) throw std::invalid_argument("アンテナ利得が不正です");
    for (double loss : {r.tx_insertion_loss_db, r.rx_insertion_loss_db})
      if (!std::isfinite(loss) || loss < 0) throw std::invalid_argument("挿入損失は0以上の有限値で指定してください");
    for (double gain : {r.rx_lna_gain_db, r.rx_conversion_gain_db})
      if (!std::isfinite(gain)) throw std::invalid_argument("受信回路の利得が不正です");
    positive(r.noise_temperature_k, "noise_temperature_k");
    for (double v : {r.noise_bandwidth_hz, r.receiver_noise_figure_db})
      if (!std::isfinite(v) || v < 0) throw std::invalid_argument("雑音帯域幅・NFは0以上の有限値です");
    if (r.required_snr_db && (!std::isfinite(*r.required_snr_db) || r.noise_bandwidth_hz <= 0))
      throw std::invalid_argument("SNR判定には有限の閾値と正の雑音帯域幅が必要です");
  }
  for (double angle : {a.fixed_elevation_deg, b.fixed_elevation_deg})
    if (!std::isfinite(angle) || std::abs(angle) > 90) throw std::invalid_argument("仰角は-90～90度です");
  positive(physics_step_s, "physics_step_s"); positive(real_time_factor, "real_time_factor");
  positive(log_interval_s, "log_interval_s"); positive(path_loss_exponent, "exponent");
  if (attenuation_limit_db != std::numeric_limits<double>::infinity())
    positive(attenuation_limit_db, "max_attenuation_db");
  positive(protocol.beacon_interval_s, "beacon_interval");
  positive(protocol.control_frame_s, "control_frame"); positive(protocol.ifs_s, "ifs");
  positive(protocol.data_frame_s, "data_frame");
  positive(protocol.response_timeout_s, "response_timeout");
  positive(protocol.ack_timeout_s, "ack_timeout");
  positive(protocol.inactivity_timeout_s, "inactivity_timeout");
  positive(protocol.channel_sample_s, "channel_sample");
  if(protocol.hrcp.empirical.enabled){
    if(!protocol.hrcp.enabled||protocol.awgn.enabled||protocol.lab_phy.enabled||preview_mode||monitor_only)
      throw std::invalid_argument("実測モデルはhrcp有効、AWGN/lab_phy/preview_mode/monitor_only無効で使用してください");
    if(protocol.hrcp.expected_rssi_admission_enabled||protocol.hrcp.monitor_waveforms)
      throw std::invalid_argument("実測モデルはExpected RSSIと波形モニターを無効にしてください");
    if(ab.required_snr_db||ba.required_snr_db)throw std::invalid_argument("実測モデルではrequired_snr_dbをnullにしてください");
    EmpiricalPhy check(protocol.hrcp.empirical);
  }
  if (protocol.hrcp.enabled) {
    if (protocol.lab_phy.enabled)
      throw std::invalid_argument("HRCPモードではlab_phyを無効にしてください");
    if (std::abs(protocol.beacon_interval_s-.000140)>1e-12)
      throw std::invalid_argument("今回のHRCP実験プロファイルはBeacon140us固定です");
  }
  if (protocol.lab_phy.enabled) {
    LabPhy phy(protocol.lab_phy);
    if (!protocol.awgn.enabled || protocol.awgn.bit_rate_bps != 3.52e9)
      throw std::invalid_argument("仮PHYにはAWGN有効・3.52 Gbpsが必要です");
    if (phy.Duration(0,false) >= protocol.beacon_interval_s)
      throw std::invalid_argument("Beacon時間が送信周期以上です");
  }
  if (protocol.awgn.enabled) {
    positive(protocol.awgn.bit_rate_bps, "awgn.bit_rate_bps");
    positive(protocol.awgn.bandwidth_ab_hz, "AWGNのAB雑音帯域");
    positive(protocol.awgn.bandwidth_ba_hz, "AWGNのBA雑音帯域");
    const double data_bits = protocol.awgn.bit_rate_bps * protocol.data_frame_s;
    if (protocol.awgn.control_frame_bits == 0 || data_bits < 1 || data_bits > 1e12)
      throw std::invalid_argument("AWGNフレーム長は1～1e12 bitの範囲にしてください");
    if (protocol.awgn.control_frame_bits > protocol.awgn.bit_rate_bps * protocol.control_frame_s)
      throw std::invalid_argument("制御フレームのbit数が送信時間内に収まりません");
  }
  if (protocol.data_frame_s + protocol.control_frame_s + 2 * protocol.ifs_s >= protocol.inactivity_timeout_s)
    throw std::invalid_argument("データとACKの交換時間より無応答タイムアウトを長くしてください");
  if (protocol.control_frame_s + protocol.ifs_s >= std::min(protocol.ack_timeout_s, protocol.response_timeout_s))
    throw std::invalid_argument("フレーム交換時間よりタイムアウトを長くしてください");
}

Geometry geometry(const ScenarioConfig &c, double time) {
  Geometry g;
  const double delta = time - c.encounter_s;
  g.a = {c.a.speed_kmh / 3.6 * delta, 0, c.a.altitude_m};
  if (c.scenario == "parallel") {
    g.b = {-c.b.speed_kmh / 3.6 * delta, c.lateral_separation_m, c.b.altitude_m};
    g.heading_b = pi;
  } else {
    g.b = {0, c.b.speed_kmh / 3.6 * delta, c.b.altitude_m};
    g.heading_b = pi / 2;
  }
  auto point_at = [](const Eigen::Vector3d &d) -> Eigen::Vector3d {
    // Gazeboのpitch正方向は機首下げ。仰角正方向（上向き）は負のpitch。
    return {0, -std::atan2(d.z(), std::hypot(d.x(), d.y())), std::atan2(d.y(), d.x())};
  };
  if (c.antenna_mode == "tracking") {
    g.antenna_a_rpy = point_at(g.b - g.a);
    g.antenna_b_rpy = point_at(g.a - g.b);
  } else {
    // 並行は両機の左側、十字はAの左側/Bの右側へ固定搭載。
    // 十字では交差後に相手のいる側を向く。最接近時の相手は真上/真下。
    const double az_a = g.heading_a + pi / 2;
    const double az_b = c.scenario == "parallel" ? -pi / 2 : g.heading_b - pi / 2;
    g.antenna_a_rpy = {0, -c.a.fixed_elevation_deg * pi / 180, az_a};
    g.antenna_b_rpy = {0, -c.b.fixed_elevation_deg * pi / 180, az_b};
  }
  return g;
}

PowerBudget calculate_power_budget(const RadioConfig &r, double path_loss,
                                  double tx_gain, double rx_gain) {
  PowerBudget b;
  b.tx_gain_dbi = tx_gain; b.rx_gain_dbi = rx_gain; b.path_loss_db = path_loss;
  b.tx_output_dbm = 10 * std::log10(r.power_mw);
  b.tx_after_loss_dbm = b.tx_output_dbm - r.tx_insertion_loss_db;
  b.eirp_dbm = b.tx_after_loss_dbm + tx_gain;
  b.isotropic_rx_dbm = b.eirp_dbm - path_loss;
  b.antenna_rx_dbm = b.isotropic_rx_dbm + rx_gain;
  b.after_lna_dbm = b.antenna_rx_dbm + r.rx_lna_gain_db;
  b.after_mixer_dbm = b.after_lna_dbm + r.rx_conversion_gain_db;
  b.receiver_input_dbm = b.after_mixer_dbm - r.rx_insertion_loss_db;
  if (r.noise_bandwidth_hz > 0) {
    b.thermal_noise_dbm = 10*std::log10(1.380649e-23 * r.noise_temperature_k * r.noise_bandwidth_hz)+30;
    // NFはアンテナ出力を基準とした受信系全体の等価値として扱う。
    b.input_referred_noise_dbm = b.thermal_noise_dbm + r.receiver_noise_figure_db;
    b.receiver_noise_dbm = b.input_referred_noise_dbm + r.rx_lna_gain_db
      + r.rx_conversion_gain_db - r.rx_insertion_loss_db;
    b.snr_db = b.receiver_input_dbm - b.receiver_noise_dbm;
  }
  return b;
}

RadioModel::RadioModel(const ScenarioConfig &c) : config_(c), pattern_(c.attenuation_limit_db) {
  pattern_.load_e_plane(c.e_plane); pattern_.load_h_plane(c.h_plane);
  pattern_boresight_gain_ = pattern_.get_gain_from_angles(0, 0).total;
  auto create = [&](const RadioConfig &r) {
    return std::make_unique<CommsCalculator>(std::make_unique<LogDistancePathLossModel>(
      r.frequency_hz, 299792458.0, c.path_loss_exponent, 1.0, -1.0),
      calculate_power_budget(r, 0, 0, 0).receiver_input_dbm, 0.0, c.mcs_table);
  };
  ab_ = create(c.ab); ba_ = create(c.ba);
}
LinkSample RadioModel::Evaluate(double time) const {
  auto g = geometry(config_, time);
  auto gains = pattern_.get_tx_rx_gains(g.a, g.antenna_a_rpy, g.b, g.antenna_b_rpy);
  auto gain = [&](double pattern_gain, std::optional<double> boresight) {
    return boresight ? *boresight + pattern_gain - pattern_boresight_gain_ : pattern_gain;
  };
  const double ab_tx = gain(gains.tx_total, config_.ab.tx_antenna_gain_dbi);
  const double ab_rx = gain(gains.rx_total, config_.ab.rx_antenna_gain_dbi);
  const double ba_tx = gain(gains.rx_total, config_.ba.tx_antenna_gain_dbi);
  const double ba_rx = gain(gains.tx_total, config_.ba.rx_antenna_gain_dbi);
  auto ab = ab_->calculate_all(g.a, g.b, ab_tx + ab_rx, false);
  auto ba = ba_->calculate_all(g.b, g.a, ba_tx + ba_rx, false);
  const auto budget_ab = calculate_power_budget(config_.ab, ab.path_loss, ab_tx, ab_rx);
  const auto budget_ba = calculate_power_budget(config_.ba, ba.path_loss, ba_tx, ba_rx);
  const bool snr_ab_ok = !config_.ab.required_snr_db || budget_ab.snr_db >= *config_.ab.required_snr_db;
  const bool snr_ba_ok = !config_.ba.required_snr_db || budget_ba.snr_db >= *config_.ba.required_snr_db;
  auto off_axis = [](const Eigen::Vector3d &direction, const Eigen::Vector3d &rpy) {
    const Eigen::Vector3d forward = AntennaPatternParser::rpy_to_rotmat(rpy.x(), rpy.y(), rpy.z()) * Eigen::Vector3d::UnitX();
    return std::acos(std::clamp(forward.dot(direction.normalized()), -1.0, 1.0)) * 180 / pi;
  };
  return {ab.distance, ab.rssi, ba.rssi,
          config_.protocol.awgn.enabled ? config_.protocol.awgn.bit_rate_bps / 1e9 : ab.throughput,
          gains.tx_total, gains.rx_total,
          ab.rssi >= config_.ab.sensitivity_dbm && snr_ab_ok,
          ba.rssi >= config_.ba.sensitivity_dbm && snr_ba_ok,
          off_axis(g.b-g.a, g.antenna_a_rpy), off_axis(g.a-g.b, g.antenna_b_rpy),
          budget_ab, budget_ba, snr_ab_ok, snr_ba_ok,
          config_.protocol.awgn.enabled ? config_.protocol.awgn.bit_rate_bps / 1e9 : ba.throughput};
}

LinkProtocol::LinkProtocol(ProtocolConfig config, Evaluate evaluate, Log log)
  : config_(config), awgn_sampler_(config.awgn.seed), evaluate_(std::move(evaluate)), log_(std::move(log)) {
  if (config.hrcp.enabled) {
    hrcp_ = std::make_unique<HrcpPairnet>(config.hrcp,evaluate_,log_);
    return;
  }
  for (double v : {config.beacon_interval_s, config.control_frame_s, config.ifs_s,
       config.data_frame_s, config.response_timeout_s, config.ack_timeout_s,
       config.inactivity_timeout_s, config.channel_sample_s}) positive(v, "protocol timing");
  if(config.lab_phy.enabled) {
    LabPhy phy(config.lab_phy);
    if(!config.awgn.enabled || config.awgn.bit_rate_bps!=3.52e9)
      throw std::invalid_argument("仮PHYにはAWGN有効・3.52 Gbpsが必要です");
    if(phy.Duration(0,false)>=config.beacon_interval_s)
      throw std::invalid_argument("Beacon時間が送信周期以上です");
    for(double t:{config.beacon_interval_s,config.ifs_s,config.response_timeout_s,config.ack_timeout_s,config.inactivity_timeout_s})
      if(std::abs(t*3.52e9-std::round(t*3.52e9))>1e-4)
        throw std::invalid_argument("仮PHYのMACタイマーは整数sampleで表せる値が必要です");
  }
  if (config.beacon_interval_s < config.control_frame_s)
    throw std::invalid_argument("Beacon周期は制御フレーム送信時間以上が必要です");
  if (config.awgn.enabled) {
    positive(config.awgn.bit_rate_bps, "AWGN bit rate");
    positive(config.awgn.bandwidth_ab_hz, "AWGN AB bandwidth");
    positive(config.awgn.bandwidth_ba_hz, "AWGN BA bandwidth");
    const double bits = config.awgn.bit_rate_bps * config.data_frame_s;
    if (config.awgn.control_frame_bits == 0 || bits < 1 || bits > 1e12 ||
        config.awgn.control_frame_bits > config.awgn.bit_rate_bps * config.control_frame_s)
      throw std::invalid_argument("AWGNフレーム長が不正です");
  }
}
void LinkProtocol::Integrate(double until) {
  if (connected_) stats_.connected_s += std::max(0.0, until - accounted_);
  accounted_ = until;
}
void LinkProtocol::Advance(double time) {
  if (hrcp_) { hrcp_->Advance(time); return; }
  if (!std::isfinite(time) || time + 1e-12 < accounted_)
    throw std::invalid_argument("シミュレーション時刻は単調増加である必要があります");
  if(config_.lab_phy.enabled && time*3.52e9>0x1.0p52)
    throw std::invalid_argument("仮PHYの時刻がsample精度保証範囲を超えています");
  while (true) {
    double beacon_end = std::numeric_limits<double>::infinity();
    for (const auto &f : pending_beacons_) beacon_end = std::min(beacon_end, f.end);
    double event = std::min({next_, next_beacon_, beacon_end});
    if (event > time + 1e-12) break;
    // 同時刻なら既存フレームの完了を先に処理。Request受信で次のBeaconを止める。
    // 加算と周期の乗算による浮動小数点誤差を同時刻の優先順位に持ち込まない。
    const bool main_event = next_ <= event + 1e-12;
    const bool beacon_receive = !main_event && beacon_end <= event + 1e-12;
    event = main_event ? next_ : (beacon_receive ? beacon_end : next_beacon_);
    Integrate(event); now_ = event;
    if (main_event) Process();
    else if (beacon_receive) ReceiveBeacon();
    else SendBeacon();
    double earliest = action_ == Action::FrameEnd ? frame_.start : now_;
    for (const auto &f : pending_beacons_) earliest = std::min(earliest, f.start);
    for (auto *tx : {&tx_a_, &tx_b_})
      while (!tx->empty() && tx->front().second < earliest - 1e-12) tx->pop_front();
  }
  Integrate(time);
}
void LinkProtocol::Emit(const std::string &type, const std::string &detail) {
  // 成功したデータブロックは時系列CSVの累積量で記録し、イベントCSVの肥大化を防ぐ。
  if ((frame_.name == "DATA" || frame_.name == "DATA_ACK") &&
      (type == "TX" || type == "RX_OK")) return;
  const auto s = evaluate_(now_);
  if (log_) log_({now_, type, frame_.name, frame_.ab ? "A_TO_B" : "B_TO_A", detail,
                 frame_.ab ? s.rssi_ab : s.rssi_ba, wait_start_, deadline_, frame_.failure,
                 frame_.start, frame_.bits, frame_.snr, frame_.eb_no, frame_.ber, frame_.per, frame_.draw,
                 frame_.tx_sample, frame_.tx_samples, frame_.rx_sample, frame_.decoded.samples,
                 frame_.decoded.bit_errors, frame_.decoded.checked_bits, frame_.decoded.sync_metric,
                 frame_.packet.sequence, frame_.packet.reference, frame_.beacon_index});
}
LinkProtocol::Frame LinkProtocol::MakeFrame(const std::string &name, bool ab, double duration, uint64_t index) {
  Frame f;
  f.name = name; f.ab = ab; f.start = now_; f.tx_end = now_ + duration;
  f.end = f.tx_end + evaluate_(now_).distance / 299792458.0;
  f.beacon_index = index;
  f.bits = config_.awgn.enabled ? (name == "DATA"
    ? static_cast<uint64_t>(std::llround(config_.awgn.bit_rate_bps * duration))
    : config_.awgn.control_frame_bits) : 0;
  if (config_.lab_phy.enabled) {
    LabPhy phy(config_.lab_phy);const unsigned type=LabPhy::Type(name);
    f.uid=++frame_counter_;f.tx_sample=std::llround(now_*3.52e9);
    f.tx_samples=phy.Samples(type,ab);f.start=f.tx_sample/3.52e9;
    f.tx_end=(f.tx_sample+f.tx_samples)/3.52e9;
    f.rx_sample=f.tx_sample+static_cast<uint64_t>(std::ceil(evaluate_(now_).distance/299792458.0*3.52e9));
    f.end=(f.rx_sample+f.tx_samples)/3.52e9;
    auto &p=f.packet;p.type=type;p.source=ab?1:2;p.destination=ab?2:1;
    p.sequence=ab?sequence_a_++:sequence_b_++;
    p.reference=frame_.decoded.ok?frame_.decoded.packet.sequence:0;
    p.body.resize(type==4?config_.lab_phy.data_bytes:(type==3||type==5?0:16));
    auto bytes=[&](size_t offset,uint64_t v,unsigned n){for(unsigned i=0;i<n;++i)p.body.at(offset+i)=(v>>(8*(n-1-i)))&255;};
    if(type==0) {
      bytes(0,std::llround(config_.beacon_interval_s*1e9),4);
      bytes(4,std::llround(config_.ifs_s*1e9),4);bytes(8,20000,4);bytes(12,1,1);
      beacon_wire_history_.emplace_back(p.sequence,now_);
      while(!beacon_wire_history_.empty() && beacon_wire_history_.front().second<now_-.5)beacon_wire_history_.pop_front();
    } else if(type==1) {
      session_nonce_=LabPhy::Mix(config_.lab_phy.seed^f.uid);
      bytes(0,p.reference,2);bytes(2,config_.lab_phy.data_bytes,2);bytes(4,0,4);bytes(8,session_nonce_,8);
    } else if(type==2) {
      // Responseは実際に復号したRequestのnonceをechoする。
      std::copy(frame_.decoded.packet.body.begin()+8,frame_.decoded.packet.body.end(),p.body.begin());
      bytes(8,0,2);bytes(10,config_.lab_phy.data_bytes,2);bytes(12,p.reference,2);
    } else if(type==4) {
      for(size_t i=0;i<p.body.size();++i)p.body[i]=LabPhy::Mix(config_.lab_phy.seed^LabPhy::Mix(f.uid)^(i/8))>>(8*(i%8));
    }
    f.bits=p.body.size()*8;
  }
  (ab ? tx_a_ : tx_b_).emplace_back(f.start, f.tx_end);
  return f;
}
void LinkProtocol::BeginFrame(const std::string &name, bool ab, double duration) {
  frame_ = MakeFrame(name, ab, duration, frame_.beacon_index);
  next_ = frame_.end; action_ = Action::FrameEnd;
  if (!connected_) state_ = name;
  Emit("TX");
}
void LinkProtocol::SendBeacon() {
  const auto saved = frame_;
  frame_ = MakeFrame("BEACON", false, config_.control_frame_s, ++beacon_counter_);
  Emit("TX");
  pending_beacons_.push_back(frame_);
  frame_ = saved;
  next_beacon_ = (std::llround(now_ / config_.beacon_interval_s) + 1) * config_.beacon_interval_s;
}
void LinkProtocol::ReceiveBeacon() {
  auto it = std::min_element(pending_beacons_.begin(), pending_beacons_.end(),
    [](const Frame &a, const Frame &b) { return a.end < b.end; });
  const auto saved = frame_;
  frame_ = *it; pending_beacons_.erase(it);
  const bool success = EvaluateFrame();
  if (success && action_ == Action::Idle && !connected_) {
    setup_start_ = frame_.start;
    wait_start_ = deadline_ = -1;
    state_ = "BEACON"; action_ = Action::Request; next_ = now_ + config_.ifs_s;
    Emit("BEACON_SELECTED");
  } else {
    if (success) Emit("BEACON_IGNORED", "A_BUSY");
    frame_ = saved;
  }
}
bool LinkProtocol::ReceiverTransmitting() const {
  const auto &transmissions = frame_.ab ? tx_b_ : tx_a_;
  const double rx_start = frame_.start + (frame_.end-frame_.tx_end);
  for (auto it = transmissions.rbegin(); it != transmissions.rend(); ++it) {
    if (it->second <= rx_start + 1e-12) break;
    // 半開区間。同時刻に送信終了→受信開始する場合は重複なし。
    if (it->first < frame_.end - 1e-12 && it->second > rx_start + 1e-12) return true;
  }
  return false;
}
void LinkProtocol::Wait(Action action, const std::string &state, double start, double duration) {
  action_ = action; state_ = state; wait_start_ = start; deadline_ = start + duration;
  next_ = std::max(now_, deadline_);
  Emit("WAIT_STARTED");
}
void LinkProtocol::Restart(const std::string &reason) {
  if (connected_) {
    ++stats_.disconnections;
    Emit("DISCONNECTED", reason);
  } else {
    ++stats_.timeouts;
    Emit("TIMEOUT", reason);
  }
  connected_ = false; pending_mb_ = 0; state_ = "SCANNING";
  wait_start_ = deadline_ = frame_.failure = -1;
  // Beaconは共通の周期格子で送出。再試行時刻へ無条件に合わせない。
  if (!std::isfinite(next_beacon_))
    next_beacon_ = (std::floor(now_ / config_.beacon_interval_s) + 1) * config_.beacon_interval_s;
  next_ = std::numeric_limits<double>::infinity(); action_ = Action::Idle;
}
bool LinkProtocol::EvaluateFrame() {
  bool success = true;
  frame_.failure = -1;
  frame_.rate = std::numeric_limits<double>::infinity();
  double minimum_snr = std::numeric_limits<double>::infinity();
  auto sample = [&](double t) {
    const auto s = evaluate_(t);
    const bool ok = frame_.ab ? s.ab_ok : s.ba_ok;
    if (!ok && (frame_.failure < 0 || t < frame_.failure)) frame_.failure = t;
    success = success && ok;
    frame_.rate = std::min(frame_.rate, frame_.ab ? s.rate_ab_gbps : s.rate_ba_gbps);
    if (config_.awgn.enabled) {
      double snr = frame_.ab ? s.budget_ab.snr_db : s.budget_ba.snr_db;
      if (!std::isfinite(snr)) throw std::runtime_error("AWGN判定に必要なSNRが計算されていません");
      minimum_snr = std::min(minimum_snr, snr);
    }
  };
  // 物理ステップでのサンプルに加え、短いフレームも開始・中間・受信完了を評価。
  sample(frame_.start); sample((frame_.start + now_) / 2); sample(now_);
  for (double t = frame_.start + config_.channel_sample_s; t < now_; t += config_.channel_sample_s) sample(t);
  const bool half_duplex = ReceiverTransmitting();
  if (half_duplex) { success = false; frame_.failure = now_; }
  const bool no_mcs = !config_.awgn.enabled && success && frame_.name == "DATA" && frame_.rate <= 0;
  if (no_mcs) { success = false; frame_.failure = frame_.start; }
  bool awgn_error = false;
  if (config_.awgn.enabled) {
    frame_.snr = minimum_snr;
    const double bandwidth = frame_.ab ? config_.awgn.bandwidth_ab_hz : config_.awgn.bandwidth_ba_hz;
    frame_.eb_no = minimum_snr + 10*std::log10(bandwidth/config_.awgn.bit_rate_bps);
    frame_.ber = qpsk_awgn_ber(frame_.eb_no);
    frame_.per = config_.lab_phy.enabled ? std::numeric_limits<double>::quiet_NaN()
      : independent_bit_packet_error(frame_.ber, frame_.bits);
    // 感度を満たすフレームごとに1回のみ抽選。AWGNではRSSIそのものを乱数で揺らさない。
    if (config_.lab_phy.enabled) {
      frame_.per=std::numeric_limits<double>::quiet_NaN(); // PER抽選は使わない
      if (success) {
        ++stats_.awgn_trials;
        frame_.decoded=LabPhy(config_.lab_phy).Receive(frame_.packet,frame_.ab,frame_.eb_no,frame_.rx_sample,frame_.uid);
        auto &rx=frame_.decoded;
        if (rx.ok) {
          const auto &p=rx.packet;
          bool valid=p.source==(frame_.ab?1:2)&&p.destination==(frame_.ab?2:1)&&p.type==LabPhy::Type(frame_.name);
          auto value=[&](size_t off,unsigned n){uint64_t v=0;for(unsigned i=0;i<n;++i)v=(v<<8)|p.body.at(off+i);return v;};
          if (p.type==1) {
            unsigned matches=0;
            for(const auto &entry:beacon_wire_history_)if(entry.first==value(0,2)&&now_-entry.second<=.5)++matches;
            valid=valid&&matches==1&&value(0,2)==p.reference;
          }
          if (p.type==2)valid=valid&&value(0,8)==session_nonce_&&value(12,2)==p.reference;
          // 期待する送信番号はローカルMACの待ち状態から得る。
          if (p.type==2||p.type==3||p.type==5)
            valid=valid&&p.reference==uint16_t((frame_.ab?sequence_b_:sequence_a_)-1);
          if(!valid){rx.ok=false;rx.reason="INVALID_MAC_REFERENCE";}
        }
        awgn_error=!rx.ok;
        if(awgn_error){++stats_.awgn_errors;success=false;frame_.failure=now_;}
      }
    } else if (success) {
      ++stats_.awgn_trials;
      frame_.draw = awgn_sampler_.Uniform();
      awgn_error = frame_.draw < frame_.per;
      if (awgn_error) {
        ++stats_.awgn_errors; success = false;
        // ビット誤りの瞬間はモデル化しない。CRC相当の成否が判明する受信完了時刻。
        frame_.failure = now_;
      }
    }
  }
  if (!success) ++stats_.failed_frames;
  std::string failure_reason;
  if (!success) {
    const auto failed = evaluate_(frame_.failure);
    const bool snr_ok = frame_.ab ? failed.snr_ab_ok : failed.snr_ba_ok;
    failure_reason = half_duplex ? "RECEIVER_TRANSMITTING" : awgn_error ? (config_.lab_phy.enabled?frame_.decoded.reason:"AWGN_PACKET_ERROR") :
      (no_mcs ? "NO_PAYLOAD_MCS" : (snr_ok ? "RSSI_BELOW_SENSITIVITY" : "SNR_BELOW_REQUIRED"));
  }
  if (config_.awgn.enabled) Emit("FRAME_RESULT", failure_reason);
  Emit(success ? "RX_OK" : "RX_FAILED", failure_reason);
  return success;
}
void LinkProtocol::FinishFrame() {
  const bool success = EvaluateFrame();
  next_ = now_ + config_.ifs_s;
  if (frame_.name == "REQUEST") {
    request_end_ = now_;
    if (success) {
      next_beacon_ = std::numeric_limits<double>::infinity();
      Emit("BEACON_STOPPED", "REQUEST_RECEIVED");
      action_ = Action::Response;
    }
    else Wait(Action::ResponseTimeout, "WAIT_RESPONSE", request_end_, config_.response_timeout_s);
  } else if (frame_.name == "RESPONSE") {
    response_end_ = now_;
    if (success) action_ = Action::Ack;
    else Wait(Action::ResponseTimeout, "WAIT_RESPONSE", request_end_, config_.response_timeout_s);
  } else if (frame_.name == "ACK") {
    if (success) {
      connected_ = true; state_ = "CONNECTED"; last_success_ = now_;
      ++stats_.connections; stats_.last_setup_ms = (now_ - setup_start_) * 1000;
      if (stats_.first_connected_s < 0) stats_.first_connected_s = now_;
      Emit("CONNECTED"); action_ = Action::Data;
    } else Wait(Action::AckTimeout, "WAIT_ACK", response_end_, config_.ack_timeout_s);
  } else if (frame_.name == "DATA") {
    if (success && (config_.awgn.enabled || frame_.rate > 0)) {
      pending_mb_ = config_.awgn.enabled ? static_cast<double>(frame_.bits) / 8e6
        : frame_.rate * (frame_.tx_end - frame_.start) * 125.0;
      action_ = Action::DataAck;
    } else {
      pending_mb_ = 0;
      Wait(Action::InactivityDisconnect, "WAIT_DATA_ACK", last_success_, config_.inactivity_timeout_s);
    }
  } else if (frame_.name == "DATA_ACK") {
    if (success) {
      stats_.delivered_mb += pending_mb_; pending_mb_ = 0; ++stats_.data_frames;
      last_success_ = now_; state_ = "CONNECTED"; action_ = Action::Data;
    } else {
      Wait(Action::InactivityDisconnect, "WAIT_DATA_ACK", last_success_, config_.inactivity_timeout_s);
    }
  }
}
void LinkProtocol::Process() {
  switch (action_) {
    case Action::Idle: break;
    case Action::Request: ++stats_.attempts; BeginFrame("REQUEST", true, config_.control_frame_s); break;
    case Action::Response: BeginFrame("RESPONSE", false, config_.control_frame_s); break;
    case Action::Ack: BeginFrame("ACK", true, config_.control_frame_s); break;
    case Action::Data: BeginFrame("DATA", !config_.data_from_prc, config_.data_frame_s); break;
    case Action::DataAck: BeginFrame("DATA_ACK", config_.data_from_prc, config_.control_frame_s); break;
    case Action::FrameEnd: FinishFrame(); break;
    case Action::ResponseTimeout: Restart("RESPONSE_TIMEOUT"); break;
    case Action::AckTimeout: Restart("ACK_TIMEOUT"); break;
    case Action::InactivityDisconnect: Restart("INACTIVITY_DISCONNECT"); break;
  }
}

struct DroneRun::Output {
  std::ofstream samples, events, frames;
  std::unique_ptr<HrcpSignalMonitor> preview;
  HrcpSignalMonitor::Status preview_status;
  bool preview_connected=false;
  bool preview_setup_started=false;
};
DroneRun::DroneRun(ScenarioConfig config) : config_(std::move(config)), radio_(config_) {
  config_.Validate();
  std::filesystem::create_directories(config_.output_dir);
  // 同じ出力先への再実行で過去の実験を上書きしない。
  for (const auto *name : {"timeseries.csv", "events.csv", "summary.csv", "completed"})
    if (std::filesystem::exists(std::filesystem::path(config_.output_dir) / name))
      throw std::runtime_error("出力先には既存結果があります。別のディレクトリを指定してください");
  output_ = std::make_unique<Output>();
  if(config_.monitor_only){
    if(!config_.protocol.hrcp.enabled||!config_.protocol.hrcp.monitor_enabled)
      throw std::invalid_argument("monitor_onlyには詳細HRCPモニターが必要です");
    protocol_=std::make_unique<LinkProtocol>(config_.protocol,[this](double t){return radio_.Evaluate(t);},LinkProtocol::Log{});
    return;
  }
  if(config_.preview_mode){
    if(config_.protocol.hrcp.enabled||config_.protocol.lab_phy.enabled)
      throw std::invalid_argument("簡易モニターは受信電力閾値または統計AWGNモデルの設定で使用してください");
    output_->preview=std::make_unique<HrcpSignalMonitor>(config_.output_dir+"/live_signals.json",0,0,true,false);
    output_->preview_status.admission_enabled=false;
  }
  output_->samples.open(config_.output_dir + "/timeseries.csv");
  output_->events.open(config_.output_dir + "/events.csv");
  if (config_.protocol.awgn.enabled || config_.protocol.hrcp.enabled) {
    output_->frames.open(config_.output_dir + "/frames.csv");
    if (!output_->frames) throw std::runtime_error("フレームログを開けません");
    output_->frames << std::setprecision(17)
      << "frame_start_s,frame_end_s,frame,direction,success,reason,rssi_at_end_dbm,minimum_snr_db,eb_no_db,frame_bits,ber,per,random_draw,beacon_index,phy_model,tx_start_sample,tx_samples,rx_start_sample,processed_samples,bit_errors,checked_bits,sync_metric,wire_sequence,wire_reference,mcs,empirical_curve,lookup_rssi_dbm\n";
  }
  if (!output_->samples || !output_->events) throw std::runtime_error("CSV出力先を開けません");
  output_->samples << std::fixed << std::setprecision(9);
  output_->events << std::fixed << std::setprecision(9);
  output_->samples << "time_s,a_x_m,a_y_m,a_z_m,b_x_m,b_y_m,b_z_m,distance_m,rssi_ab_dbm,rssi_ba_dbm,gain_a_db,gain_b_db,a_azimuth_deg,a_elevation_deg,b_azimuth_deg,b_elevation_deg,off_axis_a_deg,off_axis_b_deg,margin_ab_db,margin_ba_db,state,connected,total_data_MB";
  for (const char *direction : {"ab", "ba"})
    for (const char *field : {"tx_gain_dbi", "rx_gain_dbi", "path_loss_db", "tx_output_dbm",
         "tx_after_loss_dbm", "eirp_dbm", "isotropic_rx_dbm", "antenna_rx_dbm",
         "after_lna_dbm", "after_mixer_dbm", "receiver_input_dbm", "thermal_noise_dbm",
         "input_referred_noise_dbm", "receiver_noise_dbm", "snr_db"})
      output_->samples << ',' << direction << '_' << field;
  output_->samples << ",ab_reachable,ba_reachable\n";
  output_->events << "time_s,event,frame,direction,rssi_dbm,reason,wait_start_s,deadline_s,failure_sample_s,observation_time_s,distance_m,rssi_ab_dbm,rssi_ba_dbm,margin_ab_db,margin_ba_db,off_axis_a_deg,off_axis_b_deg,snr_ab_db,snr_ba_db,ab_reachable,ba_reachable,frame_start_s,frame_bits,minimum_snr_db,eb_no_db,ber,per,random_draw,beacon_index\n";
  protocol_ = std::make_unique<LinkProtocol>(config_.protocol,
    [this](double t) { return radio_.Evaluate(t); },
    [this](const LinkEvent &e) {
      if(output_->preview){
        auto &m=*output_->preview;auto &s=output_->preview_status;
        const auto sample=static_cast<uint64_t>(std::llround(e.time*3.52e9));
        const bool ab=e.direction=="A_TO_B";
        if(e.type=="TX")(ab?s.tx_a:s.tx_b)=e.frame;
        if(e.type=="TX"&&e.frame=="BEACON"&&!output_->preview_setup_started)m.ObserveStage(sample,"BEACON");
        if(e.type=="TX"&&(e.frame=="REQUEST"||e.frame=="RESPONSE"||e.frame=="ACK")){
          output_->preview_setup_started=true;m.ObserveStage(sample,e.frame);
        }
        const bool frame_result=e.type=="FRAME_RESULT"||(!config_.protocol.awgn.enabled&&(e.type=="RX_OK"||e.type=="RX_FAILED"));
        if(frame_result&&e.frame=="REQUEST")m.ObserveStage(sample,"RESPONSE");
        if(e.type=="CONNECTED"){output_->preview_connected=true;m.SetupConnected(sample,true);m.ObserveStage(sample,"DATA");}
        if(e.type=="DISCONNECTED"||e.type=="TIMEOUT"){output_->preview_connected=false;output_->preview_setup_started=false;m.ResetSetup(sample);m.ObserveStage(sample,"BEACON");}
        const bool control=e.frame=="BEACON"||e.frame=="REQUEST"||e.frame=="RESPONSE"||e.frame=="ACK";
        if(frame_result&&control&&!output_->preview_connected){
          const bool ok=e.type!="RX_FAILED"&&e.detail.empty();auto &r=ab?s.ab:s.ba;
          r={true,ok,sample,e.frame,e.detail};(ab?s.rx_b:s.rx_a)=e.frame+(ok?" / OK":" / "+e.detail);
          m.SetupResult(sample,ab,ok,e.frame,e.detail);
          if(!ok)m.MarkFailure(static_cast<uint64_t>(std::llround(e.frame_start_s*3.52e9)),sample,ab,e.rssi,e.frame,e.detail);
        }
      }
      if (e.type == "FRAME_RESULT") {
        output_->frames << e.frame_start_s << ',' << e.time << ',' << e.frame << ',' << e.direction
          << ',' << e.detail.empty() << ',' << e.detail << ',' << e.rssi << ',' << e.minimum_snr_db
          << ',' << e.eb_no_db << ',' << e.frame_bits << ',' << e.ber << ',' << e.per << ',' << e.random_draw << ',' << e.beacon_index
          << ',' << (config_.protocol.hrcp.empirical.enabled?"empirical_packet_v1":config_.protocol.hrcp.enabled?"hrcp_iq_continuous_ldpc_v1":config_.protocol.lab_phy.enabled?"lab_iq_rect_v0_uncoded":"statistical_qpsk")
          << ',' << e.tx_sample << ',' << e.tx_samples << ',' << e.rx_sample << ',' << e.processed_samples
          << ',' << e.bit_errors << ',' << e.checked_bits << ',' << e.sync_metric
          << ',' << e.wire_sequence << ',' << e.wire_reference << ',' << e.mcs << ',' << e.empirical_curve << ',' << e.lookup_rssi_dbm << '\n';
        return;
      }
      output_->events << e.time << ',' << e.type << ',' << e.frame
      << ',' << e.direction << ',' << e.rssi << ',' << e.detail
      << ',' << e.wait_start_s << ',' << e.deadline_s << ',' << e.failure_sample_s;
      const double observation_time = e.type == "RX_FAILED" ? e.failure_sample_s : e.time;
      const auto r = radio_.Evaluate(observation_time);
      output_->events << ',' << observation_time << ',' << r.distance << ',' << r.rssi_ab << ',' << r.rssi_ba
        << ',' << r.rssi_ab-config_.ab.sensitivity_dbm << ',' << r.rssi_ba-config_.ba.sensitivity_dbm
        << ',' << r.off_axis_a_deg << ',' << r.off_axis_b_deg
        << ',' << r.budget_ab.snr_db << ',' << r.budget_ba.snr_db
        << ',' << r.ab_ok << ',' << r.ba_ok << ',' << e.frame_start_s << ',' << e.frame_bits
        << ',' << e.minimum_snr_db << ',' << e.eb_no_db << std::scientific << std::setprecision(12)
        << ',' << e.ber << ',' << e.per << ',' << e.random_draw << ',' << e.beacon_index << std::fixed << std::setprecision(9) << '\n'; });
}
DroneRun::~DroneRun() = default;
void DroneRun::Advance(double time) {
  time = std::min(time, config_.duration_s);
  if (!std::isfinite(time) || time + 1e-12 < time_)
    throw std::invalid_argument("実験時刻は単調増加である必要があります");
  if(config_.monitor_only){protocol_->Advance(time);time_=time;return;}
  while (next_log_ <= time + 1e-12) {
    protocol_->Advance(next_log_);
    auto g = geometry(config_, next_log_); auto r = radio_.Evaluate(next_log_);
    auto &s = output_->samples;
    s << next_log_;
    for (const auto &v : {g.a, g.b}) s << ',' << v.x() << ',' << v.y() << ',' << v.z();
    s << ',' << r.distance << ',' << r.rssi_ab << ',' << r.rssi_ba << ',' << r.gain_a << ',' << r.gain_b;
    for (const auto &v : {g.antenna_a_rpy, g.antenna_b_rpy}) s << ',' << v.z()*180/pi << ',' << -v.y()*180/pi;
    s << ',' << r.off_axis_a_deg << ',' << r.off_axis_b_deg
      << ',' << r.rssi_ab-config_.ab.sensitivity_dbm << ',' << r.rssi_ba-config_.ba.sensitivity_dbm;
    s << ',' << protocol_->State() << ',' << protocol_->Connected() << ',' << Stats().delivered_mb;
    for (const auto &b : {r.budget_ab, r.budget_ba})
      s << ',' << b.tx_gain_dbi << ',' << b.rx_gain_dbi << ',' << b.path_loss_db
        << ',' << b.tx_output_dbm << ',' << b.tx_after_loss_dbm << ',' << b.eirp_dbm
        << ',' << b.isotropic_rx_dbm << ',' << b.antenna_rx_dbm << ',' << b.after_lna_dbm
        << ',' << b.after_mixer_dbm << ',' << b.receiver_input_dbm << ',' << b.thermal_noise_dbm
        << ',' << b.input_referred_noise_dbm << ',' << b.receiver_noise_dbm << ',' << b.snr_db;
    s << ',' << r.ab_ok << ',' << r.ba_ok << '\n';
    next_log_ = next_log_ >= config_.duration_s ? std::numeric_limits<double>::infinity()
      : std::min((++log_index_) * config_.log_interval_s, config_.duration_s);
  }
  protocol_->Advance(time); time_ = time;
  if(output_->preview){
    const auto r=radio_.Evaluate(time);auto &s=output_->preview_status;
    s.delivered_mb=Stats().delivered_mb;s.connections=Stats().connections;
    s.timeouts=Stats().timeouts;s.disconnections=Stats().disconnections;
    const auto sample=static_cast<uint64_t>(std::llround(time*3.52e9));
    s.rssi_a_dbm=r.rssi_ba;s.rssi_b_dbm=r.rssi_ab;s.channel_sample=sample;
    s.state_a=s.state_b=protocol_->Connected()?"ASSOCIATED":"SCANNING";
    const auto state=protocol_->State();
    s.protocol_stage=protocol_->Connected()?"DATA":state=="ACK"||state=="WAIT_ACK"?"ACK":
      state=="RESPONSE"||state=="WAIT_RESPONSE"?"RESPONSE":state=="REQUEST"||state=="BEACON"?"REQUEST":"BEACON";
    if(!protocol_->Connected()&&protocol_->State()!="SCANNING"){
      s.state_a="WAIT_RESPONSE";
      if(protocol_->State()=="ACK"||protocol_->State()=="WAIT_ACK"){s.state_a="ASSOCIATED";s.state_b="WAIT_ACK";}
    }
    auto&m=*output_->preview;m.ObserveTime(sample?sample-1:0);m.PushRssi(sample,r.rssi_ba,r.rssi_ab);
    if(m.Due())m.Publish(s);
  }
}
void DroneRun::Finish() {
  if (finished_) return;
  if (time_ + 1e-9 < config_.duration_s) throw std::runtime_error("完了時刻に到達していません");
  if(config_.monitor_only){std::ofstream done(config_.output_dir+"/completed");done<<"completed\n";done.close();if(!done)throw std::runtime_error("完了記録に失敗");finished_=true;return;}
  std::ofstream summary(config_.output_dir + "/summary.csv");
  if (!summary) throw std::runtime_error("サマリーを保存できません");
  auto s = Stats();
  summary << "scenario,antenna_mode,speed_a_kmh,speed_b_kmh,total_data_MB,connected_time_s,connections,attempts,timeouts,failed_frames,acked_data_frames,first_connected_s,last_setup_ms,awgn_trials,awgn_errors,disconnections\n";
  summary << std::fixed << std::setprecision(9) << config_.scenario << ',' << config_.antenna_mode << ','
    << config_.a.speed_kmh << ',' << config_.b.speed_kmh << ',' << s.delivered_mb << ',' << s.connected_s
    << ',' << s.connections << ',' << s.attempts << ',' << s.timeouts << ',' << s.failed_frames << ','
    << s.data_frames << ',' << s.first_connected_s << ',' << s.last_setup_ms
    << ',' << s.awgn_trials << ',' << s.awgn_errors << ',' << s.disconnections << '\n';
  summary.flush(); output_->samples.flush(); output_->events.flush();
  if (!summary || !output_->samples || !output_->events) throw std::runtime_error("CSV書き込みに失敗しました");
  if (config_.protocol.awgn.enabled || config_.protocol.hrcp.enabled) {
    output_->frames.flush();
    if (!output_->frames) throw std::runtime_error("フレームログの書き込みに失敗しました");
  }
  if(output_->preview)output_->preview->Publish(output_->preview_status,true);
  std::ofstream done(config_.output_dir + "/completed"); done << "completed\n"; done.close();
  if (!done) throw std::runtime_error("完了マーカーを書き込めません");
  finished_ = true;
}
}  // namespace comms_sim
