#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <functional>
#include <cstdint>
#include <memory>
#include <optional>
#include <limits>
#include <string>
#include <vector>
#include <deque>
#include "comms_sim_pkg/antenna_pattern_parser.hpp"
#include "comms_sim_pkg/comms_calculator.hpp"
#include "comms_sim_pkg/awgn.hpp"
#include "comms_sim_pkg/lab_phy.hpp"
#include "comms_sim_pkg/hrcp_pairnet.hpp"

namespace comms_sim {

struct DroneConfig {
  double speed_kmh = 50.0;
  double altitude_m = 20.0;
  double fixed_elevation_deg = 45.0;
  // 既定の取り付け（真横・fixed_elevation_deg）からの回転量。固定アンテナのみ。
  double mount_azimuth_offset_deg = 0, mount_elevation_offset_deg = 0;
};
struct RadioConfig {
  double frequency_hz = 272e9;
  double power_mw = 1.0;
  double sensitivity_dbm = -68.5;
  // 未指定なら従来CSVの絶対利得を使用。指定時はCSVを正面基準に正規化する。
  std::optional<double> tx_antenna_gain_dbi = std::nullopt, rx_antenna_gain_dbi = std::nullopt;
  double tx_insertion_loss_db = 0, rx_insertion_loss_db = 0;  // 損失は正値
  double rx_lna_gain_db = 0, rx_conversion_gain_db = 0;
  double noise_bandwidth_hz = 0;  // 0なら従来互換で雑音計算なし
  double noise_temperature_k = 300, receiver_noise_figure_db = 0;
  std::optional<double> required_snr_db = std::nullopt;  // 未指定なら記録のみ
  // phy（thz_sc / hrcp_sc）を指定すると規格の表を使い、感度は表の最低値（MCS0）になる。
  std::string phy = "";
  unsigned bonding_channels = 1;  // thz_sc のみ 1, 2, 4
  std::string rate_table = "";    // 空なら共通の mcs_table
};
struct PowerBudget {
  double tx_gain_dbi = 0, rx_gain_dbi = 0, path_loss_db = 0;
  double tx_output_dbm = 0, tx_after_loss_dbm = 0, eirp_dbm = 0;
  double isotropic_rx_dbm = 0, antenna_rx_dbm = 0;
  double after_lna_dbm = 0, after_mixer_dbm = 0, receiver_input_dbm = 0;
  double thermal_noise_dbm = std::numeric_limits<double>::quiet_NaN();
  double input_referred_noise_dbm = std::numeric_limits<double>::quiet_NaN();
  double receiver_noise_dbm = std::numeric_limits<double>::quiet_NaN();
  double snr_db = std::numeric_limits<double>::quiet_NaN();
};
PowerBudget calculate_power_budget(const RadioConfig &radio, double path_loss_db,
                                  double tx_gain_dbi, double rx_gain_dbi);
struct ProtocolConfig {
  AwgnConfig awgn;
  LabPhyConfig lab_phy;
  HrcpConfig hrcp;
  double beacon_interval_s = 0.000140;
  double control_frame_s = 0.000020;
  double ifs_s = 0.000010;
  double data_frame_s = 0.000100;
  double response_timeout_s = 0.5;
  double ack_timeout_s = 0.5;
  double inactivity_timeout_s = 0.5;
  double channel_sample_s = 0.0001;
  bool data_from_prc = false;  // true: B(PRC)→A(PRDEV)。未指定の旧設定は従来方向を維持。
  // 簡易リンク確立：両リンクがしきい値以上の状態が simple_setup_s 続いたら接続。
  bool simple_setup = false;
  double simple_setup_s = 0.002;
};
struct ScenarioConfig {
  bool preview_mode=false;
  bool monitor_only=false;
  bool allow_pre_encounter_end=false;  // 初回確立までの再生記録用。既定では従来の終了条件を維持。
  std::string scenario = "parallel", antenna_mode = "fixed";
  std::string config_id;  // 評価する構成の名前。summary.csv にそのまま出す。
  DroneConfig a, b{50.0, 30.0, -45.0};
  RadioConfig ab, ba{60e9, 0.4, -68.5};
  ProtocolConfig protocol;
  double encounter_s = 20.0, duration_s = 40.0;
  double lateral_separation_m = 10.0;
  double physics_step_s = 0.0001, real_time_factor = 5.0;
  double log_interval_s = 0.01, path_loss_exponent = 2.0;
  double attenuation_limit_db = 30.0;
  std::string e_plane, h_plane, mcs_table, output_dir;
  static ScenarioConfig Load(const std::string &path);
  void Validate() const;
};
struct Geometry {
  Eigen::Vector3d a, b, antenna_a_rpy, antenna_b_rpy;
  double heading_a = 0, heading_b = 0;
};
Geometry geometry(const ScenarioConfig &config, double time);
// 通信コアが受け取る位置・姿勢。回転行列はワールド←各座標系、アンテナの正面は +X。
struct Poses {
  Eigen::Vector3d a, b;
  Eigen::Matrix3d body_a, body_b;
  Eigen::Matrix3d antenna_a, antenna_b;
};
// 供給元：解析軌道／Gazebo の実測値／記録の再生。
using PoseSource = std::function<Poses(double)>;
// 解析軌道（kinematic）。取り付けオフセットを含む。
Poses analytic_poses(const ScenarioConfig &config, double time);
struct LinkSample {
  double distance = 0;
  double rssi_ab = -999, rssi_ba = -999;
  double rate_ab_gbps = 0;
  double gain_a = 0, gain_b = 0;
  bool ab_ok = false, ba_ok = false;
  double off_axis_a_deg = 0, off_axis_b_deg = 0;
  PowerBudget budget_ab{}, budget_ba{};
  bool snr_ab_ok = true, snr_ba_ok = true;
  double rate_ba_gbps = 0;
  Poses poses{};
  double deflection_a_deg = 0, deflection_b_deg = 0;  // 取り付けの向きからのアンテナのぶれ
};
class RadioModel {
 public:
  // source を省くと解析軌道。外部の供給元には取り付けオフセットを回転として掛ける。
  explicit RadioModel(const ScenarioConfig &config, PoseSource source = {});
  LinkSample Evaluate(double time) const;
 private:
  ScenarioConfig config_;
  PoseSource source_;
  bool fixed_mount_ = false;
  Eigen::Matrix3d mount_a_, mount_b_, offset_a_, offset_b_;  // 機体←アンテナ、オフセットの回転
  AntennaPatternParser pattern_;
  double pattern_boresight_gain_ = 0;
  std::unique_ptr<CommsCalculator> ab_, ba_;
};
struct LinkEvent {
  double time;
  std::string type, frame, direction, detail;
  double rssi = 0;
  double wait_start_s = -1, deadline_s = -1, failure_sample_s = -1;
  double frame_start_s = -1;
  uint64_t frame_bits = 0;
  double minimum_snr_db = std::numeric_limits<double>::quiet_NaN();
  double eb_no_db = std::numeric_limits<double>::quiet_NaN();
  double ber = std::numeric_limits<double>::quiet_NaN();
  double per = std::numeric_limits<double>::quiet_NaN();
  double random_draw = -1;
  uint64_t tx_sample = 0, tx_samples = 0, rx_sample = 0, processed_samples = 0;
  uint64_t bit_errors = 0, checked_bits = 0;
  double sync_metric = 0;
  unsigned wire_sequence = 0, wire_reference = 0;
  uint64_t beacon_index = 0;  // 実験内で1始まり。応答・データも接続の起点を保持。
  std::string empirical_curve="";
  double lookup_rssi_dbm = std::numeric_limits<double>::quiet_NaN();
  int mcs = -1;
};
struct LinkStats {
  double delivered_mb = 0, connected_s = 0;
  double first_connected_s = -1, last_setup_ms = 0;
  double window_s = 0;  // 両リンクがしきい値以上だった時間（simple_setup のみ）
  unsigned attempts = 0, connections = 0, timeouts = 0, disconnections = 0;
  unsigned data_frames = 0, failed_frames = 0;
  uint64_t awgn_trials = 0, awgn_errors = 0;
};

// 簡易リンク確立モデル。channel_sample_s ごとに両リンクの受信電力を見る。
class SimpleSetup {
 public:
  using Evaluate = std::function<LinkSample(double)>;
  using Log = std::function<void(const LinkEvent &)>;
  SimpleSetup(const ProtocolConfig &config, Evaluate evaluate, Log log);
  void Advance(double time);
  bool Connected() const { return connected_; }
  const LinkStats &Stats() const { return stats_; }
  const std::string &State() const { return state_; }
 private:
  void Emit(double time, const std::string &type, const std::string &detail, double rssi);
  double step_, setup_s_;
  bool data_ba_;
  Evaluate evaluate_;
  Log log_;
  LinkStats stats_;
  uint64_t index_ = 0;
  double ok_since_ = -1;
  bool connected_ = false;
  std::string state_ = "SCANNING";
};

// イベント時刻で進める。Gazeboのステップ境界への丸めでIFS等を伸ばさない。
class LinkProtocol {
 public:
  using Evaluate = std::function<LinkSample(double)>;
  using Log = std::function<void(const LinkEvent &)>;
  LinkProtocol(ProtocolConfig config, Evaluate evaluate, Log log = {});
  void Advance(double time);
  bool Connected() const { return simple_?simple_->Connected():hrcp_?hrcp_->Connected():connected_; }
  const LinkStats &Stats() const { return simple_?simple_->Stats():hrcp_?hrcp_->Stats():stats_; }
  const std::string &State() const { return simple_?simple_->State():hrcp_?hrcp_->State():state_; }
 private:
  enum class Action { Idle, Request, Response, Ack, Data, DataAck,
                      FrameEnd, ResponseTimeout, AckTimeout, InactivityDisconnect };
  struct Frame {
    std::string name;
    bool ab = false;
    double start = 0, tx_end = 0, end = 0, rate = 0, failure = -1;
    uint64_t bits = 0, beacon_index = 0;
    double snr = std::numeric_limits<double>::quiet_NaN();
    double eb_no = std::numeric_limits<double>::quiet_NaN();
    double ber = std::numeric_limits<double>::quiet_NaN();
    double per = std::numeric_limits<double>::quiet_NaN();
    double draw = -1;
    uint64_t uid = 0, tx_sample = 0, rx_sample = 0, tx_samples = 0;
    LabPacket packet;
    LabRx decoded;
  };
  Frame MakeFrame(const std::string &name, bool ab, double duration, uint64_t index);
  bool EvaluateFrame();
  void SendBeacon();
  void ReceiveBeacon();
  bool ReceiverTransmitting() const;
  void Process();
  void BeginFrame(const std::string &frame, bool ab, double duration);
  void FinishFrame();
  void Restart(const std::string &reason);
  void Wait(Action action, const std::string &state, double start, double duration);
  void Emit(const std::string &type, const std::string &detail = "");
  void Integrate(double until);
  std::unique_ptr<HrcpPairnet> hrcp_;
  std::unique_ptr<SimpleSetup> simple_;
  ProtocolConfig config_;
  AwgnSampler awgn_sampler_;
  Evaluate evaluate_;
  Log log_;
  LinkStats stats_;
  Action action_ = Action::Idle;
  double next_ = std::numeric_limits<double>::infinity(), now_ = 0, accounted_ = 0;
  double next_beacon_ = 0;
  uint64_t beacon_counter_ = 0, frame_counter_ = 0;
  uint16_t sequence_a_ = 0, sequence_b_ = 0;
  uint64_t session_nonce_ = 0;
  std::deque<std::pair<uint16_t,double>> beacon_wire_history_;
  Frame frame_;
  std::vector<Frame> pending_beacons_;
  std::deque<std::pair<double, double>> tx_a_, tx_b_;
  double request_end_ = 0, response_end_ = 0, last_success_ = 0;
  double setup_start_ = 0, pending_mb_ = 0;
  double wait_start_ = -1, deadline_ = -1;
  bool connected_ = false;
  std::string state_ = "SCANNING";
};

// 同一コアをGazeboプラグインと単体検証から使い、集計をCSVの再探索に依存させない。
class DroneRun {
 public:
  explicit DroneRun(ScenarioConfig config, PoseSource source = {});
  ~DroneRun();
  void Advance(double time);
  void Finish();
  const ScenarioConfig &Config() const { return config_; }
  const LinkStats &Stats() const { return protocol_->Stats(); }
  // 実際の軌跡から求めた最接近距離と、アンテナ正面方向と相手方向のなす角の最小値。
  struct Encounter {
    double d_min_m = std::numeric_limits<double>::infinity();
    double min_angle_a_deg = std::numeric_limits<double>::infinity();
    double min_angle_b_deg = std::numeric_limits<double>::infinity();
    void Observe(const LinkSample &s) {
      d_min_m = std::min(d_min_m, s.distance);
      min_angle_a_deg = std::min(min_angle_a_deg, s.off_axis_a_deg);
      min_angle_b_deg = std::min(min_angle_b_deg, s.off_axis_b_deg);
    }
  };
  const Encounter &Observed() const { return encounter_; }
 private:
  LinkSample Sample(double time);
  struct Output;
  ScenarioConfig config_;
  RadioModel radio_;
  std::unique_ptr<Output> output_;
  std::unique_ptr<LinkProtocol> protocol_;
  double next_log_ = 0, time_ = 0;
  uint64_t log_index_ = 0;
  bool finished_ = false;
  Encounter encounter_;
};
// summary.csv の追加列（見出しと値）。掃引ツールと共有する。
std::string encounter_summary_header();
std::string encounter_summary_row(const ScenarioConfig &config, const LinkStats &stats,
                                  const DroneRun::Encounter &encounter);
}  // namespace comms_sim
