#pragma once
#include <cstdint>
#include "comms_sim_pkg/empirical_phy.hpp"
#include <functional>
#include <memory>
#include <string>
namespace comms_sim {
struct LinkSample;struct LinkEvent;struct LinkStats;
struct HrcpConfig {
  bool enabled=false,monitor_enabled=false;
  bool noise_enabled=true,monitor_only=false;
  bool data_from_prc=false;
  bool monitor_waveforms=false,fast_events=true;
  unsigned noise_threads=1;
  unsigned decode_threads=1;
  double monitor_rssi_interval_s=0;
  double monitor_rssi_window_s=1e-6;
  std::string monitor_path;
  uint64_t seed=20261027;
  EmpiricalConfig empirical;
  unsigned setup_timeout_ms=0; // 0は旧動作（交渉ATP）を維持。
  unsigned data_bytes=2048,atp_ms=500,requested_atp_ms=0; // 0ならatp_msを要求
  double reference_distance_m=30;
  double expected_rssi_dbm=-86;
  bool expected_rssi_admission_enabled=true; // 旧設定は維持。今回の実験設定ではfalse。
  double bandwidth_ab_hz=2.1648e9,bandwidth_ba_hz=2.1648e9;
  double channel_sample_s=.0001;
};
// 再送で延長しないsetupタイマー。接続後のRefreshとは区別する。
struct HrcpDeadline {
  static constexpr uint64_t never=UINT64_MAX;
  uint64_t start=never,deadline=never;
  void StartOnce(uint64_t now,uint64_t duration){if(start==never){start=now;deadline=now+duration;}}
  void Refresh(uint64_t now,uint64_t duration){start=now;deadline=now+duration;}
  void Clear(){start=deadline=never;}
  bool Expired(uint64_t now)const{return deadline!=never&&now>=deadline;}
};
class HrcpPairnet {
 public:
  using Evaluate=std::function<LinkSample(double)>;
  using Log=std::function<void(const LinkEvent&)>;
  HrcpPairnet(HrcpConfig config,Evaluate evaluate,Log log);
  ~HrcpPairnet();
  void Advance(double time);
  bool Connected()const;
  const LinkStats &Stats()const;
  const std::string &State()const;
 private:
  struct Impl;std::unique_ptr<Impl> impl_;
};
}
