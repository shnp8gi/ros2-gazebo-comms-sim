#pragma once
#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <string>
#include <fstream>
#include <memory>
#include "comms_sim_pkg/hrcp_rssi_meter.hpp"
namespace comms_sim {
// 通信判断から独立した観測器。保存・描画の間引きは通信sampleを省略しない。
class HrcpSignalMonitor {
 public:
  explicit HrcpSignalMonitor(std::string path,uint64_t rssi_interval=0,uint64_t rssi_window=0,bool monitor_only=false,bool noise_enabled=true);
  using Values=std::array<double,8>; // A-TX I/Q, A-RX I/Q, B-TX I/Q, B-RX I/Q
  struct Reception {
    bool known=false,ok=false;
    uint64_t sample=0;
    std::string frame,reason;
  };
  struct Status {
    std::string state_a,state_b,tx_a,rx_a,tx_b,rx_b;
    double rssi_a_dbm=-999,rssi_b_dbm=-999;
    uint64_t channel_sample=0;
    Reception ab,ba;
    bool admission_known=false,admission_ok=false;
    uint64_t admission_sample=0;
    double admission_measured_dbm=0,admission_expected_dbm=0;
    std::string protocol_stage;
    double delivered_mb=0;
    unsigned connections=0,timeouts=0,disconnections=0;
    bool admission_enabled=true;
  };
  void Push(uint64_t sample,const Values&values,bool rx_a,bool rx_b);
  void ObserveTime(uint64_t sample){sample_=sample;}
  void PushRssi(uint64_t sample,double rssi_a_dbm,double rssi_b_dbm);
  void ObserveReceivePower(uint64_t sample,const std::array<double,2>&watts,const std::array<bool,2>&available);
  void FinishRssi();
  void ObserveStage(uint64_t sample,const std::string&stage);
  void MarkFailure(uint64_t start,uint64_t end,bool ab,double rssi_dbm,
                   const std::string&frame,const std::string&reason);
  void SetupResult(uint64_t sample,bool ab,bool ok,const std::string&frame,const std::string&reason);
  void SetupConnected(uint64_t sample,bool connected);
  void ResetSetup(uint64_t sample);
  void Publish(const Status&status,bool complete=false);
  bool Due()const;
 private:
  struct Raw {uint64_t sample;Values values;};
  struct Power {uint64_t sample;std::array<double,6> values;};
  struct Rssi {uint64_t sample;double a_dbm,b_dbm;};
  Rssi latest_rssi_{};
  uint64_t rssi_interval_=0,next_rssi_=0;
  std::deque<Rssi> rssi_;
  std::unique_ptr<HrcpRssiMeter> meter_;
  std::ofstream rssi_csv_;
  bool noise_enabled_=true;
  std::ofstream replay_;
  uint64_t last_replay_status_=UINT64_MAX;
  void RecordMeasuredRssi();
  std::deque<std::pair<uint64_t,std::string>> stage_history_;
  struct Failure {uint64_t start,end;bool ab;double rssi_dbm;std::string frame,reason;};
  std::deque<Failure> failures_;
  Reception setup_ab_,setup_ba_;
  bool setup_connected_=false;
  struct SetupState {uint64_t sample;int ab,ba;bool connected;};
  std::deque<SetupState> setup_history_;
  void SaveSetup(uint64_t sample);
  std::string path_;
  std::deque<Raw> raw_,last_a_,last_b_;
  bool active_a_=false,active_b_=false;
  std::deque<Power> power_;
  std::array<double,6> sum_{};
  uint64_t sample_=0;
  unsigned count_=0;
  bool boundary_=false;
  std::chrono::steady_clock::time_point published_{};
};
}
