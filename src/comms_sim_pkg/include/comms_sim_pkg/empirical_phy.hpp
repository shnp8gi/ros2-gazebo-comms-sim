#pragma once
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace comms_sim {
struct EmpiricalConfig {
  bool enabled=false;
  std::string table,metric="ber",per_length_policy="exact";
  double offset_ab_db=0,offset_ba_db=0;
  uint64_t seed=20261029;
  unsigned control_mcs=0;
  // MCStable.csvのGbps値から、実機CSVのMCS番号へ明示的に対応付ける。
  std::map<double,unsigned> mcs_by_rate{{.5,0},{1.1,3},{2.15,6},{2.7,8},{4.6,9},{6,11}};
};
struct EmpiricalDecision {
  double ber=-1,per=1,lookup_dbm=0;
  std::string curve;
  bool below_range=false;
};
class EmpiricalPhy {
 public:
  explicit EmpiricalPhy(EmpiricalConfig config);
  EmpiricalDecision Evaluate(bool ab,const std::string&frame,unsigned mcs,
                             unsigned payload_bytes,double rssi_dbm)const;
  unsigned Mcs(double rate_gbps)const;
  static double PacketError(double bit_error,uint64_t bits);
 private:
  struct Point {double rssi,value;};
  struct Curve {std::vector<Point> points;unsigned reference_bytes=0;};
  EmpiricalConfig config_;
  std::map<std::string,Curve> curves_;
};
}
