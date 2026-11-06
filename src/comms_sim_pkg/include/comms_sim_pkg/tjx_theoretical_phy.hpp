#pragma once
#include "comms_sim_pkg/hrcp_codec.hpp"
#include <map>
#include <random>
#include <set>
#include <string>

namespace comms_sim {
struct TjxMcs {
  unsigned id, bits_per_symbol, information_bits, repetition;
};
struct TjxTheoryConfig {
  bool enabled=false, pilot=true;
  std::string table;
  unsigned control_mcs=3;
  uint64_t seed=20261102;
};
struct TjxTheoryDecision {
  double header_per=1, body_per=1, per=1, ber=0, es_no_db=0;
  bool outside_range=false;
};
// MCS番号はTJX。速度、短縮符号、繰返し、pilotの時間をここで一元化する。
const TjxMcs &tjx_mcs(unsigned id);
const std::vector<unsigned> &tjx_mcs_ids();
double tjx_rate_bps(unsigned mcs,bool pilot);
uint64_t tjx_coded_bits(unsigned mcs,unsigned body_bytes);
uint64_t tjx_frame_chips(unsigned mcs,unsigned body_bytes,bool setup,bool pilot);
std::set<std::pair<int,unsigned>> tjx_required_blocks(unsigned data_bytes,unsigned control_mcs);

struct TjxBlockResult {bool error=false;uint64_t bit_errors=0;};
// 正解bitとの比較はLUT生成時のみ。ランタイムMACには渡さない。
class TjxBlockSimulator {
 public:
  explicit TjxBlockSimulator(uint64_t seed):rng_(seed){}
  TjxBlockResult Run(int mcs,unsigned information_bits,double es_no_db);
 private:
  HrcpLdpc code11_{1056},code14_{1344};
  HrcpCodec codec_;
  std::mt19937_64 rng_;
  std::normal_distribution<double> normal_{0.,.7071067811865475244};
};
class TjxTheoreticalPhy {
 public:
  explicit TjxTheoreticalPhy(TjxTheoryConfig config);
  TjxTheoryDecision Evaluate(unsigned mcs,unsigned body_bytes,double snr_db,double bandwidth_hz)const;
  void ValidateLengths(unsigned data_bytes)const;
 private:
  struct Point {double es_no,per,ber;};
  Point Lookup(int mcs,unsigned bits,double es_no,bool &outside)const;
  TjxTheoryConfig config_;
  std::map<std::pair<int,unsigned>,std::vector<Point>> curves_;
};
}
