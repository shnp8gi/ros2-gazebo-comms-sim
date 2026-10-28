#pragma once
#include <cstdint>
#include <random>

namespace comms_sim {
struct AwgnConfig {
  bool enabled = false;
  uint64_t seed = 20261027;
  double bit_rate_bps = 3.52e9;
  uint64_t control_frame_bits = 512;
  double bandwidth_ab_hz = 0, bandwidth_ba_hz = 0;
};
double qpsk_awgn_ber(double eb_no_db);
double independent_bit_packet_error(double ber, uint64_t bits);
// 波形は生成せず、AWGN下のGray符号化QPSKの理論BERからフレーム誤りを抽選する。
// 信号電力/RSSIの評価と乱数を分離し、ログ回数によって受信成否が変わらないようにする。
class AwgnSampler {
 public:
  explicit AwgnSampler(uint64_t seed) : engine_(seed) {}
  double Uniform() { return static_cast<double>(engine_() >> 11) * 0x1.0p-53; }
 private:
  std::mt19937_64 engine_;
};
}  // namespace comms_sim
