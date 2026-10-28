#include "comms_sim_pkg/awgn.hpp"
#include <cmath>
#include <stdexcept>

namespace comms_sim {
double qpsk_awgn_ber(double eb_no_db) {
  if (std::isnan(eb_no_db)) throw std::invalid_argument("Eb/N0が不正です");
  return 0.5 * std::erfc(std::sqrt(std::pow(10.0, eb_no_db / 10.0)));
}
double independent_bit_packet_error(double ber, uint64_t bits) {
  if (!std::isfinite(ber) || ber < 0 || ber > 1 || bits == 0)
    throw std::invalid_argument("BERは0～1、フレーム長は正のbit数が必要です");
  if (ber == 1) return 1;
  return -std::expm1(static_cast<double>(bits) * std::log1p(-ber));
}
}  // namespace comms_sim
