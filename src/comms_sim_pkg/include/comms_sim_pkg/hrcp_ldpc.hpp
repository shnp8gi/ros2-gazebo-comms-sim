#pragma once
#include <array>
#include <bitset>
#include <cstdint>
#include <vector>

namespace comms_sim {
// IEEE 802.15.3-2023 Tables 12-17/13-10. この層は行列の添字順で扱う。
// 無線送信順への並べ替えはframe codecの責任。符号化利得の定数補正はしない。
class HrcpLdpc {
 public:
  static constexpr unsigned N = 1440;
  struct Decoded {
    bool converged = false;
    unsigned iterations = 0;
    std::vector<uint8_t> information;
  };
  explicit HrcpLdpc(unsigned information_bits);
  unsigned K() const { return k_; }
  const std::vector<std::vector<unsigned>> &Checks() const { return checks_; }
  // message[i] -> i_i。末尾の未入力情報bitを0としてshorteningする。
  std::vector<uint8_t> Encode(const std::vector<uint8_t> &message) const;
  bool SyndromeZero(const std::vector<uint8_t> &word) const;
  // LLR=log(P(0)/P(1))。shortening部分は既知0。全1440bit分の位置を渡す。
  Decoded Decode(const std::vector<double> &llr, unsigned actual_information_bits,
                 unsigned max_iterations = 50, double normalization = .8) const;
 private:
  bool SyndromeZeroUnchecked(const std::vector<uint8_t> &word) const;
  unsigned k_;
  std::vector<std::vector<unsigned>> checks_;
  std::vector<std::bitset<N>> parity_equations_;
};
}
