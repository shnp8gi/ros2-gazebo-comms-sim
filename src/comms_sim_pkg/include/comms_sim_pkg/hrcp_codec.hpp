#pragma once
#include "comms_sim_pkg/hrcp_ldpc.hpp"
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace comms_sim {
using HrcpBits = std::vector<uint8_t>;
struct HrcpHeader {
  unsigned mcs=0, seed=0;
  bool pilot=true;
  uint32_t body_bytes=0;
  std::array<uint8_t,10> mac{};
};
struct HrcpEncoded {
  HrcpHeader header;
  HrcpBits header_bits; // EH符号化264bit+stuff2bit。まだspread/mapする前。
  HrcpBits body_bits;   // shortened LDPC+stuff。1block分のbit数で整列済み。
};
struct HrcpDecodedHeader {
  bool ok=false;
  std::string reason;
  HrcpHeader header;
  unsigned corrected_bits=0;
};
struct HrcpDecodedBody {
  bool converged=false;
  unsigned iterations=0;
  std::vector<uint8_t> bytes;
};
class HrcpScrambler {
 public:
  explicit HrcpScrambler(unsigned seed);
  uint8_t Next();
  void Apply(HrcpBits &bits);
 private:
  std::array<uint8_t,15> state_{};
};
class HrcpCodec {
 public:
  explicit HrcpCodec(unsigned decode_threads=1);
  HrcpEncoded Encode(HrcpHeader header,const std::vector<uint8_t> &body) const;
  HrcpDecodedHeader DecodeHeader(const std::vector<double> &llr) const;
  HrcpDecodedBody DecodeBody(const HrcpHeader &header,const std::vector<double> &llr) const;
  static unsigned BitsPerSymbol(unsigned mcs);
  static unsigned InformationBits(unsigned mcs);
  static uint64_t CodedBodyBits(const HrcpHeader &header);
  static uint64_t BodyBlocks(const HrcpHeader &header);
  static uint64_t FrameChips(const HrcpHeader &header,bool setup);
  static HrcpBits OctetBits(const std::vector<uint8_t> &bytes);
  static uint16_t HeaderCrc(const HrcpBits &bits);
  static uint32_t FrameCrc(const std::vector<uint8_t> &bytes);
  static uint8_t SubheaderCrc(const std::array<uint8_t,3> &bytes);
  static void AppendFcs(std::vector<uint8_t> &bytes);
  static bool CheckFcs(const std::vector<uint8_t> &bytes);
  static uint8_t ExtendedHamming(unsigned input);
 private:
  const HrcpLdpc &Code(unsigned mcs) const;
  HrcpLdpc code11_,code14_;
  unsigned decode_threads_=1;
};
}
