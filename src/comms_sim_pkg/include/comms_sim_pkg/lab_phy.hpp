#pragma once
#include <complex>
#include <cstdint>
#include <string>
#include <vector>

namespace comms_sim {
// IEEE互換ではない試作用codec。規格の符号行列入手までpayload FECなし。
struct LabPhyConfig {
  bool enabled = false;
  uint64_t seed = 20261027;
  unsigned data_bytes = 2048;
  unsigned chunk_samples = 4096;
  double symbol_rate = 1.76e9;
  double sync_threshold = .45;
};
struct LabPacket {
  unsigned type = 0, source = 0, destination = 0;
  uint16_t sequence = 0, reference = 0;
  std::vector<uint8_t> body;
};
struct LabRx {
  bool ok = false;
  std::string reason;
  LabPacket packet;
  uint64_t samples = 0, bit_errors = 0, checked_bits = 0;
  double sync_metric = 0;
};
class LabPhy {
 public:
  explicit LabPhy(LabPhyConfig config);
  static unsigned Type(const std::string &name);
  static uint16_t Crc16(const std::vector<uint8_t> &bits);
  static uint32_t Crc32(const std::vector<uint8_t> &bytes);
  static uint64_t Mix(uint64_t x);
  uint64_t Samples(unsigned type, bool ab) const;
  double Duration(unsigned type, bool ab) const;
  std::vector<uint8_t> Encode(const LabPacket &packet) const;
  LabRx Decode(const std::vector<uint8_t> &bits) const;
  // フレーム受信窓内の波形。連続空間探索/CFO/LDPCは未実装。
  LabRx Receive(const LabPacket &packet, bool ab, double eb_no_db,
                uint64_t arrival_sample, uint64_t frame_uid) const;
 private:
  LabPhyConfig c_;
};
}
