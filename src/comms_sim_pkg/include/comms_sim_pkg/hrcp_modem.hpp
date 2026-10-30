#pragma once
#include "comms_sim_pkg/hrcp_codec.hpp"
#include <complex>
#include <functional>
#include <array>

namespace comms_sim {
using HrcpComplex = std::complex<double>;
// 理想Nyquistパルスの整合フィルタ出力を用いる複素ベースバンドモデル。
// RF搬送波、送信スペクトルマスク、CFO/マルチパスは評価しない。
struct HrcpReception {
  uint64_t end_chip=0, sfd_end_chip=0;
  bool header_ok=false, body_ok=false, header_event=false;
  HrcpHeader header;
  std::vector<uint8_t> body;
  std::string reason;
  double sync_metric=0, estimated_es_no=0;
  unsigned ldpc_iterations=0;
};
class HrcpModem {
 public:
  explicit HrcpModem(const HrcpCodec &codec):codec_(codec){}
  std::vector<HrcpComplex> Transmit(HrcpHeader header,const std::vector<uint8_t>&body,bool setup) const;
  static HrcpComplex Bpsk(unsigned bit,uint64_t chip);
  static HrcpComplex Map(unsigned mcs,unsigned label,uint64_t chip);
  static std::vector<double> Demap(HrcpComplex sample,HrcpComplex channel,unsigned mcs,uint64_t chip);
  static const HrcpBits &GolayA();
 private:
  const HrcpCodec &codec_;
};
// フレーム境界を引数として受け取らない連続受信機。
// 入力はE|n|^2=1の複素AWGNを含む整合フィルタ出力。位置は絶対chip番号。
// 送信時はResetを呼び、半二重の受信中断を再現する。
class HrcpStreamReceiver {
 public:
  using Callback=std::function<void(const HrcpReception&)>;
  HrcpStreamReceiver(const HrcpCodec &codec,Callback callback,unsigned max_body_bytes=4096);
  void Feed(HrcpComplex sample,uint64_t chip);
  void Reset();
  uint64_t ProcessedChips() const{return processed_;}
 private:
  void Complete();
  void Collect(HrcpComplex sample,uint64_t chip);
  const HrcpCodec &codec_;
  Callback callback_;
  unsigned max_body_bytes_;
  std::array<HrcpComplex,256> ring_{};
  uint64_t processed_=0,fill_=0,lock_next_=0,sfd_end_=0;
  HrcpComplex correlation_{}, channel_{};
  double energy_=0,old_energy_=0,metric_=0;
  unsigned repeats_=0;
  bool collecting_=false,header_decoded_=false;
  std::vector<HrcpComplex> collected_;
  size_t target_=0;
  HrcpReception result_;
};
}
