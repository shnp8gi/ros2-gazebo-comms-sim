#include "comms_sim_pkg/drone_link.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <tuple>
using namespace comms_sim;
void check(bool b,const char *s){if(!b)throw std::runtime_error(s);}
LinkSample channel(double snr){LinkSample s;s.ab_ok=s.ba_ok=true;s.distance=10;s.rate_ab_gbps=3.52;
 s.budget_ab.snr_db=s.budget_ba.snr_db=snr;s.rssi_ab=s.rssi_ba=-40;return s;}
int main(){try{
  LabPhyConfig c;c.enabled=true;LabPhy phy(c);
  std::vector<uint8_t> text{'1','2','3','4','5','6','7','8','9'},bits;
  for(auto b:text)for(int k=7;k>=0;--k)bits.push_back((b>>k)&1);
  check(LabPhy::Crc16(bits)==0x29b1,"CRC16既知ベクトル");
  check(LabPhy::Crc32(text)==0xcbf43926,"CRC32既知ベクトル");
  for(unsigned type=0;type<6;++type){
    LabPacket p{type,1,2,65535,65534,std::vector<uint8_t>(type==4?2048:(type==3||type==5?0:16),0x5a)};
    auto encoded=phy.Encode(p);auto r=phy.Decode(encoded);
    check(r.ok&&r.packet.sequence==65535&&r.packet.reference==65534&&r.packet.body==p.body,"フレーム往復");
    auto error=encoded;error[7]^=1;check(phy.Decode(error).ok,"EH単一bit訂正");
    error[8]^=1;check(!phy.Decode(error).ok,"EH二重bit検出");
    if(encoded.size()>208){error=encoded;error[210]^=1;check(phy.Decode(error).reason=="PAYLOAD_CRC_ERROR","payload破損CRC検出");}
    for(bool ab:{false,true}){
      auto clean=phy.Receive(p,ab,100,1000,1);
      check(clean.ok&&clean.bit_errors==0&&clean.samples==phy.Samples(type,ab),"高SNR波形往復とsample長");
      c.chunk_samples=7;LabPhy chunked(c);auto noisy=phy.Receive(p,ab,2,1000,1),chunk=chunked.Receive(p,ab,2,1000,1);
      check(noisy.reason==chunk.reason&&noisy.bit_errors==chunk.bit_errors&&noisy.sync_metric==chunk.sync_metric&&noisy.samples==chunk.samples,"奇数sample境界で結果不変");
      auto low=phy.Receive(p,ab,-40,1000,1);check(!low.ok,"低SNR受信失敗");
    }
  }
  // payload硬判定BERを独立理論値と照合（CRC不合格フレームも数える）。
  uint64_t errors=0,total=0;LabPacket data{4,1,2,1,0,std::vector<uint8_t>(2048,0xa5)};
  for(unsigned i=0;i<100;++i){auto r=phy.Receive(data,true,0,100000ULL*i,1);errors+=r.bit_errors;total+=r.checked_bits;}
  check(std::abs(double(errors)/total-qpsk_awgn_ber(0))<.002,"IQ AWGN BERと理論の一致");
  ProtocolConfig pc;pc.awgn.enabled=true;pc.lab_phy.enabled=true;
  pc.awgn.bandwidth_ab_hz=pc.awgn.bandwidth_ba_hz=pc.awgn.bit_rate_bps;
  LinkProtocol clean(pc,[](double){return channel(100);});clean.Advance(.0002);
  check(clean.Connected()&&clean.Stats().data_frames>0,"仮PHY接続とデータ受領");
  check(std::abs(clean.Stats().delivered_mb-clean.Stats().data_frames*.002048)<1e-12,"ヘッダーを通信量に加算しない");
  using Result=std::tuple<double,std::string,uint64_t,uint64_t>;
  std::vector<Result>a,b;
  auto record=[](std::vector<Result>&v,const LinkEvent&e){if(e.type=="FRAME_RESULT")v.emplace_back(e.time,e.detail,e.bit_errors,e.processed_samples);};
  LinkProtocol stepped(pc,[](double){return channel(8);},[&](auto&e){record(a,e);});
  LinkProtocol jump(pc,[](double){return channel(8);},[&](auto&e){record(b,e);});
  for(unsigned i=0;i<=100;++i)stepped.Advance(i*.0001);
  jump.Advance(.01);
  check(a==b,"Gazebo刻みによらずsampleとイベント結果一致");
  // Request損失中のBeacon継続と500ms待ち。感度ゲートで波形計算負荷を抑える。
  unsigned beacons=0;LinkProtocol loss(pc,[](double t){auto s=channel(100);s.ab_ok=false;if(t>.001)s.ba_ok=false;return s;},
   [&](auto&e){if(e.type=="TX"&&e.frame=="BEACON")++beacons;});loss.Advance(.501);
  check(beacons>3500&&loss.Stats().timeouts==1,"Request損失500ms timeoutとBeacon継続");
  pc.ifs_s=137e-6;
  bool request_overlap=false,beacon_overlap=false;
  LinkProtocol overlap(pc,[](double){return channel(100);},[&](auto&e){
    if(e.type=="FRAME_RESULT"&&e.detail=="RECEIVER_TRANSMITTING"){
      request_overlap|=e.frame=="REQUEST";beacon_overlap|=e.frame=="BEACON";
    }});
  overlap.Advance(.0003);
  check(request_overlap&&beacon_overlap,"異なるBeacon番号との半二重重複");
  std::cout<<"仮codec/CRC/EH、IQ AWGN BER、奇数ブロック境界、通信時刻、500ms待ちの検証成功\n";
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
