#include "comms_sim_pkg/lab_phy.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace comms_sim {
namespace {
using Bits = std::vector<uint8_t>;
void put(Bits &b, uint64_t x, unsigned n) {
  for (unsigned i=n; i>0; --i) b.push_back((x>>(i-1))&1);
}
uint64_t get(const Bits &b, size_t &p, unsigned n) {
  if (p+n>b.size()) throw std::invalid_argument("仮PHYビット列が不足しています");
  uint64_t x=0; for(unsigned i=0;i<n;++i) x=(x<<1)|b[p++]; return x;
}
Bits protect(Bits b) {
  b.resize(((b.size()+10)/11)*11,0); Bits out;
  for(size_t i=0;i<b.size();i+=11) {
    std::array<uint8_t,17> w{}; unsigned k=0;
    for(unsigned j=1;j<16;++j) if(j!=1&&j!=2&&j!=4&&j!=8) w[j]=b[i+k++];
    for(unsigned p: {1u,2u,4u,8u}) for(unsigned j=1;j<16;++j) if(j&p) w[p]^=w[j];
    for(unsigned j=1;j<16;++j) w[16]^=w[j];
    for(unsigned j=1;j<=16;++j) out.push_back(w[j]);
  }
  return out;
}
bool unprotect(const Bits &b, Bits &out) {
  if(b.size()<208) return false;
  for(size_t i=0;i<208;i+=16) {
    std::array<uint8_t,17> w{}; unsigned syndrome=0,parity=0;
    for(unsigned j=1;j<=16;++j) {w[j]=b[i+j-1];parity^=w[j]; if(j<16&&w[j]) syndrome^=j;}
    if(syndrome && !parity) return false;
    if(parity) w[syndrome?syndrome:16]^=1;
    for(unsigned j=1;j<16;++j) if(j!=1&&j!=2&&j!=4&&j!=8) out.push_back(w[j]);
  }
  return true;
}
unsigned preamble(unsigned type,bool ab) {return ab&&type!=4?5120:3328;}
std::vector<int> golay() {
  std::vector<int>a{1},b{1};
  for(unsigned i=0;i<7;++i){auto x=a,y=a;x.insert(x.end(),b.begin(),b.end());for(int v:b)y.push_back(-v);a=x;b=y;}
  a.insert(a.end(),b.begin(),b.end());return a;
}
std::complex<double> rotation(size_t m) {
  static const std::array<std::complex<double>,4> r{{{1,0},{0,1},{-1,0},{0,-1}}};return r[m%4];
}
}
uint64_t LabPhy::Mix(uint64_t x) {
  x+=0x9e3779b97f4a7c15ULL;x=(x^(x>>30))*0xbf58476d1ce4e5b9ULL;
  x=(x^(x>>27))*0x94d049bb133111ebULL;return x^(x>>31);
}
LabPhy::LabPhy(LabPhyConfig c):c_(c) {
  if(c.data_bytes<1||c.data_bytes>65535||c.chunk_samples<1||c.chunk_samples>1048576||
     !std::isfinite(c.symbol_rate)||c.symbol_rate!=1.76e9||
     !std::isfinite(c.sync_threshold)||c.sync_threshold<=0||c.sync_threshold>=1)
    throw std::invalid_argument("仮PHY設定が不正です（1.76 Gbaud固定）");
}
unsigned LabPhy::Type(const std::string &s) {
  const std::array<std::string,6> names{"BEACON","REQUEST","RESPONSE","ACK","DATA","DATA_ACK"};
  auto it=std::find(names.begin(),names.end(),s);
  if(it==names.end())throw std::invalid_argument("未知の仮PHYフレーム");
  return it-names.begin();
}
uint16_t LabPhy::Crc16(const Bits &b) {
  uint16_t crc=0xffff;
  for(auto bit:b){bool carry=((crc>>15)^bit)&1;crc<<=1;if(carry)crc^=0x1021;}
  return crc;
}
uint32_t LabPhy::Crc32(const Bits &bytes) {
  uint32_t crc=0xffffffff;
  for(auto b:bytes){crc^=b;for(unsigned k=0;k<8;++k)crc=(crc>>1)^((crc&1)?0xedb88320:0);}
  return crc^0xffffffff;
}
uint64_t LabPhy::Samples(unsigned type,bool ab) const {
  unsigned bytes=type==4?c_.data_bytes:(type==3||type==5?0:16);
  return 2ULL*(preamble(type,ab)+208+(bytes?bytes*4+16:0)+10);
}
double LabPhy::Duration(unsigned type,bool ab) const {return Samples(type,ab)/(2*c_.symbol_rate);}
std::vector<uint8_t> LabPhy::Encode(const LabPacket &p) const {
  if(p.type>5||p.source>255||p.destination>255||p.body.size()>65535)throw std::invalid_argument("仮MAC設定が範囲外です");
  Bits h;
  // 両方向とも仮の37bit PHYヘッダー。MCS等の規格コード値は未確定。
  put(h,0,4);put(h,0,3);put(h,0,1);put(h,0,4);put(h,0,2);put(h,0,1);put(h,p.body.size(),22);
  put(h,0,2);put(h,p.type,3);put(h,0,2);put(h,0,1);put(h,p.source,8);put(h,p.destination,8);
  put(h,p.sequence,16);put(h,p.reference,16);put(h,p.body.size(),16);put(h,0,8);
  put(h,Crc16(h),16);Bits result=protect(h);
  for(auto b:p.body)put(result,b,8);
  if(!p.body.empty()){auto crc=Crc32(p.body);for(unsigned i=0;i<4;++i)put(result,(crc>>(i*8))&255,8);}
  return result;
}
LabRx LabPhy::Decode(const Bits &bits) const {
  LabRx r;Bits h;
  if(!unprotect(bits,h)){r.reason="HEADER_FEC_ERROR";return r;}
  Bits content(h.begin(),h.begin()+117);size_t pos=117;
  if(Crc16(content)!=get(h,pos,16)){r.reason="HEADER_CRC_ERROR";return r;}
  pos=15;auto phy_bytes=get(h,pos,22);auto version=get(h,pos,2);r.packet.type=get(h,pos,3);
  auto policy=get(h,pos,2),reserved=get(h,pos,1);r.packet.source=get(h,pos,8);r.packet.destination=get(h,pos,8);
  r.packet.sequence=get(h,pos,16);r.packet.reference=get(h,pos,16);auto bytes=get(h,pos,16);auto flags=get(h,pos,8);
  if(version||policy||reserved||flags||r.packet.type>5||bytes!=phy_bytes||
     bits.size()!=208+8*bytes+(bytes?32:0)){r.reason="INVALID_HEADER";return r;}
  const unsigned expected_bytes=r.packet.type==4?c_.data_bytes:(r.packet.type==3||r.packet.type==5?0:16);
  if(bytes!=expected_bytes){r.reason="INVALID_BODY_LENGTH";return r;}
  pos=208;for(size_t i=0;i<bytes;++i)r.packet.body.push_back(get(bits,pos,8));
  if(bytes){uint32_t crc=0;for(unsigned i=0;i<4;++i)crc|=get(bits,pos,8)<<(i*8);
    if(crc!=Crc32(r.packet.body)){r.reason="PAYLOAD_CRC_ERROR";return r;}}
  r.ok=true;return r;
}
LabRx LabPhy::Receive(const LabPacket &packet,bool ab,double eb_no_db,uint64_t arrival,uint64_t uid) const {
  if(!std::isfinite(eb_no_db))throw std::invalid_argument("仮PHYのEb/N0が不正です");
  auto bits=Encode(packet);const size_t pre=preamble(packet.type,ab);
  const size_t symbols=pre+208+(bits.size()-208)/2+10;
  if(2*symbols!=Samples(packet.type,ab))throw std::runtime_error("仮PHY body長と送信sample数が不一致です");
  auto g=golay();Bits received;received.reserve(bits.size());
  // 矩形2sample/symbolのunit-energy pulse。2sampleを積分して復調。
  // Es/N0=2Eb/N0、1sampleの複素雑音分散N0。ヘッダーも同じsymbol電力。
  const double sigma=std::sqrt(std::pow(10.,-eb_no_db/10.)/4);
  const double invsqrt2=std::sqrt(.5);
  std::complex<double> accumulator{};double corr=0,energy=0;
  uint64_t next_sample=0;LabRx out;
  const auto key=Mix(c_.seed^(ab?0x425f5258ULL:0x415f5258ULL));
  for(uint64_t base=0;base<symbols*2;base+=c_.chunk_samples) {
    const uint64_t end=std::min<uint64_t>(symbols*2,base+c_.chunk_samples);
    for(uint64_t n=base;n<end;++n) {
      if(n!=next_sample++)throw std::runtime_error("仮PHYのsample欠番/重複");
      const size_t m=n/2;std::complex<double> tx{};
      if(m<pre) tx={double(g[(m>=pre-128?128:0)+m%128]),0};
      else if(m<pre+208) tx={bits[m-pre]?-1.:1.,0};
      else if(m<symbols-10) {
        size_t i=208+2*(m-pre-208);
        tx={bits[i+1]?-invsqrt2:invsqrt2,bits[i]?-invsqrt2:invsqrt2};
        if(!ab)tx*=rotation(m-pre-208);
      }
      // 受信機+絶対サンプル番号をキーにする。フレームUIDは雑音に混ぜない。
      uint64_t index=arrival+n;
      double u1=((Mix(index^key)>>11)+.5)*0x1.0p-53;
      double u2=((Mix(index^key^0x9e3779b97f4a7c15ULL)>>11)+.5)*0x1.0p-53;
      double mag=sigma*std::sqrt(-2*std::log(u1)),phase=6.283185307179586*u2;
      std::complex<double> noise{mag*std::cos(phase),mag*std::sin(phase)};
      accumulator+=tx*invsqrt2+noise;
      if(n%2==0)continue;
      const auto rx=accumulator*invsqrt2;accumulator={};
      if(m<pre){double expected=g[(m>=pre-128?128:0)+m%128];corr+=rx.real()*expected;energy+=std::norm(rx);}
      else if(m<pre+208)received.push_back(rx.real()<0);
      else if(m<symbols-10){auto v=ab?rx:rx*std::conj(rotation(m-pre-208));received.push_back(v.imag()<0);received.push_back(v.real()<0);}
    }
  }
  (void)uid; // 情報ビット生成には利用可。受信器に正解のフレーム番号は渡さない。
  out.samples=next_sample;out.sync_metric=corr/std::sqrt(std::max(energy*pre,1e-300));
  for(size_t i=208;i<bits.size();++i){++out.checked_bits;if(bits[i]!=received[i])++out.bit_errors;}
  if(out.sync_metric<c_.sync_threshold){out.reason="PREAMBLE_NOT_DETECTED";return out;}
  auto decoded=Decode(received);out.ok=decoded.ok;out.reason=decoded.reason;out.packet=std::move(decoded.packet);return out;
}
}
