#include "comms_sim_pkg/hrcp_modem.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace comms_sim {
namespace {
constexpr double q=.7071067811865475244;
HrcpComplex rotation(uint64_t n){static const HrcpComplex r[]={{1,0},{0,1},{-1,0},{0,-1}};return r[n%4];}
void demap_points(HrcpComplex sample,const HrcpComplex*points,unsigned bits,double*out){
  std::array<double,4> d0{{1e300,1e300,1e300,1e300}},d1=d0;
  for(unsigned label=0;label<(1u<<bits);++label){double distance=std::norm(sample-points[label]);
    for(unsigned i=0;i<bits;++i){auto &best=((label>>(bits-1-i))&1)?d1[i]:d0[i];best=std::min(best,distance);}}
  for(unsigned i=0;i<bits;++i)out[i]=std::clamp(d1[i]-d0[i],-80.,80.);
}
HrcpBits hex_bits(const char *hex){HrcpBits b;std::string number(hex);
  // Table13-13の16進数を§6.1→§4.1の数値順（LSB first）で展開。
  for(auto it=number.rbegin();it!=number.rend();++it){unsigned n=*it<='9'?*it-'0':*it-'A'+10;for(unsigned i=0;i<4;++i)b.push_back((n>>i)&1);}return b;}
void append(HrcpBits &to,const HrcpBits &from,bool negate=false){for(auto b:from)to.push_back(b^negate);}
HrcpBits preamble(bool setup){
  const auto &a=HrcpModem::GolayA();auto b=hex_bits("A5AA6669C3CC000F0F00CCC36966AAA5");
  // Table13-13: Golay構成ベクトルは右側から送信する。
  HrcpBits a256=a,b256=a;append(a256,b);append(b256,b,true);
  HrcpBits a512=a256,b512=a256;append(a512,b256);append(b512,b256,true);
  HrcpBits p;for(unsigned i=0;i<(setup?28u:14u);++i)append(p,a);
  append(p,a,true);append(p,b);append(p,a512);append(p,b512);append(p,a256);return p;
}
void pilot(std::vector<HrcpComplex>&out){for(unsigned i=0;i<8;++i)out.push_back(HrcpModem::Bpsk((0xeb>>i)&1,out.size()));}
}
const HrcpBits &HrcpModem::GolayA(){static auto a=hex_bits("5A5599963C33FFF00F00CCC36966AAA5");return a;}
HrcpComplex HrcpModem::Bpsk(unsigned bit,uint64_t chip){return (bit?1.:-1.)*HrcpComplex(q,q)*rotation(chip+1);}
HrcpComplex HrcpModem::Map(unsigned mcs,unsigned label,uint64_t chip){
  if(mcs<2){return HrcpComplex((label&2)?-q:q,(label&1)?q:-q)*rotation(chip+1);}
  if(mcs>3||label>15)throw std::invalid_argument("HRCP constellation値が範囲外です");
  static constexpr int levels[]={-3,-1,3,1};return HrcpComplex(levels[label>>2],levels[label&3])/std::sqrt(10.);
}
std::vector<double> HrcpModem::Demap(HrcpComplex sample,HrcpComplex channel,unsigned mcs,uint64_t chip){
  unsigned bits=HrcpCodec::BitsPerSymbol(mcs);std::vector<double> out(bits);
  std::array<HrcpComplex,16> points{};
  for(unsigned label=0;label<(1u<<bits);++label)points[label]=channel*Map(mcs,label,chip);
  demap_points(sample,points.data(),bits,out.data());return out;
}
std::vector<HrcpComplex> HrcpModem::Transmit(HrcpHeader header,const std::vector<uint8_t>&body,bool setup)const{
  auto encoded=codec_.Encode(header,body);header=encoded.header;
  std::vector<HrcpComplex> out;out.reserve(HrcpCodec::FrameChips(header,setup));
  for(auto bit:preamble(setup))out.push_back(Bpsk(bit,out.size()));
  HrcpBits spread;for(auto bit:encoded.header_bits)for(unsigned i=0;i<4;++i)spread.push_back(bit?i%2:1-i%2);
  for(unsigned start=0;start<spread.size();start+=56){pilot(out);for(unsigned i=0;i<56;++i)out.push_back(Bpsk(spread[start+i],out.size()));}
  const unsigned b=HrcpCodec::BitsPerSymbol(header.mcs),data_chips=header.pilot?56:64;
  for(size_t start=0;start<encoded.body_bits.size();start+=b*data_chips){
    if(header.pilot)pilot(out);
    for(unsigned i=0;i<data_chips;++i){unsigned label=0;for(unsigned j=0;j<b;++j)label=(label<<1)|encoded.body_bits[start+i*b+j];out.push_back(Map(header.mcs,label,out.size()));}}
  if(header.pilot)pilot(out);
  if(out.size()!=HrcpCodec::FrameChips(header,setup))throw std::logic_error("HRCP波形長が不一致です");
  return out;
}
HrcpStreamReceiver::HrcpStreamReceiver(const HrcpCodec&codec,Callback callback,unsigned max_body_bytes)
  :codec_(codec),callback_(std::move(callback)),max_body_bytes_(max_body_bytes){}
void HrcpStreamReceiver::Reset(){
  fill_=0;ring_.fill({});correlation_={};energy_=old_energy_=0;repeats_=0;collecting_=header_decoded_=false;collected_.clear();
}
void HrcpStreamReceiver::Feed(HrcpComplex sample,uint64_t chip){
  if(!std::isfinite(sample.real())||!std::isfinite(sample.imag()))throw std::invalid_argument("非有限の受信sampleです");
  ++processed_;if(collecting_){Collect(sample,chip);return;}
  auto lag=ring_[(fill_+128)%256],old=ring_[fill_%256];
  correlation_+=sample*std::conj(lag)-lag*std::conj(old);
  energy_+=std::norm(sample)-std::norm(lag);old_energy_+=std::norm(lag)-std::norm(old);
  ring_[fill_%256]=sample;++fill_;if(fill_<256)return;
  if(repeats_ && chip+1<lock_next_)return;
  const double repeat_metric=std::norm(correlation_)/std::max(energy_*old_energy_,1e-30);
  if(!repeats_&&repeat_metric<.20)return;
  HrcpComplex sum{};const auto &a=HrcpModem::GolayA();
  for(unsigned i=0;i<128;++i)sum+=ring_[(fill_-128+i)%256]*std::conj(HrcpModem::Bpsk(a[i],i));
  const double m=std::norm(sum)/std::max(128*energy_,1e-30);
  if(m<.30){repeats_=0;return;}
  if(!repeats_){channel_=sum/128.;metric_=m;repeats_=1;lock_next_=chip+1+128;return;}
  if(std::real(sum*std::conj(channel_))>0){channel_=.75*channel_+.25*sum/128.;++repeats_;lock_next_+=128;return;}
  if(repeats_<2){repeats_=0;return;}
  // SFDは-a128。CES1408chipの後に19blockのheader。
  sfd_end_=chip+1;collecting_=true;header_decoded_=false;collected_.clear();target_=1408+1216;
  result_={};result_.sfd_end_chip=sfd_end_;result_.sync_metric=metric_;result_.estimated_es_no=std::max(1e-30,std::norm(channel_));
}
void HrcpStreamReceiver::Collect(HrcpComplex sample,uint64_t chip){
  collected_.push_back(sample);if(collected_.size()!=target_)return;
  if(!header_decoded_){
    std::vector<double> header_llr;header_llr.reserve(266);double value=0;unsigned count=0;
    for(unsigned block=0;block<19;++block)for(unsigned j=0;j<56;++j){
      unsigned n=1408+block*64+8+j;
      const double l=-4*std::real(collected_[n]*std::conj(channel_*HrcpModem::Bpsk(1,n)));
      // spreading: bit0=1010、bit1=0101。
      value+=(count%2?-1:1)*(-l);++count;
      if(count%4==0){header_llr.push_back(value);value=0;}
    }
    auto h=codec_.DecodeHeader(header_llr);result_.header_ok=h.ok;result_.reason=h.reason;
    if(!h.ok){result_.end_chip=chip+1;Complete();return;}
    result_.header=h.header;
    if(h.header.body_bytes>max_body_bytes_){result_.reason="RECEIVER_LENGTH_LIMIT";result_.end_chip=chip+1;Complete();return;}
    result_.header_event=true;result_.end_chip=chip+1;
    if(callback_)callback_(result_);
    result_.header_event=false;
    header_decoded_=true;target_+=64*HrcpCodec::BodyBlocks(h.header)+(h.header.pilot?8:0);
    if(target_!=collected_.size())return;
  }
  std::vector<double> body_llr;auto &h=result_.header;
  const auto bits=HrcpCodec::BitsPerSymbol(h.mcs);
  body_llr.reserve(HrcpCodec::BodyBlocks(h)*(h.pilot?56:64)*bits);
  std::array<std::array<HrcpComplex,16>,4> points{};
  for(unsigned phase=0;phase<4;++phase)for(unsigned label=0;label<(1u<<bits);++label)
    points[phase][label]=channel_*HrcpModem::Map(h.mcs,label,phase);
  for(uint64_t block=0;block<HrcpCodec::BodyBlocks(h);++block)for(unsigned j=h.pilot?8:0;j<64;++j){
    auto n=1408+1216+block*64+j;
    std::array<double,4> llr{};demap_points(collected_[n],points[n%4].data(),bits,llr.data());
    body_llr.insert(body_llr.end(),llr.begin(),llr.begin()+bits);}
  auto b=codec_.DecodeBody(h,body_llr);result_.body_ok=b.converged;result_.body=std::move(b.bytes);result_.ldpc_iterations=b.iterations;
  result_.reason=b.converged?"PHY_DECODED_MAC_CRC_REQUIRED":"BODY_LDPC";result_.end_chip=chip+1;Complete();
}
void HrcpStreamReceiver::Complete(){auto result=std::move(result_);Reset();if(callback_)callback_(result);}
}
