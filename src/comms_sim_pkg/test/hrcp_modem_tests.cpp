#include "comms_sim_pkg/hrcp_modem.hpp"
#include <iostream>
#include <random>
#include <stdexcept>
using namespace comms_sim;
void require(bool x,const char*m){if(!x)throw std::runtime_error(m);}
int main(){
  HrcpCodec codec;HrcpModem modem(codec);std::mt19937 rng(42);std::normal_distribution<double> noise(0,std::sqrt(.5));
  // 与えるのは無通信区間を含む連続sampleだけ。TXのフレーム境界・正解bitsをRXに渡さない。
  for(unsigned mcs=0;mcs<4;++mcs)for(bool setup:{false,true})for(bool pilot:{false,true}){
    HrcpHeader h;h.mcs=mcs;h.pilot=pilot;h.seed=7;std::vector<uint8_t> body(139);for(auto &b:body)b=rng();
    HrcpCodec::AppendFcs(body);auto tx=modem.Transmit(h,body,setup);std::vector<HrcpReception> received;
    HrcpStreamReceiver rx(codec,[&](const auto&r){if(!r.header_event)received.push_back(r);});uint64_t n=0;
    for(unsigned i=0;i<437;++i)rx.Feed({noise(rng),noise(rng)},n++);
    for(auto s:tx)rx.Feed(10.*s+HrcpComplex(noise(rng),noise(rng)),n++);
    for(unsigned i=0;i<777;++i)rx.Feed({noise(rng),noise(rng)},n++);
    if(received.size()!=1)std::cerr<<"mcs="<<mcs<<" setup="<<setup<<" pilot="<<pilot<<" received="<<received.size()<<"\n";
    require(received.size()==1,"frame detection");require(received[0].header_ok,"header");require(received[0].body_ok,"LDPC");require(received[0].body==body,"payload");
  }
  unsigned detected=0;HrcpStreamReceiver idle(codec,[&](const auto&){++detected;});for(unsigned i=0;i<1000000;++i)idle.Feed({noise(rng),noise(rng)},i);require(!detected,"noise-only false frame");
  // 複素雑音のI/Q分散とシンボルエネルギーの正規化を独立したQPSK理論値で照合。
  const double es_no=std::pow(10.,.4);unsigned errors=0;
  for(unsigned i=0;i<50000;++i){unsigned label=rng()%4;
    auto y=std::sqrt(es_no)*HrcpModem::Map(0,label,i)+HrcpComplex(noise(rng),noise(rng));
    auto llr=HrcpModem::Demap(y,{std::sqrt(es_no),0},0,i);
    for(unsigned bit=0;bit<2;++bit)errors+=bool(llr[bit]<0)!=bool((label>>(1-bit))&1);
  }
  double expected=.5*std::erfc(std::sqrt(es_no/2));
  require(std::abs(errors/100000.-expected)<.003,"QPSK AWGN normalization");
  std::cout<<"HRCP continuous receiver tests PASS\n";
}
