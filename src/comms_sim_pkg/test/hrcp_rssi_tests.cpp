#include "comms_sim_pkg/hrcp_rssi_meter.hpp"
#include <complex>
#include <iostream>
#include <random>
#include <stdexcept>
using namespace comms_sim;
void check(bool ok,const char*why){if(!ok)throw std::runtime_error(why);}
void near(double a,double b,double tolerance){check(std::isfinite(a)&&std::abs(a-b)<tolerance,"波形RSSIの数値不一致");}
int main(){try{
  HrcpRssiMeter meter(8);
  for(unsigned n=1;n<8;n+=2)meter.Push(n,{1e-6,1e-9},{true,n<5});
  near(meter.Latest().dbm[0],-30,1e-10);near(meter.Latest().dbm[1],-60,1e-10);
  check(meter.Latest().count[0]==4&&meter.Latest().count[1]==2,"自機送信を平均の分母から除外");
  // dBmの平均ではなく、Wの算術平均。欠測は0dBmや予測値で埋めない。
  for(unsigned n=9;n<16;n+=2)meter.Push(n,{n==9?4e-6:0,0},{true,false});
  near(meter.Latest().dbm[0],-30,1e-10);check(std::isnan(meter.Latest().dbm[1]),"全窓送信中は欠測");
  meter.Push(17,{1e-9,1e-8},{true,true});check(meter.Flush(),"最終部分窓");
  check(meter.Latest().start==16&&meter.Latest().end==18,"部分窓時刻");near(meter.Latest().dbm[1],-50,1e-10);
  bool rejected=false;try{meter.Push(21,{0,0},{false,false});}catch(const std::logic_error&){rejected=true;}
  check(rejected,"sample欠落を検出");
  // 実I/Qの二乗から、S+N・雑音のみ・任意の受信増幅率の校正を確認。
  constexpr unsigned count=100000;
  HrcpRssiMeter awgn(2*count);std::mt19937_64 rng(51);std::normal_distribution<double> gaussian(0,std::sqrt(.5));
  const double signal_w=1e-9,noise_w=1e-10,bw=2.1648e9,chip_rate=1.76e9;
  const double scale=noise_w*chip_rate/bw,amplitude=std::sqrt(signal_w/scale);
  for(unsigned i=0;i<count;++i){
    std::complex<double> noise(gaussian(rng),gaussian(rng));
    awgn.Push(2*i+1,{std::norm(amplitude+noise)*scale,std::norm(noise)*scale},{true,true});
  }
  near(awgn.Latest().dbm[0],10*std::log10(signal_w+scale)+30,.05);
  near(awgn.Latest().dbm[1],10*std::log10(scale)+30,.05);
  std::cout<<"受信波形RSSI: W平均・欠測・窓境界・校正・AWGN確認成功\n";
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
