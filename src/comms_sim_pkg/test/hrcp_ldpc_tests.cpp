#include "comms_sim_pkg/hrcp_ldpc.hpp"
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
using comms_sim::HrcpLdpc;
namespace {
void check(bool b,const char *message) {if(!b) throw std::runtime_error(message);}
}
int main() {try {
  std::mt19937_64 rng(20261028);
  for (unsigned k:{1056u,1344u}) {
    HrcpLdpc code(k);
    std::vector<unsigned> weights(1440);
    for(auto &row:code.Checks())for(auto i:row)++weights[i];
    for(auto w:weights)check(w==3,"規定行列のcolumn weight");
    for(unsigned l:{1u,7u,k-1,k})for(unsigned pattern=0;pattern<3;++pattern) {
      std::vector<uint8_t> data(l);
      for(auto &b:data)b=pattern==0?0:pattern==1?1:rng()%2;
      auto word=code.Encode(data);
      check(code.SyndromeZero(word),"符号語のsyndrome");
      for(unsigned i=l;i<k;++i)check(!word[i],"shorteningの既知0");
      std::vector<double> llr(1440);
      for(unsigned i=0;i<1440;++i)llr[i]=word[i]?-12.:12.;
      auto decoded=code.Decode(llr,l);
      check(decoded.converged&&decoded.iterations==0&&decoded.information==data,"無雑音復号");
      const unsigned flipped=pattern==0?0:k+(rng()%(1440-k));
      llr[flipped]*=-.3;
      decoded=code.Decode(llr,l);
      check(decoded.converged&&decoded.information==data,"情報/parityの誤り訂正");
    }
    // 行ごとの規定式+MSBからの別実装GF(2)消去で得た、i0=1のparity fixture。
    const std::string hex=k==1056?
      "e7560e219d21934f09a4d1d001b805b8e63689c8100341a6b90c320665b0d125b648d81138055413adb6cce3ec8bf806":
      "003002ca06440c111345114061";
    auto basis=code.Encode({1});
    for(unsigned i=0;i<1440-k;++i){
      char ch=hex[hex.size()-1-i/4];unsigned digit=ch<='9'?ch-'0':ch-'a'+10;
      check(basis[k+i]==((digit>>(i%4))&1),"独立実装parity fixtureとの照合");
    }
    // 複数箇所に硬判定誤りがあるAWGN軟判定を確認する。
    std::normal_distribution<double> noise(0.,.32);
    uint64_t raw_errors=0; unsigned recovered=0;
    for(unsigned trial=0;trial<24;++trial) {
      std::vector<uint8_t> data(k);for(auto &b:data)b=rng()%2;
      auto word=code.Encode(data);std::vector<double> llr(1440);
      for(unsigned i=0;i<1440;++i){double y=(word[i]?-1.:1.)+noise(rng);llr[i]=2*y/(.32*.32);raw_errors+=(y<0)!=bool(word[i]);}
      auto d=code.Decode(llr,k);
      recovered+=d.converged&&d.information==data;
    }
    check(raw_errors>0&&recovered>=20,"AWGN下のLDPC軟判定訂正");
  }
  std::cout<<"HRCP LDPC: 行列/系統符号化/shortening/軟判定復号の検証成功\n";
}catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}}
