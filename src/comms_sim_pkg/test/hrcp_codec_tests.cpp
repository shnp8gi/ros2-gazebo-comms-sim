#include "comms_sim_pkg/hrcp_codec.hpp"
#include <iostream>
#include <stdexcept>
#include <random>
using namespace comms_sim;
void check(bool b,const char *m){if(!b)throw std::runtime_error(m);}
std::vector<double> soft(const HrcpBits &bits){std::vector<double>x;for(auto b:bits)x.push_back(b?-20.:20.);return x;}
int main(){try{
  HrcpScrambler generator(0);std::string sequence;for(unsigned i=0;i<16;++i)sequence+=char('0'+generator.Next());
  check(sequence=="0001111000111010","規格PRBS既知系列");
  std::vector<uint8_t> ascii{'1','2','3','4','5','6','7','8','9'};
  check(HrcpCodec::FrameCrc(ascii)==0x649c2fd3,"FCS送信順既知ベクトル");
  HrcpCodec::AppendFcs(ascii);check(HrcpCodec::CheckFcs(ascii),"FCS検査");
  check(ascii[9]==0x26&&ascii[10]==0x39&&ascii[11]==0xf4&&ascii[12]==0xcb,"FCSのwire octet順");
  for(unsigned a=0;a<16;++a)for(unsigned b=0;b<a;++b)
    check(__builtin_popcount(unsigned(HrcpCodec::ExtendedHamming(a)^HrcpCodec::ExtendedHamming(b)))>=4,"EH最小距離4");
  HrcpCodec codec;std::mt19937 rng(1741);
  for(unsigned mcs=0;mcs<4;++mcs)for(bool pilot:{false,true})for(unsigned size:{0u,1u,131u,132u,133u,168u,169u,2056u}) {
    HrcpHeader h;h.mcs=mcs;h.seed=rng()%16;h.pilot=pilot;h.mac[0]=1;
    std::vector<uint8_t> body(size);for(auto &v:body)v=rng()%256;
    auto tx=codec.Encode(h,body);check(tx.header_bits.size()==266,"header264+stuff2");
    auto hdr=codec.DecodeHeader(soft(tx.header_bits));check(hdr.ok&&hdr.header.mac==h.mac&&hdr.header.body_bytes==size,"header往復");
    auto errors=soft(tx.header_bits);errors[3]*=-1;check(codec.DecodeHeader(errors).ok,"EH単一誤り訂正");
    errors[4]*=-1;check(!codec.DecodeHeader(errors).ok,"EH二重誤り検出");
    auto data=codec.DecodeBody(hdr.header,soft(tx.body_bits));check(data.converged&&data.bytes==body,"全MCS/shortening本文往復");
    check(HrcpCodec::FrameChips(tx.header,true)-HrcpCodec::FrameChips(tx.header,false)==1792,"長短preamble差");
  }
  // 並列復号でもshortening・反復回数・復号bit列が同一。
  HrcpCodec parallel_codec(4);
  for(unsigned mcs=0;mcs<4;++mcs){
    HrcpHeader h;h.mcs=mcs;h.seed=7;
    auto tx=codec.Encode(h,std::vector<uint8_t>(2056,0x5a));auto values=soft(tx.body_bits);
    for(size_t i=0;i<values.size();i+=17)values[i]*=-1;
    auto serial=codec.DecodeBody(tx.header,values),parallel=parallel_codec.DecodeBody(tx.header,values);
    check(serial.bytes==parallel.bytes&&serial.converged==parallel.converged&&serial.iterations==parallel.iterations,"並列LDPCの結果一致");
  }
  // 正常な符号語へ化けたheaderでもHCSが検出する。
  auto frame=codec.Encode(HrcpHeader{},std::vector<uint8_t>(20,0x5a));
  auto corrupted=frame.header_bits;auto replacement=HrcpCodec::ExtendedHamming(15);
  for(unsigned i=0;i<8;++i)corrupted[8*15+i]=(replacement>>(7-i))&1;
  check(!codec.DecodeHeader(soft(corrupted)).ok,"EHを通るheader破損を拒否");
  std::cout<<"HRCP codec: PRBS/CRC/EH/header/LDPC/長さの検証成功\n";
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
