#include "comms_sim_pkg/hrcp_codec.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace comms_sim {
namespace {
void put(HrcpBits &bits,uint64_t value,unsigned width) {
  for(unsigned i=0;i<width;++i)bits.push_back((value>>i)&1);
}
unsigned get(const HrcpBits &bits,size_t &offset,unsigned width) {
  if(offset+width>bits.size())throw std::invalid_argument("HRCP fieldのbit数不足");
  unsigned value=0;for(unsigned i=0;i<width;++i)value|=unsigned(bits[offset++])<<i;return value;
}
void msb(HrcpBits &bits,uint64_t value,unsigned width) {
  for(unsigned i=width;i>0;--i)bits.push_back((value>>(i-1))&1);
}
uint64_t crc(const HrcpBits &bits,unsigned width,uint64_t polynomial,uint64_t initial) {
  const uint64_t mask=(uint64_t(1)<<width)-1;uint64_t value=initial;
  for(auto bit:bits){if(bit>1)throw std::invalid_argument("CRC入力がbitではありません");
    const bool carry=((value>>(width-1))^bit)&1;value=(value<<1)&mask;if(carry)value^=polynomial;}
  return value^mask;
}
void validate(const HrcpHeader &h) {
  if(h.mcs>3 || h.seed>15 || h.body_bytes>2099200)throw std::invalid_argument("対応外のHRCPヘッダー値");
}
}
HrcpScrambler::HrcpScrambler(unsigned seed) {
  if(seed>15)throw std::invalid_argument("scrambler seedは4bitです");
  state_={{1,1,0,1,0,0,0,0,1,0,1,0,0,0,0}};
  for(unsigned i=0;i<4;++i)state_[11+i]=(seed>>(3-i))&1;
}
uint8_t HrcpScrambler::Next() {
  auto next=state_[13]^state_[14];
  for(unsigned i=14;i>0;--i)state_[i]=state_[i-1];
  state_[0]=next;return next;
}
void HrcpScrambler::Apply(HrcpBits &bits) {for(auto &b:bits)b^=Next();}
HrcpCodec::HrcpCodec(unsigned threads):code11_(1056),code14_(1344),decode_threads_(threads) {
  if(!threads||threads>64)throw std::invalid_argument("復号スレッド数は1～64です");
}
const HrcpLdpc &HrcpCodec::Code(unsigned mcs) const {return InformationBits(mcs)==1056?code11_:code14_;}
unsigned HrcpCodec::BitsPerSymbol(unsigned mcs) {
  if(mcs>3)throw std::invalid_argument("HRCP初期対応MCSは0〜3です");
  return mcs<2?2:4;
}
unsigned HrcpCodec::InformationBits(unsigned mcs) {
  BitsPerSymbol(mcs);return mcs%2?1344:1056;
}
uint64_t HrcpCodec::CodedBodyBits(const HrcpHeader &h) {
  validate(h);const uint64_t bits=uint64_t(h.body_bytes)*8,k=InformationBits(h.mcs);
  return bits+((bits+k-1)/k)*(1440-k);
}
uint64_t HrcpCodec::BodyBlocks(const HrcpHeader &h) {
  auto n=CodedBodyBits(h);auto b=(h.pilot?56:64)*BitsPerSymbol(h.mcs);return (n+b-1)/b;
}
uint64_t HrcpCodec::FrameChips(const HrcpHeader &h,bool setup) {
  // PPREなし。先頭pilotを含むblockと、最後の追加pilotを数える。
  return (setup?5120:3328)+1216+64*BodyBlocks(h)+(h.pilot?8:0);
}
HrcpBits HrcpCodec::OctetBits(const std::vector<uint8_t> &bytes) {
  HrcpBits b;b.reserve(bytes.size()*8);for(auto v:bytes)put(b,v,8);return b;
}
uint16_t HrcpCodec::HeaderCrc(const HrcpBits &bits) {return crc(bits,16,0xa12b,0xffff);}
uint32_t HrcpCodec::FrameCrc(const std::vector<uint8_t> &bytes) {return crc(OctetBits(bytes),32,0x04c11db7,0xffffffff);}
uint8_t HrcpCodec::SubheaderCrc(const std::array<uint8_t,3> &bytes) {
  // §6.3.6.1: 24bit多項式の剰余の1の補数。初期状態0、出力MSB first。
  return crc(OctetBits(std::vector<uint8_t>(bytes.begin(),bytes.end())),8,0x07,0);
}
void HrcpCodec::AppendFcs(std::vector<uint8_t> &bytes) {
  HrcpBits serial;msb(serial,FrameCrc(bytes),32);
  for(unsigned i=0;i<4;++i){uint8_t v=0;for(unsigned j=0;j<8;++j)v|=serial[8*i+j]<<j;bytes.push_back(v);}
}
bool HrcpCodec::CheckFcs(const std::vector<uint8_t> &bytes) {
  if(bytes.size()<4)return false;
  std::vector<uint8_t> expected(bytes.begin(),bytes.end()-4);AppendFcs(expected);return expected==bytes;
}
uint8_t HrcpCodec::ExtendedHamming(unsigned input) {
  if(input>15)throw std::invalid_argument("拡張ハミング符号の入力は4bitです");
  // Table13-16。i0をnibbleのMSBとする。一般的なHamming配置を代用しない。
  static constexpr uint8_t parity[16]={0,14,11,5,7,9,12,2,13,3,6,8,10,4,1,15};
  return uint8_t((input<<4)|parity[input]);
}
HrcpEncoded HrcpCodec::Encode(HrcpHeader h,const std::vector<uint8_t> &body) const {
  if(body.size()>2099200)throw std::invalid_argument("HRCP本文が上限を超えます");
  h.body_bytes=body.size();validate(h);HrcpEncoded out;out.header=h;
  HrcpBits fields;put(fields,h.mcs,3);put(fields,h.pilot,1);put(fields,h.seed,4);put(fields,0,2);
  put(fields,h.body_bytes,22);put(fields,0,4);
  for(auto b:h.mac)put(fields,b,8);
  msb(fields,HeaderCrc(fields),16);
  HrcpScrambler scrambler(h.seed);
  for(unsigned i=36;i<132;++i)fields[i]^=scrambler.Next();
  for(unsigned i=0;i<132;i+=4) {
    unsigned nibble=0;for(unsigned j=0;j<4;++j)nibble=(nibble<<1)|fields[i+j];
    msb(out.header_bits,ExtendedHamming(nibble),8);
  }
  out.header_bits.push_back(0);out.header_bits.push_back(1);
  HrcpBits payload=OctetBits(body);scrambler.Apply(payload);const unsigned k=InformationBits(h.mcs);
  for(size_t start=0;start<payload.size();start+=k) {
    unsigned length=std::min<size_t>(k,payload.size()-start);
    // §13.2.3.6: r[l-1]からr[0]の順に入力。r[l..k)は既知0。
    HrcpBits indexed(length);for(unsigned i=0;i<length;++i)indexed[i]=payload[start+length-1-i];
    auto word=Code(h.mcs).Encode(indexed);
    for(unsigned i=length;i>0;--i)out.body_bits.push_back(word[i-1]);
    for(unsigned i=1440;i>k;--i)out.body_bits.push_back(word[i-1]);
  }
  const uint64_t padded=BodyBlocks(h)*(h.pilot?56:64)*BitsPerSymbol(h.mcs);
  while(out.body_bits.size()<padded)out.body_bits.push_back(scrambler.Next());
  return out;
}
HrcpDecodedHeader HrcpCodec::DecodeHeader(const std::vector<double> &llr) const {
  HrcpDecodedHeader out;
  if(llr.size()!=266){out.reason="HEADER_LENGTH";return out;}
  for(auto v:llr)if(!std::isfinite(v)){out.reason="HEADER_LLR";return out;}
  HrcpBits bits;
  for(unsigned start=0;start<264;start+=8) {
    unsigned received=0;for(unsigned j=0;j<8;++j)received=(received<<1)|(llr[start+j]<0);
    unsigned best=9,selected=0;
    for(unsigned input=0;input<16;++input) {
      unsigned d=__builtin_popcount(received^ExtendedHamming(input));
      if(d<best){best=d;selected=input;}
    }
    if(best>1){out.reason="HEADER_FEC";return out;}
    out.corrected_bits+=best;msb(bits,selected,4);
  }
  size_t offset=0;auto &h=out.header;
  h.mcs=get(bits,offset,3);h.pilot=get(bits,offset,1);h.seed=get(bits,offset,4);
  unsigned ppre=get(bits,offset,2);h.body_bytes=get(bits,offset,22);get(bits,offset,4); // reserved bitsは受信側で無視する。
  // 未対応値を正解値で補正しない。
  if(ppre || h.mcs>3 || h.body_bytes>2099200){out.reason="UNSUPPORTED_PHY_HEADER";return out;}
  HrcpScrambler scrambler(h.seed);for(unsigned i=36;i<132;++i)bits[i]^=scrambler.Next();
  for(auto &b:h.mac)b=get(bits,offset,8);
  uint16_t received_crc=0;for(unsigned j=116;j<132;++j)received_crc=(received_crc<<1)|bits[j];
  if(HeaderCrc(HrcpBits(bits.begin(),bits.begin()+116))!=received_crc){out.reason="HEADER_HCS";return out;}
  out.ok=true;return out;
}
HrcpDecodedBody HrcpCodec::DecodeBody(const HrcpHeader &h,const std::vector<double> &llr) const {
  validate(h);HrcpDecodedBody out;
  const uint64_t padded=BodyBlocks(h)*(h.pilot?56:64)*BitsPerSymbol(h.mcs);
  if(llr.size()!=padded)throw std::invalid_argument("HRCP本文のLLR長がヘッダーと不一致です");
  for(auto v:llr)if(!std::isfinite(v))throw std::invalid_argument("HRCP本文に非有限LLRがあります");
  const auto &code=Code(h.mcs);HrcpBits decoded;
  const unsigned total=h.body_bytes*8;out.converged=true;
  const int blocks=(total+code.K()-1)/code.K();
  std::vector<HrcpLdpc::Decoded> results(blocks);
#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(decode_threads_) if(decode_threads_>1 && blocks>1)
#endif
  for(int block=0;block<blocks;++block) {
    unsigned start=block*code.K();size_t cursor=static_cast<size_t>(block)*1440;
    unsigned length=std::min(code.K(),total-start);std::vector<double> indexed(1440,80.);
    for(unsigned i=length;i>0;--i)indexed[i-1]=llr[cursor++];
    for(unsigned i=1440;i>code.K();--i)indexed[i-1]=llr[cursor++];
    results[block]=code.Decode(indexed,length);
  }
  for(const auto&d:results){
    out.iterations+=d.iterations;out.converged&=d.converged;
    for(size_t i=d.information.size();i>0;--i)decoded.push_back(d.information[i-1]);
  }
  HrcpScrambler scrambler(h.seed);for(unsigned i=0;i<96;++i)scrambler.Next();scrambler.Apply(decoded);
  size_t p=0;for(unsigned i=0;i<h.body_bytes;++i)out.bytes.push_back(get(decoded,p,8));
  return out;
}
}
