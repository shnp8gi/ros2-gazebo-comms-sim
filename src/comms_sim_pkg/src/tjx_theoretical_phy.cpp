#include "comms_sim_pkg/tjx_theoretical_phy.hpp"
#include "comms_sim_pkg/hrcp_mac.hpp"
#include "comms_sim_pkg/hrcp_modem.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace comms_sim {
const std::vector<unsigned>&tjx_mcs_ids(){static const std::vector<unsigned> ids{0,3,6,8,9,11};return ids;}
const TjxMcs &tjx_mcs(unsigned id){
  static const TjxMcs modes[]={{0,1,1056,2},{3,1,1056,1},{6,2,1056,1},
                                {8,2,1344,1},{9,4,1056,1},{11,4,1344,1}};
  for(const auto&m:modes)if(m.id==id)return m;
  throw std::invalid_argument("TJX MCSは0/3/6/8/9/11です");
}
double tjx_rate_bps(unsigned id,bool pilot){const auto&m=tjx_mcs(id);
  return 1.76e9*m.bits_per_symbol*m.information_bits/1440./m.repetition*(pilot?56./64.:1.);}
uint64_t tjx_coded_bits(unsigned id,unsigned bytes){const auto&m=tjx_mcs(id);
  if(bytes>2099200)throw std::invalid_argument("TJX本文長が上限を超えます");
  uint64_t bits=uint64_t(bytes)*8;return bits+((bits+m.information_bits-1)/m.information_bits)*(1440-m.information_bits);
}
uint64_t tjx_frame_chips(unsigned id,unsigned bytes,bool setup,bool pilot){const auto&m=tjx_mcs(id);
  const auto bits=tjx_coded_bits(id,bytes)*m.repetition;
  const unsigned block_bits=(pilot?56:64)*m.bits_per_symbol;
  return (setup?5120:3328)+1216+64*((bits+block_bits-1)/block_bits)+(pilot?8:0);
}
std::set<std::pair<int,unsigned>> tjx_required_blocks(unsigned data_bytes,unsigned control){
  if(!data_bytes||data_bytes>2048)throw std::invalid_argument("TJX DATA長は1〜2048 bytesです");
  std::set<std::pair<int,unsigned>> keys{{-1,132}};
  auto add=[&](unsigned id,unsigned bytes){unsigned k=tjx_mcs(id).information_bits;
    for(unsigned rest=bytes*8;rest;){unsigned n=std::min(k,rest);keys.emplace(id,n);rest-=n;}};
  for(auto id:tjx_mcs_ids())add(id,hrcp_subframe(0,std::vector<uint8_t>(data_bytes)).size());
  add(control,hrcp_beacon_encode(HrcpBeacon{}).size());
  add(control,hrcp_command(0,0,hrcp_request_payload(500)).size());
  add(control,hrcp_command(0,1,hrcp_response_payload(1,500)).size());
  add(control,hrcp_command(1,2,{0}).size());
  return keys;
}
TjxBlockResult TjxBlockSimulator::Run(int id,unsigned length,double es_no_db){
  if(!std::isfinite(es_no_db)||es_no_db< -100||es_no_db>100)throw std::invalid_argument("Es/N0範囲外");
  const double amp=std::sqrt(std::pow(10.,es_no_db/10.));
  if(id==-1){
    if(length!=132)throw std::invalid_argument("EHヘッダーは132情報bitです");
    HrcpHeader h;h.seed=rng_()%16;for(auto &b:h.mac)b=rng_();
    const auto encoded=codec_.Encode(h,{});std::vector<double> llr;
    // 4-chip spreadingを等価な軟合成で再現。同期・チャネルは既知とする。
    for(auto bit:encoded.header_bits){double value=0;
      for(unsigned repeat=0;repeat<4;++repeat)value+=4*amp*((bit?-amp:amp)+normal_(rng_));
      llr.push_back(value);}
    const auto d=codec_.DecodeHeader(llr);
    return {!d.ok||d.header.mac!=h.mac||d.header.seed!=h.seed||d.header.body_bytes!=0||d.header.mcs!=0||d.header.pilot!=h.pilot,0};
  }
  const auto&m=tjx_mcs(id);if(!length||length>m.information_bits)throw std::invalid_argument("短縮LDPC長が範囲外です");
  const auto&code=m.information_bits==1056?code11_:code14_;
  HrcpBits message(length);for(auto&b:message)b=rng_()&1;
  auto word=code.Encode(message);std::vector<unsigned> order;
  for(unsigned i=length;i>0;--i)order.push_back(i-1);
  for(unsigned i=1440;i>m.information_bits;--i)order.push_back(i-1);
  std::vector<double> llr(1440,80.);
  for(size_t start=0;start<order.size();start+=m.bits_per_symbol){
    unsigned label=0;for(unsigned j=0;j<m.bits_per_symbol;++j)label=(label<<1)|(start+j<order.size()?word[order[start+j]]:0);
    if(m.bits_per_symbol==1){double value=0;
      for(unsigned r=0;r<m.repetition;++r)value+=4*amp*((label?-amp:amp)+normal_(rng_));
      llr[order[start]]=std::clamp(value,-80.,80.);
    }else{
      const unsigned internal=m.bits_per_symbol==2?0:2;
      auto sample=amp*HrcpModem::Map(internal,label,start/m.bits_per_symbol)+HrcpComplex(normal_(rng_),normal_(rng_));
      const auto values=HrcpModem::Demap(sample,{amp,0},internal,start/m.bits_per_symbol);
      for(unsigned j=0;j<m.bits_per_symbol&&start+j<order.size();++j)llr[order[start+j]]=values[j];
    }
  }
  const auto d=code.Decode(llr,length,50,.8);TjxBlockResult result;
  for(unsigned i=0;i<length;++i)result.bit_errors+=d.information[i]!=message[i];
  result.error=!d.converged||result.bit_errors>0;return result;
}
TjxTheoreticalPhy::TjxTheoreticalPhy(TjxTheoryConfig config):config_(std::move(config)){
  tjx_mcs(config_.control_mcs);auto root=YAML::LoadFile(config_.table);
  if(root["model"].as<std::string>()!="tjx_ldpc_awgn_v1"||root["chip_rate_hz"].as<double>()!=1.76e9||
     root["decoder_iterations"].as<unsigned>()!=50||root["decoder_normalization"].as<double>()!=.8||
     root["channel"].as<std::string>()!="ideal_sync_known_channel_awgn")
    throw std::invalid_argument("TJX LUTのモデル定義が一致しません");
  for(const auto&row:root["points"]){int mcs=row["mcs"].as<int>();unsigned bits=row["information_bits"].as<unsigned>();
    if(mcs==-1?bits!=132:(!bits||bits>tjx_mcs(mcs).information_bits))throw std::invalid_argument("TJX LUTの符号長が不正です");
    double esno=row["es_no_db"].as<double>();auto trials=row["trials"].as<uint64_t>();
    auto errors=row["block_errors"].as<uint64_t>(),bit_errors=row["bit_errors"].as<uint64_t>();
    if(!std::isfinite(esno)||!trials||errors>trials||double(bit_errors)>double(trials)*bits)
      throw std::invalid_argument("TJX LUTの試行数・誤り数が不正です");
    curves_[{mcs,bits}].push_back({esno,double(errors)/trials,double(bit_errors)/trials/bits});
  }
  if(curves_.empty())throw std::invalid_argument("TJX LUTが空です");
  for(auto &[key,points]:curves_){(void)key;std::sort(points.begin(),points.end(),[](auto&a,auto&b){return a.es_no<b.es_no;});
    if(points.size()<2)throw std::invalid_argument("TJX LUTには各曲線2点以上必要です");
    for(size_t i=1;i<points.size();++i)if(points[i-1].es_no==points[i].es_no)throw std::invalid_argument("TJX LUTのEs/N0が重複しています");
  }
}
void TjxTheoreticalPhy::ValidateLengths(unsigned bytes)const{
  for(auto key:tjx_required_blocks(bytes,config_.control_mcs))if(!curves_.count(key))
    throw std::invalid_argument("必要なTJX LUT曲線がありません: MCS="+std::to_string(key.first)+" bits="+std::to_string(key.second));
}
TjxTheoreticalPhy::Point TjxTheoreticalPhy::Lookup(int mcs,unsigned bits,double esno,bool &outside)const{
  auto found=curves_.find({mcs,bits});if(found==curves_.end())throw std::invalid_argument("短縮符号長に対応するTJX LUTがありません");
  const auto&p=found->second;
  if(esno<p.front().es_no){outside=true;return {esno,1,std::numeric_limits<double>::quiet_NaN()};}
  if(esno>p.back().es_no){outside=true;return p.back();}
  auto hi=std::lower_bound(p.begin(),p.end(),esno,[](auto&a,double x){return a.es_no<x;});
  if(hi==p.begin()||hi->es_no==esno)return *hi;
  auto lo=hi-1;double a=(esno-lo->es_no)/(hi->es_no-lo->es_no);
  return {esno,lo->per+a*(hi->per-lo->per),lo->ber+a*(hi->ber-lo->ber)};
}
TjxTheoryDecision TjxTheoreticalPhy::Evaluate(unsigned id,unsigned bytes,double snr,double bw)const{
  const auto&m=tjx_mcs(id);tjx_coded_bits(id,bytes);
  if(!std::isfinite(snr)||!std::isfinite(bw)||bw<=0)throw std::invalid_argument("TJXモデルには有限SNRと正の雑音帯域が必要です");
  TjxTheoryDecision d;d.es_no_db=snr+10*std::log10(bw/1.76e9);
  d.header_per=Lookup(-1,132,d.es_no_db,d.outside_range).per;
  double success=1,errors=0;for(unsigned rest=bytes*8;rest;){unsigned n=std::min(m.information_bits,rest);
    auto point=Lookup(id,n,d.es_no_db,d.outside_range);success*=1-point.per;errors+=n*point.ber;rest-=n;}
  d.body_per=1-success;d.per=1-(1-d.header_per)*success;d.ber=bytes?errors/(bytes*8.):0;return d;
}
}
