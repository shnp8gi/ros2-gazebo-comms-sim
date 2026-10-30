#include "comms_sim_pkg/empirical_phy.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace comms_sim {
namespace {
std::string trim(std::string s){auto first=s.find_first_not_of(" \t\r\n");
  return first==std::string::npos?"":s.substr(first,s.find_last_not_of(" \t\r\n")-first+1);}
std::vector<std::string> split(const std::string&s){std::vector<std::string> out;std::stringstream in(s);std::string v;
  while(std::getline(in,v,','))out.push_back(trim(v));
  if(!s.empty()&&s.back()==',')out.emplace_back();
  return out;}
double number(const std::string&s){size_t pos=0;double value=std::stod(s,&pos);
  if(pos!=s.size()||!std::isfinite(value))throw std::invalid_argument("CSV数値が不正です: "+s);
  return value;}
unsigned integer(const std::string&s){double v=number(s);if(v<0||v>100000000||v!=std::floor(v))throw std::invalid_argument("CSV整数が不正です: "+s);return unsigned(v);}
std::string key(bool ab,const std::string&frame,const std::string&mcs){return (ab?"Q_A_TO_B/":"P_B_TO_A/")+frame+"/"+mcs;}
}
double EmpiricalPhy::PacketError(double b,uint64_t bits){
  if(!std::isfinite(b)||b<0||b>1||!bits)throw std::invalid_argument("BERまたはbit数が不正です");
  return b==1?1:-std::expm1(double(bits)*std::log1p(-b));
}
EmpiricalPhy::EmpiricalPhy(EmpiricalConfig c):config_(std::move(c)){
  if(config_.metric!="ber"&&config_.metric!="per")throw std::invalid_argument("empirical_phy.metricはberまたはperです");
  if(config_.per_length_policy!="exact"&&config_.per_length_policy!="independent")throw std::invalid_argument("per_length_policyはexactまたはindependentです");
  if(!std::isfinite(config_.offset_ab_db)||!std::isfinite(config_.offset_ba_db)||config_.mcs_by_rate.empty())throw std::invalid_argument("実測モデルの補正値/MCS対応が不正です");
  for(const auto &[rate,mcs]:config_.mcs_by_rate)if(!std::isfinite(rate)||rate<=0||mcs>255)throw std::invalid_argument("MCS対応が不正です");
  std::ifstream in(config_.table);if(!in)throw std::invalid_argument("実測CSVを開けません: "+config_.table);
  std::string line;if(!std::getline(in,line))throw std::invalid_argument("実測CSVが空です");
  if(line.compare(0,3,"\xef\xbb\xbf")==0)line.erase(0,3);
  auto names=split(line);std::map<std::string,size_t> columns;
  for(size_t i=0;i<names.size();++i)if(!columns.emplace(names[i],i).second)throw std::invalid_argument("CSVの列名が重複しています");
  for(const auto &name:std::vector<std::string>{"direction","frame","mcs","rssi_dbm",config_.metric,"reference_bytes"})
    if(!columns.count(name))throw std::invalid_argument("実測CSVの列がありません: "+name);
  unsigned row=1;
  while(std::getline(in,line)){++row;if(trim(line).empty())continue;
    try{
      auto values=split(line);if(values.size()!=names.size())throw std::invalid_argument("列数が一致しません");
      auto value=[&](const std::string&name){return values.at(columns.at(name));};
      const auto direction=value("direction"),frame=value("frame"),mcs=value("mcs");
      if(direction!="P_B_TO_A"&&direction!="Q_A_TO_B")throw std::invalid_argument("方向はP_B_TO_AまたはQ_A_TO_Bです");
      const std::vector<std::string> frames{"ALL","CONTROL","BEACON","REQUEST","RESPONSE","ACK","DATA","DATA_ACK","DISASSOCIATE"};
      if(std::find(frames.begin(),frames.end(),frame)==frames.end())throw std::invalid_argument("frameが不正です");
      if(mcs!="*"&&(integer(mcs)>255||std::to_string(integer(mcs))!=mcs))throw std::invalid_argument("mcsが不正です");
      double rssi=number(value("rssi_dbm")),error=number(value(config_.metric));
      if(error<0||error>1)throw std::invalid_argument("BER/PERは0〜1です（百分率は不可）");
      unsigned length=value("reference_bytes").empty()?0:integer(value("reference_bytes"));
      if(config_.metric=="per"&&!length)throw std::invalid_argument("実測PERにはreference_bytesが必要です");
      auto &curve=curves_[key(direction=="Q_A_TO_B",frame,mcs)];
      if(!curve.points.empty()&&curve.reference_bytes!=length)throw std::invalid_argument("同一曲線のreference_bytesが混在しています");
      curve.reference_bytes=length;curve.points.push_back({rssi,error});
    }catch(const std::exception&e){throw std::invalid_argument("実測CSV "+std::to_string(row)+"行目: "+e.what());}
  }
  if(curves_.empty())throw std::invalid_argument("実測CSVに測定点がありません");
  for(auto &[name,curve]:curves_){
    std::sort(curve.points.begin(),curve.points.end(),[](auto a,auto b){return a.rssi<b.rssi;});
    if(curve.points.size()<2)throw std::invalid_argument("曲線には2点以上必要です: "+name);
    for(size_t i=1;i<curve.points.size();++i)if(curve.points[i].rssi==curve.points[i-1].rssi)throw std::invalid_argument("同一曲線の受信電力が重複しています: "+name);
  }
}
unsigned EmpiricalPhy::Mcs(double rate)const{
  for(auto [value,mcs]:config_.mcs_by_rate)if(std::abs(value-rate)<1e-8)return mcs;
  throw std::invalid_argument("MCStable.csvの速度に対応する実測MCS番号がありません: "+std::to_string(rate));
}
EmpiricalDecision EmpiricalPhy::Evaluate(bool ab,const std::string&frame,unsigned mcs,unsigned bytes,double rssi)const{
  if(!bytes||!std::isfinite(rssi))throw std::invalid_argument("実測モデルの受信電力/長さが不正です");
  std::vector<std::string> frames{frame};if(frame!="DATA")frames.push_back("CONTROL");frames.push_back("ALL");
  const Curve *curve=nullptr;EmpiricalDecision d;
  for(const auto&f:frames){for(const auto&m:{std::to_string(mcs),std::string("*")}){
    auto name=key(ab,f,m);auto it=curves_.find(name);if(it!=curves_.end()){curve=&it->second;d.curve=name;break;}}
    if(curve)break;
  }
  if(!curve)throw std::invalid_argument("受信曲線がありません: "+key(ab,frame,std::to_string(mcs)));
  if(config_.metric=="per"&&config_.per_length_policy=="exact"&&bytes!=curve->reference_bytes)
    throw std::invalid_argument("実測PERの長さ不一致: "+d.curve+" 実測="+std::to_string(curve->reference_bytes)+"B 対象="+std::to_string(bytes)+"B。対応する測定点を用意するかindependent近似を明示してください");
  d.lookup_dbm=rssi+(ab?config_.offset_ab_db:config_.offset_ba_db);
  const auto &p=curve->points;
  if(d.lookup_dbm<p.front().rssi){d.below_range=true;d.per=1;return d;}
  double error=p.back().value;
  if(d.lookup_dbm<p.back().rssi){auto hi=std::upper_bound(p.begin(),p.end(),d.lookup_dbm,[](double x,auto point){return x<point.rssi;});auto lo=hi-1;
    double f=(d.lookup_dbm-lo->rssi)/(hi->rssi-lo->rssi);
    // 正のBERは対数補間。ゼロを含む区間とPERは線形補間。
    error=config_.metric=="ber"&&lo->value>0&&hi->value>0?
      std::exp(std::log(lo->value)*(1-f)+std::log(hi->value)*f):lo->value*(1-f)+hi->value*f;
  }
  if(config_.metric=="ber"){d.ber=error;d.per=PacketError(error,uint64_t(bytes)*8);}
  else d.per=error==1?1:-std::expm1(std::log1p(-error)*double(bytes)/curve->reference_bytes);
  return d;
}
}
