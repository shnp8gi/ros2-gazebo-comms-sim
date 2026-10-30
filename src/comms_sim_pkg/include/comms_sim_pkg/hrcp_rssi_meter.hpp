#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace comms_sim {
// 整合フィルタの位相0出力を全点積算。単位Wで平均してからdBmへ変換する。
// 受信できない自機送信中は積算対象外。無送信中の雑音は積算する。
class HrcpRssiMeter {
 public:
  struct Reading {
    uint64_t start=0,end=0;
    std::array<uint64_t,2> count{};
    std::array<double,2> watts{};
    std::array<double,2> dbm{{NAN,NAN}};
  };
  explicit HrcpRssiMeter(uint64_t window=3520):window_(window),end_(window){
    if(window<2||window%2)throw std::invalid_argument("RSSI窓は2以上の偶数sampleで指定してください");
  }
  bool Push(uint64_t sample,const std::array<double,2>&watts,const std::array<bool,2>&available){
    if(sample!=next_sample_)throw std::logic_error("RSSI波形sampleの欠落・重複があります");
    next_sample_+=2;
    for(unsigned ep=0;ep<2;++ep)if(available[ep]){
      if(!std::isfinite(watts[ep])||watts[ep]<0)throw std::invalid_argument("RSSI入力電力が不正です");
      sum_[ep]+=watts[ep];++count_[ep];
    }
    if(sample+1==end_){Finish(end_);return true;}return false;
  }
  bool Flush(){
    const uint64_t end=next_sample_-1;
    if(end<=start_)return false;
    Finish(end);return true;
  }
  const Reading& Latest()const{return last_;}
  uint64_t Window()const{return window_;}
 private:
  void Finish(uint64_t end){
    last_={};last_.start=start_;last_.end=end;last_.count=count_;
    for(unsigned ep=0;ep<2;++ep){
      last_.watts[ep]=count_[ep]?sum_[ep]/count_[ep]:0;
      last_.dbm[ep]=count_[ep]&&last_.watts[ep]>0?10*std::log10(last_.watts[ep])+30:NAN;
    }
    sum_={};count_={};start_=end;end_=end+window_;
  }
  uint64_t window_,start_=0,end_,next_sample_=1;
  std::array<double,2> sum_{};
  std::array<uint64_t,2> count_{};
  Reading last_{};
};
}
