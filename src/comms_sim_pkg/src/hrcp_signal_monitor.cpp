#include "comms_sim_pkg/hrcp_signal_monitor.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>
namespace comms_sim {
HrcpSignalMonitor::HrcpSignalMonitor(std::string path,uint64_t interval,uint64_t window,bool monitor_only,bool noise_enabled):rssi_interval_(interval),noise_enabled_(noise_enabled),path_(std::move(path)){
  if(monitor_only){replay_.open(std::filesystem::path(path_).parent_path()/"monitor_replay.jsonl");if(!replay_)throw std::runtime_error("モニター再生記録を開けません");replay_<<std::setprecision(17);}
  if(window){
    meter_=std::make_unique<HrcpRssiMeter>(window);
    if(!monitor_only){
    rssi_csv_.open(std::filesystem::path(path_).parent_path()/(std::filesystem::path(path_).stem().string()+"_rssi.csv"));
    if(!rssi_csv_)throw std::runtime_error("波形RSSIのCSVを開けません");
    rssi_csv_<<"start_sample,end_sample,a_valid_chips,b_valid_chips,a_power_w,b_power_w,a_rssi_dbm,b_rssi_dbm\n"<<std::setprecision(17);
    }
  }
}
void HrcpSignalMonitor::ObserveReceivePower(uint64_t sample,const std::array<double,2>&watts,const std::array<bool,2>&available){
  if(meter_&&meter_->Push(sample,watts,available))RecordMeasuredRssi();
}
void HrcpSignalMonitor::RecordMeasuredRssi(){
  const auto&r=meter_->Latest();
  const bool save=r.end-1>=next_rssi_;
  PushRssi(r.end-1,r.dbm[0],r.dbm[1]);
  if(save&&rssi_csv_.is_open()){
    rssi_csv_<<r.start<<','<<r.end<<','<<r.count[0]<<','<<r.count[1]<<','<<r.watts[0]<<','<<r.watts[1];
    for(auto v:r.dbm){rssi_csv_<<',';if(std::isfinite(v))rssi_csv_<<v;}rssi_csv_<<'\n';
  }
}
void HrcpSignalMonitor::FinishRssi(){if(meter_&&meter_->Flush())RecordMeasuredRssi();}
void HrcpSignalMonitor::ObserveStage(uint64_t sample,const std::string&stage){
  if(stage_history_.empty()||stage_history_.back().second!=stage){
    stage_history_.emplace_back(sample,stage);if(stage_history_.size()>10000)stage_history_.pop_front();
    if(replay_.is_open())replay_<<"[\"stage\","<<sample<<",\""<<stage<<"\"]\n";
  }
}
void HrcpSignalMonitor::SaveSetup(uint64_t sample){
  auto code=[](const Reception&r){return r.known?(r.ok?1:-1):0;};
  SetupState s{sample,code(setup_ab_),code(setup_ba_),setup_connected_};
  if(!setup_history_.empty()){const auto&old=setup_history_.back();
    if(old.ab==s.ab&&old.ba==s.ba&&old.connected==s.connected)return;}
  setup_history_.push_back(s);if(setup_history_.size()>10000)setup_history_.pop_front();
  if(replay_.is_open())replay_<<"[\"setup\","<<sample<<','<<s.ab<<','<<s.ba<<','<<s.connected<<"]\n";
}
void HrcpSignalMonitor::SetupResult(uint64_t sample,bool ab,bool ok,const std::string&frame,const std::string&reason){
  if(setup_connected_)return;
  (ab?setup_ab_:setup_ba_)={true,ok,sample,frame,reason};SaveSetup(sample);
}
void HrcpSignalMonitor::SetupConnected(uint64_t sample,bool connected){
  if(connected){setup_connected_=true;SaveSetup(sample);}
  else ResetSetup(sample);
}
void HrcpSignalMonitor::ResetSetup(uint64_t sample){
  setup_connected_=false;setup_ab_={};setup_ba_={};SaveSetup(sample);
}
void HrcpSignalMonitor::PushRssi(uint64_t sample,double a,double b){
  latest_rssi_={sample,a,b};
  if(sample>=next_rssi_){rssi_.push_back(latest_rssi_);next_rssi_=sample+rssi_interval_;if(rssi_.size()>10000)rssi_.pop_front();
    if(replay_.is_open()){replay_<<"[\"rssi\","<<sample<<',';if(std::isfinite(a))replay_<<a;else replay_<<"null";replay_<<',';if(std::isfinite(b))replay_<<b;else replay_<<"null";
      if(meter_){replay_<<','<<meter_->Latest().count[0]<<','<<meter_->Latest().count[1];}
      replay_<<"]\n";}
  }
}
void HrcpSignalMonitor::MarkFailure(uint64_t start,uint64_t end,bool ab,double rssi,
                                  const std::string&frame,const std::string&reason){
  failures_.push_back({start,end,ab,rssi,frame,reason});if(failures_.size()>10000)failures_.pop_front();
}
void HrcpSignalMonitor::Push(uint64_t sample,const Values&values,bool rx_a,bool rx_b){
  sample_=sample;raw_.push_back({sample,values});if(raw_.size()>1024)raw_.pop_front();
  bool a=values[0]*values[0]+values[1]*values[1]>0,b=values[4]*values[4]+values[5]*values[5]>0;
  if(active_a_&&!a)last_a_=raw_;
  if(active_b_&&!b)last_b_=raw_;
  active_a_=a;active_b_=b;
  for(unsigned i=0;i<4;++i)sum_[i]+=values[2*i]*values[2*i]+values[2*i+1]*values[2*i+1];
  sum_[4]+=rx_a;sum_[5]+=rx_b;boundary_=false;
  if(++count_==352){ // 100nsごとの平均電力、最新204.8us分。
    for(auto &v:sum_)v/=count_;
    power_.push_back({sample-175,sum_});if(power_.size()>2048)power_.pop_front();
    sum_.fill(0);count_=0;boundary_=true;
  }
}
bool HrcpSignalMonitor::Due()const{
  return std::chrono::steady_clock::now()-published_>=std::chrono::milliseconds(200);
}
void HrcpSignalMonitor::Publish(const Status&s,bool complete){
  auto quote=[](const std::string&v){std::string out="\"";for(char c:v){if(c=='"'||c=='\\')out+='\\';if(c=='\n'){out+="\\n";continue;}out+=c;}return out+'"';};
  const auto tmp=path_+".tmp";std::ofstream out(tmp);if(!out)throw std::runtime_error("信号モニターの保存先を開けません");
  auto number=[&](double v){if(std::isfinite(v))out<<v;else out<<"null";};
  out<<std::setprecision(10)<<"{\"schema\":\"hrcp_signals_v1\",\"sample_rate_hz\":3520000000,\"sample\":"<<sample_
     <<",\"complete\":"<<(complete?"true":"false")<<",\"protocol_stage\":"<<quote(s.protocol_stage)<<",\"state_a\":"<<quote(s.state_a)<<",\"state_b\":"<<quote(s.state_b)
     <<",\"tx_a\":"<<quote(s.tx_a)<<",\"rx_a\":"<<quote(s.rx_a)<<",\"tx_b\":"<<quote(s.tx_b)<<",\"rx_b\":"<<quote(s.rx_b)
     <<",\"rssi_source\":"<<quote(meter_?"rx_iq_matched_filter":"unavailable")
     <<",\"predicted_signal_a_dbm\":"<<s.rssi_a_dbm<<",\"predicted_signal_b_dbm\":"<<s.rssi_b_dbm
     <<",\"channel_sample\":"<<s.channel_sample;
  out<<",\"rssi_a_dbm\":";number(meter_?meter_->Latest().dbm[0]:NAN);
  out<<",\"rssi_b_dbm\":";number(meter_?meter_->Latest().dbm[1]:NAN);
  out<<",\"rssi_window_samples\":"<<(meter_?meter_->Window():0)
     <<",\"rssi_bandwidth_hz\":1760000000,\"rssi_includes_noise\":"<<(noise_enabled_?"true":"false")
     <<",\"delivered_mb\":"<<s.delivered_mb<<",\"connections\":"<<s.connections<<",\"timeouts\":"<<s.timeouts<<",\"disconnections\":"<<s.disconnections;
  if(meter_){const auto&r=meter_->Latest();out<<",\"rssi_window_start\":"<<r.start<<",\"rssi_window_end\":"<<r.end
     <<",\"rssi_valid_chips\":["<<r.count[0]<<','<<r.count[1]<<']';}
  auto reception=[&](const char*key,const Reception&r){
    out<<",\""<<key<<"\":{\"known\":"<<(r.known?"true":"false")<<",\"ok\":"<<(r.ok?"true":"false")
       <<",\"sample\":"<<r.sample<<",\"frame\":"<<quote(r.frame)<<",\"reason\":"<<quote(r.reason)<<'}';};
  reception("reception_ab",s.ab);reception("reception_ba",s.ba);
  reception("setup_ab",setup_ab_);reception("setup_ba",setup_ba_);
  out<<",\"admission\":{\"enabled\":"<<(s.admission_enabled?"true":"false")<<",\"known\":"<<(s.admission_known?"true":"false")<<",\"ok\":"<<(s.admission_ok?"true":"false")
     <<",\"sample\":"<<s.admission_sample<<",\"measured_dbm\":"<<s.admission_measured_dbm
     <<",\"expected_dbm\":"<<s.admission_expected_dbm<<"},\"power\":[";
  bool first=true;for(const auto&p:power_){if(!first)out<<',';first=false;out<<'['<<p.sample;for(auto v:p.values)out<<','<<v;out<<']';}
  auto write_raw=[&](const char*key,const std::deque<Raw>&rows){out<<"],\""<<key<<"\":[";bool first_row=true;for(const auto&r:rows){if(!first_row)out<<',';first_row=false;out<<'['<<r.sample;for(auto v:r.values)out<<','<<v;out<<']';}};
  write_raw("iq",raw_);write_raw("iq_last_a",last_a_);write_raw("iq_last_b",last_b_);
  out<<"],\"rssi_history\":[";
  if(!meter_)out<<"],\"predicted_signal_history\":[";
  first=true;
  auto rssi_row=[&](const Rssi&r){if(!first)out<<',';first=false;out<<'['<<r.sample<<',';number(r.a_dbm);out<<',';number(r.b_dbm);out<<']';};
  for(const auto&r:rssi_)rssi_row(r);
  if(!rssi_.empty()&&latest_rssi_.sample>rssi_.back().sample)rssi_row(latest_rssi_);
  out<<"],\"failures\":[";first=true;
  for(const auto&f:failures_){if(!first)out<<',';first=false;
    out<<'['<<f.start<<','<<f.end<<','<<(f.ab?1:0)<<','<<f.rssi_dbm<<','<<quote(f.frame)<<','<<quote(f.reason)<<']';}
  out<<"],\"setup_history\":[";first=true;
  for(const auto&r:setup_history_){if(!first)out<<',';first=false;
    out<<'['<<r.sample<<','<<r.ab<<','<<r.ba<<','<<(r.connected?1:0)<<']';}
  out<<"],\"stage_history\":[";first=true;
  for(const auto&r:stage_history_){if(!first)out<<',';first=false;out<<'['<<r.first<<','<<quote(r.second)<<']';}
  out<<"]}\n";
  out.close();if(!out)throw std::runtime_error("信号モニターの書き込みに失敗しました");
  if(rssi_csv_.is_open()){rssi_csv_.flush();if(!rssi_csv_)throw std::runtime_error("波形RSSIのCSV書き込みに失敗しました");}
  if(replay_.is_open()){
    if(last_replay_status_!=sample_||complete){
      replay_<<"[\"status\","<<sample_<<','<<quote(s.state_a)<<','<<quote(s.state_b)<<','<<quote(s.protocol_stage)<<','<<s.delivered_mb<<','<<s.connections<<','<<s.timeouts<<','<<s.disconnections<<','<<(complete?1:0)
        <<",{\"tx_a\":"<<quote(s.tx_a)<<",\"tx_b\":"<<quote(s.tx_b)<<",\"rx_a\":"<<quote(s.rx_a)<<",\"rx_b\":"<<quote(s.rx_b)
        <<",\"admission\":{\"enabled\":"<<(s.admission_enabled?"true":"false")<<",\"known\":"<<(s.admission_known?"true":"false")<<",\"ok\":"<<(s.admission_ok?"true":"false")
        <<",\"sample\":"<<s.admission_sample<<",\"measured_dbm\":"<<s.admission_measured_dbm<<",\"expected_dbm\":"<<s.admission_expected_dbm<<"}}]\n";
      last_replay_status_=sample_;
    }
    replay_.flush();if(!replay_)throw std::runtime_error("モニター再生記録に失敗しました");
  }
  std::filesystem::rename(tmp,path_);published_=std::chrono::steady_clock::now();
}
}
