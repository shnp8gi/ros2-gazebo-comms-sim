#include "comms_sim_pkg/drone_link.hpp"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <filesystem>
#include <yaml-cpp/yaml.h>

int main(int argc,char**argv){
 try{
  if(argc!=4)throw std::invalid_argument("使い方: tjx_theory_run config.yaml output_dir seed（理論/実測PERモデル）");
  auto c=comms_sim::ScenarioConfig::Load(argv[1]);
  const bool theory=c.protocol.hrcp.theoretical.enabled;
  if(theory==c.protocol.hrcp.empirical.enabled)throw std::invalid_argument("理論モデルまたは実測モデルの一方を有効にしてください");
  const auto seed=std::stoull(argv[3]);
  if(theory)c.protocol.hrcp.theoretical.seed=seed;else c.protocol.hrcp.empirical.seed=seed;
  c.protocol.hrcp.seed=seed^0x12345678;
  c.protocol.hrcp.monitor_enabled=false;
  auto path=std::filesystem::path(argv[2]);if(std::filesystem::exists(path))throw std::invalid_argument("結果ディレクトリが既に存在します");
  std::filesystem::create_directories(path);
  auto saved=YAML::LoadFile(argv[1]);saved[theory?"theoretical_phy":"empirical_phy"]["seed"]=seed;
  saved["awgn"]["seed"]=c.protocol.hrcp.seed;saved["hrcp"]["monitor_enabled"]=false;
  if(theory)saved["theoretical_phy"]["table"]=c.protocol.hrcp.theoretical.table;
  else {saved["empirical_phy"]["table"]=c.protocol.hrcp.empirical.table;saved["mcs_table"]=c.mcs_table;}
  saved["antenna"]["e_plane"]=c.e_plane;saved["antenna"]["h_plane"]=c.h_plane;saved["output_dir"]=path.string();
  std::ofstream(path/"config.yaml")<<saved;
  std::ofstream series(path/"timeseries.csv"),events(path/"events.csv"),hist(path/"mcs_statistics.csv");
  if(!series||!events||!hist)throw std::runtime_error("結果ファイルを開けません");
  series<<std::setprecision(17)<<"time_s,rssi_ab_dbm,rssi_ba_dbm,snr_ab_db,snr_ba_db,total_data_MiB,connected,gain_a_db,gain_b_db,off_axis_a_deg,off_axis_b_deg,distance_m\n";
  events<<std::setprecision(17)<<"time_s,event,frame,direction,detail\n";
  struct Counts{uint64_t n=0,errors=0,outside=0;double per=0,duration=0;};std::map<std::pair<std::string,int>,Counts> counts;
  unsigned initial_timeouts=0;bool first_connection_seen=false;double first_request=-1;
  comms_sim::RadioModel radio(c);
  comms_sim::LinkProtocol link(c.protocol,[&](double t){return radio.Evaluate(t);},[&](const comms_sim::LinkEvent&e){
    if(!first_connection_seen&&e.type=="TIMEOUT")++initial_timeouts;
    if(e.type=="CONNECTED")first_connection_seen=true;
    if(e.type=="TX"&&e.frame=="REQUEST"&&first_request<0)first_request=e.time;
    if(e.type=="FRAME_RESULT"){auto&v=counts[{e.frame,e.mcs}];++v.n;v.errors+=!e.detail.empty();v.outside+=e.lut_outside_range;v.per+=e.per;v.duration+=e.tx_samples/3.52e9;}
    else if(e.type=="CONNECTED"||e.type=="DISCONNECTED"||e.type=="TIMEOUT")events<<e.time<<','<<e.type<<','<<e.frame<<','<<e.direction<<','<<e.detail<<'\n';
  });
  double active=0,first=-1,last=-1,previous_time=0,previous_mb=0,peak=-1e300;
  for(uint64_t n=0;;++n){double t=std::min(n*c.log_interval_s,c.duration_s);link.Advance(t);
    const auto&s=link.Stats();if(s.delivered_mb>previous_mb){active+=t-previous_time;if(first<0)first=previous_time;last=t;}
    previous_time=t;previous_mb=s.delivered_mb;auto r=radio.Evaluate(t);peak=std::max({peak,r.rssi_ab,r.rssi_ba});
    series<<t<<','<<r.rssi_ab<<','<<r.rssi_ba<<','<<r.budget_ab.snr_db<<','<<r.budget_ba.snr_db<<','<<s.delivered_mb*1e6/1048576.<<','<<link.Connected()
      <<','<<r.gain_a<<','<<r.gain_b<<','<<r.off_axis_a_deg<<','<<r.off_axis_b_deg<<','<<r.distance<<'\n';
    if(t>=c.duration_s)break;
  }
  hist<<std::setprecision(17)<<"frame,mcs,frames,failed_frames,lut_outside_frames,mean_predicted_per,mean_tx_duration_s\n";
  for(const auto &[key,v]:counts)hist<<key.first<<','<<key.second<<','<<v.n<<','<<v.errors<<','<<v.outside<<','<<v.per/v.n<<','<<v.duration/v.n<<'\n';
  const auto&s=link.Stats();YAML::Node out;
  out["model"]=theory?"tjx_ldpc_awgn_v1":"empirical_packet_v1";out["seed"]=seed;
  out["total_data_MiB"]=s.delivered_mb*1e6/1048576.;out["connected_time_s"]=s.connected_s;
  out["active_transfer_bins_s"]=active;out["transfer_bin_span_s"]=first<0?0:last-first;
  out["first_connected_s"]=s.first_connected_s;out["first_transfer_bin_start_s"]=first;out["last_transfer_bin_end_s"]=last;
  out["transfer_bin_width_s"]=c.log_interval_s;out["duration_s"]=c.duration_s;
  out["mean_goodput_Mbps"]=s.delivered_mb*8/c.duration_s;out["peak_received_dbm_sampled"]=peak;
  out["connections"]=s.connections;out["timeouts"]=s.timeouts;out["disconnections"]=s.disconnections;
  out["timeouts_before_first_connection"]=initial_timeouts;out["connected_once"]=first_connection_seen;
  out["first_request_s"]=first_request;
  out["initial_connection_observation_s"]=first_connection_seen?s.first_connected_s:c.duration_s;
  out["data_direction"]=c.protocol.data_from_prc?"B_TO_A":"A_TO_B";
  std::ofstream summary(path/"summary.yaml");summary<<out;summary.close();
  if(!summary)throw std::runtime_error("集計書込み失敗");
  std::cout<<out<<'\n';
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
}
