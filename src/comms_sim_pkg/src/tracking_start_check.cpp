// 保存した入力を変更せず、開始状態と初回接続を通信コアで点検する。
#include "comms_sim_pkg/drone_link.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

int main(int argc,char **argv) {
  try {
    if(argc!=2)throw std::invalid_argument("使用法: tracking_start_check 入力.yaml");
    auto c=comms_sim::ScenarioConfig::Load(argv[1]);
    c.protocol.hrcp.monitor_enabled=false;c.protocol.hrcp.monitor_path.clear();
    comms_sim::RadioModel radio(c);auto initial=radio.Evaluate(0);
    double p_per=initial.ba_ok?0:1,q_per=initial.ab_ok?0:1;
    if(c.protocol.hrcp.empirical.enabled) {
      comms_sim::EmpiricalPhy phy(c.protocol.hrcp.empirical);
      p_per=phy.Evaluate(false,"BEACON",3,36,initial.rssi_ba).per;
      q_per=phy.Evaluate(true,"REQUEST",3,27,initial.rssi_ab).per;
    }
    auto input=YAML::LoadFile(argv[1]);
    double q_check_s=-1,q_check_dbm=-999;
    if(input["tracking_start"]&&input["tracking_start"]["q_threshold_lead_s"]) {
      q_check_s=input["tracking_start"]["q_threshold_lead_s"].as<double>();
      q_check_dbm=radio.Evaluate(q_check_s).rssi_ab;
      if(initial.ab_ok||q_per!=1||std::abs(q_check_dbm-c.ab.sensitivity_dbm)>1e-7||
         radio.Evaluate(q_check_s-.001).rssi_ab>=c.ab.sensitivity_dbm||
         radio.Evaluate(q_check_s+.001).rssi_ab<=c.ab.sensitivity_dbm)
        throw std::runtime_error("Qのしきい値到達時刻が指定した開始条件と一致しません");
    }else if(initial.ba_ok||initial.ab_ok||p_per!=1||q_per!=1)
      throw std::runtime_error("開始時点で受信可能です。通信圏外開始になっていません");
    unsigned timeouts=0;double connected=-1,beacon=-1,request=-1;
    comms_sim::LinkProtocol link(c.protocol,[&](double t){return radio.Evaluate(t);},
      [&](const comms_sim::LinkEvent &e){
        if(connected>=0)return;
        if(e.type=="TIMEOUT")++timeouts;
        if(e.type=="CONNECTED")connected=e.time;
        if(e.type=="RX_OK"&&e.frame=="BEACON"&&beacon<0)beacon=e.time;
        if(e.type=="TX"&&e.frame=="REQUEST"&&request<0)request=e.time;
      });
    for(uint64_t step=0;connected<0;++step) {
      const double t=std::min(step*c.log_interval_s,c.duration_s);
      link.Advance(t);if(t>=c.duration_s)break;
    }
    auto distance=[&](double t){return t>=0?radio.Evaluate(t).distance:-1;};
    std::cout<<std::setprecision(17)
      <<"model,scenario,speed_mps,start_distance_m,start_p_dbm,start_q_dbm,start_beacon_per,start_request_per,first_beacon_s,first_beacon_distance_m,first_request_s,first_connected_s,first_connected_distance_m,timeouts_before_first_connection,q_threshold_check_s,q_threshold_check_dbm\n"
      <<(c.protocol.hrcp.empirical.enabled?"statistical":"threshold")<<','<<c.scenario<<','<<c.a.speed_kmh/3.6<<','
      <<initial.distance<<','<<initial.rssi_ba<<','<<initial.rssi_ab<<','<<p_per<<','<<q_per<<','
      <<beacon<<','<<distance(beacon)<<','<<request<<','<<connected<<','<<distance(connected)<<','<<timeouts<<','<<q_check_s<<','<<q_check_dbm<<'\n';
  }catch(const std::exception&e){std::cerr<<"開始条件の確認に失敗: "<<e.what()<<'\n';return 1;}
}
