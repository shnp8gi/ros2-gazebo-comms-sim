// 任意の開始位置・速度で、既存通信コアの初回リンク確立までを反復計算。
#include "comms_sim_pkg/drone_link.hpp"
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace {
// 初回接続後のDATA計算を省くための、この計算器だけで使う終了通知。
// LinkProtocolの状態を再利用せず破棄するので、通信本体には変更を加えない。
struct InitialConnectionComplete {};
}

int main(int argc,char **argv) {
  try {
    if(argc!=3)throw std::invalid_argument("使用法: initial_connection_sweep manifest.yaml 新規結果.csv");
    if(std::filesystem::exists(argv[2]))throw std::invalid_argument("既存結果は上書きしません");
    auto manifest=YAML::LoadFile(argv[1]);std::ofstream out(argv[2]);
    if(!out)throw std::runtime_error("結果CSVを開けません");
    out<<std::setprecision(17)<<"id,model,scenario,antenna_mode,speed_mps,start_path_m,start_distance_m,end_path_m,seed,timeouts,connected,first_connected_s,first_request_s,initial_p_dbm,initial_q_dbm,observation_duration_s,wall_seconds\n";
    for(const auto &job:manifest["runs"]) {
      auto c=comms_sim::ScenarioConfig::Load(job["config"].as<std::string>());
      const double speed=job["speed_mps"].as<double>(),path=job["start_path_m"].as<double>();
      const double end_path=job["end_path_m"]?job["end_path_m"].as<double>():path;
      const auto seed=job["seed"].as<uint64_t>();
      c.a.speed_kmh=c.b.speed_kmh=speed*3.6;
      c.encounter_s=path/speed;c.duration_s=(path+end_path)/speed;
      c.scenario=job["scenario"].as<std::string>();c.antenna_mode=job["antenna_mode"].as<std::string>();
      c.protocol.hrcp.empirical.seed=seed;c.protocol.hrcp.monitor_enabled=false;c.protocol.hrcp.monitor_path.clear();
      c.Validate();comms_sim::RadioModel radio(c);auto initial=radio.Evaluate(0);
      unsigned timeouts=0;double connected=-1,request=-1;
      const auto start=std::chrono::steady_clock::now();
      comms_sim::LinkProtocol link(c.protocol,[&](double t){return radio.Evaluate(t);},
        [&](const comms_sim::LinkEvent&e){
          if(connected>=0)return;
          if(e.type=="TIMEOUT")++timeouts;
          if(e.type=="CONNECTED"){connected=e.time;throw InitialConnectionComplete{};}
          if(e.type=="TX"&&e.frame=="REQUEST"&&request<0)request=e.time;
        });
      try {
        for(uint64_t step=0;connected<0;++step){
          double t=std::min(step*c.log_interval_s,c.duration_s);link.Advance(t);if(t>=c.duration_s)break;
        }
      } catch(const InitialConnectionComplete &) {
        // 接続時刻とそれ以前のTimeoutは記録済み。次のseedは新しいコアで計算。
      }
      double wall=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
      out<<job["id"].as<std::string>()<<','<<job["model"].as<std::string>()<<','<<c.scenario<<','<<c.antenna_mode<<','
         <<speed<<','<<path<<','<<initial.distance<<','<<end_path<<','<<seed<<','<<timeouts<<','<<(connected>=0)<<','<<connected<<','
         <<request<<','<<initial.rssi_ba<<','<<initial.rssi_ab<<','<<c.duration_s<<','<<wall<<'\n';out.flush();
      if(!out)throw std::runtime_error("結果の書込みに失敗しました");
    }
  }catch(const std::exception&e){std::cerr<<"初回接続の計算に失敗: "<<e.what()<<'\n';return 1;}
}
