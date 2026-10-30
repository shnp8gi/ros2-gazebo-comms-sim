// Gazeboと同じRadioModel/LinkProtocolを使う初回リンク確立の反復計算器。
// 軌道・MAC・受信判定は変更せず、描画と大量のフレームログだけ省く。
#include "comms_sim_pkg/drone_link.hpp"
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

int main(int argc,char **argv) {
  try {
    if(argc!=3)throw std::invalid_argument("使用法: timeout_boundary_sweep manifest.yaml results.csv");
    if(std::filesystem::exists(argv[2]))throw std::invalid_argument("既存結果は上書きしません");
    auto manifest=YAML::LoadFile(argv[1]);
    std::ofstream out(argv[2]);
    if(!out)throw std::runtime_error("結果CSVを作成できません");
    out<<std::setprecision(17);
    out<<"id,model,scenario,antenna_mode,speed_mps,seed,timeouts,connected,first_connected_s,first_request_s,first_request_rssi_dbm,attempts_before_connection,observation_limit_s,stopped_s,wall_seconds\n";
    for(const auto &job:manifest["runs"]) {
      auto c=comms_sim::ScenarioConfig::Load(job["config"].as<std::string>());
      const double speed=job["speed_mps"].as<double>();
      const auto seed=job["seed"].as<uint64_t>();
      c.a.speed_kmh=c.b.speed_kmh=speed*3.6;
      c.duration_s=50/speed;c.encounter_s=25/speed;
      c.scenario=job["scenario"].as<std::string>();
      c.antenna_mode=job["antenna_mode"].as<std::string>();
      c.protocol.hrcp.empirical.seed=seed;
      c.protocol.hrcp.monitor_enabled=false;c.protocol.hrcp.monitor_path.clear();
      c.Validate();
      comms_sim::RadioModel radio(c);
      unsigned timeouts=0,attempts=0;
      double connected=-1,request=-1,request_rssi=-999,stopped=0;
      const auto start=std::chrono::steady_clock::now();
      comms_sim::LinkProtocol link(c.protocol,[&](double t){return radio.Evaluate(t);},
        [&](const comms_sim::LinkEvent &e){
          if(connected>=0)return;
          if(e.type=="TIMEOUT")++timeouts;
          if(e.type=="CONNECTED")connected=e.time;
          if(e.type=="TX"&&e.frame=="REQUEST") {
            ++attempts;
            if(request<0){request=e.time;request_rssi=e.rssi;}
          }
        });
      // Advanceは内部イベント時刻を保つ。接続後の最大10msは集計しない。
      for(uint64_t step=0;connected<0;++step) {
        stopped=std::min(step*c.log_interval_s,c.duration_s);
        link.Advance(stopped);
        if(stopped>=c.duration_s)break;
      }
      const double wall=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
      out<<job["id"].as<std::string>()<<','<<job["model"].as<std::string>()<<','
         <<c.scenario<<','<<c.antenna_mode<<','<<speed<<','<<seed<<','<<timeouts<<','
         <<(connected>=0)<<','<<connected<<','<<request<<','<<request_rssi<<','<<attempts<<','
         <<c.duration_s<<','<<stopped<<','<<wall<<'\n';
      out.flush();
      if(!out)throw std::runtime_error("結果CSVの書込みに失敗しました");
    }
  }catch(const std::exception &e){std::cerr<<"境界計算に失敗: "<<e.what()<<'\n';return 1;}
}
