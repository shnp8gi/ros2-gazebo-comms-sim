#include "comms_sim_pkg/drone_link.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

int main(int argc,char**argv){try{
  if(argc!=3)throw std::invalid_argument("使い方: radio_profile config.yaml new_output.csv");
  if(std::filesystem::exists(argv[2]))throw std::invalid_argument("既存の結果は上書きしません");
  const auto c=comms_sim::ScenarioConfig::Load(argv[1]);comms_sim::RadioModel radio(c);
  std::ofstream out(argv[2]);if(!out)throw std::runtime_error("出力先を開けません");
  out<<std::setprecision(17)<<"time_s,remaining_path_m,distance_m,p_dbm,q_dbm,p_snr_db,q_snr_db,gain_a_dbi,gain_b_dbi,off_axis_a_deg,off_axis_b_deg\n";
  for(uint64_t i=0;;++i){double t=std::min(i*c.log_interval_s,c.duration_s);auto r=radio.Evaluate(t);
    out<<t<<','<<(c.encounter_s-t)*c.a.speed_kmh/3.6<<','<<r.distance<<','<<r.rssi_ba<<','<<r.rssi_ab
      <<','<<r.budget_ba.snr_db<<','<<r.budget_ab.snr_db<<','<<r.gain_a<<','<<r.gain_b<<','<<r.off_axis_a_deg<<','<<r.off_axis_b_deg<<'\n';
    if(t>=c.duration_s)break;}
  if(!out)throw std::runtime_error("RF時系列の書込みに失敗しました");
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
