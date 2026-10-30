// 通信本体と同じ詳細PHY/MACを、既知の減衰障害で通して確認する実行器。
// 航路の通信範囲やGazebo自体の検証ではない。ATPは500msのまま。
#include "comms_sim_pkg/drone_link.hpp"
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace comms_sim;
namespace {
constexpr double recovery_s=.001, outage_s=.003, duration_s=.505;
constexpr double attenuation_db=100, hz=3.52e9;
void require(bool value,const char*message){if(!value)throw std::runtime_error(message);}
}
int main(int argc,char**argv){
 try{
  if(argc!=3)throw std::invalid_argument("使用法: hrcp_protocol_check 入力YAML 新規出力ディレクトリ");
  const auto source=std::filesystem::absolute(argv[1]);
  const auto output=std::filesystem::absolute(argv[2]);
  require(!std::filesystem::exists(output),"出力先が既に存在します。新しい名前を指定してください");
  auto c=ScenarioConfig::Load(source.string());
  c.scenario="parallel";c.antenna_mode="fixed";c.a.speed_kmh=c.b.speed_kmh=3.6;
  c.duration_s=duration_s;c.encounter_s=duration_s/2;c.output_dir=output.string();
  c.preview_mode=false;c.monitor_only=true;c.protocol.data_from_prc=true;
  auto &h=c.protocol.hrcp;
  h.enabled=true;h.data_from_prc=true;h.noise_enabled=true;h.expected_rssi_admission_enabled=false;
  h.atp_ms=h.requested_atp_ms=500;h.monitor_enabled=h.monitor_only=true;h.monitor_waveforms=false;
  h.channel_sample_s=1e-5;h.monitor_rssi_interval_s=1e-4;h.noise_threads=h.decode_threads=1;
  h.monitor_path=(output/"live_signals.json").string();
  std::filesystem::create_directories(output);
  auto y=YAML::LoadFile(source.string());
  y["scenario"]=c.scenario;y["antenna_mode"]=c.antenna_mode;
  y["drone_a"]["speed_kmh"]=y["drone_b"]["speed_kmh"]=3.6;
  y["duration_s"]=duration_s;y["encounter_time_s"]=c.encounter_s;
  y["output_dir"]=output.string();y["preview_mode"]=false;y["monitor_only"]=true;
  y["awgn"]["enabled"]=true;y["protocol"]["data_from_prc"]=true;
  y["hrcp"]["enabled"]=true;y["hrcp"]["expected_rssi_admission_enabled"]=false;
  y["hrcp"]["atp_ms"]=y["hrcp"]["requested_atp_ms"]=500;
  y["hrcp"]["monitor_enabled"]=true;y["hrcp"]["monitor_waveforms"]=false;
  y["hrcp"]["noise_threads"]=y["hrcp"]["decode_threads"]=1;
  y["hrcp"]["monitor_rssi_interval_s"]=h.monitor_rssi_interval_s;
  y["hrcp"]["monitor_rssi_window_s"]=h.monitor_rssi_window_s;
  y["hrcp"]["channel_sample_s"]=h.channel_sample_s;
  y["antenna"]["e_plane"]=c.e_plane;y["antenna"]["h_plane"]=c.h_plane;y["mcs_table"]=c.mcs_table;
  y["evaluation_scope"]="protocol_fault_injection";
  y["phy_error_model"]="hrcp_iq_continuous_ldpc_v1";
  y["protocol_revision"]="hrcp_independent_endpoints_v1";
  y["fault_injection"]["q_recovery_s"]=recovery_s;y["fault_injection"]["both_outage_s"]=outage_s;
  y["fault_injection"]["attenuation_db"]=attenuation_db;
  y["standard_exceptions"].push_back("expected_RSSI_admission_disabled_to_match_device");
  // 再生速度だけ変更する。計算時刻や試験中の500msを圧縮しない。
  for(auto segment: {std::pair<double,double>{.004,.0005},{.501,.1},{duration_s,.001}}){
    YAML::Node row;row["until_s"]=segment.first;row["speed"]=segment.second;y["guided_replay"].push_back(row);
  }
  {std::ofstream f(output/"config.yaml");f<<y;}
  RadioModel radio(c);
  std::ofstream events(output/"protocol_events.csv");events<<std::setprecision(17);
  events<<"time_s,event,frame,direction,reason,wait_start_s,deadline_s\n";
  bool beacon_ok=false,request_failed=false,data_ok=false;unsigned requests=0,expiries=0;
  double first_failure=-1,connected_at=-1,disconnected_at=-1;
  LinkStats stats;
  auto start=std::chrono::steady_clock::now();
  {
   HrcpPairnet link(h,[&](double t){
    auto s=radio.Evaluate(t);
    auto weaken=[&](bool ab){
      auto &b=ab?s.budget_ab:s.budget_ba;const auto&r=ab?c.ab:c.ba;
      b=calculate_power_budget(r,b.path_loss_db+attenuation_db,b.tx_gain_dbi,b.rx_gain_dbi);
      (ab?s.rssi_ab:s.rssi_ba)=b.receiver_input_dbm;
    };
    if(t<recovery_s||t>=outage_s)weaken(true);
    if(t>=outage_s)weaken(false);
    return s;
   },[&](const LinkEvent&e){
    events<<e.time<<','<<e.type<<','<<e.frame<<','<<e.direction<<','<<e.detail<<','<<e.wait_start_s<<','<<e.deadline_s<<'\n';
    if(e.type=="RX_OK"&&e.frame=="BEACON")beacon_ok=true;
    if(e.type=="TX"&&e.frame=="REQUEST")++requests;
    if(e.type=="FRAME_RESULT"&&e.frame=="REQUEST"&&!e.detail.empty()&&e.time<recovery_s){
      request_failed=true;if(first_failure<0)first_failure=e.time;
    }
    if(e.type=="RX_OK"&&e.frame=="DATA"&&e.direction=="B_TO_A")data_ok=true;
    if(e.type=="CONNECTED")connected_at=e.time;
    if(e.type=="DISCONNECTED")disconnected_at=e.time;
    if(e.type=="ATP_DISCONNECT"){
      ++expiries;
      require(std::abs(e.deadline_s-e.wait_start_s-.5)<2/hz,"ATPが500msではありません");
      require(std::abs(e.time-e.deadline_s)<2/hz,"期限と実際の切断時刻が一致しません");
    }
   });
   for(unsigned step=1;step<=505;++step){
    link.Advance(step*.001);
    if(step%10==0||step<=4||step==505){
      const double wall=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
      std::cout<<"進捗 "<<step<<" / 505 ms、実時間 "<<wall<<"秒、残り概算 "
               <<wall*(505.-step)/step<<"秒、接続="<<link.Connected()<<std::endl;
    }
   }
   stats=link.Stats();
   require(!link.Connected(),"試験終端で切断していません");
  }
  require(beacon_ok&&request_failed&&requests>=2,"Beacon成功・Request失敗・再送を確認できません");
  require(connected_at>=recovery_s&&connected_at<outage_s,"障害回復後に接続していません");
  require(data_ok&&stats.delivered_mb>0,"BからAへの正常データ転送を確認できません");
  require(stats.connections==1&&stats.disconnections==1&&stats.timeouts==0,"接続・切断の回数が想定外です");
  require(expiries==2&&disconnected_at>=.5,"両端の500ms切断を確認できません");
  YAML::Node result;result["validated"]=true;result["atp_ms"]=500;
  result["first_request_failure_s"]=first_failure;result["connected_s"]=connected_at;
  result["disconnected_s"]=disconnected_at;result["delivered_mb"]=stats.delivered_mb;
  result["request_transmissions"]=requests;result["endpoint_expiries"]=expiries;
  result["wall_time_s"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  {std::ofstream f(output/"verification.yaml");f<<result;}
  {std::ofstream f(output/"completed");f<<"completed\n";}
  std::cout<<"検証成功：P方向Beacon受信、Q方向Request失敗、再送、接続、配送、両端ATP500ms満了\n";
 }catch(const std::exception&e){std::cerr<<"通し試験失敗: "<<e.what()<<'\n';return 1;}
}
