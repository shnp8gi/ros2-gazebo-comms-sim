#include "comms_sim_pkg/empirical_phy.hpp"
#include "comms_sim_pkg/drone_link.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <tuple>
#include <unistd.h>
using namespace comms_sim;
void check(bool value,const char *message){if(!value)throw std::runtime_error(message);}
void near(double a,double b){check(std::abs(a-b)<1e-10,"期待する数値と一致しません");}
template<class F>void rejects(F action){bool rejected=false;try{action();}catch(const std::exception&){rejected=true;}check(rejected,"不正な入力が受理されました");}
int main(){try{
  auto path=std::filesystem::temp_directory_path()/("empirical-tests-"+std::to_string(getpid())+".csv");
  auto write=[&](const std::string&body){std::ofstream out(path);out<<"direction,frame,mcs,rssi_dbm,ber,per,reference_bytes\n"<<body;};
  write("P_B_TO_A,ALL,*,-80,0.1,0.8,100\nP_B_TO_A,ALL,*,-60,0.001,0.2,100\nQ_A_TO_B,ALL,*,-80,0.2,0.9,100\nQ_A_TO_B,ALL,*,-60,0.002,0.3,100\n");
  EmpiricalConfig c;c.enabled=true;c.table=path;c.metric="ber";
  EmpiricalPhy ber(c);near(ber.Evaluate(false,"BEACON",0,100,-70).ber,.01);
  near(ber.Evaluate(true,"REQUEST",0,100,-70).ber,.02);
  near(ber.Evaluate(false,"DATA",11,100,-70).per,-std::expm1(800*std::log1p(-.01)));
  near(EmpiricalPhy::PacketError(0,800),0);near(EmpiricalPhy::PacketError(1,800),1);
  near(EmpiricalPhy::PacketError(1e-15,800),8e-13);
  near(ber.Evaluate(false,"BEACON",0,100,-81).per,1);
  near(ber.Evaluate(false,"BEACON",0,100,-50).ber,.001);
  c.offset_ba_db=10;EmpiricalPhy shifted(c);near(shifted.Evaluate(false,"BEACON",0,100,-80).ber,.01);c.offset_ba_db=0;
  check(ber.Mcs(6)==11&&ber.Mcs(.5)==0,"MCSテーブルの対応が不一致です");rejects([&]{ber.Mcs(7);});
  c.metric="per";EmpiricalPhy per(c);near(per.Evaluate(false,"DATA",11,100,-70).per,.5);
  rejects([&]{per.Evaluate(false,"DATA",11,200,-70);});
  c.per_length_policy="independent";EmpiricalPhy scaled(c);near(scaled.Evaluate(false,"DATA",11,200,-70).per,.75);
  rejects([&]{scaled.Evaluate(false,"DATA",11,0,-70);});
  write("P_B_TO_A,ALL,*,-80,0.1,,0\nP_B_TO_A,ALL,*,-80,0.2,,0\n");c.metric="ber";rejects([&]{EmpiricalPhy invalid(c);});
  write("P_B_TO_A,ALL,*,-80,1.1,,0\nP_B_TO_A,ALL,*,-60,0,,0\n");rejects([&]{EmpiricalPhy invalid(c);});
  // packet backendと波形backendで同じMACの接続/再送/ATP処理を使用する。
  auto flat=[&](double error){std::ofstream out(path);out<<"direction,frame,mcs,rssi_dbm,ber,per,reference_bytes\n";
    for(const auto&direction:{"P_B_TO_A","Q_A_TO_B"})for(auto rssi:{-70,-40})out<<direction<<",ALL,*,"<<rssi<<','<<error<<",,0\n";};
  flat(0);
  HrcpConfig h;h.enabled=true;h.noise_enabled=false;h.expected_rssi_admission_enabled=false;
  h.empirical=c;h.data_from_prc=true;h.atp_ms=500;h.setup_timeout_ms=500;h.channel_sample_s=.001;
  auto channel=[](double){LinkSample s;s.rssi_ab=s.rssi_ba=-50;s.rate_ab_gbps=.5;s.rate_ba_gbps=6;
    // NF/SNRは未定義でも統計受信器は動作する。
    s.budget_ab.snr_db=s.budget_ba.snr_db=std::numeric_limits<double>::quiet_NaN();return s;};
  std::vector<LinkEvent> events;
  HrcpPairnet link(h,channel,[&](const auto&e){events.push_back(e);});link.Advance(.001);
  check(link.Connected()&&link.Stats().delivered_mb>0,"正常な曲線で接続・転送できません");
  check(link.Stats().failed_frames==0,"良好なチャネルで不要な受信失敗があります");
  bool data=false;for(const auto&e:events)if(e.type=="FRAME_RESULT"){
    check(e.per==0&&e.random_draw>=0&&e.random_draw<1&&e.processed_samples==0,"確率判定の診断情報が不正です");
    if(e.frame=="DATA"){data=true;check(e.mcs==11&&e.frame_bits==2048*8,"DATA方向のMCS/bit数が不一致です");}
  }check(data,"DATAがありません");
  // Pのみ受信→Q回復→接続。Beacon/Requestをまとめた接続抽選にしない。
  unsigned lost_requests=0,received_beacons=0;
  HrcpPairnet retry(h,[&](double t){auto s=channel(t);if(t<.001)s.rssi_ab=-90;return s;},[&](const auto&e){
    lost_requests+=e.type=="FRAME_RESULT"&&e.frame=="REQUEST"&&!e.detail.empty();
    received_beacons+=e.type=="RX_OK"&&e.frame=="BEACON";
  });retry.Advance(.002);check(retry.Connected()&&lost_requests>1&&received_beacons>1,"P1/Q0からの再送・接続が不正です");
  HrcpPairnet no_beacon(h,[&](double t){auto s=channel(t);s.rssi_ba=-90;return s;},{});
  no_beacon.Advance(.002);check(no_beacon.Stats().attempts==0,"Beacon未受信でRequestを送っています");
  std::vector<LinkEvent> expired;
  HrcpPairnet disconnect(h,[&](double t){auto s=channel(t);if(t>.002)s.rssi_ab=s.rssi_ba=-90;return s;},[&](const auto&e){expired.push_back(e);});
  disconnect.Advance(.503);check(disconnect.Stats().connections==1&&disconnect.Stats().disconnections==1,"500msの自然な切断がありません");
  bool checked=false;for(const auto&e:expired)if(e.type=="ATP_DISCONNECT"){
    near(e.time-e.wait_start_s,.5);checked=true;
  }check(checked,"ATP満了記録がありません");
  auto short_setup=h;short_setup.setup_timeout_ms=1;
  bool setup_expired=false;
  HrcpPairnet setup_timer(short_setup,[&](double t){auto s=channel(t);s.rssi_ab=-90;return s;},[&](const auto&e){
    if(e.type=="TIMEOUT"){near(e.time-e.wait_start_s,.001);setup_expired=true;}
  });setup_timer.Advance(.0012);check(setup_expired,"接続待ちと接続後ATPを分離できていません");
  // 確率試行はチャンク・観測の回数から独立。同じseedは同じイベント列。
  flat(.00001);std::vector<LinkEvent> first,second,third;
  HrcpPairnet full(h,channel,[&](const auto&e){first.push_back(e);});full.Advance(.003);
  HrcpPairnet chunks(h,channel,[&](const auto&e){second.push_back(e);});for(unsigned i=1;i<=30;++i)chunks.Advance(i*.0001);
  check(first.size()==second.size(),"Advance分割でイベント数が変化しました");
  for(size_t i=0;i<first.size();++i){const auto&a=first[i];const auto&b=second[i];
    check(std::tie(a.time,a.type,a.frame,a.detail,a.random_draw)==std::tie(b.time,b.type,b.frame,b.detail,b.random_draw),"Advance分割で受信判定が変化しました");}
  ++h.empirical.seed;HrcpPairnet another(h,channel,[&](const auto&e){third.push_back(e);});another.Advance(.003);
  check(full.Stats().failed_frames>0,"確率的な失敗を検証できていません");
  auto draw=[](const auto&v){for(const auto&e:v)if(e.type=="FRAME_RESULT")return e.random_draw;return -1.;};
  check(draw(first)!=draw(third),"seed変更が反映されません");
  // 実測PER経路ではbitごとの復号を実行せず、指定PERでpacket単位に抽選する。
  write("P_B_TO_A,ALL,*,-70,,0.2,2048\nP_B_TO_A,ALL,*,-40,,0.2,2048\nQ_A_TO_B,ALL,*,-70,,0.2,2048\nQ_A_TO_B,ALL,*,-40,,0.2,2048\n");
  h.empirical.metric="per";h.empirical.per_length_policy="independent";
  unsigned data_trials=0,data_errors=0;
  HrcpPairnet per_link(h,channel,[&](const auto&e){if(e.type=="FRAME_RESULT"&&e.frame=="DATA"){
    near(e.per,.2);check(std::isnan(e.ber),"PER入力から架空のBERを出しています");
    ++data_trials;data_errors+=!e.detail.empty();
  }});per_link.Advance(.02);
  check(per_link.Connected()&&data_trials>500&&std::abs(double(data_errors)/data_trials-.2)<.05,"実測PERの試行率が不一致です");
  write("P_B_TO_A,ALL,*,-70,,0.2,2048\nP_B_TO_A,ALL,*,-40,,0.2,2048\n");
  rejects([&]{HrcpPairnet missing(h,channel,{});});
  std::filesystem::remove(path);
  std::cout<<"実測曲線・確率判定・共通MAC・500ms切断・seed再現性テスト成功\n";
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
