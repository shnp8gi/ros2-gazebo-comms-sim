#include "comms_sim_pkg/drone_link.hpp"
#include "comms_sim_pkg/hrcp_mac.hpp"
#include <iostream>
#include <stdexcept>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
using namespace comms_sim;
void require(bool x,const char*m){if(!x)throw std::runtime_error(m);}
int main(){
 HrcpDeadline timer;timer.StartOnce(10,1760000000);timer.StartOnce(1000,1760000000);require(timer.start==10&&timer.deadline==1760000010,"retry extended ATP");require(!timer.Expired(1760000009)&&timer.Expired(1760000010),"ATP boundary");timer.Refresh(2000,1760000000);require(timer.deadline==1760002000,"maintenance refresh");
 HrcpBeacon beacon;auto encoded=hrcp_beacon_encode(beacon);HrcpBeacon decoded;require(hrcp_beacon_decode(encoded,decoded)&&decoded.slot_tenths_us==105&&decoded.period_us==140,"beacon wire");encoded[8]^=1;require(!hrcp_beacon_decode(encoded,decoded),"beacon FCS");
 HrcpConfig c;std::vector<LinkEvent> events;
 auto channel=[](double){LinkSample s;s.distance=10;s.rssi_ab=-50;s.rssi_ba=-60;s.budget_ab.snr_db=s.budget_ba.snr_db=25;s.budget_ba.isotropic_rx_dbm=-70;s.budget_ba.input_referred_noise_dbm=-80;s.budget_ba.rx_gain_dbi=23;s.budget_ba.receiver_noise_dbm=-85;s.budget_ab.receiver_noise_dbm=-75;return s;};
 auto baseline=c;baseline.fast_events=false;
 HrcpPairnet link(baseline,channel,[&](const auto&e){events.push_back(e);});link.Advance(.0004);

 require(link.Connected(),"association");require(link.Stats().delivered_mb>0,"data delivery");require(link.Stats().timeouts==0,"unexpected timeout");
 // 雑音スイッチOFFでも波形・同期・LDPC・CRC・MACを通る。
 auto quiet_config=c;quiet_config.noise_enabled=false;
 HrcpPairnet noiseless_link(quiet_config,channel,{});noiseless_link.Advance(.0004);
 require(noiseless_link.Connected()&&noiseless_link.Stats().delivered_mb>0&&noiseless_link.Stats().awgn_errors==0,"noiseless waveform PHY");
 HrcpPairnet quiet_chunks(quiet_config,channel,{});for(unsigned i=1;i<=8;++i)quiet_chunks.Advance(i*.00005);
 require(noiseless_link.Stats().delivered_mb==quiet_chunks.Stats().delivered_mb&&noiseless_link.Stats().awgn_trials==quiet_chunks.Stats().awgn_trials,"noiseless chunk invariance");

 // PRC(B)からDATAを送りPRDEV(A)で受信。ACK・MCS更新・分割実行も同じ方向で動作する。
 auto prc_config=c;prc_config.data_from_prc=true;std::vector<LinkEvent> prc_events;
 HrcpPairnet prc_data(prc_config,channel,[&](const auto&e){prc_events.push_back(e);});prc_data.Advance(.0004);
 require(prc_data.Connected()&&prc_data.Stats().delivered_mb>0&&prc_data.Stats().data_frames>0,"PRC payload/ACK delivery");
 bool saw_prc_data=false,saw_device_ack=false,saw_adaptation=false;
 for(const auto&e:prc_events)if(e.type=="TX"){
   if(e.frame=="DATA"){require(e.direction=="B_TO_A","PRC DATA direction");saw_prc_data=true;saw_adaptation|=e.mcs>0;}
   if(e.frame=="DATA_ACK"&&e.direction=="A_TO_B")saw_device_ack=true;
   if(e.frame=="BEACON"||e.frame=="RESPONSE")require(e.direction=="B_TO_A","PRC setup direction");
   if(e.frame=="REQUEST"||e.frame=="ACK")require(e.direction=="A_TO_B","PRDEV setup direction");
 }
 require(saw_prc_data&&saw_device_ack&&saw_adaptation,"PRC DATA/ACK/MCS not exercised");
 HrcpPairnet prc_chunks(prc_config,channel,{});for(unsigned i=1;i<=8;++i)prc_chunks.Advance(i*.00005);
 require(prc_chunks.Stats().delivered_mb==prc_data.Stats().delivered_mb&&prc_chunks.Stats().awgn_errors==prc_data.Stats().awgn_errors,"PRC chunk-dependent result");
 std::vector<LinkEvent> prc_recovery_events;
 auto prc_loss_config=prc_config;prc_loss_config.channel_sample_s=1e-7;
 HrcpPairnet prc_loss(prc_loss_config,[&](double t){auto s=channel(t);if(t>=.00016&&t<.00018)s.budget_ba.snr_db=-50;return s;},[&](const auto&e){prc_recovery_events.push_back(e);});prc_loss.Advance(.0004);
 require(prc_loss.Connected()&&prc_loss.Stats().delivered_mb>0&&prc_loss.Stats().failed_frames>0&&prc_loss.Stats().timeouts==0,"PRC data loss recovery");
 bool pending_recovery=false,saw_prc_recovery=false;
 for(const auto&e:prc_recovery_events){
   if(e.type=="RIFS_RETRY"&&e.direction=="B_TO_A")pending_recovery=true;
   else if(pending_recovery&&e.type=="TX"&&e.direction=="B_TO_A"){
     require(e.frame=="DATA_ACK"&&e.tx_samples==9104,"PRC recovery carried payload");pending_recovery=false;saw_prc_recovery=true;
   }
 }
 require(saw_prc_recovery,"PRC recovery not exercised");

 HrcpPairnet chunked(c,channel,{});for(unsigned i=1;i<=8;++i)chunked.Advance(i*.00005);
 require(chunked.Stats().delivered_mb==link.Stats().delivered_mb&&chunked.Stats().awgn_errors==link.Stats().awgn_errors,"chunk-dependent result");
#ifdef _OPENMP
 auto parallel_config=c;parallel_config.noise_threads=4;parallel_config.decode_threads=4;std::vector<LinkEvent> parallel_events;
 HrcpPairnet parallel_noise(parallel_config,channel,[&](const auto&e){parallel_events.push_back(e);});
 for(unsigned i=1;i<=8;++i)parallel_noise.Advance(i*.00005);
 require(parallel_events.size()==events.size(),"parallel noise/event skipping changed event count");
 for(size_t i=0;i<events.size();++i){const auto&a=events[i];const auto&b=parallel_events[i];
   require(a.time==b.time&&a.type==b.type&&a.frame==b.frame&&a.detail==b.detail&&a.tx_sample==b.tx_sample&&a.rx_sample==b.rx_sample&&a.sync_metric==b.sync_metric,"parallel noise/event skipping changed event");}
#endif
 // 観測のON/OFFで通信結果が変わらないことを確認。
 auto monitored_config=c;monitored_config.monitor_enabled=true;
 monitored_config.monitor_waveforms=true;
 monitored_config.monitor_path="/tmp/hrcp-signal-monitor-"+std::to_string(getpid())+".json";
 {
   HrcpPairnet monitored(monitored_config,channel,{});monitored.Advance(.0004);
   require(monitored.Stats().delivered_mb==link.Stats().delivered_mb&&monitored.Stats().awgn_errors==link.Stats().awgn_errors,"monitor changed radio result");
 }
 auto snapshot=YAML::LoadFile(monitored_config.monitor_path);
 require(snapshot["complete"].as<bool>()&&snapshot["iq"].size()==1024&&snapshot["power"].size()==2048,"signal monitor snapshot");
 require(snapshot["rssi_source"].as<std::string>()=="rx_iq_matched_filter"&&snapshot["rssi_window_samples"].as<unsigned>()==3520,"monitor RSSI source/window");
 require(snapshot["predicted_signal_a_dbm"].as<double>()==-60&&snapshot["predicted_signal_b_dbm"].as<double>()==-50,"prediction separated from measured RSSI");
 require(snapshot["rssi_history"].size()>1&&snapshot["reception_ab"]["ok"].as<bool>()&&snapshot["reception_ba"]["ok"].as<bool>(),"monitor RSSI history and reception");
 // 未送信を失敗扱いにせず、Beacon復号成功と要求許可条件未達を区別する。
 auto rejected_config=monitored_config;rejected_config.expected_rssi_dbm=30;
 {
   HrcpPairnet rejected(rejected_config,channel,{});rejected.Advance(.0001);
 }
 auto rejected_snapshot=YAML::LoadFile(rejected_config.monitor_path);
 require(!rejected_snapshot["reception_ab"]["known"].as<bool>()&&rejected_snapshot["reception_ba"]["ok"].as<bool>(),"monitor unknown direction / decoded Beacon");
 // Beacon間の無信号区間は、信号予測値ではなく実雑音波形の電力になる。
 double noise_a=0,noise_b=0;unsigned quiet=0;
 for(const auto&r:rejected_snapshot["rssi_history"])if(r[0].as<uint64_t>()>3520000000ULL/20000){
   noise_a+=std::pow(10.,r[1].as<double>()/10.);noise_b+=std::pow(10.,r[2].as<double>()/10.);++quiet;
 }
 require(quiet>10,"noise-only RSSI not observed");
 const double bandwidth_offset=10*std::log10(1.76e9/c.bandwidth_ba_hz);
 require(std::abs(10*std::log10(noise_a/quiet)-(-85+bandwidth_offset))<.2&&std::abs(10*std::log10(noise_b/quiet)-(-75+bandwidth_offset))<.2,"waveform RSSI noise calibration");
 require(rejected_snapshot["admission"]["known"].as<bool>()&&!rejected_snapshot["admission"]["ok"].as<bool>()&&rejected_snapshot["failures"][0][5].as<std::string>()=="EXPECTED_RSSI_REJECT","monitor admission rejection");
 // 実機に合わせた無効化では、高いExpected RSSIでも正常BeaconからRequestへ進む。
 auto bypass_config=rejected_config;bypass_config.expected_rssi_admission_enabled=false;
 std::vector<LinkEvent> bypass_events;
 {
   HrcpPairnet bypass(bypass_config,channel,[&](const auto&e){bypass_events.push_back(e);});bypass.Advance(.0004);
   require(bypass.Connected()&&bypass.Stats().delivered_mb>0,"期待RSSI無効時に接続・配送できません");
 }
 auto bypass_snapshot=YAML::LoadFile(bypass_config.monitor_path);
 require(!bypass_snapshot["admission"]["enabled"].as<bool>()&&!bypass_snapshot["admission"]["known"].as<bool>(),"無効な期待RSSIを判定済みと表示しています");
 bool bypass_request=false;
 for(const auto&e:bypass_events){
   require(e.type!="ADMISSION_REJECT","無効な期待RSSIで拒否しています");
   bypass_request|=e.type=="TX"&&e.frame=="REQUEST";
 }
 require(bypass_request,"正常BeaconからRequestを送信していません");
 // 無効化してもPHYの失敗は迂回しない。Beaconを復号できなければRequestを送らない。
 unsigned invalid_beacon_requests=0;
 HrcpPairnet invalid_beacon(bypass_config,[&](double t){auto s=channel(t);s.budget_ba.snr_db=-50;return s;},[&](const auto&e){
   invalid_beacon_requests+=e.type=="TX"&&e.frame=="REQUEST";
 });invalid_beacon.Advance(.0001);
 require(!invalid_beacon.Connected()&&invalid_beacon_requests==0,"Beacon復号失敗でもRequestを送信しています");
 // 初回Requestを妨害。次Beaconで再送しても最初の500ms期限は同じ。
 c.channel_sample_s=1e-7;std::vector<LinkEvent> retry_events;
 auto retry_config=c;retry_config.monitor_enabled=true;retry_config.monitor_path=monitored_config.monitor_path;
 retry_config.expected_rssi_admission_enabled=false; // 今回の設定でもP=1/Q=0と再送を検証。
 unsigned retry_attempts=0;
 {
 HrcpPairnet retry(retry_config,[&](double t){auto s=channel(t);if(t<.0001)s.budget_ab.snr_db=-50;return s;},[&](const auto&e){retry_events.push_back(e);});retry.Advance(.0005);
 require(retry.Connected(),"request retry did not recover");
 retry_attempts=retry.Stats().attempts;
 }
 auto retry_snapshot=YAML::LoadFile(retry_config.monitor_path);
 bool saw_failed_request=false;
 for(const auto&f:retry_snapshot["failures"])if(f[2].as<unsigned>()==1&&f[4].as<std::string>()=="REQUEST")saw_failed_request=true;
 require(saw_failed_request,"monitor missed undetected Request");
 bool unilateral=false,setup_closed=false;
 for(const auto&r:retry_snapshot["setup_history"]){
   unilateral|=r[1].as<int>()==-1&&r[2].as<int>()==1&&!r[3].as<int>();
   setup_closed|=r[3].as<int>()==1;
 }
 require(unilateral&&setup_closed,"monitor unilateral setup interval / connection closure");
 unsigned requests=0,beacons=0;for(auto&e:retry_events)if(e.type=="TX"){requests+=e.frame=="REQUEST";beacons+=e.frame=="BEACON";}
 require(requests>=2&&beacons>=2&&retry_attempts==1,"request retry state/index");
 // 初回Responseだけを妨害し、RIFSによるResponse再送で接続する。
 std::vector<LinkEvent> recovery_events;
 HrcpPairnet recovery(c,[&](double t){auto s=channel(t);if(t>=.00014&&t<.000145)s.budget_ba.snr_db=-50;return s;},[&](const auto&e){recovery_events.push_back(e);});recovery.Advance(.0004);
 require(recovery.Connected(),"response RIFS recovery");
 unsigned responses=0;for(auto&e:recovery_events)if(e.type=="TX"&&e.frame=="RESPONSE")++responses;
 require(responses>=2&&recovery.Stats().connections==1,"response retry count");
 // 接続後のDATA誤りを500ms停止へ変換しない。
 {
 HrcpPairnet data_loss(retry_config,[&](double t){auto s=channel(t);if(t>=.00016&&t<.00018)s.budget_ab.snr_db=-50;return s;},{});data_loss.Advance(.0004);
 require(data_loss.Connected()&&data_loss.Stats().delivered_mb>0&&data_loss.Stats().timeouts==0,"data recovery");
 require(data_loss.Stats().failed_frames>0,"data impairment not exercised");
 }
 auto data_snapshot=YAML::LoadFile(retry_config.monitor_path);
 require(data_snapshot["failures"].size()==0,"post-connection data failures entered setup highlight");
 for(const auto&r:data_snapshot["setup_history"])require(r[1].as<int>()>=0&&r[2].as<int>()>=0,"data failure affected setup status");

 // DATAのACKを受け損なったAのRIFS回復に本文を入れない。
 std::vector<LinkEvent> header_recovery_events;
 HrcpPairnet header_recovery(c,[&](double t){auto s=channel(t);if(t>=.000168&&t<.000195)s.budget_ba.snr_db=-50;return s;},[&](const auto&e){header_recovery_events.push_back(e);});header_recovery.Advance(.0004);
 bool check_next=false,saw_header=false;
 for(const auto&e:header_recovery_events){
   if(e.type=="RIFS_RETRY"&&e.direction=="A_TO_B")check_next=true;
   else if(check_next&&e.type=="TX"&&e.direction=="A_TO_B"){
     require(e.frame=="DATA_ACK"&&e.tx_samples==9104,"RIFS recovery carried payload");saw_header=true;check_next=false;
   }
 }
 require(saw_header&&header_recovery.Stats().delivered_mb>0,"header-only recovery missing");
 // 試験だけATPを1msへ短縮して満了のFSMを検証。初期設定500msの値は上の整数境界で検査。
 c.atp_ms=1;std::vector<LinkEvent> expired_events;
 HrcpPairnet expiry(c,[&](double t){auto s=channel(t);s.budget_ab.snr_db=-50;return s;},[&](const auto&e){expired_events.push_back(e);});expiry.Advance(.0012);
 require(expiry.Stats().timeouts==1,"setup expiry count");
 double first_request=-1;bool checked=false;
 for(auto&e:expired_events){if(e.type=="TX"&&e.frame=="REQUEST"&&first_request<0)first_request=e.time+double(e.tx_samples)/3.52e9;
   if(e.type=="TIMEOUT"){require(std::abs(e.time-first_request-.001)<1e-10,"retry extended stage deadline");checked=true;}}
 require(checked,"no expiry event");
 c.requested_atp_ms=2; // Bは1msを確定し、Aは要求2msをそのまま使わない。
 HrcpPairnet disconnect(c,[&](double t){auto s=channel(t);if(t>.00018)s.budget_ab.snr_db=s.budget_ba.snr_db=-50;return s;},{});disconnect.Advance(.0014);
 require(disconnect.Stats().connections==1&&disconnect.Stats().disconnections==1&&disconnect.Stats().timeouts==0,"disconnect counted as setup timeout");

 std::vector<LinkEvent> stale_events;
 HrcpPairnet stale(c,[&](double t){auto s=channel(t);if(t>.00018&&t<.0012)s.budget_ab.snr_db=-50;return s;},[&](const auto&e){stale_events.push_back(e);});stale.Advance(.0015);
 bool disassociated=false;for(auto&e:stale_events)disassociated|=e.type=="TX"&&e.frame=="DISASSOCIATE";
 require(disassociated,"expired peer was not sent Disassociation Request");
 std::cout<<"HRCP pairnet tests PASS\n";
}
