#include "comms_sim_pkg/hrcp_pairnet.hpp"
#include "comms_sim_pkg/drone_link.hpp"
#include "comms_sim_pkg/hrcp_mac.hpp"
#include "comms_sim_pkg/hrcp_modem.hpp"
#include "comms_sim_pkg/hrcp_signal_monitor.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <random>
#include <stdexcept>
namespace comms_sim {
namespace {
constexpr double clock_hz=3.52e9;
constexpr uint64_t period=492800,sifs=8800,never=UINT64_MAX;
// 最短間隔を短縮しないようsample境界へ切上げ。B:8.01us、A:19.05us。
constexpr uint64_t rifs_a=(1905*176+4)/5,rifs_b=(801*176+4)/5;
uint64_t ticks(double seconds){if(!std::isfinite(seconds)||seconds<0||seconds*clock_hz>0x1p52)throw std::invalid_argument("HRCP時刻範囲外");return uint64_t(std::llround(seconds*clock_hz));}
uint64_t mix(uint64_t x){x+=0x9e3779b97f4a7c15ULL;x=(x^(x>>30))*0xbf58476d1ce4e5b9ULL;x=(x^(x>>27))*0x94d049bb133111ebULL;return x^(x>>31);}
// 絶対sampleと受信機を鍵にする。chunk分割、状態遷移、再送回数で雑音系列が変わらない。
HrcpComplex noise(uint64_t seed,uint64_t sample,unsigned receiver){
  uint64_t key=mix(seed^(uint64_t(receiver+1)<<60)^sample);
  double u=((mix(key)>>11)+.5)*0x1p-53,v=((mix(key^0xd1b54a32d192ed03ULL)>>11)+.5)*0x1p-53;
  double r=std::sqrt(-std::log(u)),angle=6.2831853071795864769*v;return {r*std::cos(angle),r*std::sin(angle)};
}
std::string name(const HrcpMac&m,const std::vector<uint8_t>&body){
  if(m.type==0)return "BEACON";
  if(!m.subframes)return m.type==3?"ACK":"DATA_ACK";
  if(m.type==4)return "DATA";
  auto b=hrcp_body_decode(m,body);return b.ok?(b.command==0?"REQUEST":b.command==1?"RESPONSE":b.command==2?"DISASSOCIATE":"COMMAND"):"COMMAND";
}
}
struct HrcpPairnet::Impl {
  enum class Action{None,Beacon,Request,Response,CommandAck,Data,DataAck,Disassociate};
  struct Endpoint {
    bool associated=false,started=false;
    std::string last_tx="NONE",last_rx="NONE";
    unsigned atp_ms=500,devid=0,expired_id=0,reject_destination=0,last_command=1023,last_data=1023,data_sequence=0,mcs=0,successes=0;
    uint64_t tx_end=0,send_at=never,recover_at=never,last_header_end=0,selected_beacon=0,data_generation=0;
    Action send=Action::None,recover=Action::None;
    HrcpDeadline timer;
  };
  struct Wave {bool ab=false;uint64_t start=0,arrival=0,end=0,index=0;std::vector<HrcpComplex> chips;
    bool reported=false,received=false,half_duplex=false,link_setup=false;
    std::string frame,reason="NO_SYNC_OR_HEADER";
    uint64_t bits=0;double minimum_snr=0;
    uint64_t tx_samples=0,header_end=0;
    bool packet_ok=false,header_sent=false,packet_sent=false;
    double sampled_rssi=0,draw=-1;
    EmpiricalDecision decision;
    HrcpReception packet;
    HrcpReception decoded;unsigned sequence=0,reference=0,mcs=0;
  };
  std::unique_ptr<HrcpSignalMonitor> monitor;
  std::unique_ptr<EmpiricalPhy> empirical;
  uint64_t packet_number=0;
  HrcpSignalMonitor::Reception observed_ab,observed_ba;
  bool admission_known=false,admission_ok=false;
  uint64_t admission_sample=0;
  double admission_measured=0,admission_expected=0;
  HrcpConfig c;Evaluate evaluate;Log log;HrcpCodec codec;HrcpModem modem;
  std::unique_ptr<HrcpStreamReceiver> receivers[2][2];
  Endpoint a,b;HrcpBeacon beacon;LinkStats stats;std::string state="SCANNING";
  std::deque<Wave> waves;
  struct PendingRx {unsigned endpoint,phase;HrcpReception value;};
  std::deque<PendingRx> pending_rx;
  HrcpComplex previous[2]{};
  uint64_t connected_samples=0,channel_sample=0;
  uint64_t next_event=0,next_monitor=0,noise_start=0,noise_end=0;
  std::vector<std::array<HrcpComplex,2>> noise_buffer;
  uint64_t now=0,next_beacon=0,beacon_index=0,next_channel=0,setup_start=0,last_full_rx[2]={never,never};
  double amp[2]={0,0},rx_watts_per_unit[2]={0,0};LinkSample channel;std::mt19937_64 rng;
  bool observed_connected=false;
  Impl(HrcpConfig config,Evaluate e,Log l):c(config),evaluate(std::move(e)),log(std::move(l)),codec(config.decode_threads),modem(codec),rng(c.seed){
    if(c.empirical.enabled){
      if(c.noise_enabled||c.expected_rssi_admission_enabled||c.monitor_waveforms)
        throw std::invalid_argument("実測モデルではAWGN/Expected RSSI/波形表示を無効にしてください");
      empirical=std::make_unique<EmpiricalPhy>(c.empirical);
    }
    if(c.setup_timeout_ms>65535)throw std::invalid_argument("setup_timeout_msは0〜65535です");
    if(c.noise_threads<1||c.noise_threads>64)throw std::invalid_argument("雑音生成スレッド数は1～64です");
#ifndef _OPENMP
    if(c.noise_threads>1)throw std::invalid_argument("このビルドはOpenMPに対応していません");
#endif
    if(c.noise_enabled&&c.noise_threads>1)noise_buffer.resize(65536);
    if(!c.data_bytes||c.data_bytes>2048||c.requested_atp_ms>65535||!c.atp_ms||c.atp_ms>65535||!std::isfinite(c.expected_rssi_dbm)||c.expected_rssi_dbm< -225||c.expected_rssi_dbm>30||!std::isfinite(c.channel_sample_s)||c.channel_sample_s<=0||c.bandwidth_ab_hz<=0||c.bandwidth_ba_hz<=0)throw std::invalid_argument("HRCP設定範囲外");
    if(c.monitor_enabled){if(c.monitor_path.empty())throw std::invalid_argument("信号モニター出力先が未指定です");monitor=std::make_unique<HrcpSignalMonitor>(c.monitor_path,ticks(c.monitor_rssi_interval_s),empirical?0:ticks(c.monitor_rssi_window_s),c.monitor_only||bool(empirical),c.noise_enabled);}
    a.atp_ms=c.requested_atp_ms?c.requested_atp_ms:c.atp_ms;b.atp_ms=c.atp_ms;
    // 無効時もBeaconのフィールド幅は維持。-225は未使用値で、閾値の引下げで代用しない。
    beacon.period_us=period/3520;beacon.atp_ms=c.atp_ms;beacon.expected_rssi_dbm=c.expected_rssi_admission_enabled?int(std::ceil(c.expected_rssi_dbm)):-225;NewId();
    if(!empirical)for(unsigned endpoint=0;endpoint<2;++endpoint)for(unsigned phase=0;phase<2;++phase)
      receivers[endpoint][phase]=std::make_unique<HrcpStreamReceiver>(codec,[this,endpoint,phase](const HrcpReception&r){if(r.header_event)Receive(endpoint,phase,r);else {pending_rx.push_back({endpoint,phase,r});next_event=std::min(next_event,now+1);}});
    if(empirical){
      // 必要な曲線・PER長さの不足は、長いシナリオを開始する前に検出する。
      auto check=[&](bool ab,const char*frame,unsigned bytes){empirical->Evaluate(ab,frame,c.empirical.control_mcs,bytes,-60);};
      check(false,"BEACON",hrcp_beacon_encode(beacon).size());
      check(true,"REQUEST",hrcp_command(0,0,hrcp_request_payload(c.atp_ms)).size());
      check(false,"RESPONSE",hrcp_command(0,1,hrcp_response_payload(1,c.atp_ms)).size());
      check(true,"ACK",17);check(true,"DATA_ACK",17);check(false,"DATA_ACK",17);
      check(false,"DISASSOCIATE",hrcp_command(1,2,{0}).size());
      for(auto [rate,mcs]:c.empirical.mcs_by_rate)empirical->Evaluate(!c.data_from_prc,"DATA",mcs,c.data_bytes,-60);
    }
    HrcpHeader h;for(const auto &body:{hrcp_beacon_encode(beacon),hrcp_command(0,0,hrcp_request_payload(c.atp_ms))}){
      h.body_bytes=body.size();if(double(HrcpCodec::FrameChips(h,true))/1.76e9>8e-6)throw std::invalid_argument("Beacon/Requestが8usを超えます");}
  }
  const char* ProtocolStage()const{
    return observed_connected?"DATA":a.associated?"ACK":
      now<a.tx_end&&a.last_tx=="REQUEST"?"REQUEST":a.started||b.started?"RESPONSE":
      a.send==Action::Request?"REQUEST":"BEACON";
  }
  HrcpSignalMonitor::Status MonitorStatus()const{
    auto state=[](const Endpoint&e,bool is_a){return e.associated?"ASSOCIATED":e.started?(is_a?"WAIT_RESPONSE":"WAIT_ACK"):"SCANNING";};
    HrcpSignalMonitor::Status status{state(a,true),state(b,false),a.last_tx,a.last_rx,b.last_tx,b.last_rx,
            channel.rssi_ba,channel.rssi_ab,channel_sample,observed_ab,observed_ba,
            admission_known,admission_ok,admission_sample,admission_measured,admission_expected,""};
    status.protocol_stage=ProtocolStage();
    status.delivered_mb=stats.delivered_mb;status.connections=stats.connections;status.timeouts=stats.timeouts;status.disconnections=stats.disconnections;
    status.admission_enabled=c.expected_rssi_admission_enabled;
    return status;
  }
  ~Impl(){if(monitor){try{monitor->FinishRssi();monitor->ObserveTime(now?now-1:0);monitor->Publish(MonitorStatus(),true);}catch(...){}}}
  uint64_t Atp(const Endpoint&e)const{return ticks(e.atp_ms*.001);}
  bool DataSender(bool is_a)const{return is_a!=c.data_from_prc;}
  void NewId(){unsigned old=beacon.next_id;do{beacon.next_id=1+rng()%236;}while(beacon.next_id==old);b.devid=beacon.next_id;}
  void Emit(const std::string&type,const std::string&detail,bool ab,const std::string&frame="",uint64_t index=0){
    if(!log)return;
    LinkEvent event{double(now)/clock_hz,type,frame,ab?"A_TO_B":"B_TO_A",detail};
    event.rssi=ab?channel.rssi_ab:channel.rssi_ba;event.minimum_snr_db=ab?channel.budget_ab.snr_db:channel.budget_ba.snr_db;
    event.beacon_index=index;event.tx_sample=now;event.rx_sample=now;
    const auto &endpoint=ab?a:b;
    if(endpoint.timer.start!=never){event.wait_start_s=double(endpoint.timer.start)/clock_hz;event.deadline_s=double(endpoint.timer.deadline)/clock_hz;}
    log(event);
  }
  void Schedule(Endpoint&e,Action action,uint64_t when){e.send=action;e.send_at=std::max(when,e.tx_end);}
  void UpdateObserver(){
    bool connected=a.associated&&b.associated;
    if(monitor&&connected!=observed_connected)monitor->SetupConnected(now,connected);
    if(connected&&!observed_connected){++stats.connections;stats.last_setup_ms=double(now-setup_start)/clock_hz*1000;if(stats.first_connected_s<0)stats.first_connected_s=double(now)/clock_hz;Emit("CONNECTED","両機のローカル接続が成立",false);}
    if(!connected&&observed_connected){++stats.disconnections;Emit("DISCONNECTED","接続済み機体のATP満了; setup timeoutとは別集計",false);}
    observed_connected=connected;
    state=connected?"CONNECTED":b.started?"WAIT_ACK":a.started?"WAIT_RESPONSE":"SCANNING";
  }
  void Expire(Endpoint&e,bool is_a){
    if(!e.timer.Expired(now))return;
    bool maintenance=e.associated;
    if(monitor)monitor->ResetSetup(now);
    Emit(maintenance?"ATP_DISCONNECT":"TIMEOUT",is_a?"A_ATP_EXPIRED":"B_ATP_EXPIRED",is_a);
    if(!maintenance)++stats.timeouts;
    e.associated=false;e.started=false;e.atp_ms=is_a?(c.requested_atp_ms?c.requested_atp_ms:c.atp_ms):c.atp_ms;e.timer.Clear();e.send_at=e.recover_at=never;e.last_command=e.last_data=1023;e.data_sequence=e.mcs=e.successes=0;
    if(!is_a){b.expired_id=b.devid;NewId();next_beacon=((now+period-1)/period)*period;}
    UpdateObserver();
  }
  void Send(bool is_a,Action action){
    Endpoint &e=is_a?a:b;if(now<e.tx_end)throw std::logic_error("送信が自己重複しました");
    HrcpMac mac;mac.source=is_a?a.devid:0;mac.destination=is_a?0:b.devid;
    mac.last_sequence=e.last_command;std::vector<uint8_t> body;bool setup=!e.associated;uint64_t index=0;
    if(action==Action::Beacon){mac.type=0;mac.destination=255;mac.last_sequence=1023;body=hrcp_beacon_encode(beacon);index=++beacon_index;setup=true;}
    else if(action==Action::Request){mac.type=3;mac.subframes=1;body=hrcp_command(0,0,hrcp_request_payload(a.atp_ms));index=a.selected_beacon;
      if(!a.started){a.started=true;setup_start=now;++stats.attempts;Emit("ATTEMPT","Request初回送信; 再送ではATPを延長しない",true,"REQUEST",index);}setup=true;}
    else if(action==Action::Response){mac.type=3;mac.subframes=1;mac.ack_policy=2;body=hrcp_command(0,1,hrcp_response_payload(b.devid,b.atp_ms));setup=true;}
    else if(action==Action::CommandAck){mac.type=3;mac.ack_policy=2;setup=false;}
    else if(action==Action::Data){mac.type=4;mac.ack_policy=2;mac.subframes=1;mac.last_type=1;mac.last_sequence=e.last_data;
      std::vector<uint8_t> payload(c.data_bytes);for(unsigned i=0;i<c.data_bytes;++i)payload[i]=mix(c.seed^(e.data_generation<<32)^i);body=hrcp_subframe(e.data_sequence,payload);}
    else if(action==Action::Disassociate){mac.type=3;mac.subframes=1;mac.destination=e.reject_destination;body=hrcp_command(1,2,{0});setup=true;}
    else if(action==Action::DataAck){mac.type=4;mac.ack_policy=2;mac.last_type=1;mac.last_sequence=e.last_data;}
    else return;
    const auto frame_channel=empirical?evaluate(double(now)/clock_hz):channel;
    double rate=0;unsigned actual_mcs=c.empirical.control_mcs;
    if(empirical&&action==Action::Data){rate=is_a?frame_channel.rate_ab_gbps:frame_channel.rate_ba_gbps;
      actual_mcs=empirical->Mcs(rate>0?rate:c.empirical.mcs_by_rate.begin()->first);}
    HrcpHeader header;header.mac=hrcp_mac_encode(mac);header.seed=rng()%16;header.mcs=action==Action::Data?e.mcs:0;
    Wave w;w.ab=is_a;w.start=now;w.index=index;header.body_bytes=body.size();
    w.arrival=now+uint64_t(std::ceil(frame_channel.distance/299792458.*clock_hz));
    if(empirical){
      // パケット単位の受信器。成功時だけ、同じMACへheaderとbodyを実時間順に渡す。
      // 波形は生成しない。RSSIは送信開始時の値をフレーム内で保持する近似。
      header.mcs=0; // 内部MACへ渡す形式。実機MCS番号はw.mcsとCSVに別途保存する。
      const uint64_t header_samples=2*((setup?5120:3328)+1216);
      w.tx_samples=action==Action::Data?header_samples+uint64_t(std::ceil(body.size()*8*clock_hz/((rate>0?rate:c.empirical.mcs_by_rate.begin()->first)*1e9)))+16:
        2*HrcpCodec::FrameChips(header,setup);
      w.header_end=w.arrival+header_samples;w.end=w.arrival+w.tx_samples;
      w.sampled_rssi=is_a?frame_channel.rssi_ab:frame_channel.rssi_ba;
      const unsigned model_bytes=action==Action::Data?c.data_bytes:body.empty()?17:body.size();
      w.bits=uint64_t(model_bytes)*8;
      w.decision=empirical->Evaluate(is_a,name(mac,body),actual_mcs,model_bytes,w.sampled_rssi);
      w.draw=(mix(c.empirical.seed^(++packet_number*0x9e3779b97f4a7c15ULL))>>11)*0x1p-53;
      w.packet_ok=w.draw>=w.decision.per;
      w.reason=w.decision.below_range?"EMPIRICAL_BELOW_RANGE":"EMPIRICAL_PACKET_ERROR";
      if(action==Action::Data&&rate<=0){w.packet_ok=false;w.reason="NO_PAYLOAD_MCS";}
      w.packet.header=header;w.packet.body=body;w.packet.header_ok=w.packet.body_ok=true;
      w.packet.sfd_end_chip=(w.arrival+128)/2;
      w.half_duplex=(is_a?b:a).tx_end>w.arrival;
      for(auto &old:waves)if(old.ab!=is_a&&now<old.end&&now+w.tx_samples>old.arrival)old.half_duplex=true;
    }else{
      w.chips=modem.Transmit(header,body,setup);w.tx_samples=2*w.chips.size();w.end=w.arrival+w.tx_samples;
      w.bits=w.chips.size()+HrcpCodec::BodyBlocks(header)*(header.pilot?56:64)*(HrcpCodec::BitsPerSymbol(header.mcs)-1);
    }
    w.minimum_snr=is_a?channel.budget_ab.snr_db:channel.budget_ba.snr_db;
    w.frame=name(mac,body);e.last_tx=w.frame;w.sequence=action==Action::Data?e.data_sequence:0;w.reference=mac.last_sequence;w.mcs=empirical?actual_mcs:header.mcs;
    w.link_setup=!observed_connected&&(action==Action::Beacon||action==Action::Request||action==Action::Response||action==Action::CommandAck);
    e.tx_end=now+w.tx_samples;e.send_at=never;e.send=Action::None;e.recover_at=never;
    if(action!=Action::Beacon&&action!=Action::Request&&action!=Action::Disassociate){e.recover=action;e.recover_at=e.tx_end+(is_a?rifs_a:rifs_b);}
    if(action==Action::Request||action==Action::Response)e.timer.StartOnce(e.tx_end,c.setup_timeout_ms?ticks(c.setup_timeout_ms*.001):Atp(e));
    if(action==Action::CommandAck&&e.timer.start==never)e.timer.Refresh(e.tx_end,Atp(e));
    if(!empirical){receivers[is_a?0:1][0]->Reset();receivers[is_a?0:1][1]->Reset();}
    if(log){LinkEvent event{double(now)/clock_hz,"TX",name(mac,body),is_a?"A_TO_B":"B_TO_A","HRCP_SC"};event.tx_sample=now;event.tx_samples=w.tx_samples;event.rx_sample=w.arrival;event.frame_start_s=double(now)/clock_hz;event.frame_bits=w.bits;event.mcs=w.mcs;event.beacon_index=index;event.wire_sequence=action==Action::Data?e.data_sequence:0;event.wire_reference=mac.last_sequence;if(e.timer.start!=never){event.wait_start_s=double(e.timer.start)/clock_hz;event.deadline_s=double(e.timer.deadline)/clock_hz;}event.rssi=is_a?channel.rssi_ab:channel.rssi_ba;event.minimum_snr_db=is_a?channel.budget_ab.snr_db:channel.budget_ba.snr_db;log(event);}
    waves.push_back(std::move(w));UpdateObserver();
  }
  void Receive(unsigned endpoint,unsigned phase,const HrcpReception&r){
    bool is_a=endpoint==0;Endpoint &e=is_a?a:b;
    uint64_t end=2*r.end_chip+phase;
    if(!r.header_event){ // 観測ログだけでTXとの対応付けを行う。MACの判断に利用しない。
      for(auto &w:waves)if(w.ab!=is_a){
        uint64_t sfd=2*r.sfd_end_chip+phase;
        if(sfd>=w.arrival&&sfd<w.end){
          w.decoded=r;HrcpMac observed;HrcpBeacon bc;
          w.received=r.header_ok&&r.body_ok&&hrcp_mac_decode(r.header.mac,observed)&&
            (observed.type==0?hrcp_beacon_decode(r.body,bc):hrcp_body_decode(observed,r.body).ok);
          w.reason=w.received?"":r.reason;
          if(!w.received&&r.header_ok&&r.body_ok)w.reason=observed.type==0?"BEACON_MAC_OR_FCS":hrcp_body_decode(observed,r.body).reason;
          break;
        }
      }
    }
    if(end<e.tx_end)return;
    HrcpMac mac;
    if(!r.header_event)e.last_rx=r.header_ok?(r.body_ok?"PHY_DECODED":r.reason):r.reason;
    if(!r.header_ok||!hrcp_mac_decode(r.header.mac,mac)){
      if(!r.header_event){Emit("RX_FAIL",r.reason,!is_a);}return;}
    const bool stale=!is_a&&!e.started&&!e.associated&&e.expired_id&&mac.source==e.expired_id;
    if(mac.pairnet!=0x3101||(mac.source!=(is_a?0:e.devid)&&!stale)||(mac.destination!=(is_a?e.devid:0)&&!(is_a&&mac.type==0&&mac.destination==255)))return;
    if(r.header_event){
      // 有効な宛先headerを受けた時点でRIFS再送を止める。長いDATAの末尾まで待たない。
      if(mac.type!=0){e.recover_at=never;e.last_header_end=end;}return;
    }
    if(last_full_rx[endpoint]!=never&&end>=last_full_rx[endpoint]&&end-last_full_rx[endpoint]<=2)return;
    last_full_rx[endpoint]=end;
    if(mac.type==0&&is_a){HrcpBeacon decoded;
      if(!r.body_ok||!hrcp_beacon_decode(r.body,decoded)){Emit("RX_FAIL","BEACON_CRC",false,"BEACON");return;}
      if(c.expected_rssi_admission_enabled){
        // RSSIはアンテナ端へ戻し、受信利得補正を施す。PHY成否の閾値ではない。
        const double measured_isotropic=10*std::log10(r.estimated_es_no*1.76e9/c.bandwidth_ba_hz)
          +channel.budget_ba.input_referred_noise_dbm-channel.budget_ba.rx_gain_dbi;
        admission_known=true;admission_ok=measured_isotropic>=decoded.expected_rssi_dbm;
        admission_sample=end;admission_measured=measured_isotropic;admission_expected=decoded.expected_rssi_dbm;
        if(measured_isotropic<decoded.expected_rssi_dbm){
          if(monitor&&!observed_connected)monitor->MarkFailure(end,end,false,channel.rssi_ba,"BEACON","EXPECTED_RSSI_REJECT");
          Emit("ADMISSION_REJECT","Expected RSSI条件未達",false,"BEACON");return;}
      }
      if(decoded.address!=beacon.address||e.associated||(e.started&&e.devid!=decoded.next_id))return;
      e.devid=decoded.next_id;
      uint64_t selected=0;for(const auto &w:waves)if(!w.ab&&end>=w.end-2&&end<=w.end+2)selected=w.index;
      e.selected_beacon=selected;
      unsigned slot=rng()%decoded.slots;Schedule(e,Action::Request,end+sifs+ticks(slot*decoded.slot_tenths_us*1e-7));
      e.last_rx="BEACON / OK";
      Emit("RX_OK","Beacon復号; アクセススロット="+std::to_string(slot),false,"BEACON",selected);return;
    }
    auto parsed=hrcp_body_decode(mac,r.body);bool valid=r.body_ok&&parsed.ok;
    e.last_rx=name(mac,r.body)+(valid?" / OK":" / "+(r.body_ok?parsed.reason:r.reason));
    Emit(valid?"RX_OK":"RX_FAIL",valid?"MAC_VALID":r.body_ok?parsed.reason:r.reason,!is_a,name(mac,r.body),is_a?e.selected_beacon:0);
    if(e.associated&&valid)e.timer.Refresh(end,Atp(e)); // 正常なMAC受信だけ更新。header内ACKは本文成否と独立。
    if(stale){if(valid){b.reject_destination=mac.source;Schedule(b,Action::Disassociate,end+sifs);}return;}
    if(is_a&&valid&&mac.type==3&&mac.subframes&&parsed.command==2&&parsed.payload.size()==1){
      a.associated=a.started=false;a.timer.Clear();a.send_at=a.recover_at=never;a.last_command=a.last_data=1023;a.data_sequence=a.mcs=a.successes=0;
      a.atp_ms=c.requested_atp_ms?c.requested_atp_ms:c.atp_ms;UpdateObserver();return;
    }
    if(!is_a&&!b.associated&&valid&&mac.type==3&&mac.subframes&&parsed.command==0){
      unsigned atp=0;if(!hrcp_request_valid(parsed.payload,atp)||mac.source!=beacon.next_id)return;
      b.atp_ms=std::min(c.atp_ms,atp);b.started=true;b.last_command=parsed.sequence;next_beacon=never;
      Schedule(b,Action::Response,((end+period-1)/period)*period);UpdateObserver();return;
    }
    if(is_a&&a.started&&valid&&mac.type==3&&mac.subframes&&parsed.command==1){
      unsigned atp=0;if(!hrcp_response_valid(parsed.payload,a.devid,atp))return;
      a.atp_ms=atp;a.last_command=parsed.sequence;a.associated=true;a.timer.Clear();Schedule(a,Action::CommandAck,end+sifs);UpdateObserver();return;
    }
    if(!is_a&&b.started&&!b.associated&&valid&&((mac.type==3&&mac.subframes==0&&mac.last_type==0&&mac.last_sequence==0)||(mac.type==4&&mac.subframes))){
      b.associated=true;b.timer.Refresh(end,Atp(b));UpdateObserver();
    }
    if(!e.associated){if(!is_a&&b.started&&mac.ack_policy==2)Schedule(b,Action::Response,end+rifs_b);return;}
    if(!DataSender(is_a)&&mac.type==4&&mac.subframes&&valid){
      unsigned expected=(e.last_data+1)%1024;
      if(parsed.sequence==expected){e.last_data=parsed.sequence;stats.delivered_mb+=parsed.payload.size()/1e6;}
    }
    if(DataSender(is_a)&&mac.last_type==1&&mac.last_sequence==e.data_sequence){
      ++stats.data_frames;++e.data_generation;e.data_sequence=(e.data_sequence+1)%1024;
      if(++e.successes>=10){e.mcs=std::min(3u,e.mcs+1);e.successes=0;}
    }else if(DataSender(is_a)&&mac.type==4&&mac.last_type==1){e.mcs=e.mcs? e.mcs-1:0;e.successes=0;}
    if(mac.ack_policy==2)Schedule(e,DataSender(is_a)?Action::Data:Action::DataAck,end+sifs);
  }
  void Dispatch(){
    while(!pending_rx.empty()){
      auto first=pending_rx.front();uint64_t first_end=2*first.value.end_chip+first.phase;
      if(now<first_end+2)break;
      pending_rx.pop_front();
      auto quality=[](const HrcpReception&r){
        HrcpMac mac;if(!r.header_ok||!r.body_ok||!hrcp_mac_decode(r.header.mac,mac))return -2.;
        HrcpBeacon bc;bool valid=mac.type==0?hrcp_beacon_decode(r.body,bc):hrcp_body_decode(mac,r.body).ok;
        return (valid?1.:0.)+r.sync_metric;
      };
      for(auto it=pending_rx.begin();it!=pending_rx.end();){
        uint64_t other=2*it->value.sfd_end_chip+it->phase,origin=2*first.value.sfd_end_chip+first.phase;
        if(it->endpoint==first.endpoint&&std::abs(double(other)-double(origin))<=2){
          if(quality(it->value)>quality(first.value))first=*it;
          it=pending_rx.erase(it);
        }else ++it;
      }
      Receive(first.endpoint,first.phase,first.value);
    }
  }
  void PacketEvents(){
    for(auto &w:waves){
      if(!w.header_sent&&now>=w.header_end){
        w.header_sent=true;
        if(w.packet_ok&&!w.half_duplex){auto r=w.packet;r.header_event=true;r.end_chip=w.header_end/2;Receive(w.ab?1:0,w.header_end%2,r);}
      }
      if(!w.packet_sent&&now>=w.end){
        w.packet_sent=true;
        if(w.packet_ok&&!w.half_duplex){auto r=w.packet;r.end_chip=w.end/2;Receive(w.ab?1:0,w.end%2,r);}
      }
    }
  }
  void Events(){
    Dispatch();
    if(empirical)PacketEvents();
    if(now>=next_channel){channel=evaluate(double(now)/clock_hz);channel_sample=now;next_channel=now+ticks(c.channel_sample_s);
      if(!std::isfinite(channel.distance)||channel.distance<0)throw std::invalid_argument("伝搬距離が不正です");
      if(!empirical)for(unsigned i=0;i<2;++i){double snr=i?channel.budget_ab.snr_db:channel.budget_ba.snr_db,bw=i?c.bandwidth_ab_hz:c.bandwidth_ba_hz;
        if(!std::isfinite(snr))throw std::invalid_argument("HRCPには有限のSNRが必要です");
        amp[i]=std::sqrt(std::pow(10.,snr/10.)*bw/1.76e9);
        // 位相0の整合フィルタ出力: 雑音の正規化電力は1、等価雑音帯域は1.76GHz。
        // 受信回路出力点へW校正する。信号と雑音を分離せず実I/Qの二乗を積算する。
        const double noise_dbm=i?channel.budget_ab.receiver_noise_dbm:channel.budget_ba.receiver_noise_dbm;
        rx_watts_per_unit[i]=std::pow(10.,(noise_dbm-30)/10.)*1.76e9/bw;
      }
      for(auto &w:waves)if(now<w.end)w.minimum_snr=std::min(w.minimum_snr,w.ab?channel.budget_ab.snr_db:channel.budget_ba.snr_db);
    }
    Expire(a,true);Expire(b,false);
    if(now==next_beacon){if(now>=b.tx_end)Send(false,Action::Beacon);next_beacon+=period;}
    for(bool is_a:{true,false}){auto &e=is_a?a:b;
      if(now==e.send_at){auto action=e.send;Send(is_a,action);}
      else if(now==e.recover_at){e.recover_at=never;Emit("RIFS_RETRY","再送によるATP延長なし",is_a);if(DataSender(is_a)){e.mcs=e.mcs?e.mcs-1:0;e.successes=0;}// §7.6.5/7.12.7.2:接続後の非同期回復は本文なし。確認後SIFSでDATAへ戻る。
        Send(is_a,e.associated&&e.recover==Action::Data?Action::DataAck:e.recover);}}
    for(auto &w:waves)if(!w.reported&&now>=w.end+4){
      w.reported=true;++stats.awgn_trials;if(!w.received){++stats.failed_frames;++stats.awgn_errors;}
      if(monitor){auto &o=w.ab?observed_ab:observed_ba;
        o={true,w.received,w.end,w.frame,w.received?"":w.half_duplex?"HALF_DUPLEX_OVERLAP":w.reason};
        if(w.link_setup){
          bool qualified=w.received;std::string reason=o.reason;
          if(w.frame=="BEACON"&&w.received&&admission_known&&std::abs(double(admission_sample)-double(w.end))<=2){
            qualified=admission_ok;if(!qualified)reason="EXPECTED_RSSI_REJECT";}
          monitor->SetupResult(now,w.ab,qualified,w.frame,reason);
          if(!w.received)monitor->MarkFailure(w.arrival,w.end,w.ab,w.ab?channel.rssi_ab:channel.rssi_ba,w.frame,o.reason);
        }}
      if(log){LinkEvent event{double(w.end)/clock_hz,"FRAME_RESULT",w.frame,w.ab?"A_TO_B":"B_TO_A",w.received?"":w.half_duplex?"HALF_DUPLEX_OVERLAP":w.reason};
        event.frame_start_s=double(w.start)/clock_hz;event.tx_sample=w.start;event.tx_samples=w.tx_samples;event.rx_sample=w.arrival;event.processed_samples=empirical?0:w.tx_samples;event.frame_bits=w.bits;event.mcs=w.mcs;event.beacon_index=w.index;
        event.minimum_snr_db=w.minimum_snr;event.rssi=w.ab?channel.rssi_ab:channel.rssi_ba;event.sync_metric=w.decoded.sync_metric;event.wire_sequence=w.sequence;event.wire_reference=w.reference;
        if(empirical){event.rssi=w.sampled_rssi;event.minimum_snr_db=std::numeric_limits<double>::quiet_NaN();
          event.ber=w.decision.ber<0?std::numeric_limits<double>::quiet_NaN():w.decision.ber;
          event.per=w.decision.per;event.random_draw=w.draw;event.empirical_curve=w.decision.curve;event.lookup_rssi_dbm=w.decision.lookup_dbm;}
        log(event);}
    }
    while(!waves.empty()&&waves.front().reported)waves.pop_front();
    if(monitor)monitor->ObserveStage(now,ProtocolStage());
    if(empirical&&monitor&&now>=next_monitor){
      next_monitor=now+ticks(std::max(c.monitor_rssi_interval_s,.00001));
      monitor->ObserveTime(now?now-1:0);monitor->PushRssi(now,channel.rssi_ba,channel.rssi_ab);
      if(monitor->Due())monitor->Publish(MonitorStatus());
    }
    next_event=std::min({next_channel,next_beacon,a.send_at,b.send_at,a.recover_at,b.recover_at,a.timer.deadline,b.timer.deadline});
    for(const auto&w:waves)if(!w.reported)next_event=std::min(next_event,w.end+4);
    if(empirical){
      if(monitor)next_event=std::min(next_event,next_monitor);
      for(const auto &w:waves){if(!w.header_sent)next_event=std::min(next_event,w.header_end);if(!w.packet_sent)next_event=std::min(next_event,w.end);}
    }
    if(!pending_rx.empty()){const auto&r=pending_rx.front();next_event=std::min(next_event,2*r.value.end_chip+r.phase+2);}
    if(next_event<=now)next_event=now+1;
  }
  void Advance(double time){
    uint64_t until=ticks(time);if(until<now)throw std::invalid_argument("HRCP時刻を逆行できません");
    if(empirical){
      while(now<until){if(now>=next_event)Events();auto stop=std::min(until,next_event);
        if(observed_connected)connected_samples+=stop-now;
        now=stop;}
      stats.connected_s=double(connected_samples)/clock_hz;Events();return;
    }
    while(now<until){if(!c.fast_events||now>=next_event)Events();
      if(c.noise_enabled&&c.noise_threads>1&&now>=noise_end){
        noise_start=now;noise_end=std::min(until,now+noise_buffer.size());
        const auto count=static_cast<int64_t>(noise_end-noise_start);
#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(c.noise_threads)
#endif
        for(int64_t i=0;i<count;++i)for(unsigned ep=0;ep<2;++ep)noise_buffer[i][ep]=noise(c.seed,noise_start+i,ep);
      }
      if(observed_connected)++connected_samples;
      HrcpSignalMonitor::Values monitored{};
      if(monitor&&c.monitor_waveforms)for(const auto&w:waves)if(now>=w.start&&now<w.start+2*w.chips.size()){
        auto tx=w.chips[(now-w.start)/2]*.7071067811865475244;unsigned k=w.ab?0:4;monitored[k]+=tx.real();monitored[k+1]+=tx.imag();}
      HrcpComplex signal[2]{};for(auto&w:waves)if(now>=w.arrival&&now<w.end){signal[w.ab?1:0]+=w.chips[(now-w.arrival)/2]*(.7071067811865475244*amp[w.ab?1:0]);
        w.half_duplex|=now<(w.ab?b:a).tx_end;}
      std::array<double,2> rx_power_w{};std::array<bool,2> rx_available{};
      for(unsigned ep=0;ep<2;++ep){auto &e=ep?b:a;auto sample=signal[ep]+(c.noise_enabled?(c.noise_threads>1?noise_buffer[now-noise_start][ep]:noise(c.seed,now,ep)):HrcpComplex{});
        if(now>=e.tx_end&&now>e.tx_end){unsigned phase=(now+1)%2;
          if(now>=1){auto chip=phase?(now-2)/2:(now-1)/2;
            auto filtered=(sample+previous[ep])*.7071067811865475244;
            receivers[ep][phase]->Feed(filtered,chip);
            if(monitor&&phase==0){rx_power_w[ep]=std::norm(filtered)*rx_watts_per_unit[ep];rx_available[ep]=true;}
          }}
        if(monitor&&c.monitor_waveforms){unsigned k=ep?6:2;monitored[k]=sample.real();monitored[k+1]=sample.imag();}
        previous[ep]=sample;}
      if(monitor&&(now%2==1))monitor->ObserveReceivePower(now,rx_power_w,rx_available);
      if(monitor&&c.monitor_waveforms)monitor->Push(now,monitored,now>a.tx_end,now>b.tx_end);
      if(monitor&&now>=next_monitor){next_monitor=now+35200;monitor->ObserveTime(now);if(monitor->Due())monitor->Publish(MonitorStatus());}
      ++now;
    }
    stats.connected_s=double(connected_samples)/clock_hz;
    Events(); // ちょうどuntilの送信・ATP満了も、このAdvanceで確定する。
  }
};
HrcpPairnet::HrcpPairnet(HrcpConfig c,Evaluate e,Log l):impl_(std::make_unique<Impl>(c,std::move(e),std::move(l))){}
HrcpPairnet::~HrcpPairnet()=default;
void HrcpPairnet::Advance(double t){impl_->Advance(t);}
bool HrcpPairnet::Connected()const{return impl_->observed_connected;}
const LinkStats&HrcpPairnet::Stats()const{return impl_->stats;}
const std::string&HrcpPairnet::State()const{return impl_->state;}
}
