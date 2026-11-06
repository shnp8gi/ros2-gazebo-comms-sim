#include "comms_sim_pkg/tjx_theoretical_phy.hpp"
#include "comms_sim_pkg/drone_link.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
using namespace comms_sim;
void require(bool b,const char*m){if(!b)throw std::runtime_error(m);}
void near(double a,double b,double eps=1e-9){require(std::abs(a-b)<eps,"数値が不一致です");}
template<class F>void rejects(F f){bool bad=false;try{f();}catch(const std::exception&){bad=true;}require(bad,"不正値を受理しました");}
int main(){try{
  near(tjx_rate_bps(3,false),1.76e9*11/15,1e-6);
  near(tjx_rate_bps(0,false),tjx_rate_bps(3,false)/2,1e-6);
  near(tjx_rate_bps(8,true),1.76e9*2*14/15*7/8,1e-6);
  for(auto [tjx,internal]:std::vector<std::pair<unsigned,unsigned>>{{6,0},{8,1},{9,2},{11,3}})
    for(unsigned bytes:{0u,13u,27u,36u,2056u})for(bool pilot:{false,true})for(bool setup:{false,true}){
      HrcpHeader h;h.mcs=internal;h.pilot=pilot;h.body_bytes=bytes;
      require(tjx_frame_chips(tjx,bytes,setup,pilot)==HrcpCodec::FrameChips(h,setup),"既存規格フレーム時間と不一致");
    }
  require(tjx_frame_chips(0,2056,false,true)>tjx_frame_chips(3,2056,false,true),"繰返し送信時間");
  rejects([]{tjx_mcs(1);});rejects([]{tjx_required_blocks(0,3);});
  // 全MCSで無雑音寄りの復号、短縮符号、低SNRでの失敗、seed再現を検証。
  TjxBlockSimulator simulator(123),again(123);
  for(auto id:tjx_mcs_ids())for(auto bits:{104u,tjx_mcs(id).information_bits}){
    auto high=simulator.Run(id,bits,30),same=again.Run(id,bits,30);
    require(!high.error&&!high.bit_errors&&high.error==same.error,"高SNR復号失敗");
    auto low=simulator.Run(id,bits,-30),low_same=again.Run(id,bits,-30);
    require(low.error&&low.bit_errors&&low.bit_errors==low_same.bit_errors,"低SNR・乱数再現失敗");
  }
  require(!simulator.Run(-1,132,30).error&&simulator.Run(-1,132,-30).error,"EHヘッダー復号");
  TjxBlockSimulator rep0(321),rep3(321);unsigned failures0=0,failures3=0;
  for(unsigned i=0;i<60;++i){failures0+=rep0.Run(0,1056,-2).error;failures3+=rep3.Run(3,1056,-2).error;}
  require(failures0<failures3,"MCS0の反復合成利得がありません");
  auto path=std::filesystem::temp_directory_path()/("tjx-theory-test-"+std::to_string(getpid())+".yaml");
  YAML::Node root;root["model"]="tjx_ldpc_awgn_v1";root["chip_rate_hz"]=1.76e9;
  root["decoder_iterations"]=50;root["decoder_normalization"]=.8;root["channel"]="ideal_sync_known_channel_awgn";
  for(auto [mcs,bits]:tjx_required_blocks(2048,3))for(int esno:{-20,0,20}){
    YAML::Node p;p["mcs"]=mcs;p["information_bits"]=bits;p["es_no_db"]=esno;p["trials"]=100;
    p["block_errors"]=esno<0?100:esno==0?10:0;p["bit_errors"]=0;root["points"].push_back(p);}
  std::ofstream(path)<<root;
  TjxTheoryConfig c;c.enabled=true;c.table=path;c.control_mcs=3;
  TjxTheoreticalPhy phy(c);phy.ValidateLengths(2048);
  auto d=phy.Evaluate(3,264,0,1.76e9); // 1056情報bit×2、header1個。
  near(d.header_per,.1);near(d.body_per,.19);near(d.per,.271);
  near(phy.Evaluate(3,0,0,1.76e9).per,.1);
  near(phy.Evaluate(3,264,-10*std::log10(2.),3.52e9).per,d.per);
  require(phy.Evaluate(3,264,-30,1.76e9).outside_range,"LUT範囲外の記録");
  near(phy.Evaluate(3,264,-30,1.76e9).per,1);
  rejects([&]{phy.Evaluate(3,1,0,1.76e9);});rejects([&]{phy.Evaluate(3,0,NAN,1.76e9);});
  rejects([&]{phy.Evaluate(3,0,0,0);});
  auto channel=[](double){LinkSample s;s.distance=10;s.rssi_ab=s.rssi_ba=-100;
    s.rate_ab_gbps=s.rate_ba_gbps=0;s.budget_ab.snr_db=s.budget_ba.snr_db=20;return s;};
  HrcpConfig h;h.noise_enabled=false;h.expected_rssi_admission_enabled=false;h.theoretical=c;h.data_from_prc=true;
  h.bandwidth_ab_hz=h.bandwidth_ba_hz=1.76e9;h.channel_sample_s=.0001;
  std::set<int> modes;std::vector<LinkEvent> events;
  HrcpPairnet link(h,channel,[&](const LinkEvent&e){events.push_back(e);if(e.type=="TX"&&e.frame=="DATA")modes.insert(e.mcs);});link.Advance(.002);
  require(link.Connected()&&link.Stats().delivered_mb>0&&link.Stats().failed_frames==0,"理論モデルMAC統合・旧閾値依存");
  require(modes==std::set<int>({0,3,6,8,9,11}),"TJX 6段階のMCS適応");
  HrcpPairnet chunks(h,channel,{});for(unsigned i=1;i<=20;++i)chunks.Advance(i*.0001);
  require(chunks.Stats().delivered_mb==link.Stats().delivered_mb&&chunks.Stats().awgn_trials==link.Stats().awgn_trials,"時刻分割で結果が変わりました");
  HrcpPairnet blocked(h,[&](double t){auto s=channel(t);s.budget_ab.snr_db=s.budget_ba.snr_db=-30;return s;},{});blocked.Advance(.002);
  require(!blocked.Connected()&&blocked.Stats().delivered_mb==0&&blocked.Stats().failed_frames>0,"低SNRで通信成功しました");
  for(const auto&e:events)if(e.type=="FRAME_RESULT"){
    near(e.lookup_es_no_db,20);near(e.per,0);require(e.processed_samples==0,"LUTモードで連続波形処理");}
  // 中間確率でもchunk分割で抽選順・成否が変わらないことを確認。
  auto uncertain=[&](double t){auto s=channel(t);s.budget_ab.snr_db=s.budget_ba.snr_db=0;return s;};
  HrcpPairnet random_link(h,uncertain,{});random_link.Advance(.004);
  HrcpPairnet random_chunks(h,uncertain,{});for(unsigned i=1;i<=40;++i)random_chunks.Advance(i*.0001);
  require(random_link.Stats().delivered_mb>0&&random_link.Stats().failed_frames>0,"確率的な成功・失敗がありません");
  require(random_link.Stats().delivered_mb==random_chunks.Stats().delivered_mb&&random_link.Stats().failed_frames==random_chunks.Stats().failed_frames,"確率抽選がchunk分割で変化");
  auto other=h;++other.theoretical.seed;HrcpPairnet other_seed(other,uncertain,{});other_seed.Advance(.004);
  require(other_seed.Stats().delivered_mb!=random_link.Stats().delivered_mb||other_seed.Stats().failed_frames!=random_link.Stats().failed_frames,"seed変更が反映されません");
  root["points"].push_back(root["points"][0]);std::ofstream(path)<<root;
  rejects([&]{TjxTheoreticalPhy duplicate(c);});
  std::filesystem::remove(path);
  std::cout<<"TJX理論PHY: 復号・速度・長さ・LUT・MAC統合の検査成功\n";
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
