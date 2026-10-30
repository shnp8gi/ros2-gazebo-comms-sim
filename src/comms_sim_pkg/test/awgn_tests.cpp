#include "comms_sim_pkg/drone_link.hpp"
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include <vector>
using namespace comms_sim;
void check(bool value, const char *message) { if (!value) throw std::runtime_error(message); }
void near(double a, double b, double tol=1e-10) { check(std::isfinite(a) && std::abs(a-b)<tol,"AWGN数値不一致"); }
LinkSample sample(double snr) {
  LinkSample s;
  s.rssi_ab=s.rssi_ba=-40; s.ab_ok=s.ba_ok=true; s.rate_ab_gbps=3.52;
  s.budget_ab.snr_db=s.budget_ba.snr_db=snr;
  return s;
}
int main() {
  try {
    near(qpsk_awgn_ber(0),0.07864960352514257);
    near(qpsk_awgn_ber(10),3.872108215522037e-6,1e-14);
    near(independent_bit_packet_error(0,512),0);
    near(independent_bit_packet_error(1,512),1);
    near(independent_bit_packet_error(1e-4,512),1-std::pow(1-1e-4,512));
    check(independent_bit_packet_error(1e-6,352000)>independent_bit_packet_error(1e-6,512),"長いフレームほどPERが高い");
    // 実際のGaussian雑音を加えた1軸のQPSK判定で理論BERを検証。
    std::mt19937_64 rng(719); std::normal_distribution<double> noise(0,std::sqrt(.5));
    unsigned errors=0;
    for(unsigned i=0;i<400000;++i) if(1+noise(rng)<0) ++errors;
    near(errors/400000.,qpsk_awgn_ber(0),.0025);
    AwgnSampler random(123), repeat(123), other(124);
    unsigned packets=0; bool different=false;
    for(unsigned i=0;i<100000;++i) {
      double u=random.Uniform(); near(u,repeat.Uniform());
      different=different || u!=other.Uniform();
      if(u<.37) ++packets;
    }
    check(different,"シードで乱数列が変わる"); near(packets/100000.,.37,.006);
    ProtocolConfig p; p.awgn.enabled=true;
    p.awgn.bandwidth_ab_hz=p.awgn.bandwidth_ba_hz=p.awgn.bit_rate_bps;
    LinkProtocol clean(p,[](double){return sample(100);});
    clean.Advance(.00025); check(clean.Connected(),"高SNRで接続");
    near(clean.Stats().delivered_mb,.044); near(clean.Stats().awgn_errors,0);
    auto prc_config=p;prc_config.data_from_prc=true;
    bool data_seen=false,ack_seen=false;
    LinkProtocol prc(prc_config,[](double){return sample(100);},[&](const auto&e){
      if(e.type=="FRAME_RESULT"&&e.frame=="DATA"){check(e.direction=="B_TO_A","PRCからDATA送信");data_seen=true;}
      if(e.type=="FRAME_RESULT"&&e.frame=="DATA_ACK"){check(e.direction=="A_TO_B","PRDEVからACK返信");ack_seen=true;}
    });
    prc.Advance(.00025);check(data_seen&&ack_seen&&prc.Connected(),"簡易モデルのPRCデータ方向");
    near(prc.Stats().delivered_mb,.044);
    LinkProtocol noisy(p,[](double){return sample(-100);});
    noisy.Advance(.1); check(!noisy.Connected(),"低SNRでBeacon失敗");
    check(noisy.Stats().awgn_errors>0,"AWGNによる誤りが必要"); near(noisy.Stats().delivered_mb,0);
    std::vector<double> draws_a,draws_b;
    auto record=[](std::vector<double>& draws, const LinkEvent& e) {
      if(e.type=="FRAME_RESULT") {
        draws.push_back(e.random_draw);
        near(e.eb_no_db,8);
        near(e.per,independent_bit_packet_error(e.ber,e.frame_bits));
        check((e.random_draw<e.per)==(e.detail=="AWGN_PACKET_ERROR"),"抽選と失敗理由の一致");
      }
    };
    LinkProtocol a(p,[](double){return sample(8);},[&](const auto& e){record(draws_a,e);});
    LinkProtocol b(p,[](double){return sample(8);},[&](const auto& e){record(draws_b,e);});
    for(int i=0;i<=20000;++i) a.Advance(i*.0001);
    b.Advance(2);
    check(draws_a==draws_b,"更新刻みによらず同一乱数列");
    check(a.Stats().awgn_errors==b.Stats().awgn_errors,"更新刻みによらず同一失敗回数");
    near(a.Stats().delivered_mb,b.Stats().delivered_mb);
    check(a.Stats().awgn_errors>0 && a.Stats().connections>0,"接続とAWGN失敗を検証");
    std::cout << "QPSK理論BER/実Gaussian試行、PER、再現性、フレーム抽選と固定レートの検証成功\n";
  } catch(const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
