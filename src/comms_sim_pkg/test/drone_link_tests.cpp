#include "comms_sim_pkg/drone_link.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iomanip>
using namespace comms_sim;
void check(bool value, const char *message) { if (!value) throw std::runtime_error(message); }
void near_at(int line, double a, double b, double tolerance = 1e-9) {
  if (std::abs(a-b) <= tolerance) return;
  std::ostringstream message;
  message << "数値不一致: line=" << line << std::setprecision(17) << ", actual=" << a << ", expected=" << b;
  throw std::runtime_error(message.str());
}
#define near(...) near_at(__LINE__, __VA_ARGS__)
LinkSample good(double) { return {0, -40, -40, 1, 20, 20, true, true}; }
int main(int argc, char **argv) {
  try {
    check(argc == 2, "設定ディレクトリが必要です");
    ScenarioConfig c;
    auto g = geometry(c, 20);
    near((g.a-g.b).norm(), std::sqrt(200));
    c.scenario = "cross";
    near((geometry(c,20).a-geometry(c,20).b).norm(), 10);
    c.a.speed_kmh = 36; c.b.speed_kmh = 72;
    g = geometry(c, 21); near(g.a.x(), 10); near(g.b.y(), 20);
    c.antenna_mode = "tracking";
    AntennaPatternParser parser;
    for (const char *scenario : {"parallel", "cross"}) {
      c.scenario = scenario;
      for (double t : {0., 19., 20., 21., 40.}) {
        g = geometry(c, t);
        auto a = parser.calculate_antenna_frame_angles(g.a,g.b,g.antenna_a_rpy);
        auto b = parser.calculate_antenna_frame_angles(g.b,g.a,g.antenna_b_rpy);
        near(a.first,0); near(a.second,0); near(b.first,0); near(b.second,0);
      }
    }
    c.antenna_mode = "fixed"; g = geometry(c, 20);
    near(g.antenna_a_rpy.y(), -M_PI/4); near(g.antenna_b_rpy.y(), M_PI/4);
    near(g.antenna_a_rpy.z(), M_PI/2); near(g.antenna_b_rpy.z(), 0);
    // 十字の固定搭載は機首に対しA左90度/B右90度。追従せず姿勢を維持。
    near((geometry(c, 0).antenna_a_rpy-g.antenna_a_rpy).norm(),0);
    near((geometry(c, 40).antenna_b_rpy-g.antenna_b_rpy).norm(),0);
    auto after_cross = geometry(c, 21);
    for (bool is_a : {true, false}) {
      Eigen::Vector3d delta = is_a ? Eigen::Vector3d(after_cross.b-after_cross.a)
                                   : Eigen::Vector3d(after_cross.a-after_cross.b);
      const auto &rpy = is_a ? after_cross.antenna_a_rpy : after_cross.antenna_b_rpy;
      Eigen::Vector3d axis = AntennaPatternParser::rpy_to_rotmat(rpy.x(),rpy.y(),rpy.z())*Eigen::Vector3d::UnitX();
      check(axis.dot(delta)>0, "十字の固定アンテナは交差後に相手のいる側を向く");
    }
    c.scenario = "parallel";
    g = geometry(c, 20);
    auto fixed_a = parser.calculate_antenna_frame_angles(g.a, g.b, g.antenna_a_rpy);
    auto fixed_b = parser.calculate_antenna_frame_angles(g.b, g.a, g.antenna_b_rpy);
    near(fixed_a.first, 0); near(fixed_a.second, 0);
    near(fixed_b.first, 0); near(fixed_b.second, 0);
    // 追従ではなく、最接近の前後も同じ向きを保つ。
    near((geometry(c, 0).antenna_a_rpy-g.antenna_a_rpy).norm(),0);
    near((geometry(c, 40).antenna_b_rpy-g.antenna_b_rpy).norm(),0);
    c.Validate();
    c.a.speed_kmh = -1;
    bool invalid = false; try { c.Validate(); } catch (const std::invalid_argument &) { invalid = true; }
    check(invalid,"負の速度を拒否");
    c = ScenarioConfig{};
    c.e_plane = std::string(argv[1])+"/e_plane.csv";
    c.h_plane = std::string(argv[1])+"/h_plane.csv";
    c.mcs_table = std::string(argv[1])+"/MCStable.csv";
    c.antenna_mode = "tracking";
    RadioModel radio(c); auto r = radio.Evaluate(20);
    near(r.rssi_ab-r.rssi_ba, 10*std::log10(1/0.4)-20*std::log10(272./60));
    CommsCalculator calc(nullptr, 0, 0, c.mcs_table);
    auto base = calc.calculate_all(Eigen::Vector3d::Zero(),Eigen::Vector3d::UnitX(),0,false);
    auto threshold = calc.calculate_all(Eigen::Vector3d::Zero(),Eigen::Vector3d::UnitX(), -68.5-base.rssi,false);
    near(threshold.throughput, 0.5);
    ProtocolConfig p;
    std::vector<LinkEvent> events;
    LinkProtocol link(p, good, [&](const LinkEvent &e){events.push_back(e);});
    link.Advance(109e-6); check(!link.Connected(), "ACK受信前に接続しない"); near(link.Stats().delivered_mb,0);
    link.Advance(110e-6); check(link.Connected(), "ACK受信で接続");
    near(link.Stats().first_connected_s,110e-6);
    link.Advance(249e-6); near(link.Stats().delivered_mb,0);
    link.Advance(250e-6); near(link.Stats().delivered_mb,0.0125);
    std::vector<std::string> order;
    for (const auto &e:events) if(e.type=="RX_OK") order.push_back(e.frame);
    check(order==std::vector<std::string>({"BEACON","REQUEST","RESPONSE","ACK"}),"接続フレーム順序");
    for (const auto &e : events) check(e.beacon_index == 1, "最初の接続では全フレームにBeacon番号1を保持");
    // Request未受信中はBだけが周期送信を継続。Aは新しいBeaconを無視する。
    std::vector<LinkEvent> periodic_events;
    LinkProtocol periodic(p, [](double t) { auto s=good(t); s.ab_ok=false; return s; },
      [&](const LinkEvent &e) { periodic_events.push_back(e); });
    periodic.Advance(.0006);
    unsigned beacon_txs=0, ignored=0, requests=0;
    for (const auto &e : periodic_events) {
      if (e.type=="TX" && e.frame=="BEACON") {
        ++beacon_txs; near(e.time,(beacon_txs-1)*140e-6);
        check(e.beacon_index==beacon_txs,"Beaconインデックスは送信ごとに増加");
      }
      if (e.type=="TX" && e.frame=="REQUEST") { ++requests; check(e.beacon_index==1,"Requestの起点を保持"); }
      if (e.type=="BEACON_IGNORED") ++ignored;
    }
    check(beacon_txs==5 && requests==1 && ignored==4,"Request消失でも140us周期を維持しAは再要求しない");
    check(periodic.State()=="WAIT_RESPONSE","BeaconでResponse待ちを上書きしない");
    periodic.Advance(.500050001); near(periodic.Stats().timeouts,1);
    const auto &timeout=periodic_events.back();
    check(timeout.type=="TIMEOUT" && timeout.beacon_index==1 && timeout.frame=="REQUEST","タイムアウトも元Requestに対応");
    near(timeout.wait_start_s,50e-6); near(timeout.time,.500050);
    // 500ms後に再試行し、新しいBeacon番号で接続できる。
    std::vector<LinkEvent> retry_events;
    LinkProtocol retry(p, [](double t) { auto s=good(t); s.ab_ok=t>.001; return s; },
      [&](const LinkEvent &e) { retry_events.push_back(e); });
    retry.Advance(.5005); near(retry.Stats().connections,1); near(retry.Stats().timeouts,1);
    uint64_t retry_index=0;
    for (const auto &e : retry_events) {
      if (e.type=="TX" && e.frame=="REQUEST" && e.beacon_index>1) retry_index=e.beacon_index;
      if (e.type=="CONNECTED") check(e.beacon_index==retry_index && retry_index>1,"再接続に使ったBeacon番号を引き継ぐ");
      if (e.type=="TX" && e.frame=="BEACON") check(e.time<.500131,"Request受信後はBeacon停止");
    }
    // 応答間隔を延ばしてRequest送信と次Beacon送信を重ね、両端末の半二重動作を確認。
    auto overlap_config=p; overlap_config.ifs_s=120e-6;
    std::vector<LinkEvent> overlap_events;
    LinkProtocol overlap(overlap_config,good,[&](const LinkEvent &e){overlap_events.push_back(e);});
    overlap.Advance(.0003);
    bool a_busy=false,b_busy=false;
    for (const auto &e : overlap_events) if(e.type=="RX_FAILED" && e.detail=="RECEIVER_TRANSMITTING") {
      if (e.frame=="BEACON") { a_busy=true; check(e.beacon_index==2,"受信できないのは新しいBeacon #2"); }
      if (e.frame=="REQUEST") { b_busy=true; check(e.beacon_index==1,"Request #1とBeacon #2の跨インデックス重複"); }
    }
    check(a_busy && b_busy,"A/Bとも送信中の受信フレームは失敗");
    // Requestの受信完了と次Beacon予定時刻が同じなら受信完了を優先し停止する。
    auto boundary_config=p; boundary_config.ifs_s=100e-6;
    unsigned boundary_beacons=0;
    LinkProtocol boundary(boundary_config,good,[&](const LinkEvent &e){
      if(e.type=="TX" && e.frame=="BEACON") ++boundary_beacons;
    });
    boundary.Advance(.0004); check(boundary.Connected(),"境界時刻でRequestを正常受信");
    check(boundary_beacons==1,"Request受信と同時刻のBeaconは送らない");
    LinkProtocol stepped(p,good), jumped(p,good);
    for(int i=0;i<=1000;++i) stepped.Advance(i*1e-4);
    jumped.Advance(0.1);
    near(stepped.Stats().delivered_mb,jumped.Stats().delivered_mb);
    near(stepped.Stats().connected_s,jumped.Stats().connected_s);
    // Request消失: 送受信両方向が成立しなければデータを送れない。
    LinkProtocol no_ab(p,[](double t){auto s=good(t);s.ab_ok=false;return s;});
    no_ab.Advance(0.500049); near(no_ab.Stats().timeouts,0);
    no_ab.Advance(0.500050001); near(no_ab.Stats().timeouts,1);
    check(!no_ab.Connected(),"片方向だけでは接続不可"); near(no_ab.Stats().delivered_mb,0);
    LinkProtocol no_ba(p,[](double t){auto s=good(t);s.ba_ok=false;return s;});
    no_ba.Advance(1); near(no_ba.Stats().attempts,0);
    // Responseだけを落とし、Request完了から500msでタイムアウト。
    LinkProtocol lost_response(p,[](double t){auto s=good(t);s.ba_ok=!(t>=70e-6 && t<=91e-6);return s;});
    lost_response.Advance(0.500049); near(lost_response.Stats().timeouts,0);
    lost_response.Advance(0.500050001); near(lost_response.Stats().timeouts,1);
    // 接続用ACKだけを落とし、Response完了から500ms。
    LinkProtocol lost_ack(p,[](double t){auto s=good(t);s.ab_ok=!(t>=99e-6 && t<=111e-6);return s;});
    lost_ack.Advance(0.500079); near(lost_ack.Stats().timeouts,0); check(!lost_ack.Connected(),"ACK消失で接続不可");
    lost_ack.Advance(0.500080001); near(lost_ack.Stats().timeouts,1);
    // 最初のデータACKを落とす。未確認のpayloadは加算しない。
    std::vector<LinkEvent> inactivity_events;
    LinkProtocol inactivity(p,[](double t){auto s=good(t);s.ba_ok=t<200e-6;return s;},
      [&](const LinkEvent &e){inactivity_events.push_back(e);});
    inactivity.Advance(0.500109); check(inactivity.Connected(),"500ms直前は論理接続を維持");
    near(inactivity.Stats().delivered_mb,0);
    inactivity.Advance(0.500110001); check(!inactivity.Connected(),"500msで切断");
    near(inactivity.Stats().timeouts,0); near(inactivity.Stats().disconnections,1);
    near(inactivity.Stats().connected_s,0.5);
    bool saw_disconnect = false;
    for (const auto &e : inactivity_events) {
      check(e.type != "TIMEOUT", "データ送信後の切断をタイムアウトとして記録しない");
      if (e.type != "DISCONNECTED") continue;
      saw_disconnect = true;
      near(e.wait_start_s, 110e-6); near(e.deadline_s, 0.500110);
      near(e.time-e.wait_start_s, 0.5); near(e.failure_sample_s, 230e-6);
      check(e.frame == "DATA_ACK" && e.direction == "B_TO_A", "失敗方向の記録");
      check(e.detail == "INACTIVITY_DISCONNECT", "切断理由の記録");
    }
    check(saw_disconnect, "接続後の切断記録が必要");
    LinkProtocol first_data_lost(p,[](double t){auto s=good(t);s.ab_ok=t<120e-6;return s;});
    first_data_lost.Advance(.500110001);
    near(first_data_lost.Stats().timeouts,0); near(first_data_lost.Stats().disconnections,1);
    near(first_data_lost.Stats().delivered_mb,0);  // 初回DATAが失敗しても接続後の切断扱い。
    // 伝搬遅延は各フレームに加算する。
    LinkProtocol propagation(p,[](double t){auto s=good(t);s.distance=299.792458;return s;});
    propagation.Advance(113e-6); check(!propagation.Connected(),"伝搬遅延前に接続しない");
    propagation.Advance(114e-6); check(propagation.Connected(),"伝搬遅延を含む接続時刻");
    // ログ間隔で割り切れない時間でもサマリーと完了判定を保存する。
    auto temp = std::filesystem::temp_directory_path()/"drone_link_test_results";
    std::filesystem::remove_all(temp);
    c.output_dir=temp.string(); c.duration_s=.023; c.encounter_s=.01;
    { DroneRun run(c); run.Advance(.023); run.Finish(); }
    check(std::filesystem::exists(temp/"completed"),"完了マーカー");
    bool duplicate=false; try { DroneRun run(c); } catch (const std::runtime_error &) { duplicate=true; }
    check(duplicate,"既存結果を上書きしない");
    std::filesystem::remove_all(temp);
    std::cout << "ドローン軌道・周波数別損失・接続手順・500ms境界・集計テスト成功\n";
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
