// 拡張仕様書 §10.1 の単体テスト（通信層）。
#include "comms_sim_pkg/drone_link.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
using namespace comms_sim;
void check(bool value, const char *message) { if (!value) throw std::runtime_error(message); }
void near_at(int line, double a, double b, double tolerance = 1e-9) {
  if (std::abs(a-b) <= tolerance) return;
  std::ostringstream message;
  message << "数値不一致: line=" << line << std::setprecision(17) << ", actual=" << a << ", expected=" << b;
  throw std::runtime_error(message.str());
}
#define near(...) near_at(__LINE__, __VA_ARGS__)

int main(int argc, char **argv) {
  try {
    check(argc == 2, "設定ディレクトリが必要です");
    const std::string dir = argv[1];
    const double rad = M_PI / 180;

    // --- 取り付けオフセット 0、揺れなし：最接近時のなす角が 0°
    auto c2 = ScenarioConfig::Load(dir + "/drone_simple_c2.yaml");
    {
      RadioModel radio(c2);
      const auto s = radio.Evaluate(c2.encounter_s);
      near(s.distance, 5, 1e-9);
      near(s.off_axis_a_deg, 0, 1e-6); near(s.off_axis_b_deg, 0, 1e-6);
      near(s.deflection_a_deg, 0, 1e-6); near(s.deflection_b_deg, 0, 1e-6);
    }
    // --- オフセットを与えると、最接近時のなす角が幾何どおりになる
    {
      auto c = c2;
      c.a.mount_elevation_offset_deg = 7; c.b.mount_azimuth_offset_deg = 10;
      const auto s = RadioModel(c).Evaluate(c.encounter_s);
      near(s.off_axis_a_deg, 7, 1e-6);
      // 仰角 45° の向きを方位だけ δ 回すと、なす角は acos(0.5 cosδ + 0.5)
      near(s.off_axis_b_deg, std::acos(0.5 * std::cos(10 * rad) + 0.5) / rad, 1e-6);
      // 外部の供給元（既定の取り付けの姿勢）にオフセットを回転として掛けても同じ結果になる
      RadioModel external(c, [&](double t) { return analytic_poses(c2, t); });
      for (double t : {0., 12.5, 20., 27.}) {
        const auto x = external.Evaluate(t), y = RadioModel(c).Evaluate(t);
        near(x.rssi_ab, y.rssi_ab, 1e-9); near(x.rssi_ba, y.rssi_ba, 1e-9);
        near(x.off_axis_a_deg, y.off_axis_a_deg, 1e-6); near(x.off_axis_b_deg, y.off_axis_b_deg, 1e-6);
        near(x.deflection_a_deg, 0, 1e-6); near(x.deflection_b_deg, 0, 1e-6);
      }
      // 供給元のアンテナだけを傾けると、その分がぶれ角として出る
      RadioModel shaken(c2, [&](double t) {
        auto p = analytic_poses(c2, t);
        p.antenna_a = p.antenna_a * AntennaPatternParser::rpy_to_rotmat(0, 0.5 * rad, 0);
        return p; });
      near(shaken.Evaluate(20).deflection_a_deg, 0.5, 1e-6);
      near(shaken.Evaluate(20).off_axis_a_deg, 0.5, 1e-6);
      c.antenna_mode = "tracking";
      bool rejected = false; try { c.Validate(); } catch (const std::invalid_argument &) { rejected = true; }
      check(rejected, "追従アンテナで取り付けオフセットを受理しています");
    }

    // --- ボンディング N：同じ RSSI に対して、N に対応する表のレートが返る
    struct Row { const char *file; unsigned n; double sensitivity; double at_m60, at_m52, at_m40; };
    for (const auto &row : {Row{"drone_simple_c2.yaml", 1, -67, 2.87, 6.78, 8.62},
                            Row{"drone_simple_c3_2.yaml", 2, -63, 4.52, 11.50, 17.25},
                            Row{"drone_simple_c3_4.yaml", 4, -60, 4.52, 18.07, 34.50}}) {
      const auto c = ScenarioConfig::Load(dir + "/" + row.file);
      check(c.ba.bonding_channels == row.n && c.ba.phy == "thz_sc", "下りリンクのボンディング設定");
      near(c.ba.sensitivity_dbm, row.sensitivity);
      near(c.ab.sensitivity_dbm, -61);  // 戻りリンクは HRCP-SC の MCS0
      CommsCalculator table(nullptr, 0, 0, c.ba.rate_table);
      near(table.calculate_throughput(-60), row.at_m60);
      near(table.calculate_throughput(-52), row.at_m52);
      near(table.calculate_throughput(-40), row.at_m40);
      near(table.calculate_throughput(row.sensitivity - 1e-6), 0);
      // 総送信電力は一定：N を変えても受信電力は変わらない
      near(RadioModel(c).Evaluate(20).rssi_ba, RadioModel(c2).Evaluate(20).rssi_ba);
    }
    {
      const auto c1 = ScenarioConfig::Load(dir + "/drone_simple_c1.yaml");
      check(c1.ba.phy == "hrcp_sc" && c1.ab.phy == "hrcp_sc", "構成①は両方向 60 GHz");
      near(c1.ba.sensitivity_dbm, -61); near(c1.ab.sensitivity_dbm, -61);
      CommsCalculator table(nullptr, 0, 0, c1.ba.rate_table);
      near(table.calculate_throughput(-61), 2.26); near(table.calculate_throughput(-39), 11.50);
      near(table.calculate_throughput(-61.001), 0);
      // 両方向とも同じ回路なので受信電力も同じ
      const auto s = RadioModel(c1).Evaluate(20);
      near(s.rssi_ab, s.rssi_ba);
    }

    // --- 簡易リンク確立：両リンクがしきい値以上になってから 2 ms 後に接続し、下回ると切断
    {
      ProtocolConfig p; p.simple_setup = true; p.data_from_prc = true; p.channel_sample_s = 1e-4;
      // 戻りリンクは 1 ms から、下りリンクは 3 ms から 20 ms まで。30〜31 ms は短すぎて接続に至らない。
      auto evaluate = [](double t) {
        LinkSample s{};
        const long k = std::lround(t * 1e4);
        s.ab_ok = k >= 10; s.ba_ok = (k >= 30 && k < 200) || (k >= 300 && k < 310) || k >= 400;
        s.rate_ba_gbps = 4; s.rate_ab_gbps = 100;  // データは下りリンクのレートで積む
        return s;
      };
      std::vector<LinkEvent> events;
      LinkProtocol link(p, evaluate, [&](const LinkEvent &e) { events.push_back(e); });
      link.Advance(0.0050); check(!link.Connected(), "2 ms 経つ前に接続しています");
      near(link.Stats().delivered_mb, 0);
      link.Advance(0.0051); check(link.Connected(), "2 ms 後に接続していません");
      near(link.Stats().first_connected_s, 0.005); near(link.Stats().last_setup_ms, 2);
      link.Advance(0.0200);
      near(link.Stats().delivered_mb, 4 * 0.015 * 125, 1e-9);  // 5〜20 ms
      near(link.Stats().window_s, 0.017, 1e-12);                // 3〜20 ms
      link.Advance(0.0201); check(!link.Connected(), "下りリンクの低下で切断していません");
      link.Advance(0.0400);
      check(link.Stats().connections == 1 && link.Stats().disconnections == 1, "短い回復で接続しています");
      near(link.Stats().window_s, 0.018, 1e-12);
      // 再びしきい値以上になったら、2 ms の確立をやり直す
      link.Advance(0.0420); check(!link.Connected(), "再接続が早すぎます");
      link.Advance(0.0421); check(link.Connected() && link.Stats().connections == 2, "再接続していません");
      link.Advance(0.0500);
      near(link.Stats().delivered_mb, 4 * (0.015 + 0.008) * 125, 1e-9);
      near(link.Stats().window_s, 0.028, 1e-12);
      check(events.size() == 3 && events[0].type == "CONNECTED" && events[1].type == "DISCONNECTED"
            && events[1].detail == "BA_BELOW_THRESHOLD" && events[2].type == "CONNECTED", "イベントの記録");
    }

    // --- 集計：平均スループット = 総転送量 ÷ 両リンクがしきい値以上だった時間
    {
      LinkStats s; s.delivered_mb = 250; s.window_s = 2; s.last_setup_ms = 2;
      DroneRun::Encounter e; e.d_min_m = 5; e.min_angle_a_deg = 1; e.min_angle_b_deg = 2;
      const auto row = encounter_summary_row(c2, s, e);
      check(row == "2,1,5.000000000,1.000000000,2.000000000,2.000000000,250.000000000,1.000000000,2.000000000",
            "summary の追加列");
      s.window_s = 0; s.delivered_mb = 0;
      check(encounter_summary_row(c2, s, e).find(",0.000000000,0.000000000,0.000000000,2.000000000")
            != std::string::npos, "通信できる時間が 0 のとき平均スループットは 0");
    }
    std::cout << "取り付けオフセット・ボンディング・簡易リンク確立・集計テスト成功\n";
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
