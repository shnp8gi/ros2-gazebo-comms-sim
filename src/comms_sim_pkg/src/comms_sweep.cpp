// Gazebo を使わずに通信だけを計算する掃引。条件ごとに1行の集計を CSV に書く。
// manifest.yaml の runs: の各要素
//   id, config（設定ファイル）, d_min_m, speed_mps,
//   offset_a_az_deg, offset_a_el_deg, offset_b_az_deg, offset_b_el_deg（すべて省略可）
#include "comms_sim_pkg/drone_link.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace {
struct Job {
  std::string id, config;
  double d_min = -1, speed = -1;
  double a_az = 0, a_el = 0, b_az = 0, b_el = 0;
};
std::string run(const Job &job) {
  using namespace comms_sim;
  auto c = ScenarioConfig::Load(job.config);
  if (job.speed > 0) c.a.speed_kmh = c.b.speed_kmh = job.speed * 3.6;
  if (job.d_min > 0) {
    // 横方向間隔と高度差をどちらも d_min/√2 にして、最接近時の相手方向を 45° に保つ。
    if (c.scenario != "parallel") throw std::invalid_argument("d_min_m は並行すれ違いでのみ使えます");
    c.lateral_separation_m = job.d_min / std::sqrt(2.0);
    c.b.altitude_m = c.a.altitude_m + job.d_min / std::sqrt(2.0);
  }
  c.a.mount_azimuth_offset_deg = job.a_az; c.a.mount_elevation_offset_deg = job.a_el;
  c.b.mount_azimuth_offset_deg = job.b_az; c.b.mount_elevation_offset_deg = job.b_el;
  c.Validate();
  RadioModel radio(c);
  DroneRun::Encounter encounter;
  LinkProtocol link(c.protocol, [&](double t) {
    auto s = radio.Evaluate(t); encounter.Observe(s); return s; });
  link.Advance(c.duration_s);
  const auto &s = link.Stats();
  std::ostringstream row;
  row << std::fixed << std::setprecision(9) << job.id << ',' << c.a.speed_kmh / 3.6
      << ',' << job.a_az << ',' << job.a_el << ',' << job.b_az << ',' << job.b_el
      << ',' << s.connections << ',' << s.disconnections << ',' << s.first_connected_s
      << ',' << s.connected_s << ',' << encounter_summary_row(c, s, encounter);
  return row.str();
}
}  // namespace

int main(int argc, char **argv) {
  try {
    if (argc != 3) throw std::invalid_argument("使用法: comms_sweep manifest.yaml 新規結果.csv");
    if (std::filesystem::exists(argv[2])) throw std::invalid_argument("既存結果は上書きしません");
    std::vector<Job> jobs;
    for (const auto &node : YAML::LoadFile(argv[1])["runs"]) {
      Job j;
      j.id = node["id"].as<std::string>(); j.config = node["config"].as<std::string>();
      auto number = [&](const char *key, double fallback) {
        return node[key] ? node[key].as<double>() : fallback; };
      j.d_min = number("d_min_m", -1); j.speed = number("speed_mps", -1);
      j.a_az = number("offset_a_az_deg", 0); j.a_el = number("offset_a_el_deg", 0);
      j.b_az = number("offset_b_az_deg", 0); j.b_el = number("offset_b_el_deg", 0);
      jobs.push_back(j);
    }
    std::vector<std::string> rows(jobs.size()), errors(jobs.size());
    #pragma omp parallel for schedule(dynamic)
    for (size_t i = 0; i < jobs.size(); ++i) {
      try { rows[i] = run(jobs[i]); } catch (const std::exception &e) { errors[i] = e.what(); }
    }
    for (size_t i = 0; i < jobs.size(); ++i)
      if (!errors[i].empty()) throw std::runtime_error(jobs[i].id + ": " + errors[i]);
    std::ofstream out(argv[2]);
    out << "id,speed_mps,offset_a_az_deg,offset_a_el_deg,offset_b_az_deg,offset_b_el_deg,"
           "connections,disconnections,first_connected_s,connected_time_s,"
        << comms_sim::encounter_summary_header() << '\n';
    for (const auto &row : rows) out << row << '\n';
    out.flush();
    if (!out) throw std::runtime_error("結果の書込みに失敗しました");
  } catch (const std::exception &e) { std::cerr << "掃引に失敗: " << e.what() << '\n'; return 1; }
}
