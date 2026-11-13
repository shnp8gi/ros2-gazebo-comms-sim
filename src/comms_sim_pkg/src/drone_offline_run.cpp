// Gazebo を使わずに1条件を実行し、通常と同じ timeseries.csv / events.csv / summary.csv を出力する。
#include "comms_sim_pkg/drone_link.hpp"
#include <algorithm>
#include <iostream>
#include <stdexcept>

int main(int argc, char **argv) {
  try {
    if (argc != 3) throw std::invalid_argument("使用法: drone_offline_run 設定.yaml 新規出力ディレクトリ");
    auto config = comms_sim::ScenarioConfig::Load(argv[1]);
    config.output_dir = argv[2];
    comms_sim::DroneRun run(config);
    for (uint64_t k = 1;; ++k) {
      const double t = std::min(k * config.log_interval_s, config.duration_s);
      run.Advance(t);
      if (t >= config.duration_s) break;
    }
    run.Finish();
    std::cout << "完了: " << run.Stats().delivered_mb << " MB" << std::endl;
  } catch (const std::exception &e) { std::cerr << "実行に失敗: " << e.what() << '\n'; return 1; }
}
