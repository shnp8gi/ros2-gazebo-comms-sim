/**
 * kkf_golden_dump
 * ---------------
 * 責務: C++リファレンス実装 (kkf/*.hpp) の決定論フィクスチャに対する出力ダンプのみ。
 * Python 移植 (comms_sim_pkg.kkf_core) とのゴールデンテスト
 * (tools/tests/golden_kkf_test.py) が本出力と数値一致することを検証する。
 *
 * フィクスチャは乱数を使わず手続き的に生成する (両言語で同一式を実装)。
 * 出力形式:
 *   P,<s>,<mean>,<variance>      … KKF予測 (t=2.5, s=0,25,...,250)
 *   D,<a0> <a1> ... <aK-1>,<total_value>   … ビタビDP解
 */
#include <cstdio>
#include <cmath>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "comms_sim_pkg/kkf/RoadCoordinate.hpp"
#include "comms_sim_pkg/kkf/BasisFunction.hpp"
#include "comms_sim_pkg/kkf/KrigedKalmanFilter.hpp"
#include "comms_sim_pkg/kkf/HandoverPlanner.hpp"

int main() {
    using namespace tx_controller::kkf;

    // --- KKF フィクスチャ ---
    std::vector<Eigen::Vector3d> polyline = {{0, 0, 0}, {1000, 0, 0}};
    RoadCoordinate road(polyline);
    Eigen::Vector3d rsu_pos(150.0, 10.0, 1.0);

    KkfParams params;
    params.process_noise_q = 1e-4;
    params.initial_state_var = 100.0;
    params.sigma_nu = 4.0;
    params.corr_length_s_m = 20.0;
    params.corr_length_t_s = 5.0;
    params.residual_buffer_size = 64;
    params.initial_state_mean = Eigen::Vector2d(-30.0, -20.0);

    auto basis = std::make_shared<LogDistanceBasis>(road, rsu_pos);
    KrigedKalmanFilter filter(basis, params);

    for (int k = 0; k < 40; ++k) {
        double t = 0.05 * (k + 1);
        std::vector<Observation> obs;
        for (double s : {5.0 * k, 5.0 * k + 8.0}) {
            Observation o;
            o.s = s;
            o.t = t;
            o.z = -60.0 + 5.0 * std::sin(0.2 * s) + 2.0 * std::sin(1.3 * t);
            o.noise_var = 4.0;
            obs.push_back(o);
        }
        filter.Update(t, obs);
    }

    for (int i = 0; i <= 10; ++i) {
        double s = 25.0 * i;
        auto pred = filter.PredictAt(s, 2.5);
        std::printf("P,%.1f,%.12e,%.12e\n", s, pred.mean, pred.variance);
    }

    // --- プランナ フィクスチャ ---
    const int A = 4, K = 15;
    std::vector<std::vector<double>> lcb(A, std::vector<double>(K));
    for (int a = 0; a < A; ++a) {
        for (int k = 0; k < K; ++k) {
            lcb[a][k] = 10.0 * std::sin(1.7 * a + 0.31 * k);
        }
    }
    auto plan = HandoverPlanner::Solve(lcb, 0.3, 5.0, 1);
    std::printf("D,");
    for (int k = 0; k < K; ++k) {
        std::printf("%d%s", plan.assignments[k], (k + 1 < K) ? " " : "");
    }
    std::printf(",%.12e\n", plan.total_value);
    return 0;
}
