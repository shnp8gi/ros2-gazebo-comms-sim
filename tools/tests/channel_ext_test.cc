// FrozenFieldShadowingModel / RateModel 単体テスト (本番仕様 §4.2, §4.3)。
// ヘッダオンリー実装のためコンテナ内で直接コンパイルして実行する:
//   docker compose exec -T sim bash -c "cd /workspace && \
//     g++ -std=c++17 -O2 -I src/comms_sim_pkg/include -I /usr/include/eigen3 \
//         tools/tests/channel_ext_test.cc -lyaml-cpp -o /tmp/channel_ext_test && \
//     /tmp/channel_ext_test"
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "comms_sim_pkg/channel/ShadowingModel.hpp"
#include "comms_sim_pkg/RateModel.hpp"

using comms_sim::FrozenFieldShadowingModel;
using comms_sim::GudmundsonShadowingModel;
using comms_sim::McsTableRateModel;
using comms_sim::ShannonRateModel;

static int failures = 0;

static void check(bool ok, const std::string& msg) {
    if (!ok) {
        std::printf("  FAIL: %s\n", msg.c_str());
        ++failures;
    }
}

int main() {
    const Eigen::Vector3d bs0(0.0, 8.0, 2.5);
    const Eigen::Vector3d bs1(60.0, 8.0, 2.5);

    // 1) 決定論性: 別インスタンス (= 別run・別車両プラグイン) で完全一致
    {
        FrozenFieldShadowingModel a(4.0, 6.0, 1);
        FrozenFieldShadowingModel b(4.0, 6.0, 1);
        double max_diff = 0.0;
        for (double u = -200.0; u <= 200.0; u += 0.7) {
            Eigen::Vector3d pos(u, 0.0, 1.35);
            // link_id が違っても同じ場 (環境固有であり、リンク固有でない)
            double va = a.SampleDb(0, pos, bs0);
            double vb = b.SampleDb(7, pos, bs0);
            max_diff = std::max(max_diff, std::abs(va - vb));
        }
        std::printf("  決定論性: 別インスタンス間 max|Δ| = %.3g dB\n", max_diff);
        check(max_diff == 0.0, "同一 environment_seed で場が一致しない");
    }

    // 2) 環境シードが違えば場が変わる / BS が違えば独立の場
    {
        FrozenFieldShadowingModel a(4.0, 6.0, 1);
        FrozenFieldShadowingModel c(4.0, 6.0, 2);
        double diff_seed = 0.0, diff_bs = 0.0;
        for (double u = -100.0; u <= 100.0; u += 1.0) {
            Eigen::Vector3d pos(u, 0.0, 1.35);
            diff_seed += std::abs(a.SampleDb(0, pos, bs0) - c.SampleDb(0, pos, bs0));
            diff_bs += std::abs(a.SampleDb(0, pos, bs0) - a.SampleDb(0, pos, bs1));
        }
        check(diff_seed > 1.0, "environment_seed を変えても場が変わらない");
        check(diff_bs > 1.0, "BS (rx位置) を変えても場が変わらない");
    }

    // 3) 統計: σ ≈ 4dB、相関長 6m で lag=6m の自己相関 ≈ exp(-1)
    {
        FrozenFieldShadowingModel a(4.0, 6.0, 12345, 1.5, 0, -6000.0, 6000.0);
        std::vector<double> x;
        for (double u = -5990.0; u <= 5990.0; u += 1.5) {
            x.push_back(a.SampleDb(0, Eigen::Vector3d(u, 0.0, 1.35), bs0));
        }
        double mean = 0.0;
        for (double v : x) mean += v;
        mean /= x.size();
        double var = 0.0;
        for (double v : x) var += (v - mean) * (v - mean);
        var /= x.size();
        int lag = 4;  // 4 × 1.5m = 6m = 相関長
        double cov = 0.0;
        for (std::size_t i = 0; i + lag < x.size(); ++i) {
            cov += (x[i] - mean) * (x[i + lag] - mean);
        }
        cov /= (x.size() - lag);
        double rho = cov / var;
        std::printf("  統計: std=%.2f dB (期待4.0), rho(6m)=%.3f (期待%.3f)\n",
                    std::sqrt(var), rho, std::exp(-1.0));
        check(std::abs(std::sqrt(var) - 4.0) < 0.8, "場の標準偏差が σ=4dB から乖離");
        check(std::abs(rho - std::exp(-1.0)) < 0.12, "相関長 6m が実現されていない");
    }

    // 4) 補間の連続性 (格子間で不連続ジャンプしない)
    {
        FrozenFieldShadowingModel a(4.0, 6.0, 1);
        double max_step = 0.0;
        for (double u = -50.0; u <= 50.0; u += 0.05) {
            double v1 = a.SampleDb(0, Eigen::Vector3d(u, 0, 1.35), bs0);
            double v2 = a.SampleDb(0, Eigen::Vector3d(u + 0.05, 0, 1.35), bs0);
            max_step = std::max(max_step, std::abs(v2 - v1));
        }
        std::printf("  連続性: 5cm 移動の max|Δ| = %.3f dB\n", max_step);
        check(max_step < 1.0, "補間が不連続 (5cm で 1dB 超のジャンプ)");
    }

    // 5) Gudmundson が新シグネチャでも従来動作 (走行内相関・リンク独立)
    {
        GudmundsonShadowingModel g(4.0, 10.0, 42);
        double v1 = g.SampleDb(0, Eigen::Vector3d(0, 0, 1.35), bs0);
        double v2 = g.SampleDb(0, Eigen::Vector3d(0.1, 0, 1.35), bs0);
        check(std::abs(v2 - v1) < 4.0, "Gudmundson: 10cm 移動で相関が切れている");
    }

    // 6) ShannonRateModel: 閾値・飽和・値 (単位は Gbps = プラグインのデータ会計前提)
    {
        ShannonRateModel r(100.0e6, -95.0, 1.0, 0.0, 50.0);
        check(r.MinRssiDbm() == -95.0, "Shannon: MinRssi != noise_floor + snr_min");
        check(r.RateGbps(-95.1) == 0.0, "Shannon: 閾値未満が 0 でない");
        double rate0 = r.RateGbps(-95.0);   // SNR 0dB → 0.1 Gbps
        check(std::abs(rate0 - 0.1) < 1e-9, "Shannon: SNR 0dB のレートが 0.1Gbps でない");
        double rate28 = r.RateGbps(-67.0);  // SNR 28dB → ~0.93 Gbps
        std::printf("  Shannon: SNR28dB → %.3f Gbps (期待 ~0.934)\n", rate28);
        check(std::abs(rate28 - 0.934) < 0.005, "Shannon: SNR 28dB のレートが異常");
        check(r.RateGbps(-30.0) == r.RateGbps(-45.0), "Shannon: snr_cap で飽和しない");
    }

    // 7) McsTableRateModel: 既定テーブルが旧実装と同一
    {
        McsTableRateModel m("");
        check(m.MinRssiDbm() == -61.0 && m.MaxRssiDbm() == -39.0,
              "MCS: 既定テーブルの端点が旧実装と異なる");
        check(m.RateGbps(-62.0) == 0.0, "MCS: 閾値未満が 0 でない");
        check(std::abs(m.RateGbps(-60.0) - 2.5813) < 1e-9, "MCS: -60dBm の段が異なる");
        check(std::abs(m.RateGbps(-39.0) - 13.1413) < 1e-9, "MCS: 飽和値が異なる");
    }

    if (failures > 0) {
        std::printf("FAIL: %d 件\n", failures);
        return 1;
    }
    std::printf("PASS: channel_ext_test 全項目合格\n");
    return 0;
}
