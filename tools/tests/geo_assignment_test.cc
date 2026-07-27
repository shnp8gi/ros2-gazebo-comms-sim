// GeoAssignmentRegistry のビットマスク DP が厳密な最大重みマッチングを与えるかを
// 総当たりと照合する。TxControllerPlugin.cc は gz-sim 依存が重くリンクできないため、
// DP 本体と同一のアルゴリズムをここに写して検証する (アルゴリズムの回帰テスト)。
// 実装を変更したら本ファイルの Solve も追従すること。
//
//   docker compose exec -T sim bash -c "cd /workspace && \
//     g++ -std=c++17 -O2 tools/tests/geo_assignment_test.cc -o /tmp/gat && /tmp/gat"
#include <algorithm>
#include <cstdio>
#include <map>
#include <numeric>
#include <random>
#include <functional>
#include <string>
#include <vector>

static int failures = 0;

static void check(bool ok, const std::string& msg) {
    if (!ok) { std::printf("  FAIL: %s\n", msg.c_str()); ++failures; }
}

// 効用の規約: 接続閾値からのマージン (rssi - rmin) [dB] = 常に非負。
// 生の RSSI (dBm, 負値) を使うと「誰も割り当てない (合計0)」が最適になる縮退が
// 起きるため。ineligible は -1e9。
// --- TxControllerPlugin.cc の GeoAssignmentRegistry::Solve と同一のアルゴリズム ---
static std::map<int, int> SolveDp(const std::vector<std::vector<double>>& utils, int B) {
    const double kMinUtil = -1e8;
    const double kNeg = -1e18;
    const int full = 1 << B;
    const int V = static_cast<int>(utils.size());
    std::vector<std::vector<double>> dp(V + 1, std::vector<double>(full, kNeg));
    std::vector<std::vector<int>> choice(V, std::vector<int>(full, -2));
    dp[0][0] = 0.0;
    for (int k = 0; k < V; ++k) {
        const auto& u = utils[k];
        for (int mask = 0; mask < full; ++mask) {
            if (dp[k][mask] <= kNeg) continue;
            if (dp[k][mask] > dp[k + 1][mask]) { dp[k + 1][mask] = dp[k][mask]; choice[k][mask] = -1; }
            for (int b = 0; b < B && b < static_cast<int>(u.size()); ++b) {
                if (mask & (1 << b)) continue;
                if (u[b] < kMinUtil) continue;
                int nm = mask | (1 << b);
                double val = dp[k][mask] + u[b];
                if (val > dp[k + 1][nm]) { dp[k + 1][nm] = val; choice[k][nm] = b; }
            }
        }
    }
    int best_mask = 0; double best_val = kNeg;
    for (int mask = 0; mask < full; ++mask)
        if (dp[V][mask] > best_val) { best_val = dp[V][mask]; best_mask = mask; }
    std::map<int, int> out;
    int mask = best_mask;
    for (int k = V - 1; k >= 0; --k) {
        int c = choice[k][mask];
        if (c >= 0) { out[k] = c; mask &= ~(1 << c); }
    }
    return out;
}

// --- 総当たり (各車は「割り当てない」or「未使用のBS」を選ぶ) ---
static double BruteForce(const std::vector<std::vector<double>>& utils, int B) {
    const double kMinUtil = -1e8;
    const int V = static_cast<int>(utils.size());
    double best = 0.0;
    std::vector<int> assign(V, -1);
    std::function<void(int, int, double)> rec = [&](int k, int used, double acc) {
        if (k == V) { best = std::max(best, acc); return; }
        rec(k + 1, used, acc);                       // 割り当てない
        for (int b = 0; b < B; ++b) {
            if (used & (1 << b)) continue;
            if (utils[k][b] < kMinUtil) continue;
            rec(k + 1, used | (1 << b), acc + utils[k][b]);
        }
    };
    rec(0, 0, 0.0);
    return best;
}

int main() {
    // 1) 既知の最適解: 対角優位 (効用 = 閾値マージン [dB], 非負)
    {
        std::vector<std::vector<double>> u = {{18.5, 3.0, 1.0, 0.5},
                                              {2.0, 16.5, 1.5, 0.8},
                                              {1.2, 2.5, 14.5, 1.0}};
        auto a = SolveDp(u, 4);
        check(a.size() == 3 && a[0] == 0 && a[1] == 1 && a[2] == 2, "対角最適を外した");
    }
    // 2) 貪欲が失敗する構造 (全車の第1希望が同じ BS)
    {
        // v0 の第1希望も v1 の第1希望も bs0。貪欲(v0が bs0)だと 18.5+0.5=19.0、
        // 最適は v0->bs1(17.5), v1->bs0(16.5) = 34.0
        std::vector<std::vector<double>> u = {{18.5, 17.5, -1e9, -1e9},
                                              {16.5, 0.5, -1e9, -1e9}};
        auto a = SolveDp(u, 4);
        double total = 0; for (auto& kv : a) total += u[kv.first][kv.second];
        check(std::abs(total - 34.0) < 1e-9, "貪欲な取り合いで最適を外した");
    }
    // 3) 需要超過 (車 > RSU): ちょうど B 台が割り当てられ、排他が守られる
    {
        std::mt19937 rng(1);
        std::uniform_real_distribution<double> d(0.0, 26.5);
        std::vector<std::vector<double>> u(10, std::vector<double>(4));
        for (auto& r : u) for (auto& x : r) x = d(rng);
        auto a = SolveDp(u, 4);
        check(a.size() == 4, "需要超過で割当数が RSU 数と一致しない");
        std::vector<int> bs;
        for (auto& kv : a) bs.push_back(kv.second);
        std::sort(bs.begin(), bs.end());
        check(std::unique(bs.begin(), bs.end()) == bs.end(), "BS 排他違反");
    }
    // 4) 圏外 (-1e9) は絶対に採用されない
    {
        std::vector<std::vector<double>> u = {{-1e9, -1e9, -1e9, -1e9},
                                              {8.5, -1e9, -1e9, -1e9}};
        auto a = SolveDp(u, 4);
        check(a.size() == 1 && a.count(1) && a[1] == 0, "圏外ペアが採用された");
    }
    // 5) 総当たり照合 (ランダム 200 ケース、圏外混在)
    {
        std::mt19937 rng(7);
        std::uniform_real_distribution<double> d(0.0, 26.5);
        std::uniform_int_distribution<int> nv(1, 7);
        std::uniform_real_distribution<double> p(0.0, 1.0);
        for (int t = 0; t < 200; ++t) {
            int V = nv(rng), B = 4;
            std::vector<std::vector<double>> u(V, std::vector<double>(B));
            for (auto& r : u) for (auto& x : r) x = (p(rng) < 0.25) ? -1e9 : d(rng);
            auto a = SolveDp(u, B);
            double got = 0; for (auto& kv : a) got += u[kv.first][kv.second];
            double best = BruteForce(u, B);
            // 全ペア圏外なら双方 0 (割当なし)
            if (a.empty()) { check(best <= 0.0 + 1e-9, "割当なしだが総当たりは正の解を持つ"); continue; }
            check(got >= best - 1e-6, "総当たりの最適値に届かない");
        }
        std::printf("  総当たり照合 200 ケース一致\n");
    }
    if (failures) { std::printf("FAIL: %d 件\n", failures); return 1; }
    std::printf("PASS: geo_assignment_test 全項目合格\n");
    return 0;
}
