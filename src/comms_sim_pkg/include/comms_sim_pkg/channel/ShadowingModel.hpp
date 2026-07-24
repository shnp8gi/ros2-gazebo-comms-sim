#pragma once

#include <cmath>
#include <cstdint>
#include <random>
#include <unordered_map>
#include <vector>
#include <Eigen/Dense>

namespace comms_sim
{
    /**
     * IShadowingModel
     * ---------------
     * 責務: 位置に紐づく大規模シャドウイング [dB] の生成のみ。
     * 戻り値はゼロ平均の符号付き dB 値(正 = 追加損失)。
     * tx_pos = 移動側(車載)アンテナ位置、rx_pos = 固定側(基地局)アンテナ位置。
     * rx_pos は場のキーに使う実装(FrozenField)のために渡す。使わない実装は
     * 無視してよい。
     */
    class IShadowingModel {
    public:
        virtual ~IShadowingModel() = default;
        virtual double SampleDb(int link_id,
                                const Eigen::Vector3d& tx_pos,
                                const Eigen::Vector3d& rx_pos) = 0;
    };

    /**
     * GudmundsonShadowingModel
     * ------------------------
     * Gudmundson モデル: 移動距離 Δd に対し相関 ρ = exp(-Δd / d_corr) を持つ
     * 対数正規シャドウイングを AR(1) 過程として生成する。
     * 現行の白色ガウス雑音は d_corr → 0 の特殊ケースに相当する。
     * 実現値はシード(=run 毎に注入)に依存し、走行ごとに引き直される。
     */
    class GudmundsonShadowingModel : public IShadowingModel {
    public:
        GudmundsonShadowingModel(double sigma_db, double corr_distance_m, unsigned seed)
            : sigma_db(sigma_db), corr_distance_m(corr_distance_m), rng(seed) {}

        double SampleDb(int link_id,
                        const Eigen::Vector3d& tx_pos,
                        const Eigen::Vector3d& /*rx_pos*/) override {
            std::normal_distribution<double> gauss(0.0, this->sigma_db);
            auto it = this->states.find(link_id);
            if (it == this->states.end()) {
                double value = gauss(this->rng);
                this->states[link_id] = {tx_pos, value};
                return value;
            }
            LinkState& state = it->second;
            double delta_d = (tx_pos - state.last_pos).norm();
            double rho = std::exp(-delta_d / this->corr_distance_m);
            state.value = rho * state.value + std::sqrt(1.0 - rho * rho) * gauss(this->rng);
            state.last_pos = tx_pos;
            return state.value;
        }

    private:
        struct LinkState {
            Eigen::Vector3d last_pos;
            double value;
        };

        double sigma_db;
        double corr_distance_m;
        std::mt19937 rng;
        std::unordered_map<int, LinkState> states;
    };

    /**
     * FrozenFieldShadowingModel (本番仕様 §4.2)
     * -----------------------------------------
     * 走行間で固定される「環境固有」の持続シャドウ。60GHz シャドウの正体は
     * 建物・標識・街路樹等の時間的に持続する物理要因であり、走行ごとに
     * 引き直すと REM の平均場が学習できない。
     *
     * 実装: rx(基地局)アンテナ位置をキーに、道路軸への射影 u = tx_pos[axis]
     * 上の1次元場 X_j(u) を保持する。場は environment_seed と rx 位置の
     * 量子化キーから決定論生成 (格子 AR(1) + 線形補間) するため、
     *   - 同一 environment_seed なら run をまたいで同一 (学習相・評価相で固定)
     *   - 車両ごとの独立なプラグインインスタンス間でも自動的に一致
     *   - link_id・呼び出し順序・時刻に依存しない
     * run 毎に注入される channel.seed は参照しないこと (仕様のシード規約)。
     *
     * 責務: 固定場のサンプルのみ。場の物理的正当性 (σ・相関長) は設定の責務。
     */
    class FrozenFieldShadowingModel : public IShadowingModel {
    public:
        FrozenFieldShadowingModel(double sigma_db, double corr_length_m,
                                  unsigned environment_seed,
                                  double grid_m = 1.5, int axis = 0,
                                  double u_min = -1000.0, double u_max = 1000.0)
            : sigma_db(sigma_db), corr_length_m(corr_length_m),
              environment_seed(environment_seed), grid_m(grid_m), axis(axis),
              u_min(u_min), u_max(u_max) {}

        double SampleDb(int /*link_id*/,
                        const Eigen::Vector3d& tx_pos,
                        const Eigen::Vector3d& rx_pos) override {
            const std::vector<double>& field = this->FieldFor(rx_pos);
            double u = tx_pos[this->axis];
            double x = (u - this->u_min) / this->grid_m;
            if (x <= 0.0) return field.front();
            if (x >= static_cast<double>(field.size() - 1)) return field.back();
            std::size_t i = static_cast<std::size_t>(x);
            double frac = x - static_cast<double>(i);
            return field[i] * (1.0 - frac) + field[i + 1] * frac;
        }

    private:
        /// rx 位置の 0.5m 量子化キー (同一基地局は必ず同一の場を得る)
        uint64_t KeyFor(const Eigen::Vector3d& rx_pos) const {
            auto q = [](double v) -> uint64_t {
                return static_cast<uint64_t>(static_cast<int64_t>(std::llround(v * 2.0)));
            };
            uint64_t h = 1469598103934665603ULL;  // FNV-1a
            for (uint64_t part : {q(rx_pos.x()), q(rx_pos.y()), q(rx_pos.z())}) {
                for (int b = 0; b < 8; ++b) {
                    h ^= (part >> (8 * b)) & 0xFF;
                    h *= 1099511628211ULL;
                }
            }
            return h;
        }

        const std::vector<double>& FieldFor(const Eigen::Vector3d& rx_pos) {
            uint64_t key = this->KeyFor(rx_pos);
            auto it = this->fields.find(key);
            if (it != this->fields.end()) return it->second;

            // 決定論生成: seed_seq(environment_seed, key) → 格子 AR(1)。
            // 定常分散が sigma² になるよう初期値も N(0, σ) から引く。
            std::size_t n = static_cast<std::size_t>(
                std::ceil((this->u_max - this->u_min) / this->grid_m)) + 1;
            // seed_seq は下位32bitしか使わないため 64bit キーは2語に分割する
            std::seed_seq seq{static_cast<uint32_t>(this->environment_seed),
                              static_cast<uint32_t>(key & 0xFFFFFFFFu),
                              static_cast<uint32_t>(key >> 32),
                              0x5eedu};
            std::mt19937 gen(seq);
            std::normal_distribution<double> gauss(0.0, this->sigma_db);
            double rho = std::exp(-this->grid_m / this->corr_length_m);
            double innov = std::sqrt(1.0 - rho * rho);

            std::vector<double> field(n);
            field[0] = gauss(gen);
            for (std::size_t i = 1; i < n; ++i) {
                field[i] = rho * field[i - 1] + innov * gauss(gen);
            }
            return this->fields.emplace(key, std::move(field)).first->second;
        }

        double sigma_db;
        double corr_length_m;
        unsigned environment_seed;
        double grid_m;
        int axis;
        double u_min, u_max;
        std::unordered_map<uint64_t, std::vector<double>> fields;
    };
} // namespace comms_sim
