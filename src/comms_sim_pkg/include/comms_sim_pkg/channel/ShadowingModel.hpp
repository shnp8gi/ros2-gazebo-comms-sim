#pragma once

#include <random>
#include <cmath>
#include <unordered_map>
#include <Eigen/Dense>

namespace comms_sim
{
    /**
     * IShadowingModel
     * ---------------
     * 責務: 位置に紐づく大規模シャドウイング [dB] の生成のみ。
     * 戻り値はゼロ平均の符号付き dB 値(正 = 追加損失)。
     * link_id ごとに独立の空間過程を保持する。
     */
    class IShadowingModel {
    public:
        virtual ~IShadowingModel() = default;
        virtual double SampleDb(int link_id, const Eigen::Vector3d& pos) = 0;
    };

    /**
     * GudmundsonShadowingModel
     * ------------------------
     * Gudmundson モデル: 移動距離 Δd に対し相関 ρ = exp(-Δd / d_corr) を持つ
     * 対数正規シャドウイングを AR(1) 過程として生成する。
     * 現行の白色ガウス雑音は d_corr → 0 の特殊ケースに相当する。
     */
    class GudmundsonShadowingModel : public IShadowingModel {
    public:
        GudmundsonShadowingModel(double sigma_db, double corr_distance_m, unsigned seed)
            : sigma_db(sigma_db), corr_distance_m(corr_distance_m), rng(seed) {}

        double SampleDb(int link_id, const Eigen::Vector3d& pos) override {
            std::normal_distribution<double> gauss(0.0, this->sigma_db);
            auto it = this->states.find(link_id);
            if (it == this->states.end()) {
                double value = gauss(this->rng);
                this->states[link_id] = {pos, value};
                return value;
            }
            LinkState& state = it->second;
            double delta_d = (pos - state.last_pos).norm();
            double rho = std::exp(-delta_d / this->corr_distance_m);
            state.value = rho * state.value + std::sqrt(1.0 - rho * rho) * gauss(this->rng);
            state.last_pos = pos;
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
} // namespace comms_sim
