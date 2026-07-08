#pragma once

#include <random>
#include <cmath>
#include <complex>
#include <unordered_map>
#include <algorithm>

namespace comms_sim
{
    /**
     * IFadingModel
     * ------------
     * 責務: 小規模フェージングによる瞬時損失 [dB] の生成のみ。
     * LOS/NLOS 状態に応じて統計を切り替える(状態の判定は呼び出し側)。
     * 戻り値は損失 [dB](E[電力利得]=1 に正規化した利得の負の dB 値)。
     */
    class IFadingModel {
    public:
        virtual ~IFadingModel() = default;
        virtual double SampleLossDb(int link_id, bool is_los, double t) = 0;
    };

    /**
     * RicianFadingModel
     * -----------------
     * 散乱成分をコヒーレンス時間 τ_c の AR(1) 複素ガウス過程として保持し、
     *   g = | sqrt(K/(K+1)) + sqrt(1/(K+1))·h |²,  E[g] = 1
     * の Rician 電力利得から損失 −10·log10(g) を返す。
     * LOS では K = K_los、NLOS では K = K_nlos(K→0 で Rayleigh に一致)。
     */
    class RicianFadingModel : public IFadingModel {
    public:
        RicianFadingModel(double k_los_db, double k_nlos_db,
                          double coherence_time_s, unsigned seed)
            : k_los_linear(std::pow(10.0, k_los_db / 10.0)),
              k_nlos_linear(std::pow(10.0, k_nlos_db / 10.0)),
              coherence_time_s(coherence_time_s),
              rng(seed) {}

        double SampleLossDb(int link_id, bool is_los, double t) override {
            // 散乱成分 h ~ CN(0,1) を AR(1) で時間相関させる
            std::normal_distribution<double> gauss(0.0, std::sqrt(0.5));
            auto it = this->states.find(link_id);
            if (it == this->states.end()) {
                LinkState st;
                st.scatter = {gauss(this->rng), gauss(this->rng)};
                st.last_t = t;
                it = this->states.emplace(link_id, st).first;
            } else {
                LinkState& st = it->second;
                double dt = std::max(0.0, t - st.last_t);
                double rho = std::exp(-dt / this->coherence_time_s);
                std::complex<double> innovation(gauss(this->rng), gauss(this->rng));
                st.scatter = rho * st.scatter + std::sqrt(1.0 - rho * rho) * innovation;
                st.last_t = t;
            }

            double k = is_los ? this->k_los_linear : this->k_nlos_linear;
            std::complex<double> h = std::sqrt(k / (k + 1.0)) +
                                     std::sqrt(1.0 / (k + 1.0)) * it->second.scatter;
            double power_gain = std::max(std::norm(h), 1e-9);
            return -10.0 * std::log10(power_gain);
        }

    private:
        struct LinkState {
            std::complex<double> scatter;
            double last_t = 0.0;
        };

        double k_los_linear;
        double k_nlos_linear;
        double coherence_time_s;
        std::mt19937 rng;
        std::unordered_map<int, LinkState> states;
    };
} // namespace comms_sim
