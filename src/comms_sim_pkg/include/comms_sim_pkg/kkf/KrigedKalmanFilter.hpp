#pragma once

#include <deque>
#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include "comms_sim_pkg/kkf/BasisFunction.hpp"

namespace tx_controller
{
    namespace kkf
    {
        /// 単一の観測サンプル(弧長 s、時刻 t における測定値 z と観測雑音分散)
        struct Observation {
            double s = 0.0;
            double t = 0.0;
            double z = 0.0;
            double noise_var = 1.0;
        };

        /// KKF の統計パラメータ(文書 式(2),(3),(8) に対応)
        struct KkfParams {
            double process_noise_q = 1e-4;      // Q = q·I·Δt (潜在状態のランダムウォーク強度)
            double initial_state_var = 100.0;   // P0 = var·I
            double sigma_nu = 4.0;              // 残差場の標準偏差 σ_ν [dB]
            double corr_length_s_m = 20.0;      // 空間相関長 L_s [m]
            double corr_length_t_s = 5.0;       // 時間相関長 L_t [s]
            int residual_buffer_size = 64;      // クリギングに用いる残差リングバッファ長
            Eigen::VectorXd initial_state_mean; // α の事前平均(空なら零ベクトル)
        };

        /**
         * KrigedKalmanFilter
         * ------------------
         * 責務: 単一RSUの受信品質場 Z(s,t) の時空間推定のみ(文書 3章)。
         *  - 潜在状態 α のカルマンフィルタ時間更新・観測更新(式(4)〜(6)、Λ=I)
         *  - 更新後残差のリングバッファ保持と指数共分散クリギング(式(7),(8))
         * Eigen 以外に依存せず、Gazebo/ROS/YAML から完全に分離可能。
         * 基底 φ(s) は IBasisFunction として注入される(疎結合)。
         */
        class KrigedKalmanFilter {
        public:
            struct Prediction {
                double mean = -999.0;
                double variance = 1e6;
            };

            KrigedKalmanFilter(std::shared_ptr<IBasisFunction> basis, const KkfParams& params)
                : basis(std::move(basis)), params(params)
            {
                int p = this->basis->Dimension();
                this->alpha = (params.initial_state_mean.size() == p)
                                  ? params.initial_state_mean
                                  : Eigen::VectorXd::Zero(p);
                this->P = Eigen::MatrixXd::Identity(p, p) * params.initial_state_var;
            }

            /// 観測集合による1ステップ更新(時間更新+観測更新+残差登録)
            void Update(double t, const std::vector<Observation>& observations) {
                // --- 時間更新 (式(4)、Λ=I, Q=q·I·Δt) ---
                double dt = (this->last_update_t >= 0.0) ? std::max(0.0, t - this->last_update_t) : 0.0;
                this->last_update_t = t;
                int p = this->basis->Dimension();
                this->P += Eigen::MatrixXd::Identity(p, p) * (this->params.process_noise_q * dt);

                if (!observations.empty()) {
                    // --- 観測更新 (式(5),(6)) ---
                    const int n = static_cast<int>(observations.size());
                    Eigen::MatrixXd Phi(n, p);
                    Eigen::VectorXd z(n);
                    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(n, n);
                    for (int j = 0; j < n; ++j) {
                        Phi.row(j) = this->basis->Evaluate(observations[j].s).transpose();
                        z(j) = observations[j].z;
                        R(j, j) = observations[j].noise_var;
                    }
                    Eigen::MatrixXd S = Phi * this->P * Phi.transpose() + R;
                    Eigen::MatrixXd K = this->P * Phi.transpose() * S.ldlt().solve(Eigen::MatrixXd::Identity(n, n));
                    this->alpha += K * (z - Phi * this->alpha);
                    this->P = (Eigen::MatrixXd::Identity(p, p) - K * Phi) * this->P;

                    // --- 更新後残差の登録 (式(9)) ---
                    for (int j = 0; j < n; ++j) {
                        double residual = z(j) - Phi.row(j).dot(this->alpha);
                        this->residuals.push_back({observations[j].s, observations[j].t,
                                                   residual, observations[j].noise_var});
                    }
                    while (static_cast<int>(this->residuals.size()) > this->params.residual_buffer_size) {
                        this->residuals.pop_front();
                    }
                }

                this->RefreshKriging();
            }

            /// 任意の (s,t) における予測平均と予測分散 (式(7) + 基底成分の不確かさ)
            Prediction PredictAt(double s, double t) const {
                Eigen::VectorXd phi = this->basis->Evaluate(s);
                Prediction pred;
                pred.mean = phi.dot(this->alpha);
                pred.variance = phi.dot(this->P * phi) + this->params.sigma_nu * this->params.sigma_nu;

                const int n = static_cast<int>(this->residuals.size());
                if (n > 0) {
                    Eigen::VectorXd c0(n);
                    for (int j = 0; j < n; ++j) {
                        c0(j) = this->Covariance(s, t, this->residuals[j].s, this->residuals[j].t);
                    }
                    pred.mean += c0.dot(this->kriging_weights);
                    // 予測分散: σ_ν² - c0ᵀ C⁻¹ c0 (負値は数値誤差としてクランプ)
                    double reduction = c0.dot(this->C_llt.solve(c0));
                    pred.variance -= std::min(reduction, this->params.sigma_nu * this->params.sigma_nu);
                    pred.variance = std::max(pred.variance, 1e-6);
                }
                return pred;
            }

            /// 現在の残差バッファ長(診断用)
            int ObservationCount() const { return static_cast<int>(this->residuals.size()); }

        private:
            struct ResidualSample {
                double s;
                double t;
                double value;
                double noise_var;
            };

            /// 時空間分離型指数共分散 (式(8))
            double Covariance(double s1, double t1, double s2, double t2) const {
                return this->params.sigma_nu * this->params.sigma_nu *
                       std::exp(-std::abs(s1 - s2) / this->params.corr_length_s_m) *
                       std::exp(-std::abs(t1 - t2) / this->params.corr_length_t_s);
            }

            /// 残差共分散行列の分解と重み w = C⁻¹ν の再計算(更新毎に1回)
            void RefreshKriging() {
                const int n = static_cast<int>(this->residuals.size());
                if (n == 0) return;
                Eigen::MatrixXd C(n, n);
                Eigen::VectorXd nu(n);
                for (int j = 0; j < n; ++j) {
                    for (int k = 0; k < n; ++k) {
                        C(j, k) = this->Covariance(this->residuals[j].s, this->residuals[j].t,
                                                   this->residuals[k].s, this->residuals[k].t);
                    }
                    C(j, j) += this->residuals[j].noise_var;  // ナゲット(観測雑音)
                    nu(j) = this->residuals[j].value;
                }
                this->C_llt = C.ldlt();
                this->kriging_weights = this->C_llt.solve(nu);
            }

            std::shared_ptr<IBasisFunction> basis;
            KkfParams params;
            Eigen::VectorXd alpha;                       // 潜在状態 α̂
            Eigen::MatrixXd P;                           // 状態共分散 P
            double last_update_t = -1.0;
            std::deque<ResidualSample> residuals;        // 残差リングバッファ
            Eigen::LDLT<Eigen::MatrixXd> C_llt;          // 残差共分散の分解(クエリ用に保持)
            Eigen::VectorXd kriging_weights;             // w = C⁻¹ν
        };
    } // namespace kkf
} // namespace tx_controller
