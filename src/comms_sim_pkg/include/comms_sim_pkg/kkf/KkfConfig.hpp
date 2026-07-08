#pragma once

#include <yaml-cpp/yaml.h>
#include "comms_sim_pkg/kkf/KrigedKalmanFilter.hpp"

namespace tx_controller
{
    namespace kkf
    {
        /**
         * KkfConfig
         * ---------
         * 責務: kkf_predictive ポリシーの全設定値の保持と、YAMLからの読取のみ。
         * yaml-cpp への依存をこのモジュールに隔離し、統計・計画モジュールを
         * 設定ファイル形式から独立させる。
         */
        struct KkfConfig {
            // --- 実行周期 ---
            double measurement_period_s = 0.05;  // 測定レポート周期 (KKF更新周期)
            double replan_period_s = 0.2;        // 再計画周期 (MPC, 文書 5.3節)

            // --- 第3層: プランナ (文書 5章) ---
            double horizon_s = 6.0;              // 計画ホライズン [s]
            double plan_dt_s = 0.3;              // 計画ステージ幅 Δt [s]
            double kappa = 1.64;                 // 安全係数 κ (式(12))
            double switch_cost = 5.0;            // 切替コスト λ (式(13)) [dB·s相当]

            // --- 観測モデル (文書 6.1節) ---
            bool observe_all_pairs = true;       // false = P2P制約に忠実 (grant中ペアのみ観測)
            double meas_noise_std_db = 2.0;      // 観測雑音の標準偏差 [dB]
            unsigned seed = 42;                  // 観測雑音の乱数シード

            // --- 車両運動推定 ---
            double velocity_ema_beta = 0.7;      // 速度推定のEMA係数 (0=瞬時値, 1=更新なし)

            // --- 第1層: KKF統計パラメータ (文書 3章) ---
            KkfParams kkf;

            static KkfConfig FromYaml(const YAML::Node& params) {
                KkfConfig c;
                c.measurement_period_s = params["kkf_measurement_period_s"].as<double>(c.measurement_period_s);
                c.replan_period_s = params["kkf_replan_period_s"].as<double>(c.replan_period_s);
                c.horizon_s = params["kkf_horizon_s"].as<double>(c.horizon_s);
                c.plan_dt_s = params["kkf_plan_dt_s"].as<double>(c.plan_dt_s);
                c.kappa = params["kkf_kappa"].as<double>(c.kappa);
                c.switch_cost = params["kkf_switch_cost"].as<double>(c.switch_cost);
                c.observe_all_pairs = params["kkf_observe_all_pairs"].as<bool>(c.observe_all_pairs);
                c.meas_noise_std_db = params["kkf_meas_noise_std_db"].as<double>(c.meas_noise_std_db);
                c.seed = params["kkf_seed"].as<unsigned>(c.seed);
                c.velocity_ema_beta = params["kkf_velocity_ema_beta"].as<double>(c.velocity_ema_beta);
                c.kkf.process_noise_q = params["kkf_process_noise_q"].as<double>(c.kkf.process_noise_q);
                c.kkf.initial_state_var = params["kkf_initial_state_var"].as<double>(c.kkf.initial_state_var);
                c.kkf.sigma_nu = params["kkf_sigma_nu_db"].as<double>(c.kkf.sigma_nu);
                c.kkf.corr_length_s_m = params["kkf_corr_length_s_m"].as<double>(c.kkf.corr_length_s_m);
                c.kkf.corr_length_t_s = params["kkf_corr_length_t_s"].as<double>(c.kkf.corr_length_t_s);
                c.kkf.residual_buffer_size = params["kkf_residual_buffer_size"].as<int>(c.kkf.residual_buffer_size);
                return c;
            }
        };
    } // namespace kkf
} // namespace tx_controller
