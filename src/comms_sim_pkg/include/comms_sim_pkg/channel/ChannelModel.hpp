#pragma once

#include <memory>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include "comms_sim_pkg/comms_calculator.hpp"
#include "comms_sim_pkg/channel/BlockageModel.hpp"
#include "comms_sim_pkg/channel/ShadowingModel.hpp"
#include "comms_sim_pkg/channel/FadingModel.hpp"

namespace comms_sim
{
    /// 1リンク・1時刻のチャネル評価結果(成分別に保持し、分析ログに供する)
    struct ChannelSample {
        double path_loss_db = 0.0;
        bool is_los = true;
        double blockage_loss_db = 0.0;
        double shadow_db = 0.0;        // ゼロ平均・符号付き (正=追加損失)
        double fading_loss_db = 0.0;

        double TotalLossDb() const {
            return path_loss_db + blockage_loss_db + shadow_db + fading_loss_db;
        }
    };

    /**
     * ChannelModel
     * ------------
     * 責務: 距離減衰・幾何遮蔽・シャドウイング・フェージングの「合成」のみ。
     *   RSSI = TxPower + G_ant − PL(d) − L_blockage − X_shadow − L_fading
     * 各成分の計算は注入されたモデルに委譲する。blockage/shadowing/fading は
     * nullptr 可(その成分は 0 として扱う)であり、距離減衰のみの旧構成と
     * 完全な後方互換を持つ。
     */
    class ChannelModel {
    public:
        ChannelModel(std::unique_ptr<PropagationModel> path_loss_model,
                     std::unique_ptr<IBlockageModel> blockage_model,
                     std::unique_ptr<IShadowingModel> shadowing_model,
                     std::unique_ptr<IFadingModel> fading_model)
            : path_loss_model(std::move(path_loss_model)),
              blockage_model(std::move(blockage_model)),
              shadowing_model(std::move(shadowing_model)),
              fading_model(std::move(fading_model)) {}

        ChannelSample Evaluate(int link_id, double t,
                               const Eigen::Vector3d& tx_pos,
                               const Eigen::Vector3d& rx_pos,
                               const std::vector<ObstacleBox>& obstacles)
        {
            ChannelSample sample;
            sample.path_loss_db = this->path_loss_model->calculate_path_loss(
                (tx_pos - rx_pos).norm());

            if (this->blockage_model && !obstacles.empty()) {
                auto blockage = this->blockage_model->Evaluate(tx_pos, rx_pos, obstacles);
                sample.is_los = blockage.is_los;
                sample.blockage_loss_db = blockage.excess_loss_db;
            }
            if (this->shadowing_model) {
                sample.shadow_db = this->shadowing_model->SampleDb(link_id, tx_pos, rx_pos);
            }
            if (this->fading_model) {
                sample.fading_loss_db = this->fading_model->SampleLossDb(link_id, sample.is_los, t);
            }
            return sample;
        }

        const PropagationModel& GetPathLossModel() const { return *this->path_loss_model; }

    private:
        std::unique_ptr<PropagationModel> path_loss_model;
        std::unique_ptr<IBlockageModel> blockage_model;
        std::unique_ptr<IShadowingModel> shadowing_model;
        std::unique_ptr<IFadingModel> fading_model;
    };

    /**
     * ChannelModelFactory
     * -------------------
     * 責務: yaml 設定からの ChannelModel 組立のみ(yaml-cpp 依存の隔離)。
     *
     * comms_simulator_node.ros__parameters 直下の `channel:` セクションを読む:
     *   channel:
     *     blockage:  { enabled: true, max_total_loss_db: 60.0 }
     *     shadowing: { enabled: true, sigma_db: 4.0, corr_distance_m: 10.0 }
     *     fading:    { enabled: true, k_los_db: 10.0, k_nlos_db: -100.0,
     *                  coherence_time_s: 0.05 }
     *     seed: 42
     * セクションが無い場合は距離減衰のみ(旧構成と同一の振る舞い)。
     *
     * 持続シャドウ (走行間固定、本番仕様 §4.2) は type: frozen で選択する:
     *   shadowing: { enabled: true, type: frozen, sigma_db: 4.0,
     *                corr_length_m: 6.0, environment_seed: 1,
     *                grid_m: 1.5, axis: x, u_min: -1000.0, u_max: 1000.0 }
     * environment_seed のみが場を決め、run 毎に注入される channel.seed は
     * 参照しない (学習相・評価相で環境が固定される)。
     */
    class ChannelModelFactory {
    public:
        static std::unique_ptr<ChannelModel> Create(const YAML::Node& comms_params) {
            auto path_loss = PropagationModelFactory::Create(comms_params["path_loss"]);

            std::unique_ptr<IBlockageModel> blockage;
            std::unique_ptr<IShadowingModel> shadowing;
            std::unique_ptr<IFadingModel> fading;

            YAML::Node channel = comms_params["channel"];
            if (channel) {
                unsigned seed = channel["seed"].as<unsigned>(42);

                YAML::Node b = channel["blockage"];
                if (b && b["enabled"].as<bool>(true)) {
                    blockage = std::make_unique<BinaryObbBlockageModel>(
                        b["max_total_loss_db"].as<double>(60.0));
                }
                YAML::Node s = channel["shadowing"];
                if (s && s["enabled"].as<bool>(true)) {
                    std::string type = s["type"].as<std::string>("gudmundson");
                    if (type == "frozen") {
                        std::string axis_name = s["axis"].as<std::string>("x");
                        int axis = (axis_name == "y") ? 1 : (axis_name == "z") ? 2 : 0;
                        shadowing = std::make_unique<FrozenFieldShadowingModel>(
                            s["sigma_db"].as<double>(4.0),
                            s["corr_length_m"].as<double>(6.0),
                            s["environment_seed"].as<unsigned>(1),
                            s["grid_m"].as<double>(1.5),
                            axis,
                            s["u_min"].as<double>(-1000.0),
                            s["u_max"].as<double>(1000.0));
                    } else {
                        shadowing = std::make_unique<GudmundsonShadowingModel>(
                            s["sigma_db"].as<double>(4.0),
                            s["corr_distance_m"].as<double>(10.0),
                            seed + 1);
                    }
                }
                YAML::Node f = channel["fading"];
                if (f && f["enabled"].as<bool>(true)) {
                    fading = std::make_unique<RicianFadingModel>(
                        f["k_los_db"].as<double>(10.0),
                        f["k_nlos_db"].as<double>(-100.0),
                        f["coherence_time_s"].as<double>(0.05),
                        seed + 2);
                }
            } else {
                // channel: 未指定でも遮蔽体が存在すれば幾何遮蔽は物理として働かせる
                blockage = std::make_unique<BinaryObbBlockageModel>();
            }

            return std::make_unique<ChannelModel>(
                std::move(path_loss), std::move(blockage),
                std::move(shadowing), std::move(fading));
        }
    };
} // namespace comms_sim
