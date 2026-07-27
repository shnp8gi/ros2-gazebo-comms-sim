#pragma once

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Pose.hh>
#include "comms_sim_pkg/channel/BlockageModel.hpp"
#include "comms_sim_pkg/Utils.hpp"

namespace tx_controller
{
    /**
     * BlockageEnvironment
     * -------------------
     * 責務: 設定された遮蔽体エンティティの Gazebo 上の実 pose を ECM から取得し、
     * チャネルモデルが解釈できる ObstacleBox リストへ変換することのみ。
     * (幾何遮蔽の評価は comms_sim::IBlockageModel の責務)
     *
     * 設定形式 (sim_params.yaml, scenario_loader が生成):
     *   blockers:
     *     truck_1: { size: [12.0, 2.5, 3.8], loss_db: 30.0 }
     *     person_1: { size: [0.5, 0.5, 1.7], loss_db: 15.0 }
     * size はモデル全体の寸法 [m]。OBB 中心はモデル原点から
     * center_offset (省略時 [0, 0, size_z/2] = 原点が接地面中心の慣例) だけ
     * ずらした位置とする。
     */
    class BlockageEnvironment {
    public:
        void Configure(const YAML::Node& config) {
            this->entries.clear();
            if (!config["blockers"]) return;
            for (auto it : config["blockers"]) {
                BlockerEntry entry;
                entry.box.name = it.first.as<std::string>();
                auto blocker_cfg = it.second;
                auto size = blocker_cfg["size"].as<std::vector<double>>();
                entry.box.half_extents =
                    Eigen::Vector3d(size[0] / 2.0, size[1] / 2.0, size[2] / 2.0);
                entry.box.loss_db = blocker_cfg["loss_db"].as<double>(20.0);
                if (blocker_cfg["center_offset"]) {
                    auto off = blocker_cfg["center_offset"].as<std::vector<double>>();
                    entry.center_offset = Eigen::Vector3d(off[0], off[1], off[2]);
                } else {
                    entry.center_offset = Eigen::Vector3d(0.0, 0.0, size[2] / 2.0);
                }
                this->entries.push_back(entry);
            }
        }

        bool Empty() const { return this->entries.empty(); }

        /// 毎ステップ呼び出し: 遮蔽体の現在 pose を反映した ObstacleBox リストを更新
        ///
        /// @param exclude_name 除外するモデル名 (既定 "" = 除外なし)。
        ///   対象車自身も遮蔽体として登録される構成 (role: tx + blockage 属性、
        ///   都市部2車線仕様 §2.1) では、自車の OBB が自分のリンクを必ず遮って
        ///   しまう。呼び出し側 (TxControllerPlugin) が自分のモデル名を渡すことで
        ///   これを防ぐ。
        ///
        ///   除外を「障害物リストの構築」の責務としているのは、IBlockageModel や
        ///   ChannelModel に「リンクの所有者」を教えずに済ませるため
        ///   (両者は幾何しか知らないべきで、シグネチャも変えなくてよい)。
        ///   毎 tick のリストコピーも発生しない。
        void Refresh(gz::sim::EntityComponentManager& _ecm,
                     const std::string& exclude_name = "") {
            this->obstacles.clear();
            for (auto& entry : this->entries) {
                if (!exclude_name.empty() && entry.box.name == exclude_name) continue;
                if (entry.entity == gz::sim::kNullEntity) {
                    entry.entity = _ecm.EntityByComponents(
                        gz::sim::components::Name(entry.box.name),
                        gz::sim::components::Model());
                    if (entry.entity == gz::sim::kNullEntity) continue;  // 未スポーン
                }
                auto pose_comp = _ecm.Component<gz::sim::components::Pose>(entry.entity);
                if (!pose_comp) continue;

                gz::math::Pose3d pose = pose_comp->Data();
                Eigen::Matrix3d rot = utils::rpy_to_rotmat(
                    pose.Rot().Roll(), pose.Rot().Pitch(), pose.Rot().Yaw());
                Eigen::Vector3d origin(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

                entry.box.rotation = rot;
                entry.box.center = origin + rot * entry.center_offset;
                this->obstacles.push_back(entry.box);
            }
        }

        const std::vector<comms_sim::ObstacleBox>& Obstacles() const {
            return this->obstacles;
        }

    private:
        struct BlockerEntry {
            comms_sim::ObstacleBox box;
            Eigen::Vector3d center_offset = Eigen::Vector3d::Zero();
            gz::sim::Entity entity = gz::sim::kNullEntity;
        };

        std::vector<BlockerEntry> entries;
        std::vector<comms_sim::ObstacleBox> obstacles;
    };
} // namespace tx_controller
