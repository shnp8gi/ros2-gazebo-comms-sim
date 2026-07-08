#pragma once

#include <string>
#include <vector>
#include <algorithm>
#include <memory>
#include <Eigen/Dense>

namespace comms_sim
{
    /// 遮蔽体の幾何表現: 有向バウンディングボックス (OBB) + 種別ごとの超過損失
    struct ObstacleBox {
        std::string name;
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();  // box→world
        Eigen::Vector3d half_extents = Eigen::Vector3d::Zero();
        double loss_db = 20.0;
    };

    /// 1リンク分の遮蔽評価結果
    struct BlockageResult {
        bool is_los = true;
        double excess_loss_db = 0.0;
        std::vector<std::string> blocker_names;
    };

    /**
     * IBlockageModel
     * --------------
     * 責務: リンク線分と遮蔽体群の幾何関係から LOS/NLOS と超過損失を評価することのみ。
     * 遮蔽体の位置取得(ECM連携)や損失の合成は呼び出し側の責務。
     * knife-edge 回折等の連続値モデルは本インターフェースの別実装として追加する
     * (戻り値 BlockageResult は連続値化しても不変)。
     */
    class IBlockageModel {
    public:
        virtual ~IBlockageModel() = default;
        virtual BlockageResult Evaluate(const Eigen::Vector3d& tx_pos,
                                        const Eigen::Vector3d& rx_pos,
                                        const std::vector<ObstacleBox>& obstacles) const = 0;
    };

    /**
     * BinaryObbBlockageModel
     * ----------------------
     * 二値 LOS/NLOS モデル: リンク線分が OBB と交差すれば NLOS とし、
     * 遮蔽体種別ごとの固定超過損失を加算する(60GHz では第1フレネル半径が
     * 数cmのため、線分交差判定は物理的に十分な近似)。
     * 複数遮蔽体の損失は加算し、max_total_loss_db でクリップする。
     */
    class BinaryObbBlockageModel : public IBlockageModel {
    public:
        explicit BinaryObbBlockageModel(double max_total_loss_db = 60.0)
            : max_total_loss_db(max_total_loss_db) {}

        BlockageResult Evaluate(const Eigen::Vector3d& tx_pos,
                                const Eigen::Vector3d& rx_pos,
                                const std::vector<ObstacleBox>& obstacles) const override
        {
            BlockageResult result;
            for (const auto& box : obstacles) {
                if (SegmentIntersectsObb(tx_pos, rx_pos, box)) {
                    result.is_los = false;
                    result.excess_loss_db += box.loss_db;
                    result.blocker_names.push_back(box.name);
                }
            }
            result.excess_loss_db = std::min(result.excess_loss_db, this->max_total_loss_db);
            return result;
        }

    private:
        /// slab法による線分×OBB交差判定(線分をbox座標系へ変換しAABB判定)
        static bool SegmentIntersectsObb(const Eigen::Vector3d& p_world0,
                                         const Eigen::Vector3d& p_world1,
                                         const ObstacleBox& box)
        {
            Eigen::Vector3d p0 = box.rotation.transpose() * (p_world0 - box.center);
            Eigen::Vector3d p1 = box.rotation.transpose() * (p_world1 - box.center);
            Eigen::Vector3d dir = p1 - p0;

            double t_min = 0.0;
            double t_max = 1.0;
            for (int axis = 0; axis < 3; ++axis) {
                double extent = box.half_extents[axis];
                if (std::abs(dir[axis]) < 1e-12) {
                    if (p0[axis] < -extent || p0[axis] > extent) return false;
                } else {
                    double t1 = (-extent - p0[axis]) / dir[axis];
                    double t2 = (extent - p0[axis]) / dir[axis];
                    if (t1 > t2) std::swap(t1, t2);
                    t_min = std::max(t_min, t1);
                    t_max = std::min(t_max, t2);
                    if (t_min > t_max) return false;
                }
            }
            return true;
        }

        double max_total_loss_db;
    };
} // namespace comms_sim
