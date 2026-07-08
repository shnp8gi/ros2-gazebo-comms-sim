#pragma once

#include <cmath>
#include <memory>
#include <algorithm>
#include <Eigen/Dense>
#include "comms_sim_pkg/kkf/RoadCoordinate.hpp"

namespace tx_controller
{
    namespace kkf
    {
        /**
         * IBasisFunction
         * --------------
         * 責務: 弧長座標 s から基底ベクトル φ(s) ∈ R^p への写像の定義のみ(文書 式(1))。
         * KKF本体は基底の具体形を知らず、この抽象を通してのみ φ(s) を評価する。
         * FRK(文書 7章)への拡張は本インターフェースの別実装として追加できる。
         */
        class IBasisFunction {
        public:
            virtual ~IBasisFunction() = default;
            virtual int Dimension() const = 0;
            virtual Eigen::VectorXd Evaluate(double s) const = 0;
        };

        /**
         * LogDistanceBasis
         * ----------------
         * 対数距離基底 φ(s) = [1, log10(max(d(s), d_min))]^T。
         * α = [基準受信レベル, 距離減衰の傾き] に対応し、文書 3.1節の
         * 「大域的傾向(基準受信レベル・距離減衰の傾き)」を表現する。
         * d(s) は経路上の位置と対象RSUアンテナ位置とのユークリッド距離。
         */
        class LogDistanceBasis : public IBasisFunction {
        public:
            LogDistanceBasis(const RoadCoordinate& road, const Eigen::Vector3d& rsu_position,
                             double min_distance_m = 1.0)
                : road(road), rsu_position(rsu_position), min_distance_m(min_distance_m) {}

            int Dimension() const override { return 2; }

            Eigen::VectorXd Evaluate(double s) const override {
                double d = (this->road.PositionAt(s) - this->rsu_position).norm();
                d = std::max(d, this->min_distance_m);
                Eigen::VectorXd phi(2);
                phi << 1.0, std::log10(d);
                return phi;
            }

        private:
            RoadCoordinate road;
            Eigen::Vector3d rsu_position;
            double min_distance_m;
        };
    } // namespace kkf
} // namespace tx_controller
