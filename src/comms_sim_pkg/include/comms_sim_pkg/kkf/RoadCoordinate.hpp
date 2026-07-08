#pragma once

#include <vector>
#include <cmath>
#include <Eigen/Dense>

namespace tx_controller
{
    namespace kkf
    {
        /**
         * RoadCoordinate
         * --------------
         * 責務: 既知の走行経路(折れ線)に対する「世界座標 ⇔ 弧長座標 s」の相互変換のみ。
         * 通信・スケジューリングの知識を持たない純粋な幾何モジュール。
         *
         * 鉄道車両では走行経路が事前に既知であるため、経路の形状情報を
         * 予測座標系として利用することは物理的に正当である(速度・時刻は含まない)。
         */
        class RoadCoordinate {
        public:
            RoadCoordinate() = default;

            explicit RoadCoordinate(const std::vector<Eigen::Vector3d>& polyline) {
                this->points = polyline;
                this->cum_lengths.clear();
                double acc = 0.0;
                this->cum_lengths.push_back(0.0);
                for (size_t i = 1; i < this->points.size(); ++i) {
                    acc += (this->points[i] - this->points[i - 1]).norm();
                    this->cum_lengths.push_back(acc);
                }
            }

            bool IsValid() const { return this->points.size() >= 2; }

            double TotalLength() const {
                return this->cum_lengths.empty() ? 0.0 : this->cum_lengths.back();
            }

            /// 世界座標を経路上へ射影し弧長 s を返す(経路外は最近傍セグメントへ射影)
            double Project(const Eigen::Vector3d& world_pos) const {
                double best_s = 0.0;
                double best_dist_sq = std::numeric_limits<double>::infinity();
                for (size_t i = 0; i + 1 < this->points.size(); ++i) {
                    const Eigen::Vector3d& a = this->points[i];
                    const Eigen::Vector3d& b = this->points[i + 1];
                    Eigen::Vector3d ab = b - a;
                    double len_sq = ab.squaredNorm();
                    double t = (len_sq > 1e-12) ? (world_pos - a).dot(ab) / len_sq : 0.0;
                    t = std::max(0.0, std::min(1.0, t));
                    Eigen::Vector3d proj = a + t * ab;
                    double dist_sq = (world_pos - proj).squaredNorm();
                    if (dist_sq < best_dist_sq) {
                        best_dist_sq = dist_sq;
                        best_s = this->cum_lengths[i] + t * std::sqrt(len_sq);
                    }
                }
                return best_s;
            }

            /// 弧長 s に対応する経路上の世界座標を返す(端点外は端点にクランプ)
            Eigen::Vector3d PositionAt(double s) const {
                if (this->points.empty()) return Eigen::Vector3d::Zero();
                if (s <= 0.0) return this->points.front();
                if (s >= this->TotalLength()) return this->points.back();
                for (size_t i = 0; i + 1 < this->points.size(); ++i) {
                    if (s <= this->cum_lengths[i + 1]) {
                        double seg_len = this->cum_lengths[i + 1] - this->cum_lengths[i];
                        double t = (seg_len > 1e-12) ? (s - this->cum_lengths[i]) / seg_len : 0.0;
                        return this->points[i] + t * (this->points[i + 1] - this->points[i]);
                    }
                }
                return this->points.back();
            }

        private:
            std::vector<Eigen::Vector3d> points;
            std::vector<double> cum_lengths;
        };
    } // namespace kkf
} // namespace tx_controller
