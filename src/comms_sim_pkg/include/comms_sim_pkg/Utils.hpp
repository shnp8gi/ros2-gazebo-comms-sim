#pragma once

#include <string>
#include <vector>
#include <cmath>
#include <filesystem>
#include <Eigen/Dense>
#include "comms_sim_pkg/DataTypes.hpp"

namespace tx_controller
{
    namespace utils
    {
        inline Eigen::Matrix3d rpy_to_rotmat(double r, double p, double y) {
            Eigen::Matrix3d rx, ry, rz;
            rx << 1, 0, 0, 0, cos(r), -sin(r), 0, sin(r), cos(r);
            ry << cos(p), 0, sin(p), 0, 1, 0, -sin(p), 0, cos(p);
            rz << cos(y), -sin(y), 0, sin(y), cos(y), 0, 0, 0, 1;
            return rz * ry * rx;
        }

        inline std::string resolve_path(const std::string& raw_path) {
            if (raw_path.empty()) return raw_path;
            if (std::filesystem::exists(raw_path)) return raw_path;
            
            std::string path = raw_path;
            size_t pos = path.find("/workspace/config/");
            if (pos != std::string::npos) {
                std::string resolved = path;
                resolved.replace(pos, 18, "/workspace/src/comms_sim_pkg/config/");
                if (std::filesystem::exists(resolved)) {
                    return resolved;
                }
                resolved = path;
                resolved.replace(pos, 18, "/workspace/install/comms_sim_pkg/share/comms_sim_pkg/config/");
                if (std::filesystem::exists(resolved)) {
                    return resolved;
                }
            }
            
            pos = path.find("workspace/config/");
            if (pos != std::string::npos) {
                std::string resolved = path;
                resolved.replace(pos, 17, "workspace/src/comms_sim_pkg/config/");
                if (std::filesystem::exists(resolved)) {
                    return resolved;
                }
                resolved = path;
                resolved.replace(pos, 17, "workspace/install/comms_sim_pkg/share/comms_sim_pkg/config/");
                if (std::filesystem::exists(resolved)) {
                    return resolved;
                }
            }
            
            return raw_path;
        }

        inline std::vector<TrajectorySample> sample_trajectory(const std::vector<Eigen::Vector3d>& polyline_points, double resolution) {
            std::vector<TrajectorySample> samples;
            if (polyline_points.empty()) return samples;
            
            double dist_to_next = 0.0;
            for (size_t i = 0; i < polyline_points.size() - 1; ++i) {
                Eigen::Vector3d pt_a = polyline_points[i];
                Eigen::Vector3d pt_b = polyline_points[i+1];
                
                Eigen::Vector3d dir_vec = pt_b - pt_a;
                double length = dir_vec.norm();
                if (length < 1e-6) continue;
                
                Eigen::Vector3d unit_dir = dir_vec / length;
                double yaw = std::atan2(dir_vec.y(), dir_vec.x());
                
                double t = dist_to_next;
                while (t <= length) {
                    Eigen::Vector3d pt = pt_a + t * unit_dir;
                    samples.push_back({pt, yaw});
                    t += resolution;
                }
                dist_to_next = t - length;
            }
            return samples;
        }
    } // namespace utils
} // namespace tx_controller
