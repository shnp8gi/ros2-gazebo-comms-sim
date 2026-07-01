#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include "comms_sim_pkg/DataTypes.hpp"

namespace tx_controller
{
    struct MotionCommand {
        double linear_velocity;
        double angular_velocity;
    };

    struct MotionState {
        double x;
        double y;
        double yaw;
    };

    class VehicleMotionController {
    public:
        VehicleMotionController() = default;

        void Configure(const std::vector<Waypoint>& waypoints_in,
                       double waypoint_tolerance_in,
                       double max_angular_velocity_in,
                       double heading_gain_in,
                       double max_acceleration_in,
                       bool is_shinkansen_in) 
        {
            this->waypoints = waypoints_in;
            this->waypoint_tolerance = waypoint_tolerance_in;
            this->max_angular_velocity = max_angular_velocity_in;
            this->heading_gain = heading_gain_in;
            this->max_acceleration = max_acceleration_in;
            this->is_shinkansen = is_shinkansen_in;
            this->current_waypoint_idx = 0;
            this->current_v = 0.0;
            this->mission_complete = false;

            this->total_path_distance = 0.0;
            for (size_t i = 1; i < this->waypoints.size(); ++i) {
                double dx = this->waypoints[i].x - this->waypoints[i-1].x;
                double dy = this->waypoints[i].y - this->waypoints[i-1].y;
                this->total_path_distance += std::sqrt(dx*dx + dy*dy);
            }
        }

        MotionCommand CalculateCommand(const MotionState& current_state, double dt) {
            MotionCommand cmd{0.0, 0.0};

            if (this->mission_complete) {
                this->current_v = 0.0;
                cmd.linear_velocity = 0.0;
                cmd.angular_velocity = 0.0;
                return cmd;
            }

            if (this->current_waypoint_idx >= this->waypoints.size() || this->waypoints.empty()) {
                this->mission_complete = true;
                this->current_v = 0.0;
                cmd.linear_velocity = 0.0;
                cmd.angular_velocity = 0.0;
                return cmd;
            }

            auto target = this->waypoints[this->current_waypoint_idx];
            double dx = target.x - current_state.x;
            double dy = target.y - current_state.y;
            double distance = std::sqrt(dx*dx + dy*dy);
            double target_heading = std::atan2(dy, dx);

            // Check waypoint arrival
            if (distance < this->waypoint_tolerance) {
                this->current_waypoint_idx++;
                // Returning current velocity to maintain momentum, direction will correct next tick
                cmd.linear_velocity = this->current_v;
                cmd.angular_velocity = 0.0;
                return cmd;
            }

            // Compute angular velocity
            double heading_error = target_heading - current_state.yaw;
            while (heading_error > M_PI) heading_error -= 2 * M_PI;
            while (heading_error < -M_PI) heading_error += 2 * M_PI;

            if (this->is_shinkansen) heading_error = 0.0;

            double w = this->heading_gain * heading_error;
            w = std::max(-this->max_angular_velocity, std::min(this->max_angular_velocity, w));
            if (this->is_shinkansen) w = 0.0;

            // Compute linear velocity
            double turn_factor = 1.0 - std::min(1.0, std::abs(heading_error) / (M_PI / 2.0));
            double target_v = target.v * std::max(0.3, turn_factor);

            if (dt > 0) {
                double accel_step = this->max_acceleration * dt;
                if (target_v > this->current_v) {
                    this->current_v = std::min(this->current_v + accel_step, target_v);
                } else {
                    this->current_v = std::max(this->current_v - accel_step, target_v);
                }
            }

            cmd.linear_velocity = this->current_v;
            cmd.angular_velocity = w;
            return cmd;
        }

        bool IsMissionComplete() const {
            return this->mission_complete;
        }

        size_t GetCurrentWaypointIndex() const {
            return this->current_waypoint_idx;
        }

        double GetTotalPathDistance() const {
            return this->total_path_distance;
        }

        const std::vector<Waypoint>& GetWaypoints() const {
            return this->waypoints;
        }

    private:
        std::vector<Waypoint> waypoints;
        double waypoint_tolerance = 1.0;
        double max_angular_velocity = 1.0;
        double heading_gain = 2.0;
        double max_acceleration = 1.0;
        bool is_shinkansen = false;

        size_t current_waypoint_idx = 0;
        double current_v = 0.0;
        bool mission_complete = false;
        double total_path_distance = 0.0;
    };
} // namespace tx_controller
