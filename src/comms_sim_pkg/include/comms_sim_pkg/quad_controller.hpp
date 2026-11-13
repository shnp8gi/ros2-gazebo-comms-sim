#pragma once
// クアッド X 型の多段 PID 制御則。Gazebo にも ROS にも依存しない（Eigen のみ）。
//   目標の位置・速度 → [位置] → 目標速度 → [速度] → 目標の傾き・推力
//                   → [姿勢] → 目標の角速度 → [角速度] → [ミキサ] → ロータ回転数
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>

namespace comms_sim {

struct QuadParams {
  double mass = 1.5;                                   // 全備質量 [kg]
  Eigen::Vector3d inertia{0.03, 0.03, 0.05};           // [kg·m²]
  double arm = 0.25;                                   // 中心〜ロータ [m]
  double motor_constant = 8.54858e-6;                  // 推力 = motor_constant·ω² [N]
  double moment_constant = 0.016;                      // 反トルク = moment_constant·推力 [N·m]
  double max_rot_velocity = 1000;                      // [rad/s]
  double gravity = 9.81;
  // ゲイン。外側ほど遅くして、ループ同士の帯域を離す。
  double kp_position = 1.2;                            // [1/s]
  double kp_velocity = 3.0, ki_velocity = 1.0;         // [1/s], [1/s²]
  double kp_attitude = 10.0;                           // [1/s]
  double kp_rate = 40.0;                               // [1/s]
  double max_tilt_rad = 0.6;
  double max_velocity_integral = 3.0;                  // 積分項の上限 [m/s²]
};
struct QuadState {
  Eigen::Vector3d position = Eigen::Vector3d::Zero(), velocity = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();   // ワールド←機体
  Eigen::Vector3d body_rate = Eigen::Vector3d::Zero();      // 機体座標の角速度
};
struct QuadSetpoint {
  Eigen::Vector3d position = Eigen::Vector3d::Zero(), velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  double yaw = 0;
};

// ロータの配置（X3 と同じ並び）。0: 右前, 1: 左後（反時計回り）、2: 左前, 3: 右後（時計回り）。
struct QuadRotor { double x, y; int direction; };
inline std::array<QuadRotor, 4> quad_rotors(double arm) {
  const double l = arm / std::sqrt(2.0);
  return {{{l, -l, 1}, {-l, l, 1}, {l, l, -1}, {-l, -l, -1}}};
}

class QuadController {
 public:
  explicit QuadController(const QuadParams &params) : p_(params) {
    const auto rotors = quad_rotors(p_.arm);
    Eigen::Matrix4d a;
    for (int i = 0; i < 4; ++i) {
      a(0, i) = 1;                                               // 推力
      a(1, i) = rotors[i].y;                                     // ロール
      a(2, i) = -rotors[i].x;                                    // ピッチ
      a(3, i) = -rotors[i].direction * p_.moment_constant;       // ヨー（反トルク）
    }
    mixer_ = a.inverse();
    thrust_ = p_.mass * p_.gravity;
  }
  // 位置・速度ループ。目標の推力ベクトルと姿勢を決める。
  void UpdateOuter(const QuadState &s, const QuadSetpoint &sp, double dt) {
    const Eigen::Vector3d velocity_cmd = sp.velocity + p_.kp_position * (sp.position - s.position);
    const Eigen::Vector3d velocity_error = velocity_cmd - s.velocity;
    integral_ += p_.ki_velocity * velocity_error * dt;
    for (int i = 0; i < 3; ++i)
      integral_[i] = std::clamp(integral_[i], -p_.max_velocity_integral, p_.max_velocity_integral);
    Eigen::Vector3d accel = sp.acceleration + p_.kp_velocity * velocity_error + integral_
      + Eigen::Vector3d(0, 0, p_.gravity);
    // 傾きの上限：水平成分を鉛直成分に対して制限する。
    accel.z() = std::max(accel.z(), 0.2 * p_.gravity);
    const double horizontal = accel.head<2>().norm(), limit = accel.z() * std::tan(p_.max_tilt_rad);
    if (horizontal > limit) accel.head<2>() *= limit / horizontal;
    force_ = p_.mass * accel;
    const Eigen::Vector3d z = force_.normalized();
    const Eigen::Vector3d heading(std::cos(sp.yaw), std::sin(sp.yaw), 0);
    const Eigen::Vector3d y = z.cross(heading).normalized();
    desired_.col(0) = y.cross(z); desired_.col(1) = y; desired_.col(2) = z;
  }
  // 姿勢・角速度ループとミキサ。ロータ回転数の指令 [rad/s] を返す。
  Eigen::Vector4d UpdateInner(const QuadState &s) {
    thrust_ = std::max(0.0, force_.dot(s.rotation.col(2)));
    const Eigen::Matrix3d e = desired_.transpose() * s.rotation - s.rotation.transpose() * desired_;
    const Eigen::Vector3d attitude_error = 0.5 * Eigen::Vector3d(e(2, 1), e(0, 2), e(1, 0));
    const Eigen::Vector3d rate_cmd = -p_.kp_attitude * attitude_error;
    const Eigen::Vector3d inertia_rate = p_.inertia.cwiseProduct(s.body_rate);
    const Eigen::Vector3d torque = p_.inertia.cwiseProduct(p_.kp_rate * (rate_cmd - s.body_rate))
      + s.body_rate.cross(inertia_rate);
    const Eigen::Vector4d forces = mixer_ * Eigen::Vector4d(thrust_, torque.x(), torque.y(), torque.z());
    const double max_force = p_.motor_constant * p_.max_rot_velocity * p_.max_rot_velocity;
    Eigen::Vector4d speeds;
    for (int i = 0; i < 4; ++i)
      speeds[i] = std::sqrt(std::clamp(forces[i], 0.0, max_force) / p_.motor_constant);
    return speeds;
  }
  const Eigen::Matrix3d &DesiredRotation() const { return desired_; }
 private:
  QuadParams p_;
  Eigen::Matrix4d mixer_;
  Eigen::Vector3d integral_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d force_{0, 0, 1.5 * 9.81};
  Eigen::Matrix3d desired_ = Eigen::Matrix3d::Identity();
  double thrust_ = 0;
};

// 直線の経路。start_time まで停止し、一定の加速度で cruise_speed まで加速して等速で飛ぶ。
struct LinePath {
  Eigen::Vector3d start = Eigen::Vector3d::Zero(), direction = Eigen::Vector3d::UnitX();
  double cruise_speed = 2, acceleration = 1, start_time = 1, yaw = 0;
  // 時刻 t までに進む距離
  double Distance(double t) const {
    const double tau = t - start_time, ramp = cruise_speed / acceleration;
    if (tau <= 0) return 0;
    if (tau < ramp) return 0.5 * acceleration * tau * tau;
    return cruise_speed * (tau - 0.5 * ramp);
  }
  QuadSetpoint At(double t) const {
    const double tau = t - start_time, ramp = cruise_speed / acceleration;
    const double speed = tau <= 0 ? 0 : tau < ramp ? acceleration * tau : cruise_speed;
    const double accel = tau > 0 && tau < ramp ? acceleration : 0;
    return {start + direction * Distance(t), direction * speed, direction * accel, yaw};
  }
};
}  // namespace comms_sim
