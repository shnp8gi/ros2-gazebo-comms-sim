// ロータの回転による揺れ。羽を実際の速さで回す代わりに、回転数から計算した力を機体に加える。
//   1. アンバランス（1次成分）：回転面内で向きが回る力  F = m·e·ω² [cos θ, sin θ, 0]
//   2. 推力の脈動（翼通過成分）：ΔT = α·T·sin(N_b·θ + ψ)、T = motor_constant·ω²
// ω は MulticopterMotorModel が関節に与えている回転数（× 減速比）。θ は ω を積分して求めるので、
// 回転数が変われば揺れの周波数も変わる。<unbalance> や <thrust_ripple> を 0 にすると、その成分は無効。
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <sstream>

namespace comms_sim {
class RotorVibrationPlugin : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override {
    gz::sim::Model model(entity);
    link_ = gz::sim::Link(model.LinkByName(ecm, sdf->Get<std::string>("link_name", "base_link").first));
    unbalance_ = sdf->Get<double>("unbalance", 1e-6).first;
    ripple_ = sdf->Get<double>("thrust_ripple", 0.015).first;
    blades_ = sdf->Get<double>("blades", 2).first;
    motor_constant_ = sdf->Get<double>("motor_constant", 8.54858e-6).first;
    slowdown_ = sdf->Get<double>("rotor_velocity_slowdown", 10).first;
    std::istringstream positions(sdf->Get<std::string>("rotor_positions")), phases(sdf->Get<std::string>("phases"));
    for (int i = 0; i < 4; ++i) {
      double x, y, z;
      positions >> x >> y >> z;
      position_[i] = {x, y, z};
      joint_[i] = gz::sim::Joint(model.JointByName(ecm, "rotor_" + std::to_string(i) + "_joint"));
      if (joint_[i].Valid(ecm)) joint_[i].EnableVelocityCheck(ecm);
    }
    for (int i = 0; i < 4; ++i) phases >> angle_[i];      // φ_i：アンバランスの初期位相
    for (int i = 0; i < 4; ++i) phases >> ripple_phase_[i];  // ψ_i：推力脈動の位相
    if (!positions || !phases || !link_.Valid(ecm))
      std::cerr << "揺れ: 設定が不足しています（rotor_positions、phases、機体のリンク）" << std::endl;
    else ready_ = true;
  }
  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    if (info.paused || !ready_) return;
    const auto pose = link_.WorldPose(ecm);
    if (!pose) return;
    const double dt = std::chrono::duration<double>(info.dt).count();
    gz::math::Vector3d force, torque;  // 機体座標
    for (int i = 0; i < 4; ++i) {
      const auto velocity = joint_[i].Velocity(ecm);
      if (!velocity || velocity->empty()) continue;
      const double omega = (*velocity)[0] * slowdown_;  // 符号つき（回転方向）
      angle_[i] += omega * dt;
      const double square = omega * omega;
      const gz::math::Vector3d f(
        unbalance_ * square * std::cos(angle_[i]), unbalance_ * square * std::sin(angle_[i]),
        ripple_ * motor_constant_ * square * std::sin(blades_ * angle_[i] + ripple_phase_[i]));
      force += f; torque += position_[i].Cross(f);
    }
    link_.AddWorldWrench(ecm, pose->Rot().RotateVector(force), pose->Rot().RotateVector(torque));
  }
 private:
  gz::sim::Link link_{gz::sim::kNullEntity};
  std::array<gz::sim::Joint, 4> joint_;
  std::array<gz::math::Vector3d, 4> position_;
  std::array<double, 4> angle_{}, ripple_phase_{};
  double unbalance_ = 0, ripple_ = 0, blades_ = 2, motor_constant_ = 0, slowdown_ = 10;
  bool ready_ = false;
};
}  // namespace comms_sim
GZ_ADD_PLUGIN(comms_sim::RotorVibrationPlugin, gz::sim::System,
              comms_sim::RotorVibrationPlugin::ISystemConfigure,
              comms_sim::RotorVibrationPlugin::ISystemPreUpdate)
