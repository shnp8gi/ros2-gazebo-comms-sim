// 多段 PID の制御器を Gazebo と同じプロセスで動かす。
// 物理の刻みに同期して動くので、実時間係数に上限がなくても結果が再現する。
// 外とのやり取り（拡張仕様書 §5.2）：
//   /<機体>/state     gz.msgs.Odometry   位置・速度・姿勢・角速度（真値）
//   /<機体>/setpoint  gz.msgs.Odometry   目標の位置・速度・ヨー
//   /<機体>/rotor_cmd gz.msgs.Actuators  ロータ回転数の指令。MulticopterMotorModel が受け取る。
// <external_control>true</external_control> にすると指令を書かず、外部の制御器に任せる。
#include "comms_sim_pkg/quad_controller.hpp"
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Actuators.hh>
#include <gz/transport/Node.hh>
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

namespace comms_sim {
class QuadControllerPlugin : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override {
    model_ = gz::sim::Model(entity);
    name_ = model_.Name(ecm);
    auto number = [&](const char *key, double fallback) { return sdf->Get<double>(key, fallback).first; };
    QuadParams p;
    p.mass = number("mass", p.mass); p.arm = number("arm", p.arm);
    p.inertia = {number("ixx", p.inertia.x()), number("iyy", p.inertia.y()), number("izz", p.inertia.z())};
    p.motor_constant = number("motor_constant", p.motor_constant);
    p.moment_constant = number("moment_constant", p.moment_constant);
    p.max_rot_velocity = number("max_rot_velocity", p.max_rot_velocity);
    p.kp_position = number("kp_position", p.kp_position);
    p.kp_velocity = number("kp_velocity", p.kp_velocity); p.ki_velocity = number("ki_velocity", p.ki_velocity);
    p.kp_attitude = number("kp_attitude", p.kp_attitude); p.kp_rate = number("kp_rate", p.kp_rate);
    p.max_tilt_rad = number("max_tilt_deg", p.max_tilt_rad * 180 / M_PI) * M_PI / 180;
    controller_ = std::make_unique<QuadController>(p);
    const auto start = sdf->Get<gz::math::Vector3d>("line_start", gz::math::Vector3d::Zero).first;
    const auto direction = sdf->Get<gz::math::Vector3d>("line_direction", gz::math::Vector3d::UnitX).first;
    path_.start = {start.X(), start.Y(), start.Z()};
    path_.direction = Eigen::Vector3d(direction.X(), direction.Y(), direction.Z()).normalized();
    path_.cruise_speed = number("cruise_speed", path_.cruise_speed);
    path_.acceleration = number("acceleration", path_.acceleration);
    path_.start_time = number("start_time", path_.start_time);
    path_.yaw = number("yaw", std::atan2(path_.direction.y(), path_.direction.x()));
    inner_steps_ = std::max(1, sdf->Get<int>("inner_steps", 10).first);    // 姿勢・角速度：1 kHz
    outer_steps_ = std::max(1, sdf->Get<int>("outer_steps", 100).first);   // 位置・速度：100 Hz
    external_ = sdf->Get<bool>("external_control", false).first;
    rate_history_.assign(std::max(1, sdf->Get<int>("rate_window", 10).first), Eigen::Vector3d::Zero());
    link_ = gz::sim::Link(model_.LinkByName(ecm, sdf->Get<std::string>("link_name", "base_link").first));
    if (!link_.Valid(ecm)) { std::cerr << "制御器: 機体のリンクがありません: " << name_ << std::endl; return; }
    link_.EnableVelocityChecks(ecm);
    state_pub_ = node_.Advertise<gz::msgs::Odometry>("/" + name_ + "/state");
    setpoint_pub_ = node_.Advertise<gz::msgs::Odometry>("/" + name_ + "/setpoint");
    hover_ = std::sqrt(p.mass * p.gravity / 4 / p.motor_constant);
    command_ = Eigen::Vector4d::Constant(hover_);
  }
  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    if (info.paused || !controller_ || !link_.Valid(ecm)) return;
    const double time = std::chrono::duration<double>(info.simTime).count();
    const auto pose = link_.WorldPose(ecm);
    const auto velocity = link_.WorldLinearVelocity(ecm);
    const auto rate = link_.WorldAngularVelocity(ecm);
    if (pose && velocity && rate) {
      QuadState s;
      s.position = {pose->Pos().X(), pose->Pos().Y(), pose->Pos().Z()};
      s.velocity = {velocity->X(), velocity->Y(), velocity->Z()};
      const auto &q = pose->Rot();
      s.rotation = Eigen::Quaterniond(q.W(), q.X(), q.Y(), q.Z()).toRotationMatrix();
      s.body_rate = s.rotation.transpose() * Eigen::Vector3d(rate->X(), rate->Y(), rate->Z());
      const auto setpoint = path_.At(time);
      // 各ループには、その周期の間の平均を渡す。ロータの揺れ（約 100 Hz 以上）が
      // 制御周期で折り返して低い周波数の動きに化けるのを防ぐ（揺れは制御で打ち消さない）。
      outer_position_ += s.position; outer_velocity_ += s.velocity; ++outer_count_;
      inner_rate_ += s.body_rate; ++inner_count_;
      if (step_ % outer_steps_ == 0) {
        const double dt = std::chrono::duration<double>(info.dt).count() * outer_steps_;
        QuadState mean = s;
        mean.position = outer_position_ / outer_count_; mean.velocity = outer_velocity_ / outer_count_;
        // 平均は周期の中央の値なので、目標も同じだけ前の時刻のものと比べる。
        const double lag = 0.5 * (outer_count_ - 1) * std::chrono::duration<double>(info.dt).count();
        auto reference = path_.At(time - lag);
        reference.acceleration = setpoint.acceleration;
        controller_->UpdateOuter(mean, reference, dt);
        Publish(time, s, setpoint);
        outer_position_.setZero(); outer_velocity_.setZero(); outer_count_ = 0;
      }
      if (step_ % inner_steps_ == 0) {
        // 角速度は直近 rate_window_ 回分（既定 10 ms = ロータ約1回転）の移動平均を使う。
        rate_history_[rate_index_++ % rate_history_.size()] = inner_rate_ / inner_count_;
        const size_t filled = std::min<size_t>(rate_index_, rate_history_.size());
        QuadState mean = s;
        mean.body_rate.setZero();
        for (size_t i = 0; i < filled; ++i) mean.body_rate += rate_history_[i];
        mean.body_rate /= filled;
        command_ = controller_->UpdateInner(mean);
        inner_rate_.setZero(); inner_count_ = 0;
      }
    }
    ++step_;
    if (external_) return;
    gz::msgs::Actuators msg;
    for (int i = 0; i < 4; ++i) msg.add_velocity(command_[i]);
    // コンポーネントに直接書く。トピック経由より優先され、同じステップで反映される。
    if (ecm.Component<gz::sim::components::Actuators>(model_.Entity()))
      ecm.SetComponentData<gz::sim::components::Actuators>(model_.Entity(), msg);
    else
      ecm.CreateComponent(model_.Entity(), gz::sim::components::Actuators(msg));
  }
 private:
  void Publish(double time, const QuadState &s, const QuadSetpoint &sp) {
    auto fill = [&](gz::msgs::Odometry &msg, const Eigen::Vector3d &p, const Eigen::Vector3d &v,
                    const Eigen::Quaterniond &q, const Eigen::Vector3d &w) {
      auto *stamp = msg.mutable_header()->mutable_stamp();
      stamp->set_sec(static_cast<int64_t>(time));
      stamp->set_nsec(static_cast<int32_t>((time - static_cast<int64_t>(time)) * 1e9));
      auto *pose = msg.mutable_pose();
      pose->mutable_position()->set_x(p.x()); pose->mutable_position()->set_y(p.y());
      pose->mutable_position()->set_z(p.z());
      pose->mutable_orientation()->set_w(q.w()); pose->mutable_orientation()->set_x(q.x());
      pose->mutable_orientation()->set_y(q.y()); pose->mutable_orientation()->set_z(q.z());
      auto *twist = msg.mutable_twist();
      twist->mutable_linear()->set_x(v.x()); twist->mutable_linear()->set_y(v.y());
      twist->mutable_linear()->set_z(v.z());
      twist->mutable_angular()->set_x(w.x()); twist->mutable_angular()->set_y(w.y());
      twist->mutable_angular()->set_z(w.z());
    };
    gz::msgs::Odometry state, setpoint;
    fill(state, s.position, s.velocity, Eigen::Quaterniond(s.rotation), s.body_rate);
    fill(setpoint, sp.position, sp.velocity,
         Eigen::Quaterniond(Eigen::AngleAxisd(sp.yaw, Eigen::Vector3d::UnitZ())), Eigen::Vector3d::Zero());
    state_pub_.Publish(state); setpoint_pub_.Publish(setpoint);
  }
  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Link link_{gz::sim::kNullEntity};
  std::string name_;
  std::unique_ptr<QuadController> controller_;
  LinePath path_;
  Eigen::Vector4d command_ = Eigen::Vector4d::Zero();
  Eigen::Vector3d outer_position_ = Eigen::Vector3d::Zero(), outer_velocity_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d inner_rate_ = Eigen::Vector3d::Zero();
  int outer_count_ = 0, inner_count_ = 0;
  std::vector<Eigen::Vector3d> rate_history_;
  size_t rate_index_ = 0;
  double hover_ = 0;
  int inner_steps_ = 10, outer_steps_ = 100;
  uint64_t step_ = 0;
  bool external_ = false;
  gz::transport::Node node_;
  gz::transport::Node::Publisher state_pub_, setpoint_pub_;
};
}  // namespace comms_sim
GZ_ADD_PLUGIN(comms_sim::QuadControllerPlugin, gz::sim::System,
              comms_sim::QuadControllerPlugin::ISystemConfigure,
              comms_sim::QuadControllerPlugin::ISystemPreUpdate)
