#include "comms_sim_pkg/drone_link.hpp"
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Events.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/pose.pb.h>
#include <array>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace comms_sim {
// 空力・姿勢安定化は扱わず、2機の指定軌道とアンテナ姿勢を再現する。
class DroneEncounterPlugin : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate {
 public:
  void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &events) override {
    events_ = &events;
    world_ = entity;
    pause_at_encounter_ = sdf->Get<bool>("pause_at_encounter", false).first;
    try {
      run_ = std::make_unique<DroneRun>(ScenarioConfig::Load(sdf->Get<std::string>("config_file")));
      poses_.open(run_->Config().output_dir + "/gazebo_poses.csv");
      if (!poses_) throw std::runtime_error("Gazebo位置ログを開けません");
      poses_ << std::fixed << std::setprecision(9)
        << "time_s,model,x_m,y_m,z_m,roll_rad,pitch_rad,yaw_rad\n";
      pubs_[0] = node_.Advertise<gz::msgs::Pose>("/drone_a/pose");
      pubs_[1] = node_.Advertise<gz::msgs::Pose>("/drone_b/pose");
      std::cout << "ドローン実験開始: " << run_->Config().scenario << " / "
                << run_->Config().antenna_mode << std::endl;
    } catch (const std::exception &e) { Fail(e.what()); }
  }
  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    if (stopped_ || info.paused || !run_) return;
    try {
      time_ = std::min(std::chrono::duration<double>(info.simTime).count(), run_->Config().duration_s);
      const auto g = geometry(run_->Config(), time_);
      const std::array<Eigen::Vector3d, 4> positions{g.a, g.b, g.a, g.b};
      const std::array<Eigen::Vector3d, 4> rotations{
        Eigen::Vector3d(0, 0, g.heading_a), Eigen::Vector3d(0, 0, g.heading_b),
        g.antenna_a_rpy, g.antenna_b_rpy};
      for (size_t i = 0; i < names_.size(); ++i) {
        if (entities_[i] == gz::sim::kNullEntity)
          entities_[i] = gz::sim::World(world_).ModelByName(ecm, names_[i]);
        if (entities_[i] == gz::sim::kNullEntity)
          throw std::runtime_error("モデルがありません: " + names_[i]);
        const auto &p = positions[i]; const auto &r = rotations[i];
        gz::sim::Model(entities_[i]).SetWorldPoseCmd(ecm,
          gz::math::Pose3d(p.x(), p.y(), p.z(), r.x(), r.y(), r.z()));
      }
      run_->Advance(time_);
    } catch (const std::exception &e) { Fail(e.what()); }
  }
  void PostUpdate(const gz::sim::UpdateInfo &info, const gz::sim::EntityComponentManager &ecm) override {
    if (stopped_ || info.paused || !run_) return;
    try {
      if (time_ + 1e-9 >= next_pose_log_ || time_ >= run_->Config().duration_s ||
          (pause_at_encounter_ && !encounter_paused_ && time_ >= run_->Config().encounter_s)) {
        for (size_t i = 0; i < names_.size(); ++i) {
          // 命令値ではなく、Physics更新後のECM位置を記録する。
          auto pose = gz::sim::worldPose(entities_[i], ecm);
          const auto &p = pose.Pos(); const auto r = pose.Rot().Euler();
          poses_ << time_ << ',' << names_[i] << ',' << p.X() << ',' << p.Y() << ',' << p.Z()
                 << ',' << r.X() << ',' << r.Y() << ',' << r.Z() << '\n';
          if (i < 2) {
            gz::msgs::Pose msg;
            msg.set_name(names_[i]);
            auto *stamp = msg.mutable_header()->mutable_stamp();
            stamp->set_sec(static_cast<int64_t>(time_));
            stamp->set_nsec(static_cast<int32_t>((time_ - static_cast<int64_t>(time_)) * 1e9));
            msg.mutable_position()->set_x(p.X()); msg.mutable_position()->set_y(p.Y());
            msg.mutable_position()->set_z(p.Z());
            msg.mutable_orientation()->set_w(pose.Rot().W());
            msg.mutable_orientation()->set_x(pose.Rot().X());
            msg.mutable_orientation()->set_y(pose.Rot().Y());
            msg.mutable_orientation()->set_z(pose.Rot().Z());
            pubs_[i].Publish(msg);
          }
        }
        next_pose_log_ = time_ + 0.02;
      }
      if (pause_at_encounter_ && !encounter_paused_ && time_ >= run_->Config().encounter_s) {
        poses_.flush();
        encounter_paused_ = true;
        events_->Emit<gz::sim::events::Pause>(true);
        std::cout << "最接近位置で一時停止しました。GUIの再生ボタンで続行できます。" << std::endl;
      }
      if (time_ >= run_->Config().duration_s) {
        poses_.flush();
        if (!poses_) throw std::runtime_error("Gazebo位置ログを書き込めません");
        run_->Finish();
        std::cout << "ドローン実験完了: " << run_->Stats().delivered_mb << " MB" << std::endl;
        stopped_ = true;
        events_->Emit<gz::sim::events::Stop>();
      }
    } catch (const std::exception &e) { Fail(e.what()); }
  }
 private:
  void Fail(const std::string &message) {
    std::cerr << "ドローン実験エラー: " << message << std::endl;
    stopped_ = true;
    if (events_) events_->Emit<gz::sim::events::Stop>();
  }
  gz::sim::Entity world_ = gz::sim::kNullEntity;
  std::array<gz::sim::Entity, 4> entities_{};
  const std::array<std::string, 4> names_{"drone_a", "drone_b", "antenna_a", "antenna_b"};
  gz::sim::EventManager *events_ = nullptr;
  gz::transport::Node node_;
  std::array<gz::transport::Node::Publisher, 2> pubs_;
  std::unique_ptr<DroneRun> run_;
  std::ofstream poses_;
  double time_ = 0, next_pose_log_ = 0;
  bool stopped_ = false, pause_at_encounter_ = false, encounter_paused_ = false;
};
}
GZ_ADD_PLUGIN(comms_sim::DroneEncounterPlugin, gz::sim::System,
              comms_sim::DroneEncounterPlugin::ISystemConfigure,
              comms_sim::DroneEncounterPlugin::ISystemPreUpdate,
              comms_sim::DroneEncounterPlugin::ISystemPostUpdate)
