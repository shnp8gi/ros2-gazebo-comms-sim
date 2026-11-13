// 飛行の記録。物理の刻みごとに、2機の機体・アンテナの位置と姿勢、ロータ回転数を書き出す。
// 出力：trace.bin（倍精度の生データ、1行 37 列）と trace.json（列の説明）。
//   time, [機体 x y z qw qx qy qz, アンテナ x y z qw qx qy qz, ロータ回転数×4] × 2機
#include <gz/plugin/Register.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Events.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <array>
#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>

namespace comms_sim {
class FlightRecorderPlugin : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate {
 public:
  void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &events) override {
    world_ = entity; events_ = &events;
    directory_ = sdf->Get<std::string>("output_dir");
    duration_ = sdf->Get<double>("duration", 40.0).first;
    slowdown_ = sdf->Get<double>("rotor_velocity_slowdown", 10.0).first;
    out_.open(directory_ + "/trace.bin", std::ios::binary);
    if (!out_) { std::cerr << "飛行の記録を開けません: " << directory_ << std::endl; failed_ = true; }
  }
  void PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &ecm) override {
    if (ready_ || failed_) return;
    // 関節の速度は、読む側が有効にしておかないと記録されない。
    for (size_t m = 0; m < models_.size(); ++m) {
      gz::sim::Model model(gz::sim::World(world_).ModelByName(ecm, models_[m]));
      if (!model.Valid(ecm)) return;
      base_[m] = model.LinkByName(ecm, "base_link");
      antenna_[m] = model.LinkByName(ecm, "antenna_link");
      for (int r = 0; r < 4; ++r) {
        joints_[m][r] = gz::sim::Joint(model.JointByName(ecm, "rotor_" + std::to_string(r) + "_joint"));
        if (!joints_[m][r].Valid(ecm)) return;
        joints_[m][r].EnableVelocityCheck(ecm);
      }
      if (base_[m] == gz::sim::kNullEntity || antenna_[m] == gz::sim::kNullEntity) return;
    }
    ready_ = true;
  }
  void PostUpdate(const gz::sim::UpdateInfo &info, const gz::sim::EntityComponentManager &ecm) override {
    if (info.paused || stopped_ || failed_ || !ready_) return;
    const double time = std::chrono::duration<double>(info.simTime).count();
    std::array<double, 37> row{};
    size_t k = 0;
    row[k++] = time;
    for (size_t m = 0; m < models_.size(); ++m) {
      for (auto entity : {base_[m], antenna_[m]}) {
        const auto pose = gz::sim::worldPose(entity, ecm);
        row[k++] = pose.Pos().X(); row[k++] = pose.Pos().Y(); row[k++] = pose.Pos().Z();
        row[k++] = pose.Rot().W(); row[k++] = pose.Rot().X(); row[k++] = pose.Rot().Y(); row[k++] = pose.Rot().Z();
      }
      for (int r = 0; r < 4; ++r) {
        const auto velocity = joints_[m][r].Velocity(ecm);
        row[k++] = velocity && !velocity->empty() ? (*velocity)[0] * slowdown_ : 0;
      }
    }
    out_.write(reinterpret_cast<const char *>(row.data()), sizeof(row));
    ++rows_;
    if (time + 1e-9 < duration_) return;
    out_.close();
    std::ofstream meta(directory_ + "/trace.json");
    meta << "{\"rows\": " << rows_ << ", \"columns\": 37, \"dtype\": \"float64\", "
         << "\"step_s\": " << std::chrono::duration<double>(info.dt).count() << ", "
         << "\"layout\": \"time, per drone (a, b): base xyz qwxyz, antenna xyz qwxyz, rotor speed x4 [rad/s]\"}\n";
    meta.close();
    std::cout << "飛行の記録完了: " << rows_ << " 行" << std::endl;
    stopped_ = true;
    events_->Emit<gz::sim::events::Stop>();
  }
 private:
  gz::sim::Entity world_ = gz::sim::kNullEntity;
  gz::sim::EventManager *events_ = nullptr;
  const std::array<std::string, 2> models_{"drone_a", "drone_b"};
  std::array<gz::sim::Entity, 2> base_{}, antenna_{};
  std::array<std::array<gz::sim::Joint, 4>, 2> joints_;
  std::string directory_;
  std::ofstream out_;
  double duration_ = 40, slowdown_ = 10;
  uint64_t rows_ = 0;
  bool ready_ = false, stopped_ = false, failed_ = false;
};
}  // namespace comms_sim
GZ_ADD_PLUGIN(comms_sim::FlightRecorderPlugin, gz::sim::System,
              comms_sim::FlightRecorderPlugin::ISystemConfigure,
              comms_sim::FlightRecorderPlugin::ISystemPreUpdate,
              comms_sim::FlightRecorderPlugin::ISystemPostUpdate)
