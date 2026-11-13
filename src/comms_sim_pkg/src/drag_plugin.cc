// 機体の空気抵抗。F = −½ ρ C_d A |v_rel| v_rel を重心に加える。
// v_rel は対気速度（対地速度 − 風速）。風は <wind> で一定のベクトルとして与える（既定は無風）。
// これがないと等速飛行で水平方向の力が要らず、機体が前傾しない。
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <iostream>

namespace comms_sim {
class DragPlugin : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override {
    gz::sim::Model model(entity);
    link_ = gz::sim::Link(model.LinkByName(ecm, sdf->Get<std::string>("link_name", "base_link").first));
    if (!link_.Valid(ecm)) { std::cerr << "空気抵抗: 機体のリンクがありません" << std::endl; return; }
    link_.EnableVelocityChecks(ecm);
    coefficient_ = 0.5 * sdf->Get<double>("air_density", 1.2).first * sdf->Get<double>("cd_area", 0.03).first;
    wind_ = sdf->Get<gz::math::Vector3d>("wind", gz::math::Vector3d::Zero).first;
  }
  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    if (info.paused || !link_.Valid(ecm)) return;
    const auto velocity = link_.WorldLinearVelocity(ecm);
    if (!velocity) return;
    const auto relative = *velocity - wind_;
    link_.AddWorldForce(ecm, -coefficient_ * relative.Length() * relative);
  }
 private:
  gz::sim::Link link_{gz::sim::kNullEntity};
  double coefficient_ = 0;
  gz::math::Vector3d wind_;
};
}  // namespace comms_sim
GZ_ADD_PLUGIN(comms_sim::DragPlugin, gz::sim::System,
              comms_sim::DragPlugin::ISystemConfigure, comms_sim::DragPlugin::ISystemPreUpdate)
