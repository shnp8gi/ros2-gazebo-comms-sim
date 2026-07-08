/**
 * WaypointMoverPlugin
 * -------------------
 * 責務: モデルの waypoint 追従移動のみ。通信・ロギング・スケジューリングを一切持たない。
 * 移動遮蔽体(トラック・歩行者等)を通信プラグインと疎結合に動かすための軽量プラグイン。
 * 経路追従の計算は VehicleMotionController に委譲する。
 *
 * SDF パラメータ:
 *   <waypoints>x y z v x y z v ...</waypoints>   フラット列 (4値/点)
 *   <waypoint_tolerance> <heading_gain> <max_acceleration> <max_angular_velocity>
 *   <loop>true</loop>                            経路を周回する (デフォルト false)
 *   <all_ready_topic>/sim/all_ready</all_ready_topic>  空文字列なら即時開始
 */
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/boolean.pb.h>
#include <atomic>
#include <sstream>
#include <string>
#include <vector>

#include "comms_sim_pkg/DataTypes.hpp"
#include "comms_sim_pkg/VehicleMotionController.hpp"

namespace tx_controller
{
    class WaypointMoverPlugin :
        public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
    {
    public:
        void Configure(const gz::sim::Entity &_entity,
                       const std::shared_ptr<const sdf::Element> &_sdf,
                       gz::sim::EntityComponentManager &/*_ecm*/,
                       gz::sim::EventManager &/*_eventMgr*/) override
        {
            this->model = gz::sim::Model(_entity);

            if (_sdf->HasElement("waypoints")) {
                std::istringstream iss(_sdf->Get<std::string>("waypoints"));
                std::vector<double> values;
                double v;
                while (iss >> v) values.push_back(v);
                for (size_t i = 0; i + 3 < values.size(); i += 4) {
                    this->waypoints.push_back({values[i], values[i + 1], values[i + 2], values[i + 3]});
                }
            }

            this->tolerance = _sdf->HasElement("waypoint_tolerance")
                                  ? _sdf->Get<double>("waypoint_tolerance") : 1.0;
            this->heading_gain = _sdf->HasElement("heading_gain")
                                     ? _sdf->Get<double>("heading_gain") : 2.0;
            this->max_accel = _sdf->HasElement("max_acceleration")
                                  ? _sdf->Get<double>("max_acceleration") : 3.0;
            this->max_ang = _sdf->HasElement("max_angular_velocity")
                                ? _sdf->Get<double>("max_angular_velocity") : 1.0;
            this->loop = _sdf->HasElement("loop") && _sdf->Get<bool>("loop");

            this->ConfigureMotion();

            std::string all_ready_topic = _sdf->HasElement("all_ready_topic")
                                              ? _sdf->Get<std::string>("all_ready_topic")
                                              : "/sim/all_ready";
            if (all_ready_topic.empty()) {
                this->all_ready = true;
            } else {
                this->node.Subscribe<gz::msgs::Boolean>(
                    all_ready_topic,
                    [this](const gz::msgs::Boolean &msg) {
                        if (msg.data()) this->all_ready = true;
                    });
            }
        }

        void PreUpdate(const gz::sim::UpdateInfo &_info,
                       gz::sim::EntityComponentManager &_ecm) override
        {
            if (_info.paused || !this->all_ready) return;
            if (this->waypoints.empty()) return;

            auto pose_comp = _ecm.Component<gz::sim::components::Pose>(this->model.Entity());
            if (!pose_comp) return;
            gz::math::Pose3d pose = pose_comp->Data();

            MotionState state;
            state.x = pose.Pos().X();
            state.y = pose.Pos().Y();
            state.yaw = pose.Rot().Yaw();

            if (this->motion_controller.IsMissionComplete() && this->loop) {
                // 周回モード: 経路を再スタート(横断遮蔽の繰り返し評価用)
                this->ConfigureMotion();
            }

            double dt = std::chrono::duration<double>(_info.dt).count();
            MotionCommand cmd = this->motion_controller.CalculateCommand(state, dt);
            this->SetVelocity(_ecm, cmd.linear_velocity, cmd.angular_velocity);
        }

    private:
        void ConfigureMotion() {
            this->motion_controller.Configure(this->waypoints, this->tolerance,
                                              this->max_ang, this->heading_gain,
                                              this->max_accel, false);
        }

        void SetVelocity(gz::sim::EntityComponentManager &_ecm, double v, double w) {
            auto lin = _ecm.Component<gz::sim::components::LinearVelocityCmd>(this->model.Entity());
            if (!lin) {
                _ecm.CreateComponent(this->model.Entity(),
                                     gz::sim::components::LinearVelocityCmd({v, 0, 0}));
            } else {
                lin->Data() = {v, 0, 0};
            }
            auto ang = _ecm.Component<gz::sim::components::AngularVelocityCmd>(this->model.Entity());
            if (!ang) {
                _ecm.CreateComponent(this->model.Entity(),
                                     gz::sim::components::AngularVelocityCmd({0, 0, w}));
            } else {
                ang->Data() = {0, 0, w};
            }
        }

        gz::sim::Model model;
        gz::transport::Node node;
        VehicleMotionController motion_controller;
        std::vector<Waypoint> waypoints;
        double tolerance = 1.0;
        double heading_gain = 2.0;
        double max_accel = 3.0;
        double max_ang = 1.0;
        bool loop = false;
        std::atomic<bool> all_ready{false};
    };
} // namespace tx_controller

GZ_ADD_PLUGIN(
    tx_controller::WaypointMoverPlugin,
    gz::sim::System,
    tx_controller::WaypointMoverPlugin::ISystemConfigure,
    tx_controller::WaypointMoverPlugin::ISystemPreUpdate)
