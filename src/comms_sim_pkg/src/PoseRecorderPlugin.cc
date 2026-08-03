/**
 * PoseRecorderPlugin
 * ------------------
 * 責務: 全モデルの姿勢を一定周期で書き出すことだけ。通信・制御・判断を持たない。
 *
 * 動機: 通信計算は車両の運動に影響しない (VehicleMotionController は運動状態と
 * dt しか受け取らない)。したがって物理は「手法」に依存せず、交通実現 (シード)
 * にのみ依存する。姿勢の時系列を一度記録しておけば、通信とスケジューリングは
 * 後から単一プロセスで再計算できる。
 *
 * これにより:
 *   - Gazebo 実行が「手法 × シード」から「シードのみ」に減る
 *   - 再計算は同期・単一プロセスなので完全に決定的 (制御プレーンを別プロセスの
 *     ROS ノードとして非同期に動かしていたことが、負荷次第で結果が変わる交絡の
 *     原因だった)
 *
 * SDF パラメータ:
 *   <output_path>  出力先 CSV
 *   <period_s>     記録周期 [s] (既定 0.005 = 通信更新レート)
 */
#include <fstream>
#include <iomanip>
#include <string>
#include <unordered_set>

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/math/Pose3.hh>

namespace comms_sim {

class PoseRecorderPlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPostUpdate {
 public:
  void Configure(const gz::sim::Entity& /*_entity*/,
                 const std::shared_ptr<const sdf::Element>& _sdf,
                 gz::sim::EntityComponentManager& /*_ecm*/,
                 gz::sim::EventManager& /*_em*/) override {
    this->output_path = _sdf->Get<std::string>("output_path", "").first;
    this->period_s = _sdf->Get<double>("period_s", 0.005).first;
    if (this->output_path.empty()) {
      gzerr << "[PoseRecorder] output_path が未指定です。記録しません。"
            << std::endl;
      return;
    }
    this->out.open(this->output_path);
    if (!this->out) {
      gzerr << "[PoseRecorder] 開けません: " << this->output_path << std::endl;
      return;
    }
    // 再生側が「その時刻に存在したモデル」を復元できるよう、毎行にモデル名を持つ
    this->out << "t_s,model,x,y,z,roll,pitch,yaw\n";
    this->out << std::fixed << std::setprecision(6);
    this->enabled = true;
    gzmsg << "[PoseRecorder] 記録開始: " << this->output_path
          << " (周期 " << this->period_s << " s)" << std::endl;
  }

  void PostUpdate(const gz::sim::UpdateInfo& _info,
                  const gz::sim::EntityComponentManager& _ecm) override {
    if (!this->enabled || _info.paused) return;
    const double t = std::chrono::duration<double>(_info.simTime).count();
    // グリッドで判定する (累積加算だと浮動小数の誤差が溜まり、再生側の時刻
    // グリッドとずれる)
    if (t + 1e-9 < this->next_t) return;
    this->next_t = (std::floor(t / this->period_s) + 1.0) * this->period_s;

    _ecm.Each<gz::sim::components::Model, gz::sim::components::Name,
              gz::sim::components::Pose>(
        [&](const gz::sim::Entity& /*e*/,
            const gz::sim::components::Model*,
            const gz::sim::components::Name* name,
            const gz::sim::components::Pose* pose) -> bool {
          const gz::math::Pose3d& p = pose->Data();
          this->out << t << ',' << name->Data() << ','
                    << p.Pos().X() << ',' << p.Pos().Y() << ',' << p.Pos().Z()
                    << ',' << p.Rot().Roll() << ',' << p.Rot().Pitch() << ','
                    << p.Rot().Yaw() << '\n';
          return true;
        });
    ++this->rows;
  }

  ~PoseRecorderPlugin() override {
    if (this->out.is_open()) {
      this->out.flush();
      this->out.close();
      gzmsg << "[PoseRecorder] 記録完了: " << this->rows << " ステップ"
            << std::endl;
    }
  }

 private:
  std::string output_path;
  double period_s = 0.005;
  double next_t = 0.0;
  bool enabled = false;
  long rows = 0;
  std::ofstream out;
};

}  // namespace comms_sim

GZ_ADD_PLUGIN(comms_sim::PoseRecorderPlugin, gz::sim::System,
              comms_sim::PoseRecorderPlugin::ISystemConfigure,
              comms_sim::PoseRecorderPlugin::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(comms_sim::PoseRecorderPlugin,
                    "comms_sim::PoseRecorderPlugin")
