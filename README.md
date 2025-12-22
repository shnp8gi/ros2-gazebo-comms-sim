# ROS 2 Gazebo Communication Simulator

ROS 2 Humble と Gazebo Harmonic を用いた、移動車両（UGV）と固定基地局間の無線通信品質シミュレータです。

## 📋 概要

本シミュレータは、Gazebo Sim上で駆動する移動車両と固定基地局間の通信品質（RSSI/スループット）を、数理モデルを用いてシミュレーションします。

### 主な機能

- **リアルタイム通信品質計算**: RSSI、スループット、パスロスを動的に計算
- **MCSテーブルベースのスループット推定**: CSVファイルから読み込み、線形補間による推定
- **リンク確立時間のシミュレーション**: Association time（デフォルト2ms）を考慮した現実的な通信開始
- **指向性アンテナモデル**: E面/H面パターンCSVに基づく3次元ゲイン計算
- **動的伝搬路モデル**: 対数距離減衰モデル + AWGN雑音
- **マルチウェイポイント追従**: UGVの区間速度制御と自動経路追従
- **通信データ量制限**: 設定可能な送信データ上限とCSVロギング

## 🛠️ 技術スタック

| 項目 | バージョン/詳細 |
|------|----------------|
| OS | Ubuntu 22.04 LTS (Docker) |
| ROS 2 | Humble Hawksbill |
| シミュレータ | Gazebo Sim (Harmonic) |
| 言語 | Python 3 |
| 開発環境 | Docker / Docker Compose |

## 📁 ディレクトリ構造

```
os2-gazebo-comms-sim/
├── config/                        # 設定ファイル
│   ├── sim_params.yaml           # メインパラメータファイル
│   ├── e_plane.csv               # Eプレーン（垂直面）アンテナパターン
│   ├── h_plane.csv               # Hプレーン（水平面）アンテナパターン
│   └── MCStable.csv              # MCS（変調・符号化方式）テーブル
│
├── models/                        # Gazeboモデル
│   ├── antenna/                  # 基地局アンテナモデル
│   │   ├── model.config
│   │   ├── model.sdf
│   │   └── meshes/
│   └── SUV/                      # 移動車両モデル
│       ├── model.config
│       ├── model.sdf
│       └── meshes/
│
├── src/
│   └── comms_sim_pkg/            # ROS 2パッケージ
│       ├── comms_sim_pkg/        # Pythonモジュール
│       │   ├── __init__.py
│       │   ├── comms_node.py             # 通信シミュレータノード
│       │   ├── comms_calculator.py       # 通信品質計算
│       │   ├── antenna_parser.py         # アンテナパターン処理
│       │   └── ugv_controller_node.py    # UGV制御ノード
│       │
│       ├── launch/               # 起動ファイル
│       │   └── sim_launch.py
│       │
│       ├── msg/                  # カスタムメッセージ定義
│       │   └── CommsQuality.msg
│       │
│       ├── resource/             # リソースファイル
│       │   └── minimal_world.sdf # シミュレーションワールド
│       │
│       ├── CMakeLists.txt
│       ├── package.xml
│       └── setup.py
│
├── log/
│   └── sim_result/               # CSVログ出力ディレクトリ
│
├── docker-compose.yml             # Docker Compose設定
├── Dockerfile                     # Dockerイメージ定義
├── entrypoint.sh                  # コンテナエントリポイント
├── README.md                      # このファイル
├── specification.md               # 詳細仕様書
└── .gitignore
```

## 🚀 クイックスタート

### 前提条件

- Docker および Docker Compose がインストールされていること
- X11サーバー（GUIモード使用時、Linux/WSL2）

### 1. リポジトリのクローン

```bash
git clone https://github.com/your-username/ros2-gazebo-comms-sim.git
cd ros2-gazebo-comms-sim
```

### 2. Dockerイメージのビルド

```bash
docker-compose build
```

### 3. ワークスペースのビルド

```bash
docker-compose run --rm build
```

## 📺 シミュレータの実行

### GUIモード（推奨）

X11 Forwardingを使用してGazeboのGUIを表示します。

#### Linux / WSL2

```bash
# X11アクセスを許可
xhost +local:docker

# GUIモードでコンテナ起動
docker-compose run --rm sim-gui

# 以下コンテナ内
# パッケージのビルド
colcon build --cmake-args -DBUILD_TESTING=ON

# ビルドしたパッケージをROS2の環境に設定する
. install/setup.sh

# シミュレーション起動
ros2 launch comms_sim_pkg sim_launch.py
```

#### Windows (WSL2 + X Server)

1. VcXsrv等のXサーバーをインストール・起動
2. WSL2のDISPLAY変数を設 定
```bash
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0.0
```
3. 上記Linuxと同様の手順でコンテナを起動

### CUIモード（Headless）

GUIなしで高速にシミュレーションを実行します。

```bash
# CUIモードでコンテナ起動
docker-compose run --rm sim-cui

# コンテナ内でHeadlessシミュレーション起動
ros2 launch comms_sim_pkg sim_launch.py headless:=true
```

### GUI / CUI 切り替え

| モード | コマンド | 用途 |
|--------|----------|------|
| GUI | `ros2 launch comms_sim_pkg sim_launch.py` | 可視化・デバッグ |
| CUI | `ros2 launch comms_sim_pkg sim_launch.py headless:=true` | 高速実行・バッチ処理 |

## ⚙️ パラメータ設定

### config/sim_params.yaml

主要なパラメータは `config/sim_params.yaml` で設定します。このYAMLファイルは起動時に各ノードに直接読み込まれます。

#### 通信シミュレーションパラメータ

```yaml
comms_simulator_node:
  ros__parameters:
    sampling_rate: value          # 計算頻度 [Hz] (default: 1.0)
    noise_variance: value         # AWGN分散 [dB] (default: 2.0)
    e_plane_path: value           # E面アンテナゲインCSVパス (default: "/workspace/config/e_plane.csv")
    h_plane_path: value           # H面アンテナゲインCSVパス (default: "/workspace/config/h_plane.csv")
    mcs_table_path: value         # MCSテーブルパス (default: "/workspace/config/MCStable.csv")
    link_establishment_time_ms: value  # リンク確立時間 [ms] (default: 2.0)
    comm_data_limit_mb: value     # データ上限 [Mb] (default: -1.0, -1.0: 無制限)
    
    path_loss:
      d0: value                   # 基準距離 [m] (default: 1.0)
      pl0: value                  # 基準距離でのパスロス [dB] (default: 40.0)
      exponent: value             # パスロス指数 (default: 2.0)
    
    tx_power: value               # 送信電力 [dBm] (default: 20.0)
```

#### UGV制御パラメータ

```yaml
ugv_controller_node:
  ros__parameters:
    waypoints:                    # ウェイポイント [X, Y, Z, V]
      - [value, value, value, value]  # (例: [-20.0, 0.0, 0.0, 2.78])
      - [value, value, value, value]  # (例: [20.0, 0.0, 0.0, 0.0])
      # ...
    waypoint_tolerance: value     # 到達判定距離 [m] (default: 2.0)
    control_rate: value           # 制御ループ周波数 [Hz] (default: 10.0)
    max_angular_velocity: value   # 最大角速度 [rad/s] (default: 1.0)
    heading_gain: value           # 方向制御ゲイン (default: 1.5)
```

#### エンティティスポーン設定

```yaml
spawn_entities:
  antenna:
    model_uri: value              # モデルURI (default: "models://antenna")
    name: value                   # エンティティ名 (default: "antenna")
    pose: [value, value, value, value, value, value]  # [X, Y, Z, Roll, Pitch, Yaw] (default: [0.0, 6.0, 0.0, 0.0, 0.0, 0.0])
    static: value                 # 静的オブジェクト (default: true)
    antenna_height_offset: value  # アンテナ高さ [m] (default: 3.0)
  
  suv:
    model_uri: value              # モデルURI (default: "models://SUV")
    name: value                   # エンティティ名 (default: "suv")
    pose: [value, value, value, value, value, value]  # [X, Y, Z, Roll, Pitch, Yaw] (default: [-20.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    static: value                 # 静的オブジェクト (default: false)
    antenna_height_offset: value  # アンテナ高さ [m] (default: 1.9)
```

## 📊 出力データ

シミュレーション終了時、CSVファイルが `log/sim_result/` に出力されます。

### ファイル命名規則

```
YYYYMMDD_HHMMSS_LIMIT-[上限値]MB.csv
```

例: `20251217_143052_LIMIT-UNLIMITED.csv`

### CSV出力項目

| 項目 | 単位 | 説明 |
|------|------|------|
| time_s | s | シミュレーション時間 |
| ugv_x, ugv_y, ugv_z | m | UGV座標 |
| bs_x, bs_y, bs_z | m | 基地局座標 |
| distance | m | UGV-基地局間距離 |
| rssi | dBm | 受信信号強度 |
| throughput | Gbps | 瞬時スループット |
| total_data_mb | Mb | 累積送信データ量 |
| path_loss | dB | パスロス |
| e_gain, h_gain | dBi | アンテナゲイン |

## 🔌 ROSインターフェース

### トピック

| トピック名 | メッセージ型 | 方向 | 説明 |
|------------|-------------|------|------|
| `/gps/fix` | `sensor_msgs/NavSatFix` | Sub | UGV GPS位置 |
| `/imu/data` | `sensor_msgs/Imu` | Sub | UGV姿勢情報 |
| `/cmd_vel` | `geometry_msgs/Twist` | Pub | 速度指令 |
| `/comms/quality` | `comms_sim_pkg/CommsQuality` | Pub | 通信品質 |
| `/odom` | `nav_msgs/Odometry` | Sub | オドメトリ |

### カスタムメッセージ: CommsQuality

```
std_msgs/Header header
float64 distance          # 距離 [m]
float64 rssi              # RSSI [dBm]
float64 throughput        # スループット [Gbps]
float64 total_data_transmitted  # 累積データ [Mb]
float64 ugv_x, ugv_y, ugv_z
float64 base_station_x, base_station_y, base_station_z
float64 antenna_gain_e_plane, antenna_gain_h_plane
float64 path_loss
bool comm_active
```

## 🔧 伝搬路モデルの拡張（Strategy Pattern）

本シミュレータは **Strategy パターン** を採用しており、伝搬路モデルを容易に拡張・切り替えできます。

### 既存モデル

1. **LogDistancePathLossModel**: 対数距離減衰モデル（デフォルト）
2. **TwoRayGroundModel**: 2波モデル

### 新規モデルの追加方法

#### 1. 抽象基底クラスを継承

```python
# comms_sim_pkg/comms_calculator.py

from abc import ABC, abstractmethod

class PropagationModel(ABC):
    """伝搬路モデルの抽象基底クラス"""
    
    @abstractmethod
    def calculate_path_loss(self, distance: float, frequency_ghz: float = 60.0) -> float:
        """パスロスを計算"""
        pass
    
    @property
    @abstractmethod
    def model_name(self) -> str:
        """モデル名を返す"""
        pass
```

#### 2. 具象クラスを実装

```python
class NLOSModel(PropagationModel):
    """NLOS（見通し外）伝搬モデルの例"""
    
    def __init__(self, additional_loss: float = 20.0):
        self.additional_loss = additional_loss
    
    def calculate_path_loss(self, distance: float, frequency_ghz: float = 60.0) -> float:
        # 基本的な自由空間伝搬損失 + 追加損失
        fspl = 20 * np.log10(distance) + 20 * np.log10(frequency_ghz) + 92.45
        return fspl + self.additional_loss
    
    @property
    def model_name(self) -> str:
        return "NLOS Propagation Model"
```

#### 3. モデルを切り替え

```python
# comms_node.py で使用
from .comms_calculator import CommsCalculator, NLOSModel

# モデルのインスタンス化
nlos_model = NLOSModel(additional_loss=25.0)

# 計算機に設定
calculator = CommsCalculator(propagation_model=nlos_model)

# または動的に切り替え
calculator.set_propagation_model(nlos_model)
```

## 🔍 トラブルシューティング

### Gazeboが起動しない

```bash
# GZ_SIM_RESOURCE_PATHを確認
echo $GZ_SIM_RESOURCE_PATH

# モデルパスを明示的に設定
export GZ_SIM_RESOURCE_PATH=/workspace/models
```

### X11 Forwarding エラー

```bash
# Xサーバーへのアクセスを許可
xhost +local:docker

# DISPLAY変数を確認
echo $DISPLAY
```

### ビルドエラー

```bash
# クリーンビルド
cd /workspace
rm -rf build install log
colcon build --symlink-install
```

### トピックが見つからない

```bash
# 利用可能なトピックを確認
ros2 topic list

# Gazeboのトピックを確認
gz topic -l
```
