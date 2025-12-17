# ROS 2 Gazebo Communication Simulator

ROS 2 Humble と Gazebo Harmonic を用いた、移動車両（UGV）と固定基地局間の無線通信品質シミュレータです。

## 📋 概要

本シミュレータは、Gazebo Sim上で駆動する移動車両と固定基地局間の通信品質（RSSI/スループット）を、数理モデルを用いてシミュレーションします。

### 主な機能

- **通信品質シミュレーション**: 対数距離減衰モデルによるRSSI計算
- **アンテナ指向性**: E面/H面ゲインパターンの考慮（CSVファイルから読み込み）
- **AWGN雑音**: 時間変動するガウス雑音のシミュレーション
- **スループット推定**: RSSI閾値ベースのルックアップテーブル
- **ウェイポイント追従**: UGVの経路制御
- **データロギング**: CSV形式での結果出力

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
ros2-gazebo-comms-sim/
├── models/
│   ├── antenna/                # 基地局アンテナモデル
│   │   ├── model.config
│   │   ├── model.sdf
│   │   ├── meshes/
│   │   │   └── antenna.dae
│   │   └── thumbnails/
│   └── SUV/                    # 移動車両モデル
│       ├── model.config
│       ├── model.sdf
│       ├── meshes/
│       │   ├── suv.obj
│       │   └── suv.mtl
│       ├── materials/
│       │   └── textures/
│       ├── thumbnails/
│       └── metadata.pbtxt
├── config/
│   ├── sim_params.yaml         # シミュレーションパラメータ
│   ├── e_plane.csv             # E面アンテナゲイン
│   └── h_plane.csv             # H面アンテナゲイン
├── comms_sim_pkg/
│   ├── comms_sim_pkg/
│   │   ├── __init__.py
│   │   ├── comms_node.py       # 通信シミュレーションノード
│   │   ├── comms_calculator.py # 伝搬路モデル（Strategy Pattern）
│   │   ├── antenna_parser.py   # アンテナパターン解析
│   │   └── ugv_controller_node.py  # UGV制御ノード
│   ├── launch/
│   │   └── sim_launch.py       # 起動ファイル
│   ├── msg/
│   │   └── CommsQuality.msg    # カスタムメッセージ
│   ├── resource/
│   │   └── minimal_world.sdf   # Gazeboワールド
│   ├── package.xml
│   ├── CMakeLists.txt
│   └── setup.py
├── log/
│   └── sim_result/             # シミュレーション結果出力
├── Dockerfile
├── docker-compose.yml
├── entrypoint.sh
└── README.md
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

# コンテナ内でシミュレーション起動
ros2 launch comms_sim_pkg sim_launch.py
```

#### Windows (WSL2 + X Server)

1. VcXsrv等のXサーバーをインストール・起動
2. WSL2のDISPLAY変数を設定
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

主要なパラメータは `config/sim_params.yaml` で設定します。

#### 通信シミュレーションパラメータ

```yaml
comms_simulator_node:
  ros__parameters:
    sampling_rate: 1.0          # 計算頻度 [Hz]
    noise_variance: 2.0         # AWGN分散 [dB]
    comm_data_limit_mb: -1.0    # データ上限 [Mb] (-1.0: 無制限)
    tx_power: 20.0              # 送信電力 [dBm]
    
    path_loss:
      d0: 1.0                   # 基準距離 [m]
      pl0: 40.0                 # 基準距離でのパスロス [dB]
      exponent: 2.0             # パスロス指数
```

#### UGV制御パラメータ

```yaml
ugv_controller_node:
  ros__parameters:
    waypoints:                  # ウェイポイント [X, Y, Z, V]
      - [10.0, 0.0, 0.0, 2.0]
      - [50.0, 0.0, 0.0, 3.0]
      # ...
    waypoint_tolerance: 2.0     # 到達判定距離 [m]
```

#### エンティティスポーン設定

```yaml
spawn_entities:
  antenna:
    pose: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    antenna_height_offset: 1.9  # アンテナ高さ [m]
  suv:
    pose: [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    antenna_height_offset: 1.3
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

### 拡張例: 反射波モデル

```python
class ReflectionModel(PropagationModel):
    """反射波を考慮したモデル"""
    
    def __init__(self, reflection_coef: float = 0.7):
        self.reflection_coef = reflection_coef
    
    def calculate_path_loss(self, distance: float, frequency_ghz: float = 60.0) -> float:
        # 直接波
        direct_loss = 20 * np.log10(distance) + 20 * np.log10(frequency_ghz) + 92.45
        
        # 反射波（簡易モデル）
        reflected_distance = distance * 1.2  # 反射経路は長い
        reflected_loss = 20 * np.log10(reflected_distance) + 20 * np.log10(frequency_ghz) + 92.45
        reflected_power = 10 ** (-reflected_loss / 10) * self.reflection_coef
        
        # 合成
        direct_power = 10 ** (-direct_loss / 10)
        total_power = direct_power + reflected_power
        
        return -10 * np.log10(total_power)
    
    @property
    def model_name(self) -> str:
        return "Reflection Propagation Model"
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
