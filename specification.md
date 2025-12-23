<!-- filepath: /home/yagi/ros2-gazebo-comms-sim/specification.md -->
# 📡 ROS 2/Gazebo 通信シミュレータ 統合システム仕様書

## 変更履歴

| 変更日 | バージョン | 改定内容 |
| :--- | :--- | :--- |
| 2025/12/14 | ver 1.0 | 初版 |
| 2025/12/14 | ver 2.0 | UGVのマルチウェイポイント追従と区間速度制御、アンテナゲインの動的参照、通信容量上限による通信停止機能の追加、およびCSVロギング仕様の確定 |
| 2025/12/22 | ver 3.0 | MCSテーブルベースのスループット計算への変更、リンク確立時間（Association time）の実装、RSSI閾値の自動取得機能、YAMLベースの完全パラメータ化 |

---

## 1. プロジェクト概要

Gazebo Simで駆動する移動車両と固定基地局間の通信品質（RSSI/スループット）を、ROS 2ノード上で数理モデルを用いてシミュレーションし、その挙動を評価する。

---

## 2. 技術スタックと環境構築

| 分野 | 項目 | 決定事項 | 備考 |
| :--- | :--- | :--- | :--- |
| **OS** | ベースOS | Ubuntu 22.04 LTS (Dockerコンテナ内) | |
| **ROS 2** | ディストリビューション | **Humble Hawksbill (LTS)** | サポート期間: 2027年5月まで。 |
| **シミュレータ** | 種類 | **Gazebo Sim (Harmonic)** | ROS 2との連携、将来性を重視。 |
| **開発環境** | コンテナ | **Docker** (推奨) | 環境の再現性、GUI/CUI切り替えをサポート。 |
| **実装言語** | 主言語 | **Python 3** | 通信計算（数理モデル）の柔軟性と開発速度を優先。 |

---

## 3. シミュレーション環境 (World & Models)

| 項目 | 詳細 | Fuel URI / 構成 |
| :--- | :--- | :--- |
| **ワールド** | シンプルな無限平面 (Empty World + Ground Plane)。将来的な物体設置は可能とする。 | `minimal_world.sdf` |
| **移動車両 (UGV)** | 実在感のあるSUVモデルに駆動系とセンサをアタッチ。 | **Fuel: `https://app.gazebosim.org/OpenRobotics/fuel/models/SUV`** |
| **基地局** | 高さのあるアンテナ塔モデル。固定設置。 | **Fuel: `https://app.gazebosim.org/OpenRobotics/fuel/models/antenna`** |
| **モデル参照** | Gazebo Fuelからローカルにダウンロードし、Dockerでマウントして参照する。 | `GZ_SIM_RESOURCE_PATH` を設定。 |

---

## 4. ディレクトリ構成

```
ros2-gazebo-comms-sim/
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
│   │   ├── meshes/              # 3Dメッシュとテクスチャ
│   │   └── thumbnails/          # プレビュー画像
│   └── SUV/                      # 移動車両モデル
│       ├── model.config
│       ├── model.sdf
│       ├── metadata.pbtxt
│       ├── materials/
│       ├── meshes/
│       └── thumbnails/
│
├── comms_sim_pkg/            # ROS 2パッケージ
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── setup.py
│       │
│       ├── comms_sim_pkg/        # Pythonモジュール
│       │   ├── __init__.py
│       │   ├── comms_node.py             # 通信シミュレータノード
│       │   ├── comms_calculator.py       # 通信品質計算エンジン
│       │   ├── antenna_parser.py         # アンテナパターン処理
│       │   └── ugv_controller_node.py    # UGV制御ノード
│       │
│       ├── launch/               # 起動ファイル
│       │   └── sim_launch.py    # メインランチファイル
│       │
│       └── resource/             # リソースファイル
│           └── minimal_world.sdf # シミュレーションワールド
│
├── comms_sim_msgs/           # メッセージ定義パッケージ
│       ├── CMakeLists.txt
│       ├── package.xml
│       └── msg/
│           └── CommsQuality.msg # 通信品質メッセージ
│
├── log/
│   └── sim_result/               # CSVログ出力ディレクトリ
│
├── docker-compose.yml             # Docker Compose設定
├── Dockerfile                     # Dockerイメージ定義
├── entrypoint.sh                  # コンテナエントリポイント
├── .gitignore
├── README.md                      # 使用方法ガイド
├── specification.md               # このファイル（システム仕様書）
└── prompt.md                      # 開発プロンプト履歴
```

---

## 5. ロボットとセンサ構成

| エンティティ | センサ/プラグイン | ROS 2 トピック | 用途 |
| :--- | :--- | :--- | :--- |
| **車両** | Diff Drive (プラグイン) | `/cmd_vel` (Subscribe) | 運動制御。 |
| **車両** | NavSat (GNSS) | `/gps/fix` (Publish) | $x, y, z$ 位置情報の提供。 |
| **車両** | IMU | `/imu/data` (Publish) | 姿勢情報 (Roll, Pitch, Yaw) の提供。**アンテナゲイン計算**に利用。 |
| **基地局** | N/A | パラメータから座標を取得 | 静的なためセンサ不要。 |

---

## 6. 通信シミュレーション要件 (ROS 2 Node)

### 6.1. 通信ノード (`comms_simulator_node`)

| 項目 | 詳細 |
| :--- | :--- |
| **ノード名** | `comms_simulator_node` |
| **実装言語** | Python 3 |
| **役割** | Gazeboから取得した位置情報と姿勢情報に基づき、カスタムの伝搬路モデルで通信品質を計算し、ロギング及びPublishを行う。 |

### 6.2. 通信インターフェース

| 項目 | トピック名/パラメータ | メッセージ型 | 送受信 | 備考 |
| :--- | :--- | :--- | :--- | :--- |
| **車両位置** | `/gps/fix` | `sensor_msgs/NavSatFix` | Subscribe | Gazeboから車両の位置を取得。 |
| **車両姿勢** | `/imu/data` | `sensor_msgs/Imu` | Subscribe | Gazeboから車両の姿勢を取得。 |
| **通信結果** | `/comms/quality` | カスタム (`CommsQuality.msg`) | Publish | 計算されたRSSI値とスループットを出力。 |

### 6.3. 伝搬路モデル計算ロジック

| 項目 | 詳細 |
| :--- | :--- |
| **計算頻度** | **可変サンプリングレート** (`sampling_rate` パラメータで調整可能)。デフォルト 1.0 Hz。 |
| **パスロス** | **対数距離減衰モデル** ($PL(d) = PL(d_0) + 10 \times n \times \log_{10}(d/d_0)$) をベースとする。 |
| **RSSI計算** | $RSSI = P_t - PL(d) + G_a + N$ <br> - $P_t$: 送信電力 [dBm] <br> - $PL(d)$: パスロス [dB] <br> - $G_a$: アンテナゲイン [dBi] <br> - $N$: AWGN雑音 [dB] |
| **雑音 (AWGN)** | **正規分布に従うAWGN** ($\mathcal{N}(0, \sigma^2)$) を逐一加算。分散 $\sigma^2$ はYAMLで設定可能。 |
| **アンテナゲイン** | **CSVファイル**（E面/H面ゲイン）を参照。車両の姿勢（IMU）に基づく相対角度からゲインを線形補間 (`numpy.interp`) で取得する。 |
| **スループット** | **MCSテーブル（CSV）を参照し、線形補間により決定**。（6.4.項を参照） |
| **拡張性** | 伝搬路モデルの計算ロジックは、**Strategyパターン**を適用し、将来のNLOS/反射波モデルへの差し替えを容易にする。 |

### 6.4. スループット決定方式

CSVファイル（`MCStable.csv`）を参照する方式

#### MCSテーブル（`MCStable.csv`）の形式

```csv
# RSSI [dBm], Throughput [Gbps]
-39, 13.1413
-45, 9.856
-51, 6.5707
-55, 5.1627
-58, 3.2853
-61, 2.5813
```

#### スループット計算ルール

| RSSI範囲 | スループット | 計算方法 |
| :--- | :--- | :--- |
| $RSSI \le RSSI_{min}$ | 0.0 Gbps | 通信不可 |
| $RSSI_{min} < RSSI < RSSI_{max}$ | 線形補間 | `numpy.interp(rssi, mcs_rssi, mcs_throughput)` |
| $RSSI \ge RSSI_{max}$ | $Throughput_{max}$ | 飽和（最大スループット） |

**重要**: 
- $RSSI_{min}$ と $RSSI_{max}$ は**CSVファイルから自動取得**される
- 上記の例では、$RSSI_{min} = -61$ dBm、$RSSI_{max} = -39$ dBm。

### 6.5. リンク確立時間（Association Time）の実装

現実の無線通信における**リンク確立遅延**をシミュレート。

#### リンク状態遷移

```
DISCONNECTED → ESTABLISHING → CONNECTED
     ↓              ↓              ↓
  RSSI < 閾値   リンク確立中    通信可能
                (待機時間中)
```

| 状態 | 条件 | 通信可否 | 遷移条件 |
| :--- | :--- | :--- | :--- |
| **DISCONNECTED** | $RSSI \le RSSI_{min}$ | 不可 | $RSSI > RSSI_{min}$ で ESTABLISHING へ |
| **ESTABLISHING** | $RSSI > RSSI_{min}$ かつ待機中 | 不可 | 設定時間経過 → CONNECTED <br> $RSSI \le RSSI_{min}$ → DISCONNECTED |
| **CONNECTED** | $RSSI > RSSI_{min}$ かつ確立済み | 可能 | $RSSI \le RSSI_{min}$ → DISCONNECTED |

#### パラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
| :--- | :--- | :--- | :--- |
| `link_establishment_time_ms` | float | 2.0 | リンク確立時間 [ms]。YAMLで設定。 |

#### 動作シーケンス例

```
時刻 t=0.0s: RSSI = -65 dBm (< -61 dBm)
  状態: DISCONNECTED
  通信: 不可

時刻 t=1.0s: RSSI = -58 dBm (> -61 dBm)
  状態: DISCONNECTED → ESTABLISHING
  通信: 不可（リンク確立待機開始）

時刻 t=1.002s: 2 ms経過
  状態: ESTABLISHING → CONNECTED
  通信: 可能（スループット = 3.2853 Gbps）

時刻 t=5.0s: RSSI = -63 dBm (< -61 dBm)
  状態: CONNECTED → DISCONNECTED
  通信: 不可

時刻 t=8.0s: RSSI = -55 dBm (> -61 dBm)
  状態: DISCONNECTED → ESTABLISHING
  通信: 不可（再度リンク確立待機開始）

時刻 t=8.002s: 2 ms経過
  状態: ESTABLISHING → CONNECTED
  通信: 可能（スループット = 5.1627 Gbps）
```

### 6.6. パラメータ構成

すべてのパラメータは `config/sim_params.yaml` で管理される。

#### 通信シミュレータノードパラメータ

| パラメータ | 型 | デフォルト値 | 説明 |
| :--- | :--- | :--- | :--- |
| `sampling_rate` | float | 1.0 | 通信計算の更新頻度 [Hz]。 |
| `tx_power` | float | -7.0 | 送信電力 [dBm]。 |
| `noise_variance` | float | 2.0 | AWGNの分散値 [dB]。 |
| `mcs_table_path` | string | `/workspace/config/MCStable.csv` | MCSテーブルCSVファイルへのパス。 |
| `link_establishment_time_ms` | float | 2.0 | リンク確立時間 [ms]。 |
| `e_plane_path` | string | `/workspace/config/e_plane.csv` | E面ゲインCSVファイルへのパス。 |
| `h_plane_path` | string | `/workspace/config/h_plane.csv` | H面ゲインCSVファイルへのパス。 |
| `comm_data_limit_mb` | float | 100.0 | 通信データ量の上限 [Mb]。`-1.0`で無制限。 |
| `base_station_position` | list [x, y, z] | [0.0, 6.0, 0.0] | 基地局の静的なワールド座標。 |
| `base_station_antenna_offset` | float | 3.0 | 基地局アンテナの高さオフセット [m]。 |
| `ugv_antenna_offset` | float | 1.9 | UGVアンテナの高さオフセット [m]。 |

#### パスロスモデルパラメータ

| パラメータ | 型 | デフォルト値 | 説明 |
| :--- | :--- | :--- | :--- |
| `path_loss.d0` | float | 1.0 | 基準距離 [m]。 |
| `path_loss.pl0` | float | 40.0 | 基準距離でのパスロス [dB]。 |
| `path_loss.exponent` | float | 2.0 | パスロス指数 $n$。自由空間: 2.0、都市部: 2.7-3.5。 |

---

## 7. UGV制御とシミュレーションシナリオ

### 7.1. UGVの動作 (`ugv_controller_node`)

* **経路設定**: UGVは、設定ファイル (`sim_params.yaml`) から読み込んだ**マルチウェイポイントリスト**を順次追従する。
* **区間速度制御**: 各ウェイポイントは目標直線速度 (`V`) を持ち、UGVはその区間の目標速度を維持するように走行する。
    * ウェイポイントデータ形式: **`[X, Y, Z, V]`** (4要素)

#### ウェイポイントパラメータ例

```yaml
ugv_controller_node:
  ros__parameters:
    waypoints:
      - [-20.0, 0.0, 0.0, 2.78]  # 10 km/h
      - [20.0, 0.0, 0.0, 0.0]    # 停止
    waypoint_tolerance: 2.0      # [m]
    control_rate: 10.0           # [Hz]
    max_angular_velocity: 1.0    # [rad/s]
    heading_gain: 1.5            # 比例ゲイン
```

### 7.2. シミュレーション制御と終了条件

| 項目 | 詳細 |
| :--- | :--- |
| **全体終了条件** | UGVの全ウェイポイント到達をもって、シミュレーション全体を終了し、ROS 2ドメインをシャットダウンする。 |
| **通信停止条件** | 送信データ量 (`TotalDataTransmission`) が `comm_data_limit_mb` に達したとき、`comms_simulator_node` は**データ送信機能のみを停止**する。車両の移動とノードの実行は継続される。 |

---

## 8. ログ出力仕様

シミュレーション終了時、`comms_simulator_node` は収集したデータをCSV形式で出力する。

| 項目 | 詳細 |
| :--- | :--- |
| **出力タイミング** | ノードが終了するとき（`atexit`フックを使用）。 |
| **出力ディレクトリ** | `/workspace/log/sim_result/` |
| **ファイル命名規則** | `YYYYMMDD_HHMMSS_LIMIT-[上限値]MB.csv` <br> (`-1.0`の場合は `LIMIT-UNLIMITED.csv`)。 |
| **CSVヘッダー** | コメント行 (`# ...`) で以下の情報を記載: <br> - データ量制限 <br> - リンク確立時間 <br> - RSSI閾値 <br> - 伝搬路モデル名 <br> - 送信電力 <br> - 雑音分散 |

### CSV出力項目

| 項目名 | 単位 | 説明 |
| :--- | :--- | :--- |
| `time_s` | [s] | シミュレーション時刻 |
| `ugv_x`, `ugv_y`, `ugv_z` | [m] | UGV座標 |
| `bs_x`, `bs_y`, `bs_z` | [m] | 基地局座標 |
| `distance` | [m] | UGV-基地局間距離 |
| `rssi` | [dBm] | 受信信号強度 |
| `throughput` | [Gbps] | 瞬時スループット |
| `total_data_mb` | [Mb] | 累積送信データ量 |
| `path_loss` | [dB] | パスロス |
| `e_gain` | [dBi] | E面アンテナゲイン |
| `h_gain` | [dBi] | H面アンテナゲイン |
| `link_state` | - | リンク状態 (`DISCONNECTED`, `ESTABLISHING`, `CONNECTED`) |

### CSV出力例

```csv
# Communication Simulation Results
# Data Limit: 100.0 Mb
# Link Establishment Time: 2.0 ms
# RSSI Threshold: -61.0 dBm
# Model: Log-Distance Path Loss Model
# TX Power: -7.0 dBm
# Noise Variance: 2.0 dB
#
time_s,ugv_x,ugv_y,ugv_z,bs_x,bs_y,bs_z,distance,rssi,throughput,total_data_mb,path_loss,e_gain,h_gain,link_state
0.0,-20.0,0.0,0.0,0.0,6.0,3.0,20.88,-45.2,0.0,0.0,65.2,8.5,6.2,DISCONNECTED
1.0,-17.22,0.0,0.0,0.0,6.0,3.0,18.25,-42.8,0.0,0.0,60.8,9.2,6.8,ESTABLISHING
1.002,-17.22,0.0,0.0,0.0,6.0,3.0,18.25,-42.8,9.856,0.009856,60.8,9.2,6.8,CONNECTED
2.0,-14.44,0.0,0.0,0.0,6.0,3.0,15.92,-40.1,10.5,10.509856,58.1,9.8,7.1,CONNECTED
3.0,-11.66,0.0,0.0,0.0,6.0,3.0,13.75,-37.8,11.8,22.309856,55.8,10.2,7.5,CONNECTED
```

---

## 9. インターフェース

| 項目 | 詳細 |
| :--- | :--- |
| **GUIモード** | ホスト側のXサーバーを利用する**X11 Forwarding**を設定し、GazeboのGUI（クライアント）と物理サーバーを起動。 |
| **Headlessモード** | Gazeboを**Headlessモード (`gz sim -s`)**で起動し、GUI表示を省略。計算速度を優先する。YAMLの `simulation.headless` で切り替え。 |

---

## 10. 設計パターンと実装方針

### 10.1. Strategy パターン（伝搬路モデル）

**目的**: 伝搬路モデルの交換可能性を確保。

**実装**:
```python
class PropagationModel(ABC):
    @abstractmethod
    def calculate_path_loss(self, distance: float) -> float:
        pass

class LogDistancePathLossModel(PropagationModel):
    def calculate_path_loss(self, distance: float) -> float:
        return self.pl0 + 10 * self.exponent * np.log10(distance / self.d0)

class TwoRayGroundModel(PropagationModel):
    def calculate_path_loss(self, distance: float) -> float:
        # 実装...
```

### 10.2. 状態管理パターン

**目的**: リンク確立状態の明示的な管理。

**実装**:
```python
class LinkState(Enum):
    DISCONNECTED = 0
    ESTABLISHING = 1
    CONNECTED = 2

def _update_link_state(self, rssi: float, current_time: float) -> bool:
    if self.link_state == LinkState.DISCONNECTED:
        if rssi > self.rssi_threshold:
            self.link_state = LinkState.ESTABLISHING
            # ...
    elif self.link_state == LinkState.ESTABLISHING:
        # ...
```

---

## 11. テストと検証

### 11.1. 単体テスト

| 対象 | テスト項目 |
| :--- | :--- |
| `CommsCalculator` | - MCSテーブル読み込み <br> - RSSI計算の正確性 <br> - スループット線形補間 <br> - 飽和処理 |
| `AntennaPatternParser` | - CSV読み込み <br> - 角度計算 <br> - 線形補間 |
| `CommsSimulatorNode` | - リンク状態遷移 <br> - データ量管理 <br> - CSVロギング |

### 11.2. 統合テスト

| シナリオ | 確認項目 |
| :--- | :--- |
| **基本動作** | UGVが移動し、通信品質が計算され、CSVに出力される。 |
| **リンク確立** | RSSI閾値を超えてから2ms後に通信が開始される。 |
| **データ量制限** | 設定された上限に達したら通信が停止する。 |
| **再接続** | RSSI低下後、再び閾値を超えたら再度リンク確立が行われる。 |

---

## 12. 今後の拡張予定

| 項目 | 概要 |
| :--- | :--- |
| **NLOS伝搬モデル** | 障害物による遮蔽を考慮。 |
| **複数基地局** | ハンドオーバーのシミュレーション。 |
| **リアルタイム可視化** | RViz2またはGazebo GUIでの通信品質表示。 |
| **ビームフォーミング** | 指向性制御のシミュレーション。 |
| **干渉モデル** | 複数送信機による干渉の考慮。 |

---

**Document Version**: 3.0  
**Last Updated**: 2025年12月22日  
**Status**: Active Development
