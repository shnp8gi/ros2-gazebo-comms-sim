<!-- filepath: /home/yagi/ros2-gazebo-comms-sim/specification.md -->

# 📡 ROS 2/Gazebo 通信シミュレータ 統合システム仕様書

## 変更履歴

| 変更日     | バージョン | 改定内容                                                                                                                                                                                                                                                            |
| :--------- | :--------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| 2025/12/14 | ver 1.0    | 初版                                                                                                                                                                                                                                                                |
| 2025/12/14 | ver 2.0    | UGVのマルチウェイポイント追従と区間速度制御、アンテナゲインの動的参照、通信容量上限による通信停止機能の追加、およびCSVロギング仕様の確定                                                                                                                            |
| 2025/12/22 | ver 3.0    | MCSテーブルベースのスループット計算への変更、リンク確立時間（Association time）の実装、RSSI閾値の自動取得機能、YAMLベースの完全パラメータ化                                                                                                                         |
| 2026/01/04 | ver 3.1    | 実装（`sim_params.yaml`/`sim_launch.py`/`comms_node.py`）に合わせて、CSV出力先・ROSインターフェース・パラメータ定義の齟齬を修正                                                                                                                                     |
| 2026/02/16 | ver 4.0    | MCSテーブル値を現行に更新、アンテナオフセットを3D相対座標`[x,y,z]`に変更、CSV出力列名にbody/antenna/origin区分を追加、座標系説明の追加、`sampling_rate`デフォルト値修正、`base_station_pose_topic`インターフェース追加、CSV出力タイミングにmission_complete時を追記 |

---

## 1. プロジェクト概要

Gazebo Simで駆動する移動車両と固定基地局間の通信品質（RSSI/スループット）を、ROS 2ノード上で数理モデルを用いてシミュレーションし、その挙動を評価する。

---

## 2. 技術スタックと環境構築

| 分野             | 項目                   | 決定事項                            | 備考                                             |
| :--------------- | :--------------------- | :---------------------------------- | :----------------------------------------------- |
| **OS**           | ベースOS               | Ubuntu 22.04 LTS (Dockerコンテナ内) |                                                  |
| **ROS 2**        | ディストリビューション | **Humble Hawksbill (LTS)**          | サポート期間: 2027年5月まで。                    |
| **シミュレータ** | 種類                   | **Gazebo Sim (Harmonic)**           | ROS 2との連携、将来性を重視。                    |
| **開発環境**     | コンテナ               | **Docker** (推奨)                   | 環境の再現性、GUI/CUI切り替えをサポート。        |
| **実装言語**     | 主言語                 | **Python 3**                        | 通信計算（数理モデル）の柔軟性と開発速度を優先。 |

---

## 3. シミュレーション環境 (World & Models)

| 項目               | 詳細                                                                            | Fuel URI / 構成                                                        |
| :----------------- | :------------------------------------------------------------------------------ | :--------------------------------------------------------------------- |
| **ワールド**       | シンプルな無限平面 (Empty World + Ground Plane)。将来的な物体設置は可能とする。 | `minimal_world.sdf`                                                    |
| **移動車両 (UGV)** | SUVモデルに駆動系とセンサをアタッチ。                                           | **Fuel: `https://app.gazebosim.org/OpenRobotics/fuel/models/SUV`**     |
| **基地局**         | 高さのあるアンテナ塔モデル。固定設置。                                          | **Fuel: `https://app.gazebosim.org/OpenRobotics/fuel/models/antenna`** |
| **モデル参照**     | Gazebo Fuelからローカルにダウンロードし、Dockerでマウントして参照する。         | `GZ_SIM_RESOURCE_PATH` を設定。                                        |

### 3.1. 座標系とモデル原点

**ワールド座標系**: ENU（East-North-Up）に準拠。

| 軸  | 方向      |
| :-- | :-------- |
| X   | 前方 (正) |
| Y   | 左方 (正) |
| Z   | 上方 (正) |

**SUVモデル原点**: ホイールベース中央・トレッド中央・地面レベル (z=0)

```
    前方 (+X)
      ↑
FL ●─────● FR      (x=+1.5)
   │     │
   │  ★  │  ← ★ = モデル原点 (0,0,0) = 地面レベル
   │     │
RL ●─────● RR      (x=-1.5)
      ↓
    後方 (-X)
```

- chassis リンクは z=+0.5m にオフセット、車体上面は約 z≈2.0m

**`antenna_offset`**: エンティティ原点からの3D相対座標 `[x, y, z]` [m]

| 設置例                | offset値           |
| :-------------------- | :----------------- |
| SUV屋根中央にアンテナ | `[0.0, 0.0, 2.23]` |
| SUV屋根前方にアンテナ | `[1.0, 0.0, 2.23]` |

## 4. ディレクトリ構成

```
ros2-gazebo-comms-sim/
├── config/                        # 設定ファイル
│   ├── sim_params.yaml            # メインパラメータファイル
│   ├── e_plane.csv                # Eプレーン（垂直面）アンテナパターン
│   ├── h_plane.csv                # Hプレーン（水平面）アンテナパターン
│   └── MCStable.csv               # MCS（変調・符号化方式）テーブル
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
├── comms_sim_pkg/                 # ROS 2パッケージ
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── setup.py
│   │
│   ├── comms_sim_pkg/             # Pythonモジュール
│   │   ├── __init__.py
│   │   ├── comms_node.py          # 通信シミュレータノード
│   │   ├── comms_calculator.py    # 通信品質計算エンジン
│   │   ├── antenna_parser.py      # アンテナパターン処理
│   │   └── ugv_controller_node.py # UGV制御ノード
│   │
│   ├── launch/                    # 起動ファイル
│   │   └── sim_launch.py          # メインランチファイル
│   │
│   └── resource/                  # リソースファイル
│       └── minimal_world.sdf      # シミュレーションワールド
│
├── comms_sim_msgs/                # メッセージ定義パッケージ
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── msg/
│       └── CommsQuality.msg       # 通信品質メッセージ
│
├── log/                           # colcon/実行ログ
├── sim_results/                   # CSVログ出力ディレクトリ（実装の出力先）
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

| エンティティ | センサ/プラグイン       | ROS 2 トピック                               | 用途                                                                                      |
| :----------- | :---------------------- | :------------------------------------------- | :---------------------------------------------------------------------------------------- |
| **車両**     | Diff Drive (プラグイン) | `/cmd_vel` (Subscribe)                       | 運動制御。                                                                                |
| **車両**     | IMU                     | `/imu/data` (Publish)                        | 姿勢情報 (Roll, Pitch, Yaw) の提供。**アンテナゲイン計算**に利用。                        |
| **車両**     | Odometry                | `/odom` (Publish)                            | 位置（ワールド座標系に補正して利用）・速度の提供。                                        |
| **基地局**   | N/A                     | パラメータ/スポーン設定から座標を取得        | 静的なためセンサ不要。                                                                    |
| **基地局**   | N/A                     | `/base_station/pose` (Subscribe、オプション) | 基地局の姿勢をアンテナ方向計算に利用（パラメータ `base_station_pose_topic` で変更可能）。 |

> 注: `NavSat (GNSS)` / `/gps/fix` は仕様として想定しているが、現状の `sim_params.yaml`（`ros_gz_bridge.bridge_topics`）ではブリッジ設定が未定義のため、実装は `/odom` を位置入力として用いる。

---

## 6. 通信シミュレーション要件 (ROS 2 Node)

### 6.1. 通信ノード (`comms_simulator_node`)

| 項目         | 詳細                                                                                                                                      |
| :----------- | :---------------------------------------------------------------------------------------------------------------------------------------- |
| **ノード名** | `comms_simulator_node`                                                                                                                    |
| **実装言語** | Python 3                                                                                                                                  |
| **役割**     | Gazeboから取得した位置情報（/odom）と姿勢情報（/imu/data）に基づき、カスタムの伝搬路モデルで通信品質を計算し、ロギング及びPublishを行う。 |

### 6.2. 通信インターフェース

| 項目               | トピック名/パラメータ | メッセージ型                  | 送受信    | 備考                                                                  |
| :----------------- | :-------------------- | :---------------------------- | :-------- | :-------------------------------------------------------------------- |
| **車両位置**       | `/odom`               | `nav_msgs/Odometry`           | Subscribe | Gazeboから車両の位置を取得（`spawn_pose` によりワールド座標へ補正）。 |
| **車両姿勢**       | `/imu/data`           | `sensor_msgs/Imu`             | Subscribe | Gazeboから車両の姿勢を取得。                                          |
| **通信結果**       | `/comms/quality`      | カスタム (`CommsQuality.msg`) | Publish   | 計算されたRSSI値とスループットを出力。                                |
| **ミッション完了** | `/mission_complete`   | `std_msgs/Bool`               | Subscribe | UGV完走通知。受信時にCSV保存をトリガする。                            |
| **基地局姿勢**     | `/base_station/pose`  | `geometry_msgs/PoseStamped`   | Subscribe | 基地局の姿勢（オプション）。アンテナ方向計算に利用。                  |

### 6.3. 伝搬路モデル計算ロジック

| 項目               | 詳細                                                                                                                                                                |
| :----------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| **計算頻度**       | **可変サンプリングレート** (`sampling_rate` パラメータで調整可能)。デフォルトはYAMLに従う。                                                                         |
| **パスロス**       | **対数距離減衰モデル** ($PL(d) = 10 \times n \times \log_{10}(4 \pi d/\lambda)$) をベースとする。                                                                   |
| **RSSI計算**       | $RSSI = P_t - PL(d) + G_a + N$ <br> - $P_t$: 送信電力 [dBm] <br> - $PL(d)$: パスロス [dB] <br> - $G_a$: アンテナゲイン [dBi]（Tx/Rx合成） <br> - $N$: AWGN雑音 [dB] |
| **雑音 (AWGN)**    | **正規分布に従うAWGN** ($\mathcal{N}(0, \sigma^2)$) を逐一加算。分散 $\sigma^2$ はYAMLで設定可能。                                                                  |
| **アンテナゲイン** | **CSVファイル**（E面/H面ゲイン）を参照。アンテナ座標系に変換した方向ベクトルから、E面（仰角）/H面（方位角）ゲインを線形補間して取得する。                           |
| **スループット**   | **MCSテーブル（CSV）を参照し、ステップ関数により決定**。（6.4.項を参照）                                                                                            |
| **拡張性**         | 伝搬路モデルの計算ロジックは、**Strategyパターン**を適用し、将来のNLOS/反射波モデルへの差し替えを容易にする。                                                       |

### 6.4. スループット決定方式

CSVファイル（`MCStable.csv`）を参照する方式

#### MCSテーブル（`MCStable.csv`）の形式

```csv
# RSSI [dBm], Throughput [Gbps]
-50.5, 6.000
-54, 4.600
-58.5, 2.700
-62.5, 2.150
-65.5, 1.100
-68.5, 0.500
```

#### スループット計算ルール

RSSIが各MCSレベルの閾値を満たす最大のMCSレベルのスループットをそのまま適用する。

| RSSI範囲                         | スループット              | 計算方法                                    |
| :------------------------------- | :------------------------ | :------------------------------------------ |
| $RSSI \le RSSI_{min}$            | 0.0 Gbps                  | 通信不可                                    |
| $RSSI_{min} < RSSI < RSSI_{max}$ | MCSテーブル該当レベルの値 | `numpy.searchsorted` によるステップ関数参照 |
| $RSSI \ge RSSI_{max}$            | $Throughput_{max}$        | 飽和（最大スループット）                    |

**例**: MCSテーブルが上記の場合
| RSSI範囲 | 適用スループット |
| :--- | :--- |
| $RSSI \le -68.5$ | 0.0 Gbps |
| $-68.5 < RSSI < -65.5$ | 0.500 Gbps |
| $-65.5 \le RSSI < -62.5$ | 1.100 Gbps |
| $-62.5 \le RSSI < -58.5$ | 2.150 Gbps |
| $-58.5 \le RSSI < -54$ | 2.700 Gbps |
| $-54 \le RSSI < -50.5$ | 4.600 Gbps |
| $RSSI \ge -50.5$ | 6.000 Gbps |

**重要**:

- $RSSI_{min}$ と $RSSI_{max}$ は**CSVファイルから自動取得**される
- 上記の例では、$RSSI_{min} = -68.5$ dBm、$RSSI_{max} = -50.5$ dBm。

### 6.5. リンク確立時間（Association Time）の実装

現実の無線通信における**リンク確立遅延**をシミュレート。

#### リンク状態遷移

```
DISCONNECTED → ESTABLISHING → CONNECTED
     ↓              ↓              ↓
  RSSI < 閾値   リンク確立中    通信可能
                (待機時間中)
```

| 状態             | 条件                             | 通信可否 | 遷移条件                                                           |
| :--------------- | :------------------------------- | :------- | :----------------------------------------------------------------- |
| **DISCONNECTED** | $RSSI \le RSSI_{min}$            | 不可     | $RSSI > RSSI_{min}$ で ESTABLISHING へ                             |
| **ESTABLISHING** | $RSSI > RSSI_{min}$ かつ待機中   | 不可     | 設定時間経過 → CONNECTED <br> $RSSI \le RSSI_{min}$ → DISCONNECTED |
| **CONNECTED**    | $RSSI > RSSI_{min}$ かつ確立済み | 可能     | $RSSI \le RSSI_{min}$ → DISCONNECTED                               |

#### パラメータ

| パラメータ名                 | 型    | デフォルト値 | 説明                              |
| :--------------------------- | :---- | :----------- | :-------------------------------- |
| `link_establishment_time_ms` | float | 2.0          | リンク確立時間 [ms]。YAMLで設定。 |

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

| パラメータ                          | 型             | デフォルト値                                  | 説明                                                           |
| :---------------------------------- | :------------- | :-------------------------------------------- | :------------------------------------------------------------- |
| `sampling_rate`                     | float          | 500.0                                         | 通信計算の更新頻度 [Hz]。                                      |
| `tx_power`                          | float          | -7.0                                          | 送信電力 [dBm]。                                               |
| `noise_variance`                    | float          | 2.0                                           | AWGNの分散値 [dB]。                                            |
| `mcs_table_path`                    | string         | `/workspace/config/MCStable.csv`              | MCSテーブルCSVファイルへのパス。                               |
| `link_establishment_time_ms`        | float          | 2.0                                           | リンク確立時間 [ms]。                                          |
| `e_plane_path`                      | string         | `/workspace/config/e_plane.csv`               | E面ゲインCSVファイルへのパス。                                 |
| `h_plane_path`                      | string         | `/workspace/config/h_plane.csv`               | H面ゲインCSVファイルへのパス。                                 |
| `comm_data_limit_mb`                | float          | 800.0                                         | 通信データ量の上限 [Mb]。`-1.0`で無制限。                      |
| `base_station_position`             | list [x, y, z] | `spawn_entities.antenna.pose[0:3]`            | 基地局のワールド座標（スポーン設定から取得して起動時に反映）。 |
| `base_station_antenna_offset`       | list [x, y, z] | `[0.0, 0.0, 3.0]`                             | 基地局モデル原点からのアンテナ位置オフセット [m]。             |
| `ugv_spawn_pose`                    | list [x, y, z] | `spawn_entities.suv.pose[0:3]`                | UGVスポーンワールド座標（/odom → ワールド補正に使用）。        |
| `ugv_antenna_offset`                | list [x, y, z] | `[0.0, 0.0, 1.9]`                             | UGVモデル原点からのアンテナ位置オフセット [m]。                |
| `ugv_antenna_relative_rpy`          | list [r, p, y] | `spawn_entities.suv.antenna_relative_rpy`     | UGVアンテナの相対姿勢 [rad]。                                  |
| `base_station_antenna_relative_rpy` | list [r, p, y] | `spawn_entities.antenna.antenna_relative_rpy` | 基地局アンテナの相対姿勢 [rad]。                               |
| `base_station_pose_topic`           | string         | `/base_station/pose`                          | 基地局姿勢トピック名（オプション）。                           |

#### パスロスモデルパラメータ

| パラメータ            | 型    | デフォルト値 | 説明                                               |
| :-------------------- | :---- | :----------- | :------------------------------------------------- |
| `path_loss.c`         | float | 299792458    | 光速 [m/s]。                                       |
| `path_loss.frequency` | float | 6.0e10       | 使用周波数 [Hz]。                                  |
| `path_loss.exponent`  | float | 2.0          | パスロス指数 $n$。自由空間: 2.0、都市部: 2.7-3.5。 |

---

## 7. UGV制御とシミュレーションシナリオ

### 7.1. UGVの動作 (`ugv_controller_node`)

- **経路設定**: UGVは、設定ファイル (`sim_params.yaml`) から読み込んだ**マルチウェイポイントリスト**を順次追従する。
- **区間速度制御**: 各ウェイポイントは目標直線速度 (`V`) を持ち、UGVはその区間の目標速度を維持するように走行する。
  - ウェイポイントデータ形式: **`[X, Y, Z, V]`** (4要素)

#### ウェイポイントパラメータ例

```yaml
ugv_controller_node:
  ros__parameters:
    waypoints:
      - [-20.0, 0.0, 0.0, 2.78] # 10 km/h
      - [20.0, 0.0, 0.0, 0.0] # 停止
    waypoint_tolerance: 2.0 # [m]
    control_rate: 10.0 # [Hz]
    max_angular_velocity: 1.0 # [rad/s]
    heading_gain: 1.5 # 比例ゲイン
```

### 7.2. シミュレーション制御と終了条件

| 項目             | 詳細                                                                                                                                                                           |
| :--------------- | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **全体終了条件** | UGVの全ウェイポイント到達をもって、シミュレーション全体を終了し、ROS 2ドメインをシャットダウンする。                                                                           |
| **通信停止条件** | 送信データ量 (`TotalDataTransmission`) が `comm_data_limit_mb` に達したとき、`comms_simulator_node` は**データ送信機能のみを停止**する。車両の移動とノードの実行は継続される。 |

---

## 8. ログ出力仕様

`comms_simulator_node` は収集したデータをCSV形式で出力する。

| 項目                 | 詳細                                                                                                                                                                       |
| :------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **出力タイミング**   | (1) ミッション完了通知（`/mission_complete`）受信時、(2) ノード終了時（`atexit`フック）。                                                                                  |
| **出力ディレクトリ** | `/workspace/sim_results/`                                                                                                                                                  |
| **ファイル命名規則** | `YYYYMMDD_HHMMSS_LIMIT-[上限値]MB.csv` <br> (`-1.0`の場合は `LIMIT-UNLIMITED.csv`)。                                                                                       |
| **CSVヘッダー**      | コメント行 (`# ...`) で以下の情報を記載: <br> - データ量制限 <br> - 伝搬路モデル名 <br> - 送信電力 <br> - 雑音分散 <br> - 各座標カラムの定義（body/antenna/origin の意味） |

### CSV出力項目

各座標カラムの意味:

- **`ugv_body`**: UGV車両モデル原点（ホイールベース中央・地面レベル）
- **`ugv_antenna`**: UGVアンテナ位置 = `ugv_body` + `antenna_offset`
- **`bs_origin`**: 基地局モデル原点
- **`bs_antenna`**: 基地局アンテナ位置 = `bs_origin` + `antenna_offset`
- **`distance`**: `ugv_antenna` と `bs_antenna` 間の3D距離

| 項目名                                            | 単位   | 説明                                                     |
| :------------------------------------------------ | :----- | :------------------------------------------------------- |
| `time_s`                                          | [s]    | シミュレーション時刻                                     |
| `ugv_body_x`, `ugv_body_y`, `ugv_body_z`          | [m]    | UGV車体原点座標                                          |
| `ugv_antenna_x`, `ugv_antenna_y`, `ugv_antenna_z` | [m]    | UGVアンテナ位置座標                                      |
| `bs_origin_x`, `bs_origin_y`, `bs_origin_z`       | [m]    | 基地局モデル原点座標                                     |
| `bs_antenna_x`, `bs_antenna_y`, `bs_antenna_z`    | [m]    | 基地局アンテナ位置座標                                   |
| `distance`                                        | [m]    | UGVアンテナ-基地局アンテナ間3D距離                       |
| `rssi`                                            | [dBm]  | 受信信号強度                                             |
| `throughput`                                      | [Gbps] | 瞬時スループット                                         |
| `total_data_mb`                                   | [Mb]   | 累積送信データ量                                         |
| `path_loss`                                       | [dB]   | パスロス                                                 |
| `e_gain`                                          | [dBi]  | E面アンテナゲイン                                        |
| `h_gain`                                          | [dBi]  | H面アンテナゲイン                                        |
| `link_state`                                      | -      | リンク状態 (`DISCONNECTED`, `ESTABLISHING`, `CONNECTED`) |

### CSV出力例

```csv
# 通信シミュレーション結果
# データ上限: 800.0 Mb
# 伝搬路モデル: Log-Distance Path Loss Model
# 送信電力: -7.0 dBm
# 雑音分散: 2.0 dB
#
# 列の座標定義:
#   ugv_body    = UGV車体モデル原点（ホイールベース中心・地面レベル）
#   ugv_antenna = UGVアンテナ位置（ugv_body + antenna_offset）
#   bs_origin   = 基地局モデル原点
#   bs_antenna  = 基地局アンテナ位置（bs_origin + antenna_offset）
#   distance    = ugv_antenna と bs_antenna 間の3D距離
#
time_s,ugv_body_x,ugv_body_y,ugv_body_z,ugv_antenna_x,ugv_antenna_y,ugv_antenna_z,bs_origin_x,bs_origin_y,bs_origin_z,bs_antenna_x,bs_antenna_y,bs_antenna_z,distance,rssi,throughput,total_data_mb,path_loss,e_gain,h_gain,link_state
0.0,-20.0,0.0,0.0,-20.0,0.0,2.23,0.0,3.0,0.0,0.0,3.0,2.79,20.42,-55.2,0.0,0.0,65.2,8.5,6.2,DISCONNECTED
```

---

## 9. インターフェース

| 項目               | 詳細                                                                                                                           |
| :----------------- | :----------------------------------------------------------------------------------------------------------------------------- |
| **GUIモード**      | ホスト側のXサーバーを利用する**X11 Forwarding**を設定し、GazeboのGUI（クライアント）と物理サーバーを起動。                     |
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
        return 10 * self.exponent * np.log10(4 * pi * distance / lambda)

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

| 対象                   | テスト項目                                                                                    |
| :--------------------- | :-------------------------------------------------------------------------------------------- |
| `CommsCalculator`      | - MCSテーブル読み込み <br> - RSSI計算の正確性 <br> - スループットステップ関数 <br> - 飽和処理 |
| `AntennaPatternParser` | - CSV読み込み <br> - 角度計算 <br> - 線形補間                                                 |
| `CommsSimulatorNode`   | - リンク状態遷移 <br> - データ量管理 <br> - CSVロギング                                       |

### 11.2. 統合テスト

| シナリオ         | 確認項目                                                 |
| :--------------- | :------------------------------------------------------- |
| **基本動作**     | UGVが移動し、通信品質が計算され、CSVに出力される。       |
| **リンク確立**   | RSSI閾値を超えてから2ms後に通信が開始される。            |
| **データ量制限** | 設定された上限に達したら通信が停止する。                 |
| **再接続**       | RSSI低下後、再び閾値を超えたら再度リンク確立が行われる。 |

---

## 12. 今後の拡張予定

| 項目                   | 概要                                    |
| :--------------------- | :-------------------------------------- |
| **NLOS伝搬モデル**     | 障害物による遮蔽を考慮。                |
| **複数基地局**         | ハンドオーバーのシミュレーション。      |
| **リアルタイム可視化** | RViz2またはGazebo GUIでの通信品質表示。 |
| **ビームフォーミング** | 指向性制御のシミュレーション。          |
| **干渉モデル**         | 複数送信機による干渉の考慮。            |

---

**Document Version**: 4.0  
**Last Updated**: 2026年2月16日  
**Status**: Active Development
