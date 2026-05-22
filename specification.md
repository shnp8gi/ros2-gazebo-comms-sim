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
| 2026/04/20 | ver 5.0    | 複数車両設定（マルチUGV）への対応、`link_controller_node`による集中スケジューリング（RSSI優先による通信アクセス権の調停）の追加、データ上限到達時の動的なリンク権限譲渡機能、`/cmd_vel`監視による時間軸(`time_s`, `vehicle_time_s`)起点のゼロ化、全ミッション完了時の自動終了、およびログノードの集約と個別CSV分離出力を追加 |
| 2026/05/01 | ver 6.0    | DockerでのNVIDIA GPUアクセラレーション対応、幾何学スコアや推定受信電力（physical_score_priority）に基づくスケジューリングポリシーの追加、切断時のプロアクティブハンドオーバー機能実装、および高速走行時のスリップ防止用加速度制限制御の追加 |
| 2026/05/15 | ver 7.0    | カスタムモデルの追加要件（SDF記述ルール）の明文化、タイヤの空転による誤差を排除するGround Truthオドメトリ（`OdometryPublisher`）の採用、`VelocityControl`を用いた物理干渉なしの厳密な加速度制御の導入、複数アンテナ搭載時のロギング終了判定の修正、およびGazebo初期化時の座標ズレを自動修正する「初期位置検証・自己補正機能」の追加 |
| 2026/05/21 | ver 8.0    | nominal RSSIヒートマップ事前計算に基づく車載アンテナのフィードフォワード制御ポリシー（`feedforward_optimal`）の追加、ロギングレベル5（ヒートマップ出力＋制御ログ）の導入、およびログ保存ディレクトリの3階層フォルダリング（`comms/`, `control/`, `heatmap/`）によるカプセル化（可視性向上）の実装 |

---

## 1. プロジェクト概要

Gazebo Simで駆動する複数台の移動車両と固定基地局間の通信品質（RSSI/スループット）を、ROS 2ノード上で数理モデルを用いてシミュレーションし、その挙動を評価する。特にIEEE 802.15.3eなどを想定した、基地局による集中スケジューリングと時分割・優先制御を模擬する。

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
   │          │
   │    ★    │  ← ★ = モデル原点 (0,0,0) = 地面レベル
   │          │
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

### 3.2. カスタムモデルの追加と要件

本システムは特定の車両モデル（SUVや新幹線）に依存せず、YAMLファイルの `model_uri` 指定によって任意の自作モデルを動的にロード可能である。
新しいモデルを追加し、UGVコントローラーで正常に制御・ロギングさせるためには、モデルの SDF ファイル (`models/<ModelName>/model.sdf`) に以下の要件を満たすプラグインを記述する必要がある。

1. **速度入力プラグイン (`cmd_vel`)**
   - 車両を動かすためのプラグイン（`gz::sim::systems::DiffDrive` や `gz::sim::systems::VelocityControl`）を含める。
   - トピック名は `<topic>cmd_vel</topic>` と固定で記述する（起動時にシステムが各車両専用のネームスペースに自動で書き換える）。
   - **VelocityControlの利用**: 車輪のスリップを排除し、指定した加速度を100%忠実に再現したい場合は、摩擦をゼロにし `VelocityControl` を採用する。

2. **位置計測プラグイン (`odom`)**
   - 車両の現在位置をシステムへフィードバックするためのプラグイン（`gz::sim::systems::OdometryPublisher` など）を含める。
   - トピック名は `<odom_topic>odom</odom_topic>` と固定で記述する。
   - **Ground Truthの利用**: 物理的な車輪の空転によるオドメトリ誤差を防ぐため、`DiffDrive` のオドメトリトピックを `odom_wheel` 等に退避させ、`OdometryPublisher` を用いて絶対座標系の正しい位置を配信することが推奨される。

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
│   │   ├── link_controller_node.py# 集中通信アクセス調停（スケジュール）ノード
│   │   ├── link_scheduling_strategy.py # スケジュール戦略パターン（RSSI優先等）
│   │   ├── sim_logger_node.py     # ログ集約・CSV自動保存ノード
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
│   └── sweep_<sweep_timestamp>/   # スイープ全体の実行ログフォルダ（単一実行時は run_<run_timestamp>）
│       ├── sweep_summary.csv      # スイープ全体のサマリーCSV（または sweep_summary_run*.csv）
│       └── runs/                  # 各個別シミュレーションランの結果
│           └── run_<run_idx>_y<Y>_a<ANGLE>/ # 個別ランのパラメータフォルダ
│               ├── comms/         # 通信品質関連の時系列データ ({vehicle_name}_connected.csv / {vehicle_name}_full.csv)
│               ├── control/       # 制御およびハンドオーバーイベントデータ (events.csv, feedforward_log.csv)
│               └── heatmap/       # アンテナ制御用事前計算ヒートマップ (rssi_heatmap.csv)
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
| **車両**     | Diff Drive / Velocity Control | `/cmd_vel` (Subscribe)                       | 運動制御。高速走行時のスリップ対策として `VelocityControl` も利用される。 |
| **車両**     | IMU                     | `/imu/data` (Publish)                        | 姿勢情報 (Roll, Pitch, Yaw) の提供。**アンテナゲイン計算**に利用。                        |
| **車両**     | OdometryPublisher       | `/odom` (Publish)                            | 位置（ワールド座標系に補正して利用）・速度の提供。車輪の空転に依存しないGround Truth座標を使用。 |
| **基地局**   | N/A                     | パラメータ/スポーン設定から座標を取得        | 静的なためセンサ不要。                                                                    |
| **基地局**   | N/A                     | `/base_station/pose` (Subscribe、オプション) | 基地局の姿勢をアンテナ方向計算に利用（パラメータ `base_station_pose_topic` で変更可能）。 |

> 注: `NavSat (GNSS)` / `/gps/fix` は仕様として想定しているが、現状の `sim_params.yaml`（`ros_gz_bridge.bridge_topics`）ではブリッジ設定が未定義のため、実装は `OdometryPublisher` 等による `/odom` を位置入力として用いる。

---

## 6. 通信シミュレーション要件 (ROS 2 Node)

### 6.1. 通信ノード (`comms_simulator_node`)

| 項目         | 詳細                                                                                                                                      |
| :----------- | :---------------------------------------------------------------------------------------------------------------------------------------- |
| **ノード名** | `comms_simulator_node`                                                                                                                    |
| **実装言語** | Python 3                                                                                                                                  |
| **役割**     | 複数分散配置される車両ごとで軌道と通信品質を計算する。基地局の調停ノード(`link_controller_node`)からのアクセス許可(`link_grant`)に従い通信状態を維持し、結果をロギングノードへパブリッシュする。<br>**堅牢性(Robustness)**: 起動直後にGazeboから受信したオドメトリとYAMLのスポーン座標を比較し、初期化のラグによる座標ズレ（10m以上）を検知した場合はオフセットを再計算して自己補正を行う初期位置検証機能を持つ。 |

### 6.1.2. リンクコントローラノード (`link_controller_node`)

| 項目         | 詳細                                                                                                                                      |
| :----------- | :---------------------------------------------------------------------------------------------------------------------------------------- |
| **ノード名** | `link_controller_node`                                                                                                                    |
| **役割**     | 基地局として特定のアクセス権(`link_grant`)を一つの車両にのみ割り当てる集中調停ノード。<br> **ポリシー一覧**: <br>- `sequential`: 順次切替 <br>- `round_robin`: 時間単位切替 <br>- `rssi_priority`: RSSI最大優先 <br>- `physical_score_priority`: 推定受信電力（ゲイン-ロス）優先 <br>- `geometric_beam_priority`: 幾何学的アライメント優先 <br>- `geometric_weighted`: 距離とアライメントの重み付け加算優先 <br>- `feedforward_optimal`: 事前計算されたnominal RSSIヒートマップ（LUT）に基づく車載アンテナのフィードフォワード制御ポリシー <br>★さらに、現在の通信局が `DISCONNECTED` 状態になった際、即座に最適な幾何学スコアを持つ後続車両へ通信権を移行させる「**プロアクティブハンドオーバー**」機能を搭載。 |

### 6.2. 通信インターフェース

| 項目               | トピック名/パラメータ | メッセージ型                  | 送受信    | 備考                                                                  |
| :----------------- | :-------------------- | :---------------------------- | :-------- | :-------------------------------------------------------------------- |
| **車両位置**       | `/odom`               | `nav_msgs/Odometry`           | Subscribe | Gazeboから車両の位置を取得。                                |
| **車両姿勢**       | `/imu/data`           | `sensor_msgs/Imu`             | Subscribe | Gazeboから車両の姿勢を取得。                                          |
| **通信結果**       | `/{vehicle}/comms/quality` | カスタム (`CommsQuality.msg`) | Publish   | 計算された通信状態をログ集約ノードへ出力。                                |
| **アクセス要求**   | `/{vehicle}/link_request` | `std_msgs/Float64`            | Publish   | コントローラへRSSIを報告して通信権限を要求。上限到達時は `-inf` を送信し権限を譲渡。 |
| **アクセス許可**   | `/{vehicle}/link_grant`   | `std_msgs/Bool`               | Subscribe | コントローラから通信権の付与を受け取る。                             |
| **ミッション完了** | `/{vehicle}/mission_complete` | `std_msgs/Bool`               | Publish/Sub | UGV完走通知。ログ集約ノードが全完了を受信し自動終了をトリガ。 |
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
| **DISCONNECTED** | $RSSI \le RSSI_{min}$            | 不可     | $RSSI > RSSI_{min}$ で ESTABLISHING へ <br> **※リンク制御側では、この状態を検知した瞬間に即座に他の最適局へ通信権を強制切替（ハンドオーバー）する処理が走ります** |
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

#### 全体シミュレーションパラメータ

| パラメータ | 型 | デフォルト値 | 説明 |
| :--- | :--- | :--- | :--- |
| `simulation.logging_level` | int | 5 | ログ記録レベル (1〜5)。<br>1: サマリーのみ<br>2: サマリー + イベントログ<br>3: サマリー + イベント + 時系列 (CONNECTEDのみ)<br>4: サマリー + イベント + 時系列 (常時)<br>5: サマリー + イベント + 時系列 (常時) + アンテナ制御ログ + ヒートマップ |
| `simulation.headless` | bool | true | ヘッドレスモード (true: GUIなし, false: GUIあり)。 |
| `simulation.real_time_factor` | float | 5.0 | リアルタイム倍率。 |

#### 通信シミュレータノードパラメータ

| パラメータ                          | 型             | デフォルト値                                  | 説明                                                           |
| :---------------------------------- | :------------- | :-------------------------------------------- | :------------------------------------------------------------- |
| `sampling_rate`                     | float          | 1000.0                                        | 通信計算の更新頻度 [Hz]。                                      |
| `tx_power`                          | float          | -7.0                                          | 送信電力 [dBm]。                                               |
| `noise_variance`                    | float          | 1.0                                           | AWGNの分散値 [dB]。                                            |
| `mcs_table_path`                    | string         | `/workspace/config/MCStable.csv`              | MCSテーブルCSVファイルへのパス。                               |
| `link_establishment_time_ms`        | float          | 2.0                                           | リンク確立時間 [ms]。                                          |
| `e_plane_path`                      | string         | `/workspace/config/e_plane.csv`               | E面ゲインCSVファイルへのパス。                                 |
| `h_plane_path`                      | string         | `/workspace/config/h_plane.csv`               | H面ゲインCSVファイルへのパス。                                 |
| `comm_data_limit_mb`                | float          | -1.0                                          | 通信データ量の上限 [Mb]。`-1.0`で無制限。                      |
| `max_antenna_attenuation`           | float          | 30.0                                          | ゲイン低下時の最大減衰量クランプ値 [dB]。                      |
| `logging_start_trigger`             | string         | `on_movement`                                 | ログ記録開始条件 (`immediate`, `on_movement`, `on_topic`)。    |
| `base_station_position`             | list [x, y, z] | `spawn_entities.antenna.pose[0:3]`            | 基地局のワールド座標（スポーン設定から取得して起動時に反映）。 |
| `base_station_antenna_offset`       | list [x, y, z] | `[0.0, 0.0, 3.0]`                             | 基地局モデル原点からのアンテナ位置オフセット [m]。             |
| `ugv_spawn_pose`                    | list [x, y, z] | `spawn_entities.suv.pose[0:3]`                | UGVスポーンワールド座標（/odom → ワールド補正に使用）。        |
| `ugv_antenna_offset`                | list [x, y, z] | `[0.0, 0.0, 1.9]`                             | UGVモデル原点からのアンテナ位置オフセット [m]。                |
| `ugv_antenna_relative_rpy`          | list [r, p, y] | `spawn_entities.suv.antenna_relative_rpy`     | UGVアンテナの相対姿勢 [rad]。                                  |
| `base_station_antenna_relative_rpy` | list [r, p, y] | `spawn_entities.antenna.antenna_relative_rpy` | 基地局アンテナの相対姿勢 [rad]。                               |
| `base_station_pose_topic`           | string         | `/base_station/pose`                          | 基地局姿勢トピック名（オプション）。                           |

#### リンクコントローラノードパラメータ

| パラメータ             | 型     | デフォルト値              | 説明                                                                     |
| :--------------------- | :----- | :------------------------ | :----------------------------------------------------------------------- |
| `scheduling_policy`    | string | `physical_score_priority` | スケジューリングポリシー。(`sequential`, `round_robin`, `rssi_priority`, `geometric_beam_priority`, `physical_score_priority`, `geometric_weighted`, `feedforward_optimal`) |
| `heatmap_resolution_m` | float  | 0.2                       | 軌道上の事前計算サンプリング解像度 [m] (`feedforward_optimal` または `logging_level >= 5` で有効)。 |
| `time_slot_duration_s` | float  | 10.0                      | `round_robin` 指定時のタイムスロット長 [s]。                             |
| `rssi_threshold`       | float  | -75                       | `rssi_priority` 指定時の最低RSSI閾値 [dBm]。                             |
| `beam_gain_threshold`  | float  | 5.0                       | 幾何学ベースポリシー時のアンテナゲイン足切り閾値 [dBi]。                 |
| `weight_distance`      | float  | 0.7                       | `geometric_weighted` ポリシー時の距離の重み係数。                        |
| `weight_angle`         | float  | 0.3                       | `geometric_weighted` ポリシー時の角度の重み係数。                        |

#### パスロスモデルパラメータ

| パラメータ            | 型    | デフォルト値 | 説明                                               |
| :-------------------- | :---- | :----------- | :------------------------------------------------- |
| `path_loss.c`         | float | 299792458    | 光速 [m/s]。                                       |
| `path_loss.frequency` | float | 6.0e10       | 使用周波数 [Hz]。                                  |
| `path_loss.exponent`  | float | 2.0          | パスロス指数 $n$。自由空間: 2.0、都市部: 2.7-3.5。 |

---

## 7. UGV制御とシミュレーションシナリオ

### 7.1. UGVの動作 (`ugv_controller_node`)

- **経路設定**: UGVは、設定ファイル (`sim_params.yaml`) から読み込んだ**マルチウェイポイントリスト**を順次追従する。（※最新版では5台のUGVが近接・高密度で走行するシナリオを設定）
- **初期位置検証と自己補正**: 起動直後、Gazeboのスポーン処理による遅延や物理エンジンの荒ぶりが原因で発生する「スポーン座標と実際のオドメトリ座標のズレ」を自動検知する。ズレが10m以上の場合は異常とみなし、ワールド座標系のオフセットを動的に再計算することで、シミュレーションの破綻を未然に防ぐ。
- **区間速度制御**: 各ウェイポイントは目標直線速度 (`V`) を持ち、UGVはその区間の目標速度を維持するように走行する。
- **スリップ防止（加速度制御）**: 高速走行（50km/h等）時の急加速によるタイヤの空転やオドメトリ誤差を防ぐため、`max_acceleration` により速度指令（`/cmd_vel`）の変化率に制限をかける。
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
| **全体終了条件** | シミュレーションに参加する**全車両 (`base_vehicle_names`) が全てのウェイポイントに到達したタイミング**をもって、`sim_logger_node`がCSVデータの強制書き出しとシステムプロセスの安全なシャットダウン(Sys.exit)を呼び出す。複数アンテナを持つ車両の場合でも、個別のアンテナ単位ではなく、1台のベース車両（実体）単位で完了を監視・集約する。 |
| **通信停止と譲渡** | 個別車両の送信データ累計 (`TotalDataTransmission`) が `comm_data_limit_mb` に達した場合、その車両は状態を `DISCONNECTED` に変更し、基地局調停ノードへの送信要求 (RSSI) を `-inf` に落とすことで、自動的に他車両へ通信権(`link_grant`)を譲る協調動作を行う。 |

---

## 8. ログ出力仕様

`comms_simulator_node` は収集したデータをCSV形式で出力する。

| 項目                 | 詳細                                                                                                                                                                       |
| :------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **保存ノード**       | 専用の `sim_logger_node.py` が担当する。`logging_level` に応じて出力データを制御する。                                                                                     |
| **出力タイミング**   | 全車両のミッション完了通知（`/mission_complete`）が揃った時、またはプロセス終了・終了割込時（`atexit`フック）。                                                                                  |
| **出力ディレクトリ** | `/workspace/sim_results/`（コンテナ内の所有権はホストマウント権限に自動追従して可読・編集可能化される。スイープ実行時：`sim_results/sweep_<sweep_timestamp>/`、単一実行時：`sim_results/run_<timestamp>/`） |
| **ファイル出力形式** | 3階層フォルダリング（`comms/`, `control/`, `heatmap/`）により整理して出力。ファイル名から重複するタイムスタンプを排除。<br>- **サマリー**: `sweep_summary.csv` または `sweep_summary_run*.csv`<br>- **通信時系列**: `comms/{vehicle_name}_connected.csv` (`logging_level` 3) または `comms/{vehicle_name}_full.csv` (`logging_level` 4以上)<br>- **イベント**: `control/events.csv` (`logging_level` 2以上)<br>- **制御ログ**: `control/feedforward_log.csv` (`logging_level` 5のみ)<br>- **ヒートマップ**: `heatmap/rssi_heatmap.csv` (`logging_level` 5のみ) |
| **時間軸の起点**     | 各車両の `/cmd_vel` メッセージを監視し、**いずれかの車両が最初に動き出した瞬間**を `time_s = 0.0` とする。また、**各車両ごとの個別開始時間**も `vehicle_time_s = 0.0` として計算される。距離への換算を容易にするための仕様。 |

### CSV出力項目

各座標カラムの意味:
- **`ugv_x_m`, `ugv_y_m`, `ugv_z_m`**: UGVアンテナ位置座標（絶対座標）
- **`bs_x_m`, `bs_y_m`, `bs_z_m`**: 基地局アンテナ位置座標（絶対座標）

#### 1. 通信品質時系列CSV (`comms/{vehicle_name}_full.csv` または `_connected.csv`)

| 項目名 | 単位 | 説明 |
| :--- | :--- | :--- |
| `time_s` | [s] | シミュレーション時刻（最初の車両が移動を開始した瞬間を 0.0 とする） |
| `vehicle_time_s` | [s] | 車両別時刻（その車両自身が移動を開始した瞬間を 0.0 とする、`full.csv`のみ） |
| `vehicle_name` | string | 車両（または車載アンテナ）名 |
| `has_link_grant` | bool | 基地局からの通信権（Grant）付与フラグ（`full.csv`のみ） |
| `distance_m` | [m] | アンテナ間3D距離 |
| `rssi_dBm` | [dBm] | 受信信号強度 |
| `throughput_Gbps` | [Gbps] | 瞬時スループット |
| `total_data_MB` | [MB] | 累積送信データ量 |
| `path_loss_dB` | [dB] | パスロス |
| `e_gain_dB` | [dBi] | E面アンテナゲイン |
| `h_gain_dB` | [dBi] | H面アンテナゲイン |
| `comm_active` | bool | 通信可否フラグ（`full.csv`のみ） |
| `ugv_x_m`, `ugv_y_m`, `ugv_z_m` | [m] | UGVアンテナ位置座標 |
| `bs_x_m`, `bs_y_m`, `bs_z_m` | [m] | 基地局アンテナ位置座標 |
| `link_state` | - | リンク状態 (`DISCONNECTED` / `ESTABLISHING` / `CONNECTED`、`full.csv`のみ) |

#### 2. イベントCSV (`control/events.csv`)

| 項目名 | 単位 | 説明 |
| :--- | :--- | :--- |
| `time_s` | [s] | 経過時間（最初の車両が移動を開始した瞬間を 0.0 とする） |
| `vehicle_name` | string | 対象車両の名前 |
| `link_state` | - | 新しいリンク状態 |
| `has_link_grant` | bool | 通信権付与フラグ |
| `rssi_dBm` | [dBm] | イベント発生時の受信信号強度 |
| `distance_m` | [m] | イベント発生時のアンテナ間距離 |

#### 3. nominal RSSI ヒートマップ (`heatmap/rssi_heatmap.csv`)

| 項目名 | 単位 | 説明 |
| :--- | :--- | :--- |
| `x_m`, `y_m`, `z_m` | [m] | 軌道上のサンプル座標 |
| `yaw_rad` | [rad] | サンプル座標における車両の進行方向（Yaw） |
| `rssi_{bs_name}_{antenna_name}` | [dBm] | 各地上局アンテナと車載アンテナペアにおける nominal RSSI (アンテナ個数に基づき動的に列数が変動) |
| `optimal_antenna` | string | その座標における nominal RSSI が最大となる車載アンテナ名 |
| `max_rssi` | [dBm] | その座標における最大の nominal RSSI |

#### 4. フィードフォワード制御ログ (`control/feedforward_log.csv`)

| 項目名 | 単位 | 説明 |
| :--- | :--- | :--- |
| `time_s` | [s] | 経過時間 |
| `ugv_x_m`, `ugv_y_m`, `ugv_z_m` | [m] | 現在の UGV アンテナ位置座標 |
| `rssi_{antenna_name}` | [dBm] | 各車載アンテナの現在の実測 RSSI (車両アンテナ構成に基づき動的に列数が変動) |
| `selected_antenna` | string | 現在選択（Grant付与）されている車載アンテナ名 |
| `rssi_optimal_dBm` | [dBm] | 現在位置に対応するヒートマップ上（LUT）の nominal optimal RSSI |

### CSV出力例

#### 通信品質時系列CSV例 (`comms/shinkansen_front_full.csv`)
```csv
time_s,vehicle_time_s,vehicle_name,has_link_grant,distance_m,rssi_dBm,throughput_Gbps,total_data_MB,path_loss_dB,e_gain_dB,h_gain_dB,comm_active,ugv_x_m,ugv_y_m,ugv_z_m,bs_x_m,bs_y_m,bs_z_m,link_state
15.0,15.0,shinkansen_front,True,20.42,-55.2,4.6,1.2,65.2,8.5,6.2,True,-20.0,0.0,0.0,0.0,3.0,0.0,CONNECTED
```

#### イベントCSV例 (`control/events.csv`)
```csv
time_s,vehicle_name,link_state,has_link_grant,rssi_dBm,distance_m
15.0,shinkansen_front,CONNECTED,True,-55.2,20.42
```

#### ヒートマップCSV例 (`heatmap/rssi_heatmap.csv`)
```csv
x_m,y_m,z_m,yaw_rad,rssi_antenna_bs_front,rssi_antenna_bs_mid,rssi_antenna_bs_rear,optimal_antenna,max_rssi
-1000.0,0.0,0.0,0.0,-85.2,-90.1,-95.3,front,-85.2
```

#### フィードフォワード制御ログ例 (`control/feedforward_log.csv`)
```csv
time_s,ugv_x_m,ugv_y_m,ugv_z_m,rssi_shinkansen_front,rssi_shinkansen_mid,rssi_shinkansen_rear,selected_antenna,rssi_optimal_dBm
15.0,-20.0,0.0,0.0,-55.2,-60.1,-65.3,shinkansen_front,-55.2
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

**Document Version**: 8.0<br>
**Last Updated**: 2026年05月21日<br>
**Status**: Active Development
