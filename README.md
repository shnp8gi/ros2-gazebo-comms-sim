# ROS 2 Gazebo Communication Simulator

ROS 2 Humble と Gazebo Harmonic を用いた、移動車両（TX/SUV）と固定基地局間の無線通信品質シミュレータです。

## 📋 概要

本シミュレータは、Gazebo Sim上で駆動する移動車両と固定基地局間の通信品質（RSSI/スループット）を、数理モデルを用いてシミュレーションします。

### 主な機能

- **リアルタイム通信品質計算**: RSSI、スループット、パスロスを動的に計算
- **MCSテーブルベースのスループット推定**: CSVファイルから読み込み、ステップ関数によるMCSインデックス選択
- **リンク確立時間のシミュレーション**: Association time（デフォルト2ms）を考慮した現実的な通信開始
- **指向性アンテナモデル**: E面/H面パターンCSVに基づく指向性ゲイン計算（Tx/Rx合成）
- **動的伝搬路モデル**: 対数距離減衰モデル + AWGN雑音（Strategyパターンで差し替え可能）
- **マルチ車両・高密度・マルチウェイポイント追従**: 複数台のTXによる同期待機、区間速度制御（スリップ防止用加速度制御対応）、および自動経路追従
- **集中スケジューリング**: 基地局側で通信権（Grant）を1台ずつ排他的に付与。RSSI優先、アンテナアライメント（幾何学スコア）、推定受信電力ベースに加え、事前計算されたnominal RSSIヒートマップに基づくフィードフォワード制御ポリシー（`feedforward_optimal`）を搭載。
- **プロアクティブハンドオーバーと自律的権限譲渡**: 通信データ上限到達時の自律的な譲渡のほか、リンク切断（DISCONNECTED）時に即座に最適な幾何学スコアを持つ後続車両へ通信権を強制切替する機能
- **GPUアクセラレーション対応**: Docker Composeでの NVIDIA GPU リソース割り当てにより、GUIモードでの高速なGazeboレンダリングをサポート
- **ミッション完了時の自動終了とCSV保存**: 全車両のミッション完了を検知して安全に自動シャットダウン。ロギングレベル1〜5に対応し、3階層のフォルダ構造（`comms/`, `control/`, `heatmap/`）で結果を出力。

## 🛠️ 技術スタック

| 項目         | バージョン/詳細           |
| ------------ | ------------------------- |
| OS           | Ubuntu 22.04 LTS (Docker) |
| ROS 2        | Humble Hawksbill          |
| シミュレータ | Gazebo Sim (Harmonic)     |
| 言語         | Python 3                  |
| 開発環境     | Docker / Docker Compose (NVIDIA GPU対応) |

## 📁 ディレクトリ構造

```
ros2-gazebo-comms-sim/
├── config/                          # 設定ファイル
│   ├── sim_params.yaml             # メインパラメータファイル
│   ├── e_plane.csv                 # E面（垂直面）アンテナパターン
│   ├── h_plane.csv                 # H面（水平面）アンテナパターン
│   └── MCStable.csv                # MCS（変調・符号化方式）テーブル
│
├── models/                          # Gazeboモデル
│   ├── antenna/                    # 基地局アンテナモデル
│   │   ├── model.config
│   │   ├── model.sdf
│   │   └── meshes/
│   └── SUV/                        # 移動車両モデル
│       ├── model.config
│       ├── model.sdf
│       └── meshes/
│
├── comms_sim_pkg/                   # ROS 2パッケージ（ノード/launch）
│   ├── comms_sim_pkg/              # Pythonモジュール
│   │   ├── comms_node.py          # 通信シミュレータノード（/comms/quality publish）
│   │   ├── comms_calculator.py    # 通信品質計算（RSSI/Throughput）
│   │   ├── antenna_parser.py      # アンテナパターン処理（E/H面CSV補間）
│   │   ├── link_controller_node.py# 基地局側調停ノード（アクセス許可付与、フィードフォワード制御、LUT事前計算）
│   │   ├── link_scheduling_strategy.py # スケジューリング戦略（feedforward_optimal等）
│   │   ├── sim_logger_node.py     # ログノード（ミッション監視、フォルダ分け保存、サマリー集計）
│   │   └── tx_controller_node.py # TX制御ノード（/cmd_vel publish）
│   ├── launch/
│   │   └── sim_launch.py          # Gazebo起動/スポーン/ブリッジ/ノード起動
│   ├── resource/
│   │   └── minimal_world.sdf
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── setup.py
│
├── comms_sim_msgs/                   # メッセージ定義パッケージ
│   └── msg/
│       └── CommsQuality.msg
│
├── sim_results/                      # ログ出力ディレクトリ
│   └── sweep_<timestamp>/            # スイープ実行時の出力フォルダ
│       ├── sweep_summary_run*.csv    # スイープ各回の要約CSV
│       └── runs/                     # 各パラメータ実行別の出力フォルダ
│           └── run_<run_idx>_y<Y>_a<ANGLE>/
│               ├── comms/            # 通信品質の時系列CSVファイル ({vehicle_name}_connected.csv / {vehicle_name}_full.csv)
│               ├── control/          # 制御・イベントのCSVファイル (events.csv, feedforward_log.csv)
│               └── heatmap/          # アンテナ制御用事前計算ヒートマップ (rssi_heatmap.csv)
│
├── docker-compose.yml
├── docker-compose.gpu.yml            # GPU設定オーバーライド
├── Dockerfile
├── entrypoint.sh
├── start.sh                          # コンテナ自動起動・GPU判定スクリプト
├── README.md
├── specification.md
└── .gitignore
```

## 🌐 座標系

本シミュレータは**ENU（East-North-Up）座標系**を採用しています。

| 軸  | 方向      |
| --- | --------- |
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

**`antenna_offset`**: エンティティ原点からアンテナ位置への3D相対座標 `[x, y, z]` [m]

| 設置例                | offset値           |
| --------------------- | ------------------ |
| SUV屋根中央にアンテナ | `[0.0, 0.0, 2.23]` |
| SUV屋根前方にアンテナ | `[1.0, 0.0, 2.23]` |

## 🚀 クイックスタート

### 前提条件

- X11サーバー（GUIモード使用時、Linux/WSL2）

### 初期セットアップ

#### 1. Gitのインストール

Ubuntu / WSL2 の場合:
```bash
sudo apt update
sudo apt install git
```
Windows の場合は [Git for Windows](https://gitforwindows.org/) などをインストールしてください。

#### 2. GitHubアカウントとSSHキーの準備

本リポジトリのクローンにSSH接続を利用するため、アカウントとキーの登録を行います。

1. **GitHubアカウントの作成**
   - [GitHub登録ページ](https://github.com/signup) よりアカウントを作成してください。
2. **SSHキーの生成** (Ubuntu/WSL2 もしくは Git Bash 等のターミナルで実行):
   ```bash
   ssh-keygen -t ed25519 -C "your_email@example.com"
   ```
   - 保存先やパスフレーズを聞かれますが、そのままEnterを押して進めて問題ありません。
3. **公開鍵（Public key）の取得**:
   ```bash
   cat ~/.ssh/id_ed25519.pub
   ```
   - 表示された `ssh-ed25519 ...` から始まる文字列全体をコピーします。
4. **GitHubへの鍵登録**:
   - GitHubの [SSH keys 設定ページ](https://github.com/settings/keys) にアクセスします。
   - 「New SSH key」をクリックし、タイトル（例: `My PC`）を入力してソースの枠内にコピーした公開鍵を貼り付け、「Add SSH key」で保存します。

#### 3. Dockerのインストール

Docker公式の手順に従って Docker をインストールしてください（`docker compose` コマンドが使用できるよう構成してください）。

- [Docker Engine インストール手順 (Ubuntu)](https://docs.docker.com/engine/install/ubuntu/)
- [Docker Desktop for Windows](https://docs.docker.com/desktop/install/windows-install/)

### 4. リポジトリのクローン

SSHを利用してリポジトリをクローンします。

```bash
git clone git@github.com:your-username/ros2-gazebo-comms-sim.git
cd ros2-gazebo-comms-sim
```

### 5. ワークスペースのビルド（初回およびコード変更時）

#### 1. イメージのビルド（初回のみ必須）
```bash
docker compose build
```

#### 2. コンテナの起動とビルドの実行
専用のビルドコンテナを使ってパッケージをビルドします。（イメージは自動でビルドされます）
```bash
docker compose run --rm build
```

## 📺 シミュレータの実行

### コンテナの起動（自動GPU判定）

`start.sh` を使用してコンテナをバックグラウンドで起動します。スクリプトが自動的にNVIDIA GPUの有無を判定し、最適な設定（CPU/GPU）で起動します。同時にX11のアクセス許可も行われます。

#### Linux / WSL2

```bash
./start.sh
```

コンテナが起動したら、以下のコマンドでコンテナ内のシェルに接続します。

```bash
docker compose exec sim bash
```

以下コンテナ内

##### ワークスペースのセットアップ

新しく開いたシェルにビルド済みのパッケージを認識させます。（※コードを修正した場合は、ここで `colcon build --symlink-install` を実行して再ビルドすることも可能です）

```bash
source install/setup.bash
```

##### シミュレーション起動

X11 Forwarding経由でもサーバー側のGPUを使って高速描画を行うため、**`vglrun -d egl`** を先頭につけてシミュレーションを実行します。（※ホスト上にXサーバーが無くても、EGLバックエンドにより直接GPUにアクセスできます）

```bash
vglrun -d egl ros2 launch comms_sim_pkg sim_launch.py
```

#### Windows (WSL2 + X Server)

1. VcXsrv等のXサーバーをインストール・起動
   - **【重要】** VcXsrv (XLaunch) の起動時（Extra settings画面）は、必ず **「Disable access control」** にチェックを入れてください。設定が漏れるとDockerからの画面描画が拒否されます。
2. WSL2のDISPLAY変数を設定

```bash
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0.0
```

3. 上記Linuxと同様の手順でコンテナを起動

### CUIモード（Headless）

GUIなしで高速にシミュレーションを実行します。GUI起動時と同様に `start.sh` で起動し `docker compose exec sim bash` で中に入った後、以下のコマンドを実行します。

```bash
ros2 launch comms_sim_pkg sim_launch.py headless:=true
```

### GUI / CUI 切り替え

| モード | コマンド                                                 | 用途                 |
| ------ | -------------------------------------------------------- | -------------------- |
| GUI    | `ros2 launch comms_sim_pkg sim_launch.py`                | 可視化・デバッグ     |
| CUI    | `ros2 launch comms_sim_pkg sim_launch.py headless:=true` | 高速実行・バッチ処理 |

## 📊 アンテナ角度・位置スイープシミュレーション (sweep_sim.py)

大量のパラメータ条件（基地局の配置位置、アンテナ角度）を一括で自動実行し、各条件における通信品質やスループットの統計データを収集・集計するためのツール `tools/sweep_sim.py` が用意されています。

### 1. 実行方法
コンテナ内に入り、以下のコマンドで実行します：
```bash
python3 tools/sweep_sim.py [オプション]
```

### 2. コマンドライン引数 (CUIオプション)
`tools/sweep_sim.py` は、以下のオプションを指定して実行できます（指定しない場合はスクリプト上部のデフォルト値が使用されます）：

| オプション | 型 | デフォルト値 | 説明 |
|---|---|---|---|
| `--start-angle` | float | `75.0` | スイープする角度の開始値 [deg] (西: `0`, 南: `90`, 東: `180`) |
| `--end-angle` | float | `105.0` | スイープする角度の終了値 [deg] |
| `--step-angle` | float | `0.1` | スイープする角度の刻み幅 [deg] |
| `--base-station-yaw` | float | `-90.0` | 基地局本体自体の向き（ヨー角） [deg] (南向き: `-90.0`, 北向き: `90.0`) |
| `--num-runs` | int | `10` | 各設定条件（タスク）を繰り返しシミュレーションする回数 |
| `--rtf` | float | `5.0` | スイープ時のシミュレーション加速倍率 (Real-Time Factor) |
| `--timeout` | int | `120` | 1シミュレーションランあたりの最大待機時間 [秒] |
| `--resume` | str | `None` | 中断したシミュレーションを再開するためのタイムスタンプまたは結果ディレクトリパス |

**実行例:**
```bash
# 角度 80° から 100° まで 1°刻みで、各条件を 5回ずつ、加速倍率 8.0倍で実行
python3 tools/sweep_sim.py --start-angle 80 --end-angle 100 --step-angle 1.0 --num-runs 5 --rtf 8.0

# 途中で中断したスイープ（例: タイムスタンプ 20260526_145419）を途中から再開
python3 tools/sweep_sim.py --resume 20260526_145419
```

### 3. スクリプト内デフォルト設定
スクリプト `tools/sweep_sim.py` の**最上部**（L15〜L37付近）に、以下のパラメータがあらかじめ定義されています。引数なしで実行した場合、これらのデフォルト値が適用されます。

```python
# 基地局のY位置リスト (m)
Y_POSITIONS = [1.0]

# 基地局自体の向き (ヨー角) (deg)
BASE_STATION_YAW_DEG = -90.0

# 角度スイープの設定 (下限, 上限, 刻み幅)
START_ANGLE = 75.0   # 下限 (deg)
END_ANGLE = 105.0    # 上限 (deg)
STEP_ANGLE = 0.1     # 刻み幅 (deg)

# パラメータスイープを繰り返す回数 (ラン数)
NUM_RUNS = 10

# 1タスクあたりの最大待機時間 [秒]
TASK_TIMEOUT_SEC = 120

# スイープ時の加速倍率 (ヘッドレス時のみ有効.1.0=リアルタイム)
SWEEP_REAL_TIME_FACTOR = 5.0
```

### 4. 実行進捗の監視
スイープシミュレーションの実行中は、別ターミナルから以下のコマンドを実行することで、進捗状況（完了したタスク数、予測残り時間など）をリアルタイムで確認できます。
```bash
python3 ./tools/sweep_progress.py --watch
```

## ⚙️ パラメータ設定

### config/sim_params.yaml

主要なパラメータは `config/sim_params.yaml` で設定します。ランチファイル `comms_sim_pkg/launch/sim_launch.py` がこのYAMLを読み込み、Gazebo起動、モデルスポーン、ros_gz_bridge、各ノード起動に反映します。

#### 全体シミュレーションパラメータ

```yaml
simulation:
  logging_level: value        # ログ記録レベル (1〜5) (default: 5)
                              # 1: サマリーのみ
                              # 2: サマリー + イベントログ
                              # 3: サマリー + イベント + 時系列 (CONNECTED時のみ)
                              # 4: サマリー + イベント + 時系列 (常時記録)
                              # 5: サマリー + イベント + 時系列 (常時記録) + アンテナ制御ログ + ヒートマップ
  headless: value             # ヘッドレスモード (true: GUIなし, false: GUIあり) (default: true)
  real_time_factor: value     # リアルタイム倍率 (default: 5.0)
```

#### 通信シミュレーションパラメータ

```yaml
comms_simulator_node:
  ros__parameters:
    sampling_rate: value # 計算頻度 [Hz] (default: 100.0)
    noise_variance: value # AWGN分散 [dB] (default: 0.0)
    e_plane_path: value # E面アンテナCSV (default: "/workspace/config/e_plane.csv")
    h_plane_path: value # H面アンテナCSV (default: "/workspace/config/h_plane.csv")
    mcs_table_path: value # MCSテーブル (default: "/workspace/config/MCStable.csv")
    link_establishment_time_ms: value # リンク確立時間 [ms] (default: 2.0)
    comm_data_limit_mb: value # データ上限 [Mb] (default: -1.0, -1.0: 無制限)
    max_antenna_attenuation: value # アンテナ最大減衰量 [dB] (default: 30.0)
    logging_start_trigger: value # ログ記録開始トリガー (immediate, on_movement, on_topic) (default: "on_movement")

    path_loss:
      frequency: value # 使用周波数 [Hz] (default: 6.0e10)
      c: value # 光速 [m/s] (default: 299792458)
      exponent: value # パスロス指数 (default: 2.0)

    tx_power: value # 送信電力 [dBm] (default: -7.0)
```

#### リンク制御パラメータ（集中スケジューラ）

```yaml
link_controller_node:
  ros__parameters:
    scheduling_policy: value # スケジューリングポリシー (例: sequential, round_robin, rssi_priority, geometric_beam_priority, physical_score_priority, geometric_weighted, feedforward_optimal)
    heatmap_resolution_m: value # 軌道上の事前計算サンプリング解像度 [m] (feedforward_optimal または logging_level >= 5 で有効) (default: 0.2)
    time_slot_duration_s: value # round_robin時のスロット長 [s] (default: 10.0)
    rssi_threshold: value # rssi_priority時の閾値 [dBm] (default: -75)
    beam_gain_threshold: value # geometric_beam_priority用の最小ゲイン閾値 [dBi] (default: 5.0)
    weight_distance: value # geometric_weighted用の距離重み係数 (default: 0.7)
    weight_angle: value # geometric_weighted用の角度重み係数 (default: 0.3)
```

※ 通信距離はワールド座標で計算されます。`/odom`（相対座標）からワールド座標へ変換するため、通常は `spawn_entities.suv.pose` と同じ値を指定します（launch が自動で渡します）。

#### TX制御パラメータ

```yaml
tx_controller_node:
  ros__parameters:
    waypoints:
      - [value, value, value, value] # [X, Y, Z, V] (ワールド座標)
      # ...
    waypoint_tolerance: value # 到達判定距離 [m] (default: 2.0)
    control_rate: value # 制御ループ周波数 [Hz] (default: 10.0)
    max_angular_velocity: value # 最大角速度 [rad/s] (default: 1.0)
    heading_gain: value # 方向制御ゲイン (default: 1.5)
```

> **Note**: `waypoints` はワールド座標として扱われます。`sim_launch.py` は `spawn_entities.suv.pose` を元に `spawn_pose` を自動生成して `tx_controller_node` に渡し、`/odom` とワールド座標の差分を補正します。

#### エンティティスポーン設定

```yaml
spawn_entities:
  antenna:
    model_uri: value # モデルURI (default: "models://antenna")
    name: value # エンティティ名 (default: "antenna")
    pose: [x, y, z, roll, pitch, yaw] # (default: [0.0, 6.0, 0.0, 0.0, 0.0, 0.0])
    static: value # 静的オブジェクト (default: true)
    antenna_height_offset: value # アンテナ高さ [m] (default: 3.0)
    antenna_relative_rpy: [r, p, y] # アンテナ相対角度 [rad] (default: [0.0, 0.0, -1.5708])

  suv:
    model_uri: value # モデルURI (default: "models://SUV")
    name: value # エンティティ名 (default: "suv")
    pose: [x, y, z, roll, pitch, yaw] # (default: [-20.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    static: value # 静的オブジェクト (default: false)
    antenna_height_offset: value # アンテナ高さ [m] (default: 1.9)
    antenna_relative_rpy: [r, p, y] # アンテナ相対角度 [rad] (default: [0.0, 0.0, 0.0])
```

## 📊 出力データ

シミュレーション実行中に収集したログは、以下のディレクトリにCSVとして出力されます。

- 出力先: `sim_results/`（`/workspace/sim_results/` に保存）
- 保存タイミング: **ミッション完了通知受信時**（全ウェイポイント到達時）およびノード終了時

### 出力ディレクトリおよびファイル構造

ロギングレベルおよび実行モード（スイープ等）に応じて、以下の3階層構造で出力されます。
個々のCSVファイル名からタイムスタンプの重複を排除し、フォルダ階層によって結果をカプセル化（可視化・分類）しています。

```
sim_results/
└── sweep_<sweep_timestamp>/                 # スイープ全体の実行ログフォルダ（単一実行時は run_<run_timestamp>）
    ├── sweep_summary_run<run_idx>.csv       # スイープ実行各回の要約結果サマリー
    └── runs/                                # 各個別シミュレーションランの結果
        └── run_<run_idx>_y<Y>_a<ANGLE>/     # 個別ランのパラメータフォルダ
            ├── comms/                       # 通信品質関連の時系列データ
            │   ├── {vehicle_name}_connected.csv  # 接続時（CONNECTED状態）のみ記録 (logging_level: 3)
            │   └── {vehicle_name}_full.csv       # 未接続時含む全期間記録 (logging_level: 4以上)
            ├── control/                     # 制御およびハンドオーバーイベントデータ
            │   ├── events.csv               # リンク状態やGrant権限の変更イベントログ (logging_level: 2以上)
            │   └── feedforward_log.csv      # フィードフォワード制御のリアルタイム決定ログ (logging_level: 5のみ)
            └── heatmap/                     # 事前計算ヒートマップデータ (logging_level: 5のみ)
                └── rssi_heatmap.csv         # 軌道上の各座標における nominal RSSI 事前計算値
```

### 各ファイルの項目定義

#### 1. サマリーCSV (`sweep_summary_run*.csv` / `sweep_summary.csv`)
各車両の通信総量、接続率等を要約したデータです。
- `run_id`: シミュレーション実行タイムスタンプ
- `y_position`: 基地局のY軸配置位置 [m]
- `antenna_angle`: 基地局アンテナの方向（Yaw） [rad]
- `vehicle_name`: 車両（または車載アンテナ）名、もしくは `shinkansen_total`
- `total_data_MB`: 累積送信データ量 [MB]
- `connected_time_s`: 接続確立（CONNECTED）の総時間 [s]
- `average_throughput_Gbps`: 接続状態における平均スループット [Gbps]
- `average_rssi_dBm`: 接続状態における平均受信信号強度 [dBm]
- `handover_count`: ハンドオーバー（接続再確立）回数

#### 2. 通信品質時系列CSV (`comms/{vehicle_name}_connected.csv` または `_full.csv`)
車両ごとの時系列通信パラメータを記録します。
※ `vehicle_time_s`, `has_link_grant`, `comm_active`, `link_state` は `logging_level: 4` 以上（`_full.csv`）でのみ記録されます。
- `time_s`: 最初の車両が走行を開始した時点を0とした経過時間 [s]
- `vehicle_time_s`: 対象車両が走行を開始した時点を0とした経過時間 [s]
- `vehicle_name`: 車両（または車載アンテナ）名
- `has_link_grant`: 基地局からの通信権（Grant）付与フラグ
- `distance_m`: アンテナ間3D距離 [m]
- `rssi_dBm`: 受信信号強度 [dBm]
- `throughput_Gbps`: 瞬時スループット [Gbps]
- `total_data_MB`: 累積送信データ量 [MB]
- `path_loss_dB`: パスロス [dB]
- `e_gain_dB`: E面アンテナゲイン [dBi]
- `h_gain_dB`: H面アンテナゲイン [dBi]
- `comm_active`: 通信可否フラグ
- `tx_x_m`, `tx_y_m`, `tx_z_m`: TXアンテナ位置座標 [m]
- `bs_x_m`, `bs_y_m`, `bs_z_m`: 基地局アンテナ位置座標 [m]
- `link_state`: リンク状態 (`DISCONNECTED` / `ESTABLISHING` / `CONNECTED`)

#### 3. イベントCSV (`control/events.csv`)
リンク権や通信状態の変化があった瞬間を記録します。
- `time_s`: 経過時間 [s]
- `vehicle_name`: 対象車両の名前
- `link_state`: 新しいリンク状態
- `has_link_grant`: 通信権付与フラグ
- `rssi_dBm`: イベント発生時の受信信号強度 [dBm]
- `distance_m`: イベント発生時のアンテナ間距離 [m]

#### 4. nominal RSSI ヒートマップ (`heatmap/rssi_heatmap.csv`)
シミュレーション起動時に、車両軌道上のサンプル座標ごとに基地局と各車載アンテナの組み合わせで計算される nominal RSSI の分布図です。
- `x_m`, `y_m`, `z_m`: 軌道上のサンプル座標 [m]
- `yaw_rad`: サンプル座標における車両の進行方向（Yaw） [rad]
- `rssi_{bs_name}_{antenna_name}`: 基地局アンテナと車載アンテナの各ペアにおける nominal RSSI [dBm]
- `optimal_antenna`: 最大の RSSI を示す車載アンテナ名
- `max_rssi`: その座標における最大 RSSI [dBm]

#### 5. フィードフォワード制御ログ (`control/feedforward_log.csv`)
`feedforward_optimal` ポリシー稼働中の、リアルタイムなアンテナ選択決定の履歴です。
- `time_s`: 経過時間 [s]
- `tx_x_m`, `tx_y_m`, `tx_z_m`: 現在の TX アンテナ位置座標 [m]
- `rssi_{antenna_name}`: 各車載アンテナの現在の実測 RSSI [dBm]
- `selected_antenna`: 現在選択（Grant付与）されている車載アンテナ名
- `rssi_optimal_dBm`: 現在位置に対応するヒートマップ上（LUT）の nominal optimal RSSI [dBm]

## 🔌 ROSインターフェース

### トピック

| トピック名           | メッセージ型                  | 方向    | 説明                                                          |
| -------------------- | ----------------------------- | ------- | ------------------------------------------------------------- |
| `/imu/data`          | `sensor_msgs/Imu`             | Sub     | TX姿勢情報（ros_gz_bridge経由）                              |
| `/odom`              | `nav_msgs/Odometry`           | Sub     | TXオドメトリ（ros_gz_bridge経由）                            |
| `/cmd_vel`           | `geometry_msgs/Twist`         | Pub     | 速度指令（ros_gz_bridge経由でGazeboへ送信）                   |
| `/comms/quality`     | `comms_sim_msgs/CommsQuality` | Pub     | 通信品質                                                      |
| `/link_request`      | `std_msgs/Float64`            | Pub     | 基地局へRSSIをスコアとして報告し通信権限を要求                |
| `/link_grant`        | `std_msgs/Bool`               | Sub     | 完了局からのアクセス許可(TDMA的スケジューリング)              |
| `/mission_complete`  | `std_msgs/Bool`               | Pub     | TX完走通知（全車完了で`sim_logger_node`が自動終了を実行）    |
| `/rx/pose` | `geometry_msgs/PoseStamped`   | Sub     | 基地局姿勢（オプション、アンテナ方向計算に使用）              |
| `/clock`             | `rosgraph_msgs/Clock`         | Sub     | シミュレーション時間（use_sim_time=true時）                   |

### カスタムメッセージ: CommsQuality

```
std_msgs/Header header
float64 distance              # TX-基地局間距離 [m]
float64 rssi                  # 受信信号強度 [dBm]
float64 throughput            # 瞬時スループット [Gbps]
float64 total_data_transmitted # 累計伝送データ量 [Mb]
float64 tx_x                 # TX位置X [m]
float64 tx_y                 # TX位置Y [m]
float64 tx_z                 # TX位置Z [m]
float64 rx_x        # 基地局位置X [m]
float64 rx_y        # 基地局位置Y [m]
float64 rx_z        # 基地局アンテナ位置Z [m]
float64 antenna_gain_e_plane   # E面ゲイン [dBi]
float64 antenna_gain_h_plane   # H面ゲイン [dBi]
float64 path_loss              # パスロス [dB]
bool comm_active               # 通信アクティブフラグ
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
