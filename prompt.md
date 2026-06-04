# ロボティクスシミュレーション環境構築タスク

あなたはロボティクスおよびシミュレーション分野のエキスパートである。
以下の「開発ガイドライン」および「システム仕様書」に基づき、**ROS 2 Humble** と **Gazebo Harmonic** を用いた通信シミュレータ構築のためのコード、設定ファイル、およびドキュメントを作成せよ。

---

## 1. 開発ガイドライン

回答作成にあたっては、以下の優先順位および方針を厳守すること。

### 1.1 情報の優先順位

1. **【仕様書の遵守】**
   * 後述する「2. システム仕様書」を最優先とする。
   * ここで定義された ROS 2 のバージョン（Humble）、ノード構成、トピック名から逸脱してはならない。

2. **【Webリサーチによる実装詳細の補完】**
   * API の使用方法、`CMakeLists.txt` の設定、SDF 記述方法などの実装詳細は、公式ドキュメント（ROS 2 Documentation、Gazebo Sim Docs）および信頼できる GitHub リポジトリを参照すること。
   * **重要**: ROS 2 はディストリビューション間の差異が大きいため、必ず **Humble / Harmonic** に対応した情報のみを用いること。

### 1.2 コーディングおよびデバッグ方針

* **使用言語**: Python（ROS 2 ノード実装）、SDF（モデル定義）
* **ROS 2 作法**: `colcon build` を前提としたパッケージ構成とし、`setup.py` および `package.xml` を含めること。
* **Gazebo 連携**: ROS 2 トピックと Gazebo プラグイン（例: `ros_gz_bridge`）の連携について、バージョン互換性を厳密に確認すること。
* **エラー対応**: 発生し得るエラーについては、一般論ではなく、ログに基づいた具体的な検索結果や Issue の解決策を提示すること。

### 1.3 禁止事項

* ROS 1（旧 ROS）の記述を含めないこと（`rospy`、`roscpp` の使用禁止。必ず `rclpy` を使用する）。
* 仕様書で定義されたセンサ構成やロボットモデルを、指示なく変更・提案しないこと。

---

## 2. システム仕様書（Knowledge）

### 1. プロジェクト概要

 Gazebo Simで駆動する移動車両と固定基地局間の通信品質（RSSI/スループット）を、ROS 2ノード上で数理モデルを用いてシミュレーションし、その挙動を評価する。

---

### 2. 技術スタックと環境構築

| 分野 | 項目 | 決定事項 | 備考 |
| :--- | :--- | :--- | :--- |
| **OS** | ベースOS | Ubuntu 22.04 LTS (Dockerコンテナ内) |
| **ROS 2** | ディストリビューション | **Humble Hawksbill (LTS)** | サポート期間: 2027年5月まで。 |
| **シミュレータ** | 種類 | **Gazebo Sim (Harmonic)** | ROS 2との連携、将来性を重視。 |
| **開発環境** | コンテナ | **Docker** (推奨) | 環境の再現性、GUI/CUI切り替えをサポート。 |
| **実装言語** | 主言語 | **Python 3** | 通信計算（数理モデル）の柔軟性と開発速度を優先。 |

---

### 3. シミュレーション環境 (World & Models)

| 項目 | 詳細 | Fuel URI / 構成 |
| :--- | :--- | :--- |
| **ワールド** | シンプルな無限平面 (Empty World + Ground Plane)。将来的な物体設置は可能とする。 | `minimal_world.sdf` |
| **移動車両 (TX)** | 実在感のあるSUVモデルに駆動系とセンサをアタッチ。 | **Fuel: `https://app.gazebosim.org/OpenRobotics/fuel/models/SUV`** |
| **基地局** | 高さのあるアンテナ塔モデル。固定設置。 | **Fuel: `https://app.gazebosim.org/OpenRobotics/fuel/models/antenna`** |
| **モデル参照** | Gazebo Fuelからローカルにダウンロードし、Dockerでマウントして参照する。 | `GZ_SIM_RESOURCE_PATH` を設定。 |

---

### 4. ロボットとセンサ構成

| エンティティ | センサ/プラグイン | ROS 2 トピック (Publish) | 用途 |
| :--- | :--- | :--- | :--- |
| **車両** | Diff Drive (プラグイン) | `/cmd_vel` (Subscribe) | 運動制御。 |
| **車両** | NavSat (GNSS) | `/gps/fix` (標準メッセージ) | $x, y, z$ 位置情報の提供。 |
| **車両** | IMU | `/imu/data` (標準メッセージ) | 姿勢情報 (Roll, Pitch, Yaw) の提供。**アンテナゲイン計算**に利用。 |
| **基地局** | N/A | パラメータサーバーから座標を取得 | 静的なためセンサ不要。 |

---

### 5. 通信シミュレーション要件 (ROS 2 Node)

#### 5.1. 通信ノード (`comms_simulator_node`)

| 項目 | 詳細 |
| :--- | :--- |
| **ノード名** | `comms_simulator_node` |
| **実装言語** | Python |
| **役割** | Gazeboから取得した位置情報と姿勢情報に基づき、カスタムの伝搬路モデルで通信品質を計算し、ロギング及びPublishを行う。 |

#### 5.2. 通信インターフェース

| 項目 | トピック名/パラメータ | メッセージ型 | 送受信 | 備考 |
| :--- | :--- | :--- | :--- | :--- |
| **車両位置** | `/gps/fix`, `/imu/data` | `sensor_msgs/NavSatFix`, `sensor_msgs/Imu` | Subscribe | Gazeboから車両の位置・姿勢を取得。 |
| **通信結果** | `/comms/quality` | カスタム (`CommsQuality.msg` 推奨) | Publish | 計算されたRSSI値とスループットを出力。 |

#### 5.3. 伝搬路モデル計算ロジック

| 項目 | 詳細 |
| :--- | :--- |
| **計算頻度** | **可変サンプリングレート** (`sampling_rate` パラメータで調整可能)。デフォルト 1.0 Hz。 |
| **パスロス** | **対数距離減衰モデル** ($PL(d)$) をベースとする。 |
| **雑音 (AWGN)** | **正規分布に従うAWGN**を逐一加算。**時間または場所によって変動**するように乱数シードや分布を動的に変更するロジックを実装。 |
| **アンテナゲイン** | **CSVファイル**（E面/H面ゲイン）を参照。車両の姿勢（IMU）に基づく相対角度からゲインを線形補間 (`scipy.interpolate.interp1d`) で取得する。 |
| **スループット** | **RSSI閾値に基づくルックアップテーブル**によりGbps単位で決定。（5.4.項を参照） |
| **拡張性** | 伝搬路モデルの計算ロジックは、**Strategyパターン**を適用し、将来のNLOS/反射波モデルへの差し替えを容易にする。 |

#### 5.4. スループット決定テーブル

スループットは、以下のRSSI閾値に基づくルックアップテーブルによって決定される。

| RSSI \[dBm\] | スループット \[Gbps\] |
| :--- | :--- |
| $RSSI > -51.0$ | $6.0$ |
| $-55.0 < RSSI \le -51.0$ | $4.7$ |
| $-58.5 < RSSI \le -55.0$ | $2.7$ |
| $-61.5 < RSSI \le -58.5$ | $2.15$ |
| $-63.5 < RSSI \le -61.5$ | $1.1$ |
| $-65.5 < RSSI \le -63.5$ | $0.5$ |
| $RSSI \le -65.5$ | $0.0$ |

#### 5.5. パラメータ構成

| パラメータ | 型 | 備考 |
| :--- | :--- | :--- |
| `sampling_rate` | float | 通信計算の更新頻度 (Hz)。 |
| `noise_variance` | float | AWGNの分散値（初期設定）。 |
| `rx_position` | list [x, y, z] | 基地局の静的なワールド座標。 |
| `e_plane_path` | string | E面ゲインCSVファイルへのパス。 |
| `h_plane_path` | string | H面ゲインCSVファイルへのパス。 |
| `suv_model_path` | string | SUVモデルのメッシュデータへのパス |
| `groundstation_model_path` | string | 基地局モデルのメッシュデータへのパス |
| `comm_data_limit_mb` | float | **通信データ量の上限** \[Mb\]。`-1.0`で無制限。 |

---

### 6. TX制御とシミュレーションシナリオ

#### 6.1. TXの動作 (`tx_controller_node`)

* **経路設定**: TXは、設定ファイル (`sim_params.yaml`) から読み込んだ**マルチウェイポイントリスト**を順次追従する。
* **区間速度制御**: 各ウェイポイントは目標直線速度 (`V`) を持ち、TXはその区間の目標速度を維持するように走行する。
  * ウェイポイントデータ形式: **`[X, Y, Z, V]`** (4要素)

#### 6.2. シミュレーション制御と終了条件

| 項目 | 詳細 |
| :--- | :--- |
| **全体終了条件** | TXの全ウェイポイント到達をもって、シミュレーション全体を終了し、ROS 2ドメインをシャットダウンする。 |
| **通信停止条件** | 送信データ量 (`TotalDataTransmittion`) が `comm_data_limit_mb` に達したとき、`comms_simulator_node` は**データ送信機能のみを停止**する。車両の移動とノードの実行は継続される。 |

---

### 7. ログ出力仕様

シミュレーション終了時、`comms_simulator_node` は収集したデータをCSV形式で出力する。

| 項目 | 詳細 |
| :--- | :--- |
| **出力タイミング** | ノードが終了するとき（`atexit`フックを使用）。 |
| **ファイル命名規則** | `YYYYMMDD_HHMMSS_LIMIT-[上限値]MB.csv` (`-1.0`の場合は `LIMIT-UNLIMITED.csv`)。 |
| **CSVヘッダー** | 最初の行に、**通信データ量の上限**をコメント行 (`# ...`) として明記する。 |
| **出力項目** | 以下の項目をサンプリングごとに出力する。 |

| 項目名 | 単位 |
| :--- | :--- |
| 時間 | \[s\] |
| TX座標 (X, Y, Z) | \[m\] |
| 基地局座標 (X, Y, Z) | \[m\] |
| RSSI | \[dBm\] |
| 瞬時スループット | \[Gbps\] |
| TotalDataTransmittion | \[Mb\] |

---

### 8. インターフェース

| 項目 | 詳細 |
| :--- | :--- |
| **GUIモード** | ホスト側のXサーバーを利用する**X11 Forwarding**を設定し、GazeboのGUI（クライアント）と物理サーバーを起動。 |
| **CUIモード** | Gazeboを**Headlessモード (`gz sim -s`)**で起動し、GUI表示を省略。計算速度を優先する。 |

---

### 9. ディレクトリ構造

ros2-gazebo-comms-sim/
├── models/
│   ├── groundstation/
│   └── SUV/
├── config/
│   ├── sim_params.yaml
│   ├── e_plane.csv
│   └── h_plane.csv
├── comms_sim_pkg/
│   ├── comms_sim_pkg/
│   │   ├── comms_node.py
│   │   ├── comms_calculator.py
│   │   ├── antenna_parser.py
│   │   └── tx_controller_node.py
│   ├── launch/
│   │   └── sim_launch.py
│   ├── resource/
│   │   └── minimal_world.sdf
│   ├── package.xml
│   └── setup.py
├──log
│   └──sim_result
├── Dockerfile
├── docker-compose.yml
├── README.md
└── .gitignore

## 3. 成果物（Deliverables）

以下を出力すること。

1. `Dockerfile` および `docker-compose.yml`
2. 使用する車とドローン用SDF
3. configフォルダを作成し中にsim_params.yamlを作成
4. Gazebo World 用 SDF（モデル include およびプラグイン設定を含む）  
5. 通信シミュレーションノードの Python 実装（クラス設計を含む）  
6. `README.md` 概要  
   * プロジェクト概要
   * シミュレータ実行方法（GUIを利用した実行）
      * GUI / CUI 切り替え手順とCUIでの実行方法
   * 通信モデル拡張手順（Strategy パターン拡張方法）

注意：上記の項目はニュアンスが繊細であり，言及が必要なもののみをリストアップしている．ゆえに出力結果が「9.1. ディレクトリ構造」のすべての構成要素を満たしているかを確認し，満たしていない場合は仕様書を参照し作成すること

---

## 4. 開発フロー

以下の手順で開発を進めること。

1. Docker 環境構築  
2. Gazebo 資産作成（`minimal_world.sdf`, `antenna.sdf`, `suv.sdf`）  
3. ROS 2 パッケージ実装（`comms_sim_pkg`）  
4. Launch ファイル作成（`sim_launch.py`）  
5. マニュアル作成（`README.md`）

この手順を行うにあたって一つの項目が終了したら一時作業を中止し，ホストに継続して次の事項の作成に取り組むかどうかを逐一確認してください．

---

## 5. コーディング規約（Python）

* **PEP 8 準拠**
* **型ヒント**: 可能な限り関数シグネチャに型ヒントを付与する。
* **ログ出力**: ROS 2 標準ロガー（`self.get_logger()`）を使用する。

---

## 6. 設計原則

* **モジュール性**: 通信計算ロジック（`comms_calculator.py`）を ROS 2 ノードから分離する。
* **拡張性**: 伝搬路モデル（`LosModel`, `NlosModel` 等）を Strategy パターンで切り替え可能とする。
* **Docker 完結性**: 単一の `Dockerfile` および `docker-compose.yml` で完結させる。

---

## 7. GitHub / Git によるプロジェクト管理

| 項目 | 内容 |
| --- | --- |
| リポジトリ | 全成果物の Single Source of Truth とする |
| ブランチ戦略 | `main` を安定版、開発はフィーチャーブランチ |
| CI/CD | GitHub Actions によるビルド・Lint 自動化 |
| `.gitignore` | ROS 2 ビルド成果物、Python キャッシュ、Docker キャッシュを除外 |
