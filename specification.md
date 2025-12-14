# 📡 ROS 2/Gazebo 通信シミュレータ 統合システム仕様書

## 1. プロジェクト概要

| 項目 | 詳細 |
| :--- | :--- |
| **目的** | Gazebo Simで駆動する移動車両と固定基地局間の通信品質（RSSI/スループット）を、ROS 2ノード上で数理モデルを用いてシミュレーションし、その挙動を評価する。 |
| **利用期間** | 2～3年間の利用を想定し、LTSバージョンを採用。 |
| **拡張性** | ドローン、マルチエージェント、高度な伝搬路モデル（NLOS、反射波）への拡張性を確保。 |
| **制約** | シミュレーション実行環境はGPU非搭載（CPU内蔵グラフィックス）を前提とし、計算負荷を最小化する設計とする。 |

---

## 2. 技術スタックと環境構築

| 分野 | 項目 | 決定事項 | 備考 |
| :--- | :--- | :--- | :--- |
| **OS** | ベースOS | Ubuntu 22.04 LTS (Dockerコンテナ内) |
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

## 4. ロボットとセンサ構成

| エンティティ | センサ/プラグイン | ROS 2 トピック (Publish) | 用途 |
| :--- | :--- | :--- | :--- |
| **車両** | Diff Drive (プラグイン) | `/cmd_vel` (Subscribe) | 運動制御。 |
| **車両** | NavSat (GNSS) | `/gps/fix` (標準メッセージ) | $x, y, z$ 位置情報の提供。 |
| **車両** | IMU | `/imu/data` (標準メッセージ) | 姿勢情報 (Roll, Pitch, Yaw) の提供。 |
| **基地局** | N/A | パラメータサーバーから座標を取得 | 静的なためセンサ不要。 |

---

## 5. 通信シミュレーション要件 (ROS 2 Node)

### 5.1. 通信ノード (`comms_simulator_node`)
| 項目 | 詳細 |
| :--- | :--- |
| **ノード名** | `comms_simulator_node` |
| **実装言語** | Python |
| **役割** | Gazeboから取得した位置情報に基づき、カスタムの伝搬路モデルで通信品質を計算する。 |

### 5.2. 通信インターフェース

| 項目 | トピック名/パラメータ | メッセージ型 | 送受信 | 備考 |
| :--- | :--- | :--- | :--- | :--- |
| **車両位置** | `/gps/fix`, `/imu/data` | `sensor_msgs/NavSatFix`, `sensor_msgs/Imu` | Subscribe | Gazeboから車両の位置・姿勢を取得。 |
| **通信結果** | `/comms/rssi`, `/comms/throughput` | カスタム (`CommsQuality.msg` 推奨) | Publish | 計算されたRSSI値とスループットを出力。 |

### 5.3. 伝搬路モデル計算ロジック

| 項目 | 詳細 |
| :--- | :--- |
| **計算頻度** | **可変サンプリングレート** (`sampling_rate` パラメータで調整可能)。デフォルト 1.0 Hz。 |
| **パスロス** | **対数距離減衰モデル** ($PL(d)$) をベースとする。 |
| **雑音 (AWGN)** | **正規分布に従うAWGN**を逐一加算。**時間または場所によって変動**するように乱数シードや分布を動的に変更するロジックを実装。 |
| **アンテナ** | お客様提供の**CSVファイル**（E面/H面ゲイン）を参照。車両の姿勢（IMU）に基づく相対角度からゲインを線形補間で取得。 |
| **拡張性** | 伝搬路モデルの計算ロジックは、**Strategyパターン**を適用し、将来のNLOS/反射波モデルへの差し替えを容易にする。 |

### 5.4. パラメータ構成

| パラメータ | 型 | 備考 |
| :--- | :--- | :--- |
| `sampling_rate` | float | 通信計算の更新頻度 (Hz)。 |
| `noise_variance` | float | AWGNの分散値（初期設定）。 |
| `base_station_position` | list [x, y, z] | 基地局の静的なワールド座標。 |
| `antenna_pattern_path` | string | CSVファイルへのパス。 |

---

## 6. Docker運用とマニュアル

| 項目 | 詳細 |
| :--- | :--- |
| **GUIモード** | ホスト側のXサーバーを利用する**X11 Forwarding**を設定し、GazeboのGUI（クライアント）と物理サーバーを起動。 |
| **CUIモード** | Gazeboを**Headlessモード (`gz sim -s`)**で起動し、GUI表示を省略。計算速度を優先する。 |
| **マニュアル** | `README.md` に、`docker compose` コマンドの引数や環境変数を用いてGUI/CUIを切り替える具体的な手順を明記する。 |

---

## 7. 実装要件とコーディング規約

### 7.1. ディレクトリ構造

ros2-gazebo-comms-sim/
├── models/
│   ├── antenna/
│   └── SUV/
├── config/
│   ├── sim_params.yaml
│   ├── sample_e_plane.csv
│   └── sample_h_plane.csv
├── comms_sim_pkg/
│   ├── comms_sim_pkg/
│   │   ├── comms_node.py
│   │   ├── comms_calculator.py
│   │   └── antenna_parser.py
│   ├── launch/
│   │   └── sim_launch.py
│   ├── resource/
│   │   └── minimal_world.sdf
│   ├── package.xml
│   └── setup.py
├── Dockerfile
├── docker-compose.yml
├── README.md
└── .gitignore

### 7.2. コーディング規約 (Python)

* **PEP 8**: 全てのコードはPEP 8に準拠すること。
* **型ヒント**: 関数シグネチャには可能な限り型ヒント（Type Hints）を使用し、可読性と保守性を高めること。
* **ログ**: ROS 2の標準ロギング機能 (`self.get_logger().info(...)`) を使用すること。

### 7.3. 設計原則

* **モジュール性**: 通信計算ロジック（`comms_calculator.py`）は、ROS 2ノード (`comms_node.py`) から完全に分離し、ROSの依存性を極力持たせないように設計する。
* **拡張性 (Strategyパターン)**: `comms_calculator.py` 内で、伝搬路モデルの計算方法（例: `LosModel`, `NlosModel`）をクラスとして定義し、実行時に切り替えられるように抽象基底クラスを設けること。
* **Docker**: ホスト環境に依存しない、単一の `Dockerfile` と `docker-compose.yml` で完結させること。

---

## 8. GitHub/Gitによるプロジェクト管理

| 項目 | 趣旨・設定 |
| :--- | :--- |
| **リポジトリ** | **全てのソースコード、設定ファイル、ドキュメントの唯一の真の情報源 (Single Source of Truth) とする。** |
| **ブランチ戦略** | `main` ブランチを安定版とし、開発はフィーチャーブランチで行い、プルリクエスト (Pull Request) 経由でのマージを必須とする。 |
| **CI/CD** | GitHub Actionsを設定し、Dockerイメージのビルド、ROS 2パッケージのビルド、PythonコードのLINTチェックを自動実行し、コード品質と環境再現性を保証する。 |
| **`.gitignore`** | ROS 2のビルドディレクトリ (`build/`, `install/`, `log/`)、Python環境 (`venv`, `__pycache__`), Dockerキャッシュなど、**生成物ファイルは厳格に除外**する。 |

---

## 9. 開発フローの指示

専任AIは、以下のステップで開発を進めること。

1.  **Docker環境の構築**: `Dockerfile`と`docker-compose.yml`を生成し、ROS 2 HumbleとGazebo Harmonicの連携環境を構築する。
2.  **Gazebo資産の作成**: `minimal_world.sdf`、`antenna.sdf`、`suv.sdf`（Fuelモデルを参照し、センサプラグイン設定を含む）を作成する。
3.  **ROS 2パッケージの実装**: `comms_sim_pkg` の構成に基づき、Pythonノード、ユーティリティ、カスタムメッセージ（必要な場合）を実装する。
4.  **Launchファイルの作成**: GUI/Headlessモードの切り替えを引数で制御できる`sim_launch.py`を作成する。
5.  **マニュアル作成**: `README.md` に、Dockerビルド/実行手順、GUI/CUI切り替え方法、およびモデルをローカルに配置するための手順（`GZ_SIM_RESOURCE_PATH`の設定）を分かりやすく記述する。