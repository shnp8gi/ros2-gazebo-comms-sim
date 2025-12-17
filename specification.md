# 📡 ROS 2/Gazebo 通信シミュレータ 統合システム仕様書

## 変更履歴

| 変更日 | バージョン | 改定内容 |
| :--- | :--- | :--- |
| 2025/12/14 | ver 1.0 | 初版 |
| 2025/12/14 | ver 2.0 | UGVのマルチウェイポイント追従と区間速度制御、アンテナゲインの動的参照、通信容量上限による通信停止機能の追加、およびCSVロギング仕様の確定 |

---

## 1. プロジェクト概要

 Gazebo Simで駆動する移動車両と固定基地局間の通信品質（RSSI/スループット）を、ROS 2ノード上で数理モデルを用いてシミュレーションし、その挙動を評価する。


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
| **車両** | IMU | `/imu/data` (標準メッセージ) | 姿勢情報 (Roll, Pitch, Yaw) の提供。**アンテナゲイン計算**に利用。 |
| **基地局** | N/A | パラメータサーバーから座標を取得 | 静的なためセンサ不要。 |

---

## 5. 通信シミュレーション要件 (ROS 2 Node)

### 5.1. 通信ノード (`comms_simulator_node`)
| 項目 | 詳細 |
| :--- | :--- |
| **ノード名** | `comms_simulator_node` |
| **実装言語** | Python |
| **役割** | Gazeboから取得した位置情報と姿勢情報に基づき、カスタムの伝搬路モデルで通信品質を計算し、ロギング及びPublishを行う。 |

### 5.2. 通信インターフェース

| 項目 | トピック名/パラメータ | メッセージ型 | 送受信 | 備考 |
| :--- | :--- | :--- | :--- | :--- |
| **車両位置** | `/gps/fix`, `/imu/data` | `sensor_msgs/NavSatFix`, `sensor_msgs/Imu` | Subscribe | Gazeboから車両の位置・姿勢を取得。 |
| **通信結果** | `/comms/quality` | カスタム (`CommsQuality.msg` 推奨) | Publish | 計算されたRSSI値とスループットを出力。 |

### 5.3. 伝搬路モデル計算ロジック

| 項目 | 詳細 |
| :--- | :--- |
| **計算頻度** | **可変サンプリングレート** (`sampling_rate` パラメータで調整可能)。デフォルト 1.0 Hz。 |
| **パスロス** | **対数距離減衰モデル** ($PL(d)$) をベースとする。 |
| **雑音 (AWGN)** | **正規分布に従うAWGN**を逐一加算。**時間または場所によって変動**するように乱数シードや分布を動的に変更するロジックを実装。 |
| **アンテナゲイン** | **CSVファイル**（E面/H面ゲイン）を参照。車両の姿勢（IMU）に基づく相対角度からゲインを線形補間 (`scipy.interpolate.interp1d`) で取得する。 |
| **スループット** | **RSSI閾値に基づくルックアップテーブル**によりGbps単位で決定。（5.4.項を参照） |
| **拡張性** | 伝搬路モデルの計算ロジックは、**Strategyパターン**を適用し、将来のNLOS/反射波モデルへの差し替えを容易にする。 |

### 5.4. スループット決定テーブル

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

### 5.5. パラメータ構成

| パラメータ | 型 | 備考 |
| :--- | :--- | :--- |
| `sampling_rate` | float | 通信計算の更新頻度 (Hz)。 |
| `noise_variance` | float | AWGNの分散値（初期設定）。 |
| `base_station_position` | list [x, y, z] | 基地局の静的なワールド座標。 |
| `e_plane_path` | string | E面ゲインCSVファイルへのパス。 |
| `h_plane_path` | string | H面ゲインCSVファイルへのパス。 |
| `suv_model_path` | string | SUVモデルのメッシュデータへのパス |
| `groundstation_model_path` | string | 基地局モデルのメッシュデータへのパス |
| `comm_data_limit_mb` | float | **通信データ量の上限** \[Mb\]。`-1.0`で無制限。 |

---

## 6. UGV制御とシミュレーションシナリオ

### 6.1. UGVの動作 (`ugv_controller_node`)

* **経路設定**: UGVは、設定ファイル (`sim_params.yaml`) から読み込んだ**マルチウェイポイントリスト**を順次追従する。
* **区間速度制御**: 各ウェイポイントは目標直線速度 (`V`) を持ち、UGVはその区間の目標速度を維持するように走行する。
    * ウェイポイントデータ形式: **`[X, Y, Z, V]`** (4要素)

### 6.2. シミュレーション制御と終了条件

| 項目 | 詳細 |
| :--- | :--- |
| **全体終了条件** | UGVの全ウェイポイント到達をもって、シミュレーション全体を終了し、ROS 2ドメインをシャットダウンする。 |
| **通信停止条件** | 送信データ量 (`TotalDataTransmittion`) が `comm_data_limit_mb` に達したとき、`comms_simulator_node` は**データ送信機能のみを停止**する。車両の移動とノードの実行は継続される。 |

---

## 7. ログ出力仕様

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
| UGV座標 (X, Y, Z) | \[m\] |
| 基地局座標 (X, Y, Z) | \[m\] |
| RSSI | \[dBm\] |
| 瞬時スループット | \[Gbps\] |
| TotalDataTransmittion | \[Mb\] |

---

## 8. インターフェース

| 項目 | 詳細 |
| :--- | :--- |
| **GUIモード** | ホスト側のXサーバーを利用する**X11 Forwarding**を設定し、GazeboのGUI（クライアント）と物理サーバーを起動。 |
| **CUIモード** | Gazeboを**Headlessモード (`gz sim -s`)**で起動し、GUI表示を省略。計算速度を優先する。 |

---

## 9. ディレクトリ構造

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
│   │   └── ugv_controller_node.py
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