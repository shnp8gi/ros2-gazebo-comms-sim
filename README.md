# ドローン2機のすれ違い通信シミュレータ

Ubuntu 22.04 / ROS 2 Humble / Gazebo Harmonicで動作する通信シミュレータ。
ドローン同士の接近・すれ違い・離脱に伴う通信状態を計算し、接続、タイムアウト、切断、転送量を比較する。
並行・交差する飛行経路と、固定・追従するアンテナを組み合わせて実験できる。

機体は指定軌道に沿って移動する。通信モデルの比較を目的としており、空力や飛行制御の再現は対象外。
モデルごとの実装範囲や制約は、[仕様書](specification.md)と各モデルの説明を参照する。

## 初めて使う場合

1. [起動](#起動)の手順で既定設定を実行する。
2. 条件を変える場合は、[設定とモデルの選択](#設定とモデルの選択)から対象のYAMLを開く。
3. 実行後は、[結果の確認](#結果の確認)に従って保存先の設定・集計・ログを見る。
4. 計算方法を調べる場合は、[実装と資料の場所](#実装と資料の場所)からソースやモデルの説明へ進む。

周波数、送信電力、速度、配置、待ち時間などの具体的な値は設定ファイルで確認する。
実行済みの条件を確認するときは、元の設定ファイルではなく、その結果と一緒に保存された `config.yaml` を使う。

## 起動

以下のコマンドはリポジトリのルートで実行する。Docker EngineとDocker Compose pluginが必要。

### 画面なしで実行する

```bash
# イメージとプログラムをビルドし、既定設定で実行
bash tools/start_headless.sh

# 飛行経路とアンテナ方式を指定
bash tools/start_headless.sh scenario:=cross antenna_mode:=tracking

# 経路とアンテナ方式の全組み合わせを実行・検証・解析
bash tools/start_headless.sh --all
```

既定の設定は [drone_params.yaml](src/comms_sim_pkg/config/drone_params.yaml)。実行終了時に結果の保存先が表示される。

### Gazeboの画面を表示する

```bash
# 初回のみイメージを作成
docker compose -f docker-compose.yml build sim

# GUIで実行
bash tools/start_gui.sh

# 最接近時に一時停止せず、最後まで再生
bash tools/start_gui.sh pause_at_encounter:=false
```

GUIはホストのデスクトップの `DISPLAY` とXauthorityを使用する。GPU連携がなくてもソフトウェア描画で表示できる。
既定では最接近時に一時停止するため、続行する場合は画面の再生ボタンを押す。終了時はGUIを閉じる。

### Dockerを準備する

導入補助スクリプトは [tools/setup_docker_host.sh](tools/setup_docker_host.sh)。内容を確認し、管理者が対象ユーザーを指定して実行する。

```bash
sudo bash tools/setup_docker_host.sh "$(id -un)"
```

Dockerグループへの追加が現在のシェルに反映されていない場合は再ログインする。一時的に実行する場合は次の形式を使う。

```bash
sg docker -c 'bash tools/start_headless.sh'
```

### 開発用コンテナだけを起動する

```bash
bash start.sh
docker compose -f docker-compose.yml exec sim bash
```

`start.sh` は開発用コンテナを起動し、GPUの利用可否を判定する。シミュレーションは開始しない。
GUIの有無はシミュレーション起動時に選ぶ。通常の実験では `start_headless.sh` または `start_gui.sh` を直接使えばよい。

## 設定とモデルの選択

設定ファイルは [src/comms_sim_pkg/config/](src/comms_sim_pkg/config/) にある。用途に合うYAMLを選び、内部の設定と参照CSVを確認する。

| 用途 | 設定・説明の入口 |
|---|---|
| 既定の統計AWGNモデルを使う | [drone_params.yaml](src/comms_sim_pkg/config/drone_params.yaml)、[基本仕様](specification.md) |
| 受信電力の閾値によるモデルを使う | [drone_threshold_parallel_1mps.yaml](src/comms_sim_pkg/config/drone_threshold_parallel_1mps.yaml) |
| BER/PERの受信曲線による確率モデルを使う | [drone_statistical_parallel_1mps.yaml](src/comms_sim_pkg/config/drone_statistical_parallel_1mps.yaml)、[統計モデルの使い方](specification_review/statistical_model_usage_ja.txt) |
| HRCPの波形・接続処理を使う | [drone_hrcp.yaml](src/comms_sim_pkg/config/drone_hrcp.yaml)、[HRCP実装の説明](specification_review/runtime_hrcp_ja.txt) |
| 試作I/Q PHYを使う | [drone_lab_phy.yaml](src/comms_sim_pkg/config/drone_lab_phy.yaml)、[試作PHYの説明](specification_review/runtime_lab_phy_ja.txt) |

`config/` には短時間の動作確認用やモニター用の設定もある。モデルによって受信判定・接続処理・集計の定義が異なるため、比較する設定の説明を先に確認する。

### 別の設定ファイルを指定する

```bash
# 単独実行：ROS launchの引数で指定
bash tools/start_headless.sh config_file:=/workspace/src/comms_sim_pkg/config/drone_lab_phy_smoke.yaml

# 全組み合わせ：実行ツールの引数で指定
bash tools/start_headless.sh --all --config src/comms_sim_pkg/config/drone_lab_phy_smoke.yaml
```

Docker内ではリポジトリのルートが `/workspace` になる。YAML内のCSVの相対パスは、そのYAMLの所在ディレクトリが基準。
設定を別の場所へコピーする場合は、参照CSVのパスも合わせて変更する。

| 変更したいもの | 確認する場所 |
|---|---|
| 飛行経路・アンテナ方式 | YAMLの `scenario`、`antenna_mode`。起動引数でも上書き可能 |
| 機体の速度・高度・配置・実行時間 | YAMLの `drone_a`、`drone_b` と軌道・時間の項目 |
| 周波数・電力・受信回路・雑音 | YAMLの `radio_ab`、`radio_ba` |
| 伝搬・アンテナ特性 | YAMLの `propagation`、`antenna` と参照CSV |
| 受信判定・接続手順 | YAMLの `awgn`、`lab_phy`、`hrcp`、`empirical_phy`、`protocol`。使用する項目はモデルによる |
| 受信曲線・レート表 | YAMLが参照するCSV。形式と適用条件は各モデルの説明 |
| 結果の保存先 | 単独実行は `output_dir:=/workspace/sim_results/my_run`、全組み合わせは `--output sim_results/my_runs` |

既存結果のある保存先への上書きは拒否する。条件を変えて再実行するときは新しい保存先を使う。

## 結果の確認

通常の結果は `sim_results/` に保存される。全組み合わせを実行すると、親ディレクトリの下に各条件の結果と比較表が作成される。

| 最初に見るもの | 確認できる内容 |
|---|---|
| `completed` | 実行が完了したことを示す記録 |
| `config.yaml` | その実行で使用した条件とモデル |
| `summary.csv` | 接続・タイムアウト・切断・転送量などの集計 |
| `timeseries.csv` | 位置・受信電力・通信状態の時間変化 |
| `events.csv` / `frames.csv` | 接続手順やフレームごとの成否・失敗理由 |
| `comparison.csv` | 全組み合わせ実行の比較表 |

通常の保存モードでは `world.sdf`、`provenance.json`、`inputs/` に実行環境や入力情報も保存する。
出力項目はモデルと保存モードによって異なる。列の意味や集計方法は [specification.md](specification.md) と使用モデルの説明を参照する。

### 検証・解析する

以下はDockerコンテナ内、または必要なPython依存関係を導入した環境で実行する。
`sim_results/my_run` は実際の保存先に置き換える。

```bash
python3 tools/validate_drone_results.py sim_results/my_run
python3 tools/analyze_drone_results.py sim_results/my_run
```

全組み合わせの実行ツールは共通の検証・解析を自動実行する。追加の解析は用途に応じて次を使う。

| 用途 | ツール |
|---|---|
| 試作I/Q PHYの追加検証 | [validate_lab_phy_results.py](tools/validate_lab_phy_results.py) |
| 受信曲線による確率モデルの追加解析 | [analyze_statistical_results.py](tools/analyze_statistical_results.py) |
| 状態遷移・受信電力の滞在割合を作図 | [plot_protocol_occupancy.py](tools/plot_protocol_occupancy.py)、[plot_state_by_rssi.py](tools/plot_state_by_rssi.py)。従来モデルの全組み合わせ結果が対象 |
| 試作I/Q PHYの信号帯別goodputを作図 | [plot_throughput_by_signal.py](tools/plot_throughput_by_signal.py)。短時間設定の全組み合わせ結果が対象 |
| 信号モニター・ログ再生 | [run_hrcp_monitor.py](tools/run_hrcp_monitor.py)、[monitor_hrcp_signals.py](tools/monitor_hrcp_signals.py) |

引数は各ツールの `--help` で確認できる。解析・作図にはPyYAML、NumPy、Matplotlibなどを使用する。
日本語表示が欠ける場合は、作図する環境に `fonts-noto-cjk` を導入する。

## ビルドとテスト

通常のビルド対象は `comms_sim_pkg`。起動スクリプトはビルドも実行する。
ビルド・テストだけを行う場合は、Dockerイメージ作成後に次を実行する。

```bash
docker compose -f docker-compose.yml run --rm --no-deps --user "$(id -u):$(id -g)" -e HOME=/tmp build
docker compose -f docker-compose.yml run --rm --no-deps --user "$(id -u):$(id -g)" -e HOME=/tmp sim \
  ctest --test-dir build/drone/comms_sim_pkg --output-on-failure
```

Docker内、または同じROS/Gazebo依存関係を導入した環境では直接ビルドできる。

```bash
source /opt/ros/humble/setup.bash
colcon build --build-base build/drone --install-base install/drone --packages-select comms_sim_pkg --symlink-install
source install/drone/setup.bash
ros2 launch comms_sim_pkg sim_launch.py headless:=true
```

テストの内容は [test/](src/comms_sim_pkg/test/)、登録一覧は [CMakeLists.txt](src/comms_sim_pkg/CMakeLists.txt) にある。
専用の実験スクリプトが別のビルド先を使う場合は、そのモデルの手順に従う。例えば [run_statistical_model.py](tools/run_statistical_model.py) は `install/no_expected_rssi` を使用するため、[専用手順](specification_review/statistical_model_usage_ja.txt)を確認する。

## 実装と資料の場所

| 調べたいこと | 場所 |
|---|---|
| 設定の読み込み・world生成・ROS bridgeの起動 | [launch/sim_launch.py](src/comms_sim_pkg/launch/sim_launch.py) |
| Gazeboの機体・アンテナ更新と通信処理の接続 | [src/DroneEncounterPlugin.cc](src/comms_sim_pkg/src/DroneEncounterPlugin.cc) |
| 軌道・無線計算・従来の接続処理 | [src/drone_link.cpp](src/comms_sim_pkg/src/drone_link.cpp) |
| アンテナ特性・伝搬計算 | [src/antenna_pattern_parser.cpp](src/comms_sim_pkg/src/antenna_pattern_parser.cpp)、[src/comms_calculator.cpp](src/comms_sim_pkg/src/comms_calculator.cpp) |
| 統計AWGN・試作PHY・受信曲線モデル | [src/awgn.cpp](src/comms_sim_pkg/src/awgn.cpp)、[src/lab_phy.cpp](src/comms_sim_pkg/src/lab_phy.cpp)、[src/empirical_phy.cpp](src/comms_sim_pkg/src/empirical_phy.cpp) |
| HRCPの通信処理 | [src/](src/comms_sim_pkg/src/) の `hrcp_*`。宣言は [include/comms_sim_pkg/](src/comms_sim_pkg/include/comms_sim_pkg/) |
| 機体・アンテナモデルとworld | [models/](src/comms_sim_pkg/models/)、[worlds/](src/comms_sim_pkg/worlds/) |
| 実行・検証・解析・測定CSVの変換 | [tools/](tools/) |
| 測定データの入力形式 | [統計モデルの使い方](specification_review/statistical_model_usage_ja.txt)、[条件表テンプレート](receiver_measurement_manifest_template.csv)、[CSV変換ツール](tools/prepare_receiver_curve.py) |
| 基本仕様とログの定義 | [specification.md](specification.md) |
| モデルごとの説明・規格照合・検討中の設計 | [specification_review/](specification_review/) |
| 資料の出典一覧 | [references/manifest.json](specification_review/references/manifest.json) |
| Dockerの構成 | [Dockerfile](Dockerfile)、[docker-compose.yml](docker-compose.yml)、[entrypoint.sh](entrypoint.sh) |

`specification_review/` には実装説明と検討中の設計案が含まれる。実行設定は `config/` のYAMLを使い、提案用のYAMLと区別する。
`sim_results/` は実験結果、`build/`・`install/`・`log/` はビルド関連の生成物。これらのデータ本体はGit管理対象外。

ドローンモデルの出典・作者・ライセンス・変更点は [ATTRIBUTION.txt](src/comms_sim_pkg/models/drone/ATTRIBUTION.txt) に記載する。
モデルはリポジトリに同梱しており、実行時の取得は不要。

## ブランチの使い分け

`main` はドローン通信実験用。車両HO（ハンドオーバー）の開発は別系統で保持する。

| ブランチ | 内容 |
|---|---|
| `feat/urban-2lane` | 都市道路・車両HO、評価・リプレイ |
| `feat/production-sim-two-phase` | 車両・RSUの二相評価 |
| `imp-var` | KKFによる予測HOと評価基盤 |

HOを扱う場合は対象ブランチのREADMEを確認する。
