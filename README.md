# ドローン2機のすれ違い通信シミュレータ

Ubuntu 22.04 / ROS 2 Humble / Gazebo Harmonic向け。
2機のドローンが飛行し、方向別の無線リンクと接続手順を計算する。接続失敗・無応答タイムアウトが起きる条件の比較を目的とする。

## 案内

- [起動](#起動)：Dockerで単独条件・4条件比較・GUIを実行する。
- [設定とPHYモード](#設定とphyモード)：既定の統計AWGNと試作I/Q PHYを選ぶ。
- [結果の読み方](#結果の読み方)・[作図](#プロトコル状態とrssi滞在割合の作図)：検証と解析の手順。
- [ビルドとテスト](#ビルドとテスト)：開発時の確認手順。
- [リポジトリ構成](#リポジトリ構成)：コード・資料・生成物の役割。
- [specification.md](specification.md)：計算式、接続手順、ログの定義とモデルの制約。

## シミュレーションの概要

| 条件 | 初期設定 |
|---|---|
| A → B | 272 GHz、−1 dBm（約0.794 mW）、データ／Request／ACK |
| B → A | 61 GHz、−8 dBm（約0.158 mW）、Beacon／Response／データACK |
| 速度 | A・B各50 km/h、個別変更可能 |
| 並行すれ違い | 逆方向飛行、高度差10 m、横間隔10 m |
| 十字すれ違い | 直交する経路、高度差10 m、同時に平面交点を通過 |
| 固定アンテナ | 低いAは仰角+45°、高いBは−45°、並行では相手側の真横、十字ではAの左側／Bの右側（交差後の相手側） |
| 追従アンテナ | 両機が常に相手を向く理想追従 |
| 待ち期限 | Response待ち・ACK待ちは500 msのタイムアウト。接続後の無応答は500 msで切断し、別集計 |
| 実行時間 | 40 s、20 sですれ違い、初期高度A=20 m／B=30 m |

接続は `Beacon → Request → Response → ACK → DATA → DATA_ACK`。
**IEEE規格に完全準拠した実装ではない。** 接続手順は802.15.3eを参考にした抽象モデル。PHYの設計目標はA→Bが802.15.3d、B→Aが802.15.3eだが、現在の実装は下記の統計モデルと独自の試作PHYである。フレーム長・周期、アンテナ特性の共用などの仮定は [specification.md](specification.md) に記載する。

## 電力計算

ユーザー提供Excel「リンク_4m_1」のアンテナ利得、LNA、変換利得、挿入損失を反映。
受信判定の信号電力はTJX入力端とし、閾値は両方向−68.5 dBmを維持する。
熱雑音は300 K・2.1648 GHzで計算し、NFは272 GHz側10 dB、61 GHz側0 dB（ユーザー指定の理想受信機、実測後に変更）。
既定設定はQPSK・誤り訂正なしの統計AWGNフレーム誤り判定を有効化。固定3.52Gbps、制御フレーム64byteの仮定。
`required_snr_db: null` は固定SNR閾値を追加しない意味で、AWGN判定は `awgn.enabled` で制御する。
式・測定基準点・仮定は [specification.md](specification.md#無線計算) を参照。

Gazeboを起動せず、同じ通信コアで距離別の正対リンクバジェットをCSV出力できる。
ビルド後のROS環境で実行する：

```bash
ros2 run comms_sim_pkg link_budget_report src/comms_sim_pkg/config/drone_params.yaml
```

旧結果は電力・周波数・受信回路の設定が異なるため、新設定の結果と区別する。

## 起動

Docker EngineとCompose pluginを導入したホストで、リポジトリのルートから実行する。
通常は以下のスクリプトだけで起動できる。`start.sh`を先に実行する必要はない。

```bash
# 並行・固定（初期設定）。イメージとプログラムをビルドして起動
bash tools/start_headless.sh

# 指定の4条件を順次実行し、位置・集計を検証して比較表と図を生成
bash tools/start_headless.sh --all

# 十字・追従、A=40 km/h / B=60 km/h
bash tools/start_headless.sh scenario:=cross antenna_mode:=tracking speed_a_kmh:=40 speed_b_kmh:=60

# 4条件すべてを独立速度で実行
bash tools/start_headless.sh --all --speed-a-kmh 40 --speed-b-kmh 60
```

Dockerグループに追加済みでも現在のシェルへ反映されていない場合は、再ログインするか以下を使う。

```bash
sg docker -c 'bash tools/start_headless.sh --all'
```

初回のDocker導入に管理者権限が必要な場合は、管理者が以下を実行する。Dockerグループはホストのroot相当の操作が可能になるため、登録するユーザーを確認する。

```bash
sudo bash tools/setup_docker_host.sh rosgazebo
```

CPUのみでヘッドレス実行できる。イメージ作成後、デスクトップ上へGUIを出す場合は以下を実行する（初回のイメージ作成だけを行う場合は `docker compose -f docker-compose.yml build sim`）。Xauthorityを読み取り専用で渡し、DockerのGPU連携がなくてもソフトウェア描画で表示する。最接近で自動的に一時停止し、画面の再生ボタンで続行できる。

```bash
# 最接近で一時停止して配置を確認
sg docker -c 'bash tools/start_gui.sh scenario:=parallel antenna_mode:=fixed'

# 最初から最後まで通して再生
sg docker -c 'bash tools/start_gui.sh scenario:=parallel antenna_mode:=fixed pause_at_encounter:=false'
```

Docker内または同じ依存関係を導入したROS環境で直接起動する場合：

```bash
source /opt/ros/humble/setup.bash
source install/drone/setup.bash
ros2 launch comms_sim_pkg sim_launch.py headless:=false scenario:=cross antenna_mode:=tracking
```

`config_file:=...` で設定ファイルを指定できる。Docker内ではホストのリポジトリルートが `/workspace` になる。出力先の指定は `output_dir:=/workspace/sim_results/my_run`。既存結果があるディレクトリへの上書きは拒否する。

比較時に変えた条件を追跡できるよう、実行時設定と入力CSVのコピー、入力CSVのSHA256を結果フォルダへ保存する。

### コンテナとシミュレーションの起動を分ける

`bash start.sh` は開発用の常駐コンテナを起動する。GPU利用の可否だけを判定し、シミュレーションは開始しない。コンテナ起動時にheadlessを選ぶ必要はなく、`DISPLAY`の有無でも自動選択しない。

| 入口 | 用途 |
|---|---|
| `bash tools/start_headless.sh` | ビルド後、`headless:=true`で実験を実行 |
| `bash tools/start_gui.sh` | ビルド後、Xauthorityを渡して`headless:=false`で実行（ソフトウェア描画） |
| `bash start.sh` | 開発用コンテナを起動。`docker compose -f docker-compose.yml exec sim bash`で入る |
| `ros2 launch comms_sim_pkg sim_launch.py` | ROS環境から直接実行。`headless`の既定値は`true` |

常駐コンテナ内からGUIを直接起動する場合は、別途X11の認証設定が必要。ホストのデスクトップへ表示するときは `tools/start_gui.sh` を使う。

## 設定とPHYモード

| 設定ファイル | PHY・用途 | 実行時間／すれ違い時刻 |
|---|---|---|
| [drone_params.yaml](src/comms_sim_pkg/config/drone_params.yaml) | 既定。QPSKの理論BER/PERからフレーム成否を抽選する統計AWGN | 40 s／20 s |
| [drone_lab_phy.yaml](src/comms_sim_pkg/config/drone_lab_phy.yaml) | 試作I/Q PHY。ビット系列、矩形パルス、AWGN、復調、CRC判定 | 40 s／20 s |
| [drone_lab_phy_smoke.yaml](src/comms_sim_pkg/config/drone_lab_phy_smoke.yaml) | 同じ試作PHYの短時間動作確認用 | 1.1 s／0.55 s |

試作PHYは `lab_phy.enabled: true` で選択し、payload FECは未実装。制御フレーム長と送信時間は実際のビット／サンプル数から計算するため、統計AWGN用の64 byte・20 µs・100 µs設定を適用しない。制約とログの意味は [試作PHYの実装説明](specification_review/runtime_lab_phy_ja.txt) を参照。

```bash
# 試作PHYの短時間4条件比較（ホストから実行）
bash tools/start_headless.sh --all --config src/comms_sim_pkg/config/drone_lab_phy_smoke.yaml

# 試作PHYの通常設定を単独実行
bash tools/start_headless.sh config_file:=/workspace/src/comms_sim_pkg/config/drone_lab_phy.yaml
```

単独実行はROSの `config_file:=...`、`--all` は実行ツールの `--config ...` を使う。前者のDocker内パスではリポジトリルートが `/workspace` になる。YAML内のアンテナCSVとMCS表の相対パスは、そのYAMLの所在ディレクトリが基準。設定を別ディレクトリへコピーするときはCSVの参照も合わせて変更する。

両モードとも固定QPSKであり、[適応MCS案](specification_review/adaptive_mcs_design_ja.txt) は実行コードへ未接続。`MCStable.csv` は統計AWGNと試作PHYではレート選択に使わない。`lab_phy.enabled: false`（または省略）かつ `awgn.enabled: false` の互換モードで使用する。

短時間設定と40秒設定は観測する軌道区間が異なる。結果を比較するときは、実行時間・PHYモード・seed・電力設定を確認する。

## 結果の読み方

通常の保存先は `sim_results/drone_日時_ID/`。4条件実行では `sim_results/drone_four_日時_ID/` 配下に各条件のフォルダ、実行ログ、`manifest.json`、`comparison.csv` を作る。

| ファイル | 内容 |
|---|---|
| `config.yaml` / `world.sdf` / `provenance.json` / `inputs/` | 実際に使った設定・world・モデル情報・入力CSVとハッシュ |
| `timeseries.csv` | 10 msごとの位置、両方向RSSI、感度との差、指向ずれ、接続状態、確認済み転送量 |
| `events.csv` | 制御フレーム送受信、受信失敗、待ち開始、タイムアウト、切断、Beacon選択／無視／停止。beacon_indexで対応付け |
| `frames.csv` | AWGN有効時の全フレームのBER/PER・乱数値・受信成否・失敗理由 |
| `connections.csv` | 成功接続ごとのBeacon開始／受信・ACK受信時刻、リンク確立時間、開始からの待ち時間 |
| `gazebo_poses.csv` | Gazeboで更新された2機・2アンテナの実際の位置姿勢（50 Hz） |
| `summary.csv` | 試行数、接続数、接続前タイムアウト数、接続後切断数、接続時間、確認済み転送量 |
| `completed` | 実行時間に到達し、CSV保存が成功した印 |
| `conditions.csv` | 実験条件を1行にした表（解析時に生成） |
| `beacon_requests.csv` | Requestごとに起点Beaconの番号・送受信時刻、Request成否、成立／タイムアウトを対応付け |
| `timeouts.csv` | 接続前のタイムアウト1件1行。失敗時と判定時の距離・RSSI・指向ずれを別々に記録 |
| `disconnections.csv` | 接続後の切断1件1行。タイムアウト件数には含めない |
| `classification_summary.csv` | 新定義のタイムアウト数と切断数。旧ログ再解析時も原本summary.csvは保持 |
| `pending_waits.csv` | 実験終了時点で判定時刻に達していない待ち。タイムアウト0件と区別する |
| `timeout_counts.csv` | 種類別回数、Beacon受信失敗数、接続試行数・成立数 |
| `timeline.png` | すれ違い時刻基準のRSSI・指向ずれ・転送量と、失敗／タイムアウトの時刻 |

4条件実行では検証・解析を自動実行する。単独実行の結果も、ROSコンテナ内で以下から検証・解析できる。

```bash
python3 tools/validate_drone_results.py sim_results/my_run
python3 tools/analyze_drone_results.py sim_results/my_run

# 試作I/Q PHYの場合は、サンプル時刻・フレーム長・転送量も追加検証
python3 tools/validate_lab_phy_results.py sim_results/my_run
```

4条件実行の自動検証は `validate_drone_results.py` による共通検証。試作PHY固有の検証は各条件のフォルダに対して上記を追加実行する。

`timeouts.csv` の `wait_start_s` は500 msの起点、`failure_sample_s` はフレーム内で受信不可を検出した最初のサンプル時刻、`timeout_s` は待ち切れた時刻。`margin_ab_db` / `margin_ba_db` は受信感度との差で、負なら受信できない。

Beaconは140 µs周期（送信開始間隔）で、BがRequestを受信するまで継続する。AはRequest送信後、新しいBeaconを無視して最初のResponseを500 ms待ち、期限を更新しない。BがRequestを受信した時点でBeaconを停止する。A/Bとも送信中は受信できず、異なるBeacon番号のフレーム間でも送受信区間が重なれば `RECEIVER_TRANSMITTING` として失敗する。

`beacon_index` は実験内で1から連番とし、再接続でリセットしない。Request・Response・ACK・接続後DATAにも起点の番号を引き継ぐ。`BEACON_SELECTED` は応答対象の選択、`BEACON_IGNORED` はAが処理中のため無視、`BEACON_STOPPED` はBによるRequest受信を表す。状態図のResponse待ち中でもBはBeaconを送信し得る。

`timeouts` は接続前のResponse/ACK待ちだけを数える。接続成立後のDATA/DATA_ACK無応答は `disconnections` に数え、初回DATA失敗で転送量が0 MBでもタイムアウトに含めない。500 msの無応答期限は維持する。

**タイムアウト0件でも接続できているとは限らない。** Beaconを一度も受信できない条件では、接続試行もタイムアウトも0件になる。必ず試行数・成立数・Beacon受信失敗数と併せて読む。AWGN有効時はフレームごとに確率的な誤りを抽選する。同じseedでは同じ結果になるため、独立試行にはseedを変える。

## ドローンモデル

ユーザー指定の [MuhammadIrhamArijan / drone](https://app.gazebosim.org/MuhammadIrhamArijan/fuel/models/drone)（Fuel version 1、Parrot Bebop 2）を [models/drone](src/comms_sim_pkg/models/drone) に保存済み。起動時にFuelへ接続する必要はない。

Fuel記載ライセンスは **CC BY 4.0**。`model.config` の作者は Olivier Crave (Parrot)。出典、取得ファイルのハッシュ、ローカル変更点は [ATTRIBUTION.txt](src/comms_sim_pkg/models/drone/ATTRIBUTION.txt) に保存。メッシュとサムネイルは原本を使用し、参照URIを変更した。重力なしのworldで可動モデルへ指定軌道を与える。ローター駆動・空力・姿勢安定化・風のシミュレーションは含めない。黄色の線と先端がアンテナ正面方向を示す。

## ビルドとテスト

古い成果物と混在しないよう、ビルド先は `build/drone`、インストール先は `install/drone`。通常のビルド対象は `comms_sim_pkg` のみで、現在のドローン実験は `comms_sim_msgs` に依存しない。

```bash
# Docker内、または同じROS/Gazebo依存関係を導入した環境
source /opt/ros/humble/setup.bash
colcon build --build-base build/drone --install-base install/drone --packages-select comms_sim_pkg --symlink-install
ctest --test-dir build/drone/comms_sim_pkg --output-on-failure
```

ホストから既存のDockerイメージでビルド・テストだけを実行する場合：

```bash
docker compose -f docker-compose.yml run --rm --no-deps --user "$(id -u):$(id -g)" -e HOME=/tmp build
docker compose -f docker-compose.yml run --rm --no-deps --user "$(id -u):$(id -g)" -e HOME=/tmp sim \
  ctest --test-dir build/drone/comms_sim_pkg --output-on-failure
```

CTestは `drone_link_tests`、`link_budget_tests`、`awgn_tests`、`lab_phy_tests` の4件。通信コアのテストは、両機の速度・最接近距離・追従方向・周波数別損失・接続順序・500 ms境界・ACK前の転送量非加算・物理更新刻みによらないイベント処理を検証する。4条件の実行ツールは、Gazebo実位置と理想軌道、アンテナ方向、時系列とサマリーの一致も検証する。

## プロトコル状態とRSSI滞在割合の作図

4条件を実行した親ディレクトリを指定する。以下の `sim_results/my_four_runs` は実際の保存先に置き換える。Python依存はPyYAML・NumPy・Matplotlib（Dockerイメージに導入済み）。日本語フォントはNoto Sans CJK JPを使用する。Dockerfileには日本語フォントの導入が含まれていないため、日本語が欠ける場合は作図する環境に `fonts-noto-cjk` を導入する。

```bash
python3 tools/plot_protocol_occupancy.py sim_results/my_four_runs
```

`protocol_analysis/` に状態遷移の全体図・接続手順の拡大図・RSSI滞在割合図をPNG/PDFで保存する。
状態区間はイベントログから復元し、RSSIは10msログを時間加重して5dB刻みで集計する。
`--bin-db 1` で刻みを変更できる。RSSIは全時間と接続中（切断待ち除外）を別々に集計し、
未成立条件の接続中割合は分母なしとして空欄にする。数値CSVと集計方法の説明も同じ場所に保存する。

RSSI帯を分母にした「そのRSSIのとき、どの状態にいたか」は次で作図する。

```bash
python3 tools/plot_state_by_rssi.py sim_results/my_four_runs
```

`state_by_rssi.png/pdf/csv` と境界拡大図、接続境界のログ図を保存。各RSSI帯の中で状態割合の合計が100%になる。

試作I/Q PHYの短時間4条件ログでは、信号帯別のgoodputと既存MCS表の参考比較も作図できる。

```bash
python3 tools/plot_throughput_by_signal.py sim_results/my_lab_phy_smoke_runs
```

`signal_throughput/` にSNR・Eb/N0・RSSI別のPNG/PDF/CSVを保存する。ACKで確認したpayloadを各信号帯の滞在時間（待機を含む）で割った値であり、PHYレートとは異なる。現在のツールは試作PHYのDATA/ACK通番を必要とし、図題も1.1秒の短時間設定向け。MCS表の速度定義とRSSI参照面は未確定のため、適応変調の実測結果として扱わない。

## リポジトリ構成

| パス | 役割 |
|---|---|
| `src/comms_sim_pkg/src/`・`include/` | C++通信コア、統計AWGN、試作PHY、Gazeboプラグイン、リンクバジェット出力 |
| `src/comms_sim_pkg/launch/` | 設定・入力の保存、world生成、GazeboとROS bridgeの起動 |
| `src/comms_sim_pkg/config/` | 実行設定、共通アンテナCSV、MCS表、Fast DDS設定 |
| `src/comms_sim_pkg/models/`・`worlds/` | 同梱モデルとworld。モデルの出典は `models/drone/ATTRIBUTION.txt` |
| `src/comms_sim_pkg/test/` | 通信・電力計算・PHYのC++テストと照合用データ |
| `src/comms_sim_msgs/` | 旧通信品質メッセージ定義。現在のドローン実験では未使用 |
| `tools/` | ホスト設定・起動、4条件実行、結果検証、解析・作図 |
| `specification.md` | 実装仕様と集計定義 |
| `specification_review/` | 規格照合、設計案、試作PHYの実装説明、出典資料。各資料の実装状況に注意 |
| `RESUME_NOTES.txt` | ローカルの作業引き継ぎメモ（Git管理対象外）。古い未実装状態や変更前の条件も含む |
| `sim_results/` | 実験ごとの設定・CSV・図。Git管理対象外（管理用ファイルを除く） |
| `build/`・`install/`・`log/` | colconの生成物。Git管理対象外 |
| `Dockerfile`・`docker-compose*.yml`・`entrypoint.sh` | 実行環境とソースのマウント設定 |

実行条件は各結果の `config.yaml`、入力の由来は `provenance.json` で確認する。`specification_review/` の提案YAMLは実行設定ではない。資料を更新するときは、実装済みの内容を `specification.md` とREADMEへ反映し、検討中の案と区別する。

`.dockerignore` はビルド成果物・実験結果・取得資料などをDockerのビルドコンテキストから除外する。ファイルの削除や、実行時の `/workspace` マウントには影響しない。

`specification_review/references/` の取得PDFと抽出テキストはローカルキャッシュとしてGit管理対象外。出典URL・SHA256・使用箇所は [manifest.json](specification_review/references/manifest.json) に記録し、設計案・根拠の対応表とともに管理する。

## ブランチの使い分け

`main` は現在のドローン2機の通信実験を開発する基準とする。進行中の車両HO（ハンドオーバー）研究は別系統で保持する。

| ブランチ | 内容 |
|---|---|
| `feat/urban-2lane` | 都市道路・車両HO、評価・リプレイ・資料整備 |
| `feat/production-sim-two-phase` | 10台×3 RSUの二相評価 |
| `imp-var` | KKFによる予測HOと評価基盤 |

HO系統の変更はドローン版へ一括マージせず、共通で必要な修正を内容ごとに検討する。取り込み済みの一時ブランチは整理し、独自変更の残るブランチは保持する。
