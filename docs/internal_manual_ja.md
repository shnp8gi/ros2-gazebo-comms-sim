# ドローン通信シミュレータ 社内利用マニュアル

対象は、Ubuntu 22.04、ROS 2 Humble、Gazebo Harmonicで動作する、ドローン2機の通信シミュレータです。初めて利用する方が、ソースの取得、環境構築、実行、シナリオ変更、結果の検証と共有まで進めるための手順を示します。ROSやGazeboの事前知識は不要ですが、Ubuntuの端末でコマンドを実行する必要があります。

通常の利用はDockerで行います。ROSとGazeboはコンテナ内に入るため、ホストへの個別インストールは不要です。最初に受信電力閾値モデルで一通り操作し、その後、目的に合う通信モデルへ切り替えてください。

手順と設定例の確認基準は、2026年10月30日時点のコミット `504c99a` のドローン実験コードです。後続の機能追加を含む版を使う場合は、その版の設定項目・検証ツールも確認してください。

この手順書と `docs/examples/` を含むリポジトリ全体を共有してください。手順書だけでは、プログラム、アンテナCSV、機体モデルが不足します。対象はドローン実験のコードで、別ブランチの車両ハンドオーバー実験は対象外です。

## 目次

1. [できることと変更できる範囲](#1-できることと変更できる範囲)
2. [初回の環境構築](#2-初回の環境構築)
3. [最初の実行と成功確認](#3-最初の実行と成功確認)
4. [自分のシナリオを作る](#4-自分のシナリオを作る)
5. [設定項目と単位](#5-設定項目と単位)
6. [通信モデルを選ぶ](#6-通信モデルを選ぶ)
7. [条件をまとめて比較する](#7-条件をまとめて比較する)
8. [画面表示とログ再生](#8-画面表示とログ再生)
9. [結果を読む](#9-結果を読む)
10. [実験を保存して共有する](#10-実験を保存して共有する)
11. [困ったときの対処](#11-困ったときの対処)
12. [実装と詳細資料](#12-実装と詳細資料)

## 1 できることと変更できる範囲

2機が接近・すれ違い・離脱する間の、受信電力、接続、再送、タイムアウト、切断、転送量を計算します。機体は指定軌道上を一定速度で移動します。空力、操縦、風に対する飛行制御の再現は対象外です。

| 変更したいもの | 設定だけでできること |
|---|---|
| 経路 | `parallel` は平行な経路を逆方向に進むすれ違い、`cross` は直交する経路の交差 |
| 速度と配置 | A/Bそれぞれの正の速度、高度、平行経路の横間隔、最接近時刻 |
| アンテナ | 機体に固定する `fixed` と、相手を常時向く理想追従の `tracking`。固定時の仰角、指向性CSV |
| 通信 | 方向ごとの周波数、送信電力、利得・損失、雑音条件、通信モデル、タイマー、データ方向 |
| 記録 | 実行時間、計算刻み、記録間隔、出力先、対応モデルのモニター |

任意の経由点、曲線飛行、加減速、停止、3機以上、障害物による遮蔽、追従の遅れは、現在のYAMLでは指定できません。こうした経路・機能を追加する場合はC++などの実装変更と再検証が必要です。YAMLに新しいキーを書くだけでは機能は増えません。

### 機体と通信方向の呼び方

| 名前 | 役割・方向 |
|---|---|
| A / PRDEV | 低い高度の機体。接続を要求する端末 |
| B / PRC | 高い高度の機体。Beaconを送る調整局 |
| AB / Q / `Q_A_TO_B` | AからBへの通信。設定は `radio_ab`、受信側はB |
| BA / P / `P_B_TO_A` | BからAへの通信。設定は `radio_ba`、受信側はA |

サンプルは `protocol.data_from_prc: true` で、DATAはB→A、通常のDATA確認応答はA→Bです。Beacon、Request、Responseなども通る必要があるため、DATA方向の電力だけ高くても接続は成立するとは限りません。

## 2 初回の環境構築

### 2.1 用意するもの

- Ubuntu 22.04のx86_64 PC。現行DockerfileはVirtualGLのamd64パッケージを取得します。
- 初回導入時の管理者権限と、GitHub、Ubuntu、ROS、Gazebo、Dockerの配布元へ接続できるネットワーク。
- Docker EngineとDocker Compose plugin。コマンドは `docker compose` を使用します。
- ソース、Dockerイメージ、ビルド生成物、実験ログを保存する空き容量。長時間実験では `frames.csv` や `events.csv` が増えるため、実行前に `df -h .`、実行後に `du -sh sim_results/対象名` で確認します。
- GUIを使う場合はUbuntuのデスクトップ環境。画面なしの実行にGPUは不要です。

### 2.2 ソースを取得する

ホストの端末で実行します。共有されたアーカイブを使う場合は、展開したリポジトリのルートへ移動してください。以降のコマンドは、特記がなければすべてこのルートで実行します。

```bash
sudo apt-get update
sudo apt-get install -y git
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/shnp8gi/ros2-gazebo-comms-sim.git
cd ros2-gazebo-comms-sim
git rev-parse HEAD
```

既に同名ディレクトリがある場合は、その中身を確認して使用するか、別の作業先へ取得します。再現実験では、共有元から渡されたブランチ・コミットのコードを使ってください。取得権限が必要な場合は、管理者からリポジトリへのアクセス権またはソース一式を受け取ります。

### 2.3 Dockerを導入する

既に使える場合は導入を省略します。

```bash
docker version
docker compose version
docker info
```

未導入なら、管理者が同梱スクリプトの内容を確認して実行します。次の例は、ログイン中の利用者をDockerグループに追加します。

```bash
less tools/setup_docker_host.sh
sudo bash tools/setup_docker_host.sh "$(id -un)"
```

導入後はログアウトして再ログインし、`docker info` が成功することを確認します。Dockerグループはホスト上でroot相当の権限を持つため、社内の管理方針に従って付与してください。スクリプトは既存の競合パッケージを検出すると削除せず停止します。既存環境への影響を管理者が確認してから整理します。

GUIや解析用のPython依存関係をホストに入れる手順は第8章です。画面なしの実行・解析はコンテナだけで進められます。

### 2.4 パスの対応を理解する

| ホストで見える場所 | Docker内で見える場所 |
|---|---|
| リポジトリのルート | `/workspace` |
| `src/comms_sim_pkg/config/` | `/workspace/src/comms_sim_pkg/config/` |
| `docs/examples/threshold_custom.yaml` | `/workspace/docs/examples/threshold_custom.yaml` |
| `sim_results/first_run/` | `/workspace/sim_results/first_run/` |

Docker内の `config_file:=` には `/workspace/...` を指定します。ホストの `/home/利用者名/...` をそのまま渡すと、コンテナから見つけられません。独自のYAMLとCSVはリポジトリ内に置くと、この対応で参照できます。

## 3 最初の実行と成功確認

### 3.1 画面なしで実行する

まず受信電力閾値モデルの既存設定を使います。各機1 m/s、平行経路、固定アンテナ、シミュレーション時間50秒の例です。初回はDockerイメージの作成とプログラムのビルドも行います。

```bash
bash tools/start_headless.sh \
  config_file:=/workspace/src/comms_sim_pkg/config/scenarios/drone_threshold_parallel_1mps.yaml \
  output_dir:=/workspace/sim_results/first_run
```

`first_run` は初回用の名前です。2回目以降は `first_run_02` など、別の名前に変更します。`output_dir` を省略すると日時と識別子を含む新しい出力先が自動作成されます。単独実行は空でない出力先を拒否し、一括実行は既存の親ディレクトリ自体を拒否します。

シミュレーション時間と実際の待ち時間は別です。ビルドや計算の速さはPC・モデルに依存します。終了すると「結果保存先」が表示され、ホストの `sim_results/first_run/` に結果が残ります。`Ctrl+C` で中断した結果は完了結果として扱いません。

設定を指定せず `bash tools/start_headless.sh` とすると、別のモデルである `drone_params.yaml` の統計AWGN設定が選ばれます。この章と同じ条件にするには、上の `config_file` を指定してください。

### 3.2 正常終了を検証して図を作る

以下もホストの端末で実行します。PythonはDocker内で動くため、ホスト側にmatplotlibなどを導入する必要はありません。

```bash
test -f sim_results/first_run/completed && echo "完了マーカーあり"
docker compose -f docker-compose.yml run --rm --no-deps \
  --user "$(id -u):$(id -g)" -e HOME=/tmp -e DISPLAY= sim \
  python3 tools/validate_drone_results.py sim_results/first_run
docker compose -f docker-compose.yml run --rm --no-deps \
  --user "$(id -u):$(id -g)" -e HOME=/tmp -e DISPLAY= sim \
  python3 tools/analyze_drone_results.py sim_results/first_run
```

次の状態になれば、基本操作は完了です。

- `completed` が存在し、検証コマンドに「検証成功」と出る。
- `config.yaml`、`summary.csv`、`timeseries.csv`、`events.csv`、`gazebo_poses.csv` が存在する。
- 解析後に `connections.csv`、`timeouts.csv`、`disconnections.csv`、`timeline.png` などが作られる。

`timeline.png` をファイルマネージャーで開くと、受信電力、アンテナのずれ、累積転送量を確認できます。検証成功は位置・時刻・ログ集計の整合性を示します。実機通信の精度を保証する検証ではありません。

上記の設定・基準コードによる動作確認では、転送量は約1058.85 MB、接続1回、接続前タイムアウト7回、切断1回でした。設定やコードを変更した結果がこの数値に一致する必要はありません。

## 4 自分のシナリオを作る

### 4.1 サンプルをコピーして編集する

編集例として [threshold_custom.yaml](examples/threshold_custom.yaml) を同梱しています。両機18 km/h、高度20/28 m、横間隔6 m、最接近5秒、終了10秒です。既存設定を残すため、同じディレクトリにコピーします。

```bash
cp -n docs/examples/threshold_custom.yaml docs/examples/my_scenario.yaml
nano docs/examples/my_scenario.yaml
```

`nano` は `Ctrl+O` → Enterで保存、`Ctrl+X` で終了します。別のテキストエディターでも構いません。既に `my_scenario.yaml` がある場合、`cp -n` は上書きしないため、別名を使うか、既存ファイルを意図して編集してください。

まず次の項目を変更します。これはファイルの一部であり、無線・プロトコルなど残りの項目はサンプルから維持します。

```yaml
scenario: parallel
antenna_mode: fixed
drone_a:
  speed_kmh: 18.0
  altitude_m: 20.0
  fixed_elevation_deg: 53.1301023542
drone_b:
  speed_kmh: 18.0
  altitude_m: 28.0
  fixed_elevation_deg: -53.1301023542
lateral_separation_m: 6.0
encounter_time_s: 5.0
duration_s: 10.0
```

YAMLはスペースで字下げし、タブは使いません。キー名は変えず、`true` / `false`、数値、`null` を不要に引用符で囲まないでください。同じキーを重複して書くと意図した値が使われない原因になります。

### 4.2 初期位置と最接近距離を決める

初期位置を直接指定するキーはありません。速度、高度、最接近時刻から計算します。`vA = drone_a.speed_kmh / 3.6`、`vB = drone_b.speed_kmh / 3.6`、`te = encounter_time_s`、横間隔を `L` とすると、時刻 `t` の位置は次のとおりです。単位はmと秒です。

| 経路 | Aの位置 | Bの位置 | 最接近距離 |
|---|---|---|---|
| `parallel` | `(vA×(t−te), 0, hA)` | `(−vB×(t−te), L, hB)` | `sqrt(L² + (hB−hA)²)` |
| `cross` | `(vA×(t−te), 0, hA)` | `(0, vB×(t−te), hB)` | `hB−hA` |

サンプルの開始位置はAが `(-25, 0, 20)`、Bが `(25, 6, 28)`、最接近距離は10 mです。開始時点で機体を遠くしたければ、速度を維持して `encounter_time_s` を大きくし、通過後も観測できるよう `duration_s` も延ばします。速度だけ変更すると開始位置も変わります。

本書のサンプルは、`fixed` の既定の水平向きを使用します。平行時はAが+Y、Bが−Y、直交時はAが+Y、Bが+Xを向きます。平行時に最接近点へ正対させたい場合の仰角は、Aが `atan((hB−hA)/L)`、Bがその負値です。サンプルの約±53.13°はこの値です。`tracking` では相手へ瞬時に向くため、固定仰角の値は姿勢決定に使われません。

### 4.3 変更した条件で実行する

```bash
bash tools/start_headless.sh \
  config_file:=/workspace/docs/examples/my_scenario.yaml \
  output_dir:=/workspace/sim_results/my_scenario_01
```

第3.2節の検証・解析コマンドの出力先を `sim_results/my_scenario_01` に変えて実行します。入力YAMLと結果内の `config.yaml` を照合し、意図した速度・高度・モデルになっていることを確認してください。

経路、アンテナ方式、速度だけなら、ファイルを変更せず起動引数で上書きできます。

```bash
bash tools/start_headless.sh \
  config_file:=/workspace/docs/examples/my_scenario.yaml \
  scenario:=cross antenna_mode:=tracking \
  speed_a_kmh:=36 speed_b_kmh:=18 \
  output_dir:=/workspace/sim_results/my_cross_tracking_01
```

**起動引数がYAMLより優先されます。** 高度、横間隔、時間、送信電力などにはlaunch引数がありません。YAMLを編集します。ROSの一般的なパラメーター形式 `ros__parameters` に包む必要はありません。

### 4.4 CSVの参照先を保つ

YAML内の相対パスの基準は、そのYAMLが置かれたディレクトリです。サンプルでは以下のように参照します。

```yaml
antenna:
  e_plane: ../../src/comms_sim_pkg/config/antennas/e_plane.csv
  h_plane: ../../src/comms_sim_pkg/config/antennas/h_plane.csv
  max_attenuation_db: 30.0
mcs_table: ../../src/comms_sim_pkg/config/phy/MCStable.csv
```

`docs/examples/` 内で別名コピーする場合は変更不要です。他の場所へ移動した場合は、これらと `empirical_phy.table` を直します。独自CSVを同じディレクトリに置けば `e_plane: my_e_plane.csv` のように書けます。CSVを使わない受信モデルでも、共通ローダーがアンテナ表とMCS表を読み込むため、参照を削除しないでください。

## 5 設定項目と単位

### 5.1 経路と時間

| キー | 単位・設定条件 |
|---|---|
| `scenario` | `parallel` / `cross` |
| `antenna_mode` | `fixed` / `tracking` |
| `drone_a.speed_kmh`, `drone_b.speed_kmh` | km/h。正の値。1 m/s = 3.6 km/h。0や負の速度は不可 |
| `drone_a.altitude_m`, `drone_b.altitude_m` | m。正の値で、必ずA < B |
| `drone_a.fixed_elevation_deg`, `drone_b.fixed_elevation_deg` | 度。−90〜+90。上向きが正 |
| `lateral_separation_m` | m。正の値。`cross` の経路には使わないが、値の検査は行われる |
| `encounter_time_s` | 秒。通常は `0 <= encounter_time_s < duration_s`。0なら航路中央から離脱を開始 |
| `duration_s` | 秒。シミュレーションの終了時刻 |
| `simulation.physics_step_s` | 秒。Gazeboの刻みとチャネル更新間隔。小さくすると計算量が増える |
| `simulation.log_interval_s` | 秒。時系列の記録間隔。通信イベント記録とは別 |
| `simulation.real_time_factor` | 正の実時間倍率の目標。CPU処理が重い場合、この速度には達しない |

通過前だけの観測には `simulation.allow_pre_encounter_end: true` で `encounter_time_s >= duration_s` を許可できます。通常のすれ違い比較では省略または `false` にします。記録間隔と終了時刻は、計算刻みの整数倍にそろえると確認しやすくなります。

長いタイムアウトを調べる場合は、失敗後にタイマー満了まで進める実行時間が必要です。2 msの動作確認では500 msのタイムアウトを観測できません。

### 5.2 無線と伝搬

以下は `radio_ab` と `radio_ba` にそれぞれ指定します。AB/BAは送信方向であり、A/B各機体の全設定ではありません。

| キー | 意味・単位 |
|---|---|
| `receiver_model` | `equivalent_nf` はアンテナ端基準＋受信系の等価NF。`rf_chain` は従来のTJX入力端基準 |
| `frequency_hz` | 搬送周波数、Hz。60.48 GHzは `60.48e9` |
| `power_mw` | 送信電力、mW。dBmを直接入れない。`mW = 10^(dBm/10)`、−8 dBmは約0.15849 mW |
| `sensitivity_dbm` | 受信電力閾値、dBm。詳細HRCP・BER/PERモデルでは受信成否の直接の判定値ではない |
| `tx_antenna_gain_dbi`, `rx_antenna_gain_dbi` | 正面アンテナ利得、dBi。方向による変化はアンテナCSVで計算 |
| `tx_insertion_loss_db` | 送信機損失、dB。0以上。`power_mw` が送信アンテナ給電端の電力なら0。受信系NFとは独立 |
| `rx_insertion_loss_db` | `rf_chain` 専用の受信挿入損失、dB。`equivalent_nf` では省略または0 |
| `rx_lna_gain_db`, `rx_conversion_gain_db` | `rf_chain` 専用の受信回路利得、dB。`equivalent_nf` では省略または0 |
| `noise_temperature_k` | kTBの雑音温度、K。既定290 K。正の値で上書き可能 |
| `noise_bandwidth_hz` | 雑音帯域、Hz。AWGN有効時は正の値 |
| `receiver_noise_figure_db` | アンテナ端換算として扱う受信系の等価NF、dB。0以上 |
| `required_snr_db` | 旧モデルの追加SNR条件。使用しない場合は `null`。BER/PERモデルでは `null` 必須 |

新規設定は `receiver_model: equivalent_nf` を使います。送信電力→送信機損失→送信アンテナ利得→通信路→受信アンテナ利得の順に信号電力を計算し、熱雑音＋受信系全体のNFからSNRを求めます。NFは信号電力から引きません。
このモデルでは感度・MCS表・受信曲線もアンテナ端を基準にそろえます。閾値のみ・実測RSSI曲線のモデルにはNFによる追加の受信判定を入れず、SNRを使う理論PHY・AWGNモデルに反映します。
モデル名を省略した旧YAMLは、受信回路利得・損失キーがあれば `rf_chain` として従来の計算を維持します。キーがなければ `equivalent_nf` です。参照面の換算とログ列の定義は [無線計算の仕様](../specification.md#アンテナ端基準等価nfモデル) を参照してください。

`propagation.exponent` は距離減衰の指数で、2.0が自由空間を基準とする設定です。壁や地形を配置しても、それだけで通信計算に遮蔽が反映されるわけではありません。

`antenna.max_attenuation_db` は指向性減衰の打切りです。既定例の30.0を `null` にすると打切りを外せます。弱い方向の結果が大きく変わるため、比較ではこの値もそろえます。

### 5.3 接続とタイマー

| 項目 | 従来の閾値・統計AWGN・lab PHY | HRCP・BER/PER |
|---|---|---|
| DATAの方向 | `protocol.data_from_prc`。`true` はB→A、`false` はA→B | 同左 |
| 接続前の待ち | `protocol.response_timeout_ms`, `ack_timeout_ms` | `hrcp.setup_timeout_ms`。省略時・0時はATPに従う |
| 接続後の切断待ち | `protocol.inactivity_timeout_ms` | `hrcp.atp_ms` と要求側の `requested_atp_ms`。省略時は要求値も `atp_ms` |
| Beacon開始間隔 | `protocol.beacon_interval_ms` | 現行プロファイルは0.14 ms固定 |
| フレーム時間 | `control_frame_us`, `ifs_us`, `data_frame_us`。lab PHYには別の波形長計算もある | 詳細モデル内で計算。従来の時間キーを変えてもHRCP時間の変更にはならない |
| DATA長 | 閾値・統計AWGNでは時間とレートから算出 | `hrcp.data_bytes`、byte |

タイマーはフレーム交換時間より長くする必要があります。従来のタイマー・時間キーはHRCP設定でも共通検査で参照するため、削除せず元の設定を維持してください。HRCPのATPは両端で合意した値を用い、再送のたびに接続待ちを延長するモデルではありません。

### 5.4 起動引数の一覧

| 引数 | 値・用途 |
|---|---|
| `config_file:=...` | 入力YAML。省略時はインストールされた `drone_params.yaml` |
| `scenario:=...`, `antenna_mode:=...` | YAMLの経路・アンテナ方式を上書き |
| `speed_a_kmh:=...`, `speed_b_kmh:=...` | YAMLの速度を上書き |
| `output_dir:=...` | 結果保存先。省略時は自動命名 |
| `headless:=true/false` | 画面なし／Gazebo GUIあり |
| `bridge:=true/false` | ROSへの `/clock` と機体poseのブリッジ。既定true |
| `pause_at_encounter:=true/false` | 最接近時にGazeboを一時停止。画面なし実行ではfalseを使う |
| `keep_gui_open:=true/false` | 実験終了後もGazebo GUIを残す |

## 6 通信モデルを選ぶ

| モデル | 出発点の設定 | 受信判定・用途 |
|---|---|---|
| 受信電力閾値 | `drone_threshold_parallel_1mps.yaml` | 閾値との比較。経路や指向性による変化を確認しやすい。簡易MAC |
| 統計AWGN | `drone_params.yaml` | SNRからGray QPSK・符号化なしの理論BER/PERを計算し抽選。簡易MAC |
| BER/PER受信曲線 | `drone_statistical_parallel_1mps.yaml` | CSV曲線からフレーム成否を抽選。HRCPのMACを共用。波形・LDPCは計算しない |
| 詳細HRCP | `drone_hrcp_smoke.yaml` | 連続I/Q、同期、ヘッダー、LDPC等。2 msの短時間確認から始める |
| 試作I/Q PHY | `drone_lab_phy_smoke.yaml` | 非標準の試作PHY。既存比較・検証向け |

表のYAMLは `src/comms_sim_pkg/config/scenarios/` にあります。既定の `drone_params.yaml` だけは `config/` 直下です。アンテナのCSVとグラフは `config/antennas/`、MCS表・受信曲線・理論LUTは `config/phy/`、DDS設定は `config/dds/` にまとめています。モデルは名前だけで切り替わらず、複数の有効フラグと依存設定で決まります。用途に合う完成済みYAMLをコピーして変更し、参照パスはコピー先のYAMLからの相対パスに調整します。

### 6.1 BER/PERモデルを実行する

編集用の [statistical_custom.yaml](examples/statistical_custom.yaml) は第4章と同じ経路を持ちます。受信曲線は人工データの `receiver_curve_demo.csv` で、`calibration_status: synthetic_demo` として記録します。実機校正済みの評価には使用できません。

この例は接続後のATPが10,000 msで、受信しにくくなった後も再送が続きます。10秒の実行でも、動作確認時には約78万フレーム、結果ディレクトリ約627 MiBになりました。ログ容量を確保してから実行してください。ATPを短くする場合は、切断条件を変更した実験として記録します。

```bash
bash tools/start_headless.sh \
  config_file:=/workspace/docs/examples/statistical_custom.yaml \
  output_dir:=/workspace/sim_results/statistical_01
docker compose -f docker-compose.yml run --rm --no-deps \
  --user "$(id -u):$(id -g)" -e HOME=/tmp -e DISPLAY= sim \
  python3 tools/validate_drone_results.py sim_results/statistical_01
docker compose -f docker-compose.yml run --rm --no-deps \
  --user "$(id -u):$(id -g)" -e HOME=/tmp -e DISPLAY= sim \
  python3 tools/analyze_statistical_results.py sim_results/statistical_01
```

「確率判定検証成功」が出て、`probability_bins.csv` と `statistical_verification.yaml` が作られます。このモデルには **`analyze_statistical_results.py` を使用**します。現行の `analyze_drone_results.py` は `hrcp.enabled` を見て波形HRCP用の解析へ分岐するため、BER/PER結果には使用できません。同じ理由で `start_headless.sh --all` と `run_hrcp_monitor.py` の自動解析もBER/PERモデルには使わず、第7章の個別実行を使います。

次の組み合わせを維持します。

- `empirical_phy.enabled: true`、`hrcp.enabled: true`。
- `awgn.enabled: false`、`lab_phy.enabled: false`、`preview_mode: false`、`monitor_only: false`。
- `hrcp.expected_rssi_admission_enabled: false`、`hrcp.monitor_waveforms: false`。
- 両方向の `required_snr_db: null`。

受信抽選のseedは `empirical_phy.seed`、MAC側の乱数は `awgn.seed` です。再現時には両方を保存します。`rssi_offset_ab_db` / `rssi_offset_ba_db` は曲線参照電力への補正で、正の値は受信しやすい側へ動かします。測定から決めていない補正は実験上の仮定として残してください。

### 6.2 自分の受信曲線やアンテナを入れる

| CSV | 形式と注意 |
|---|---|
| アンテナE面・H面 | `角度[deg],利得[dB]` の2列。各2点以上、角度重複なし。既存CSVを参考に角度範囲をそろえる。値は補間され、正面利得の設定と組み合わせて使う |
| `MCStable.csv` | `受信電力[dBm],速度[Gbps]` の2列、ヘッダーなし。条件を満たす段階の速度を使う。詳細HRCPと統計AWGNのレート決定には使わない |
| BER/PER受信曲線 | ヘッダー `direction,frame,mcs,rssi_dbm,ber,per,reference_bytes`。確率は0〜1、電力は受信回路出力に対応するdBm |

受信曲線では方向を `P_B_TO_A` / `Q_A_TO_B`、フレームを `BEACON` / `REQUEST` / `RESPONSE` / `ACK` / `DATA` / `DATA_ACK` / `DISASSOCIATE`、または共通用の `CONTROL` / `ALL` で指定します。MCSの `*` は共通曲線を意図して使う場合だけ指定します。各組み合わせに異なる電力を2点以上用意し、使う方向・フレーム・MCSをカバーします。

`empirical_phy.metric: ber` は独立な残留ビット誤りから `PER = 1 − (1−BER)^(8×byte数)` を近似します。`metric: per` と `per_length_policy: exact` は測定時と評価時の長さ一致が必要です。`independent` は長さ補正の近似を明示的に選ぶ指定であり、校正の代わりにはなりません。MCS表を変えるときは `empirical_phy.mcs_by_rate_gbps` と曲線のMCS番号も対応させます。

TJXの測定CSVを変換する場合は [条件表テンプレート](../receiver_measurement_manifest_template.csv) と [詳細手順](../specification_review/statistical_model_usage_ja.txt) を使います。ホストにPython依存関係がある場合の例です。入力・出力パスは自分のものに置き換えます。

```bash
python3 tools/prepare_receiver_curve.py measurements/my_manifest.csv \
  measurements/my_receiver_curve.csv --metric per
```

TJX内部のRSSI数値をそのままdBmとして入力しないでください。`--relative-power` の出力は相対電力表であり、シミュレータへ直接入力できません。同梱の測定データには暫定条件と使用対象外の旧測定があるため、各測定ディレクトリのREADME・無効化記録を確認します。

### 6.3 詳細HRCPを実行する

```bash
bash tools/start_headless.sh \
  config_file:=/workspace/src/comms_sim_pkg/config/scenarios/drone_hrcp_smoke.yaml \
  output_dir:=/workspace/sim_results/hrcp_smoke_01
```

検証と解析には第3.2節の2コマンドを使い、出力先を変更します。HRCPでは解析結果に `endpoint_expiries.csv` と `hrcp_counts.csv` などが作られ、従来モデルの `timeline.png` は作られません。

詳細HRCPは3.52 GHzのサンプル時計で連続処理するため、短いシミュレーションでも時間がかかります。長時間の全経路評価は、短時間の所要時間とログ量を確認してから実行します。`awgn.enabled: false` にすると雑音のない別の条件になり、弱い信号でも復号できる場合があります。高速化のつもりで変更しないでください。

`hrcp.expected_rssi_admission_enabled` はBeacon受信後にRequestを許す条件です。`sensitivity_dbm` とは別の機構です。現行例はfalseですが、キー省略時はtrueなので、コピーした設定では明示値を維持します。詳細HRCPは研究実装で、完全な規格適合、実機との相互接続、CFO・位相雑音・マルチパス・Dopplerの評価を保証しません。

### 6.4 専用の統計モデル実行ツールを使う場合

第6.1節の共通起動だけで通常利用は可能です。既存の `tools/run_statistical_model.py` は別のビルド先 `install/no_expected_rssi` を使うため、利用する場合に限り、先に次を実行します。

```bash
docker compose -f docker-compose.yml run --rm --no-deps \
  --user "$(id -u):$(id -g)" -e HOME=/tmp -e DISPLAY= sim \
  bash -c 'source /opt/ros/humble/setup.bash && colcon --log-base log/statistical_build build --build-base build/no_expected_rssi --install-base install/no_expected_rssi --packages-select comms_sim_pkg --cmake-args -DBUILD_TESTING=ON --parallel-workers 1'
```

第8.2節のホストPython依存関係も導入した上で実行します。

```bash
python3 tools/run_statistical_model.py \
  --config docs/examples/statistical_custom.yaml --seed 12345 \
  --output sim_results/statistical_seed12345
```

このツールは専用の検証・解析と `replay.sh` の生成を行います。`--seed` は受信抽選のseedだけを上書きします。`--replay` を追加するとモニターも起動します。

## 7 条件をまとめて比較する

### 7.1 経路とアンテナの4条件

受信電力閾値、統計AWGN、lab PHY、波形HRCPは次の形式を使えます。詳細HRCPではまず短時間設定を指定します。

```bash
bash tools/start_headless.sh --all \
  --config docs/examples/threshold_custom.yaml \
  --output sim_results/custom_four_01
```

`parallel_fixed`、`parallel_tracking`、`cross_fixed`、`cross_tracking` を順番に実行し、検証・解析します。全条件が成功すると親ディレクトリに `comparison.csv` と `manifest.json`、各条件のログが残ります。速度を共通に上書きする場合は `--speed-a-kmh 36 --speed-b-kmh 18` を追加します。

添付設定の動作確認では、4条件とも検証・解析が成功し、`cross_fixed` の転送量は0 MBでした。固定アンテナの向きと接続条件による結果なので、0 MBだけを理由に実行失敗と判断しないでください。

単独実行の `config_file:=...` と、一括実行の `--config ...` は構文が異なります。`--all` は `start_headless.sh` の最初の引数に置きます。

### 7.2 BER/PERモデルの4条件

このモデルは個別実行と専用解析を組み合わせます。次はホストのBash端末で実行する一式です。`statistical_four_01` は未使用の名前にします。

```bash
(
  set -e
  for scenario in parallel cross; do
    for mode in fixed tracking; do
      run_dir="sim_results/statistical_four_01/${scenario}_${mode}"
      bash tools/start_headless.sh \
        config_file:=/workspace/docs/examples/statistical_custom.yaml \
        scenario:="$scenario" antenna_mode:="$mode" \
        output_dir:="/workspace/$run_dir"
      docker compose -f docker-compose.yml run --rm --no-deps \
        --user "$(id -u):$(id -g)" -e HOME=/tmp -e DISPLAY= sim \
        python3 tools/validate_drone_results.py "$run_dir"
      docker compose -f docker-compose.yml run --rm --no-deps \
        --user "$(id -u):$(id -g)" -e HOME=/tmp -e DISPLAY= sim \
        python3 tools/analyze_statistical_results.py "$run_dir"
    done
  done
)
```

このループでは比較表は自動生成しません。各条件の `summary.csv` と `probability_bins.csv` を比較します。受信確率の影響を調べる場合は、同じ経路・設定で `empirical_phy.seed` を変え、複数回の結果を残してください。

### 7.3 比較するときの条件

速度だけ、横間隔だけ、アンテナ方式だけなど、最初は一つの条件を変えます。比較表にはモデル、通信方向、入力CSV、実行時間、seed、打切り減衰値を併記してください。モデル間では転送量の加算基準も異なるため、数値だけを並べて性能差と解釈しないようにします。

## 8 画面表示とログ再生

### 8.1 Gazeboで飛行を見る

第3章でDockerイメージを作成した後、Ubuntuデスクトップの端末で実行します。

```bash
bash tools/start_gui.sh \
  config_file:=/workspace/docs/examples/threshold_custom.yaml \
  output_dir:=/workspace/sim_results/gui_01
```

このスクリプトはプログラムをビルドしますが、Dockerイメージは作成しません。GUIから始める場合は、先に `docker compose -f docker-compose.yml build sim` を実行します。

既定では最接近時に一時停止します。Gazeboの再生ボタンで再開してください。最後まで自動で進める場合は `pause_at_encounter:=false` を追加します。完了後も画面は開いたままなので、確認後に閉じます。途中で閉じると実行も終了します。

`DISPLAY` とXauthorityが必要です。通常のGUIスクリプトはソフトウェア描画を使用し、NVIDIA連携を必須にしません。`start.sh` は開発用コンテナを起動するだけで、シミュレーションは開始しません。

### 8.2 通信モニターを使う

通信モニターはホストのPythonで表示します。利用する場合だけ、管理者権限で依存関係を導入します。

```bash
sudo apt-get update
sudo apt-get install -y python3-yaml python3-numpy python3-pandas \
  python3-matplotlib python3-pyqt5 fonts-noto-cjk
python3 -c 'import yaml, numpy, pandas, matplotlib; from PyQt5 import QtWidgets; print("モニター依存関係OK")'
```

HRCPの短時間モニターは、通常ビルド `install/drone` が作られた後に次で起動します。

```bash
python3 tools/run_hrcp_monitor.py \
  --config src/comms_sim_pkg/config/scenarios/drone_hrcp_monitor.yaml \
  --scenario cross --antenna-mode tracking
```

独自設定でログ再生したい場合は、実行前に `hrcp.monitor_enabled: true` を設定します。BER/PERサンプルでは有効になっています。記録済みの結果は次で再生します。

```bash
python3 tools/monitor_hrcp_signals.py \
  sim_results/statistical_01/live_signals.json \
  --replay --autoplay --window full --details --playback-speed 2
```

再生には `live_signals.json` と対応する `monitor_replay.jsonl` などを同じ結果ディレクトリに残します。モニターの一時停止や再生速度は表示だけの操作で、計算済みの結果は変わりません。Gazeboの一時停止とは別です。

評価・比較用では `monitor_only: false` を維持してください。`monitor_only: true` は軽量なモニター保存用で、通常のCSV一式を作らず、共通検証・解析に必要なファイルが不足します。

## 9 結果を読む

### 9.1 ファイルを確認する順番

| ファイル | 確認すること |
|---|---|
| `completed` | 終了まで計算したか。ファイルがない結果は未完了 |
| `config.yaml` | 実際に適用した設定。起動引数で上書きした値、モデル名、制約も記録 |
| `summary.csv` | 接続回数、タイムアウト、切断、転送量の全体集計 |
| `timeseries.csv` | 時刻、機体位置、距離、受信電力、状態、累積転送量 |
| `events.csv` | 接続手順・送受信・タイマー満了などの原因追跡 |
| `frames.csv` | 対応モデルのフレーム成否、失敗理由、確率、MCSなど。閾値モデルでは生成されない |
| `gazebo_poses.csv` | Gazebo上の実際の機体・アンテナ位置と姿勢 |
| `inputs/`, `provenance.json` | 実行時にコピーしたCSVとそのSHA256、入力元の記録 |
| `world.sdf` | 実行用に生成したGazebo world |
| `comparison.csv` | 対応する4条件一括実行の比較表 |

### 9.2 主要な集計値

| 列 | 意味・読み方 |
|---|---|
| `connections` | 接続成立の回数。0でも正常終了はあり得る |
| `attempts` | 接続試行数。モデルの手順によるためBeacon送信数とは限らない |
| `timeouts` | 接続前の待ちタイムアウト。受信失敗の総数ではない |
| `disconnections` | 接続後の切断回数 |
| `first_connected_s` | 最初の接続成立時刻。負値は未接続を表す |
| `last_setup_ms` | 最後の接続確立時間。従来モデルは対象Beaconの送信から、HRCP系は初回Request送信から双方の接続確認まで |
| `connected_time_s` | 接続状態の累積時間 |
| `total_data_MB` | 累積payload量。1 MB = 1,000,000 byte |
| `acked_data_frames` | ACKを確認したDATA数。HRCP系の受信payload件数とは分けて扱う |
| `awgn_trials`, `awgn_errors` | モデル依存のフレーム判定・失敗数。BER/PERモデルでも互換列名として使う |

閾値・従来統計AWGN・lab PHYはACKを確認したpayloadを加算します。詳細HRCPとBER/PERは受信側で正常受信した一意のpayloadを加算し、重複再送を除外します。ACKがまだ返っていない末尾のDATAなどで、受信量とACK確認数が一致しないことがあります。

全実行時間での平均goodputをMbpsで求めるなら `total_data_MB × 8 / duration_s` です。接続中だけの平均を求める場合は分母を `connected_time_s` にしますが、0秒の場合は計算しません。どの時間で割った値かを必ず表記します。

`rssi_ab_dbm` / `rssi_ba_dbm` はリンク計算による信号電力で、受信回路の利得・損失を含む参照点です。詳細HRCPモニターの波形由来RSSIとは区別します。`ab_reachable` / `ba_reachable` と `margin_*` は旧閾値の参考列で、HRCPやBER/PERのフレーム成功を直接表しません。

### 9.3 期待どおりにつながらないとき

まず `config.yaml` の経路・速度・DATA方向を確認し、両方向の電力とアンテナのずれを見ます。次に `events.csv` でBeacon、Request、Response、確認応答のどこで止まるかを確認します。最後に対応モデルの `frames.csv` やタイムアウト解析を見て、受信失敗・半二重の重複・タイマー満了を分けます。

受信失敗時刻とタイムアウト確定時刻には待ち時間の差があります。また、終了時点で待ちが残っている結果の「タイムアウト0回」は、タイムアウトが起きない証明ではありません。従来解析の `pending_waits.csv` や詳細系のイベント・終了状態も確認します。

## 10 実験を保存して共有する

### 10.1 残すもの

入力YAML、独自CSV、結果ディレクトリ全体、実行コマンド、コードのコミット、実験目的を一緒に残します。`provenance.json` は入力CSVのハッシュを記録しますが、ソースのコミットやDockerイメージ全体を保存するものではありません。

正常終了した結果に、ホストで次を追記すると使用バージョンを残せます。対象パスは実際の結果に変更します。

```bash
git rev-parse HEAD > sim_results/my_scenario_01/source_commit.txt
git status --short > sim_results/my_scenario_01/source_status.txt
git diff --binary > sim_results/my_scenario_01/source_changes.patch
docker image inspect ros2-gazebo-comms-sim:humble-harmonic \
  --format '{{.Id}}' > sim_results/my_scenario_01/docker_image_id.txt
```

未追跡のファイルは `git diff` に含まれないため、独自YAMLやCSVは別途含めてください。共有メモには次を記入します。

```text
目的:
コードのコミットと未コミット変更:
入力YAMLと参照CSV:
実行コマンド:
通信モデルと校正状態:
乱数seed:
変更した条件と比較対象:
結果ディレクトリ:
検証結果:
平均goodputの計算に使った時間の定義:
```

`sim_results/`、`build/`、`install/`、`log/` のデータ本体はGit管理対象外です。Gitへのpushだけでは結果は相手に届きません。結果ディレクトリを社内ストレージへ置くか、アーカイブを別途渡してください。受信曲線や実測原本の共有範囲は社内の取り扱いに合わせます。

### 10.2 別PCで再実行する

受け取った側は同じコードと入力CSVを取得し、第2章の環境構築から進めます。結果の `config.yaml` はCSVを `/workspace/元の出力先/inputs/...` のような絶対パスで参照するため、別の場所へ移したものをそのまま入力にすると参照切れになる場合があります。

元の入力YAMLと同じ起動引数を使うか、保存済み `config.yaml` を新しいシナリオ用ディレクトリにコピーし、`antenna.e_plane`、`antenna.h_plane`、`mcs_table`、必要なら `empirical_phy.table` を同梱した入力CSVへ向け直します。再実行は必ず新しい `output_dir` を指定します。

同じseedだけでは再現条件はそろいません。コード、CSV、モデル、計算刻み、MAC側と受信抽選側のseedも合わせます。Dockerfileは配布元のパッケージを取得するため、厳密な環境保存が必要な実験では、社内の方法でDockerイメージ自体も保存してください。

## 11 困ったときの対処

| 症状 | 確認・対処 |
|---|---|
| `docker: command not found` | 第2.3節で管理者にDocker導入を依頼する |
| `docker compose` が使えない | Docker Compose pluginの導入を確認する |
| Dockerソケットの `permission denied` | Dockerグループとサービスを確認し再ログインする。加入済みなら一時的に `sg docker -c 'bash tools/start_headless.sh ...'` の形式も使える |
| `apt`、`curl`、`wget` の接続・証明書エラー | 初回イメージ作成時のネットワーク、プロキシ、時刻、社内CAを管理者と確認する |
| `出力先が空ではありません` / `FileExistsError` | 前回結果を残して出力名を変える。未完了の結果も上書きしない |
| `参照ファイルがありません` / CSVを開けない | YAML所在ディレクトリからの相対パスと、Docker内の `/workspace` を確認する |
| `Aを低い機体、Bを高い機体に設定` | `0 < altitude_A < altitude_B` にする |
| `すれ違い時刻は実行時間の内側` | 通常は `0 <= encounter_time_s < duration_s` にする |
| 無効なモデルの組み合わせ | 単独のフラグだけで変更せず、第6章の対応YAMLからコピーする |
| `Package 'comms_sim_pkg' not found` | `start_headless.sh` でビルドする。手動コンテナでは `/opt/ros/humble/setup.bash` と `install/drone/setup.bash` をsourceする |
| 以前のPCのビルドパスがエラーに出る | `build/`・`install/`をコピーして流用せず、ソースを別ディレクトリへ取得してビルドする |
| ログやビルド生成物の書込み権限がない | 以前rootで作成した可能性を確認。通常スクリプトは利用者UID/GIDで実行する。管理者が対象ディレクトリを限定して所有権を修正する |
| GUIが出ない／Xauthorityがない | デスクトップ端末の `echo "$DISPLAY"`、`echo "${XAUTHORITY:-$HOME/.Xauthority}"` を確認。画面なし手順で計算できるか切り分ける |
| 最接近で止まったまま | GUIの再生ボタンを押すか、`pause_at_encounter:=false` で再実行する |
| モニターで `No module named PyQt5` 等 | 第8.2節のホストPython依存関係を導入する |
| BER/PER解析で `異なるPHYの結果が混在` | `analyze_statistical_results.py` を使う。共通解析と `--all` はこのモデルに使用しない |
| `run_statistical_model.py` がビルドを要求 | この専用ツールは `install/no_expected_rssi` が必要。第6.4節を使うか、第6.1節の共通起動で実行する |
| `summary.csv` などがない | `completed` と `monitor_only` を確認。中断・例外・ディスク不足のログも確認する |
| `--all` が `TimeoutExpired` で止まる | 各条件の実時間上限は `max(120秒, duration_s×30)`。重いHRCPでは短時間設定か単独実行へ切り替える |
| 完了したが転送量が0 | 正常な通信不成立の可能性がある。両方向の電力、指向性、受信曲線、実行区間を第9.3節で確認する |

中断は起動した端末で `Ctrl+C` を使います。残ったコンテナは `docker ps` で名前と実験の対応を確認してから、その実験のものだけ停止します。共有PCで他の利用者のコンテナを一括停止しないでください。結果やビルドディレクトリを削除する前に、保存要否と対象を確認します。

## 12 実装と詳細資料

| 調べる内容 | 入口 |
|---|---|
| 全体の概要 | [README](../README.md) |
| 基本モデル・ログ仕様 | [specification.md](../specification.md) |
| 起動引数・設定保存・CSVコピー | [sim_launch.py](../src/comms_sim_pkg/launch/sim_launch.py) |
| 設定の読込み・検査・経路・通信計算 | [drone_link.cpp](../src/comms_sim_pkg/src/drone_link.cpp) |
| Gazeboへの機体配置と完了処理 | [DroneEncounterPlugin.cc](../src/comms_sim_pkg/src/DroneEncounterPlugin.cc) |
| 標準設定 | [config](../src/comms_sim_pkg/config/) |
| BER/PERモデルと測定データの取り込み | [statistical_model_usage_ja.txt](../specification_review/statistical_model_usage_ja.txt) |
| 詳細HRCPの実装範囲と制約 | [runtime_hrcp_ja.txt](../specification_review/runtime_hrcp_ja.txt) |
| 試作I/Q PHY | [runtime_lab_phy_ja.txt](../specification_review/runtime_lab_phy_ja.txt) |
| 実行・検証・解析・キャンペーン用ツール | [tools](../tools/) |

`specification_review/` には検討案と過去時点の説明もあります。既存実験を確認するときは、その結果の `config.yaml` と使用コミットを基準にしてください。モデル別の説明にある過去の実行例・固定UID・保存先を、別PCへそのまま流用しないでください。
