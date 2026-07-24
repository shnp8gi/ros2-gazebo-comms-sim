# 本番シミュレーション実装仕様 (imp-var ブラッシュアップ版)

作成: 2026-07-23 / 対象: ros2-gazebo-comms-sim (imp-var ブランチ)
元文書: 簡易検証で確定した「本番シミュレーション構築 実装仕様」
関連: [kkf_v2i_system_design.md](kkf_v2i_system_design.md) (フェーズA〜D完了済みの現行アーキテクチャ)

---

## 0. 位置づけと設計原則

元仕様(簡易検証の確定設計)を、**現行シミュレータに組み込むための実装仕様**に再構成した。
数値はすべて簡易検証由来の初期値であり、YAML パラメータ化する(【実測】項目は実測後に差替え)。

設計原則(単一責任・保守性・汎用性の汚染防止):

1. **既存の抽象の上に実装する**。新しい振る舞いは既存インターフェース
   (`IShadowingModel`, `IRateModel`(新設), predictor 差し替え, `cases:`/`entity_groups:`,
   評価ディレクトリ契約)の**新実装クラス**として追加し、既存クラスの改変は最小にする。
2. **既存機能は削除せず設定で無効化する**(フェージング、遮蔽トラッカー、Viterbi プランナ、
   優先度DP)。road_multicar 論文の再現性と回帰テスト資産を保持するため。
3. **生成は Python 前処理、実行は既存ランタイム**。ランダム交通・パラメトリック幾何は
   sweep の config 生成段階で決定論的に展開し、C++ プラグイン/scenario_loader は不変とする。
4. **各新規モジュールは責務1つ**。「何をするか」と同時に「何をしないか」を明記する。

---

## 1. 差分サマリ (元仕様 → 現状 → 作業)

| # | 元仕様の項目 | 現状 | 作業区分 |
|---|---|---|---|
| 1 | RSU/車のアンテナ配置 (ハの字15°、任意設定) | 既存機構で表現可 (pose / relative_rpy / offset) | **設定のみ** + 生成ツール |
| 2 | 双方向指向性 (両端ゲイン積、実測パターン) | **実装済み** (`get_tx_rx_gains` が tx_total+rx_total を適用、e/h plane CSV) | **検証のみ** |
| 3 | 車高依存の動的遮蔽 (視線幾何判定) | **実装済み** (`BinaryObbBlockageModel` の3D線分×OBB交差が視線高さ判定を内包) | **検証のみ** + 車種カタログ |
| 3' | 遮蔽交通流 (複数車線・車種構成・run毎ランダム) | なし (blocker は静的定義+enabled切替のみ) | **新規** (traffic 生成) |
| 3b | 持続シャドウ (走行間固定) | Gudmundson AR(1) が **run毎シードで引き直される** = 仕様違反 | **新規** (`FrozenFieldShadowingModel`) |
| — | マルチパス/フェージング無し | Rician 実装あり | **設定のみ** (`enabled: false`) |
| — | 雑音床 + Shannon レート写像 | MCS テーブル直付け (`CommsCalculator`) | **新規** (`IRateModel` 分離) |
| — | 送信電力 −7dBm / T_est 2ms | パラメータ既存 (`tx_power`, `link_establishment_time_ms`) | **設定のみ** |
| 4 | RBF 基底 (20個) | ConstantBasis / LogDistanceBasis のみ | **新規** (`RbfBasis`) |
| 4' | σ_ν²(s) 自己組織化 (S2/W 割引累積) | なし (σ_ν は固定スカラ) | **新規** (`AleatoricVarianceMap`) |
| 5 | (α,P,S2,W) 走行間永続化+忘却 | なし (毎走行まっさら) | **新規** (`RemStore`) |
| 6 | 遮蔽トラッカー(第2層)無効化 | `kkf_tracker_enabled` で切替可 | **設定のみ** |
| — | ハンガリアン法マッチング (μ−κσ) | 逐次優先度DP + Viterbi (車毎) | **新規** (`HungarianAssigner`) |
| 7 | arm: trend / kf / kkf_cold / kkf_conv / oracle | lut(ff) / a3 / ts_kf / kkf は既存 | **新規** (trend, oracle) + **変更** (kf のフォールバック) |
| 8 | 二相評価 (学習相→CRN評価相) | 単相 sweep のみ。CRN・Wilcoxon・min_car/Jain は実装済み | **新規** (学習相の直列実行+状態受け渡し) |
| 9 | 掃引 (遮蔽密度・κ・学習走行数) | cases 記法で表現可 | **設定のみ** (traffic 密度は新パラメータ) |
| 10 | 指向性ゲイン LUT 化等の高速化 | predict_batch 等は実施済み | **必要時のみ** (P6) |

結論: **元仕様の§2(指向性)・§3(視線幾何)の大半は実装済み**。本番構築の実体は
(a) 走行間固定シャドウ、(b) run毎ランダム交通の生成、(c) REM の永続化+偶然的分散、
(d) L3 マッチング、(e) 二相評価パイプライン、の5点に絞られる。

---

## 2. 全体責務マップ

```
[シナリオ・交通生成 (Python, tools/lib)]        ← 新規はここと kkf_core に集中
  scenario_gen.py   : パラメトリック幾何 (RSU列・車列) → scenario yaml 断片の生成のみ
  traffic_gen.py    : traffic: 定義 + run seed → blocker エンティティ列への決定論展開のみ

[データプレーン (C++ プラグイン)]               ← 変更は新実装クラス2つのみ
  channel/ShadowingModel.hpp : + FrozenFieldShadowingModel (走行間固定場のサンプルのみ)
  RateModel.hpp (新設)       : IRateModel = RSSI→レート写像のみ (CommsCalculator から抽出)
  それ以外 (指向性・OBB遮蔽・T_est・BS排他・レポートPub・ExternalScheduleStrategy) は不変

[制御プレーン数理 (Python, kkf_core)]           ← ROS/gz 非依存・numpy のみ・単体テスト必須
  basis.py         : + RbfBasis (基底評価のみ)
  variance_map.py  : AleatoricVarianceMap = σ_ν²(s) の保持・更新・照会のみ (新規)
  rem_store.py     : RemStore = (α,P,S2,W,meta) の保存・復元・忘却適用のみ (新規)
  matching.py      : HungarianAssigner = 効用行列→P2P割当のみ (新規)
  kkf.py / planner.py / blockage_tracker.py : 不変 (ゴールデンテスト維持)

[制御プレーンI/O (kkf_scheduler_node.py)]       ← 差し替え点の追加のみ、数式を持たない
  make_predictor  : + TrendPredictor / OraclePredictor、ts_kf の trend フォールバック
  MpcScheduler    : assigner 切替 (hungarian | priority_dp)、RemStore の load/save 呼出

[評価パイプライン (tools/sim.py)]
  learn サブコマンド (新規) : 学習相 = 直列実行 + REM状態の run間受け渡し + スナップショット
  run/analyze/... : 不変 (指標・CRN・Wilcoxon は実装済み)
```

---

## 3. シーン幾何とシナリオ生成

### 3.1 幾何 (すべて YAML、初期値は元仕様)

- RSU: 3台、s方向60m間隔 (例 x = −60, 0, +60)、y = **8.0**、アンテナ高 2.5m【実測】。
  yaw = −π/2 + δ_rsu (δ_rsu = **+15°** = 下流側へ振る。任意設定)。
- 車: **10台**、y = 0、等速 16.67 m/s、車間 18m。**単一アンテナ**
  (offset `[0, 0, 1.35]`【実測】、relative_rpy yaw = +δ_veh、δ_veh = **+15°** = RSU側へ振る)。
  ※ 現行機構は複数アンテナ対応のまま。仕様上1本にするのは YAML の antennas を1エントリに
  するだけであり、コード変更はない。
- 遮蔽車線: y = 2.5 (並走渋滞・低速) / 4.5・6.0 (対向)【実測: 対象道路の車線構成】。

視線幾何の検算 (RSU h=2.5, 車載 h=1.35, y_R=8 の直線補間。OBB交差が自動でこれを実現):

| 遮蔽車線 y | 視線高さ [m] | 乗用車1.5m | バス3.2m | トラック3.8m |
|---|---|---|---|---|
| 2.5 | 1.71 | 遮蔽しない | 遮蔽 | 遮蔽 |
| 4.5 | 2.00 | 遮蔽しない | 遮蔽 | 遮蔽 |
| 6.0 | 2.21 | 遮蔽しない | 遮蔽 | 遮蔽 |

→ 元仕様どおり「大型車のみ遮蔽」が幾何から自然に出る。**専用の視線判定コードは書かない**
(元仕様§3の1D射影式は簡易検証の近似であり、本番は既存の3D OBB交差の方が正確)。

### 3.2 シナリオ生成 (新規: `tools/lib/scenario_gen.py`)

- 責務: 幾何パラメータ (RSU台数/間隔/y/δ_rsu、車台数/車間/速度/δ_veh/アンテナ高) から
  entities リストを決定論生成することのみ。物理・通信の知識を持たない。
- 出力はチェックインされる scenario yaml (`config/scenarios/road_10car.yaml`)。
  生成スクリプトは再生成の道具であり、**真実は生成された yaml** (評価ディレクトリ契約と整合)。
- 既存 road_5car.yaml は回帰資産としてそのまま残す。

### 3.3 遮蔽交通流 (新規: `tools/lib/traffic_gen.py`)

- scenario yaml に `traffic:` セクションを新設:

```yaml
traffic:
  lanes:
    - { y: 2.5, direction: +1, speed_mps: 5.0,  headway_s: {dist: lognormal, mean: 4.0, sigma: 0.8} }
    - { y: 4.5, direction: -1, speed_mps: 16.7, headway_s: {dist: exponential, mean: 6.0} }
    - { y: 6.0, direction: -1, speed_mps: 16.7, headway_s: {dist: exponential, mean: 8.0} }
  vehicle_mix:            # 都市部相当の初期値【実測: 交通実態】
    - { model: SedanBlocker, ratio: 0.75 }   # 車高1.5m / 8dB (視線下=遮蔽しない)
    - { model: TruckBlocker, ratio: 0.15 }   # 3.8m / 26dB
    - { model: BusBlocker,   ratio: 0.10 }   # 3.2m / 22dB
  window_s: [0, 60]       # この時間窓に RSU 区間を通過する車のみ生成 (エンティティ数の抑制)
  tx_entry_jitter_s: 0.0  # 対象車の進入タイミングジッタ (元仕様の「要検討」→ 掃引変数化)
```

- 展開規則: `traffic_gen.expand(scenario, traffic_seed)` が上記を blocker エンティティ列
  (pose + waypoints + model) に**決定論展開**する。呼び出し点は sweep_sim の config 生成直後
  (`inject_run_seeds` と同じ場所)。traffic_seed = run seed 由来 → **run毎に異なる交通、
  run_idx のみに依存するので方式間で同一実現 (CRN 成立)**。
- 責務外: 展開後のエンティティを動かすのは既存 `WaypointMoverPlugin`、遮蔽計算は既存
  `BlockageEnvironment`。ランタイム変更なし。
- モデルは全車種 `models://GhostBox` (collision なし) + `model_catalog.blockage` 属性。
  **SimpleBox (collision あり) を混雑交通に使うと車線間・車間の形状重なりが接触ソルバに
  載り RTF が 1/30 以下に落ちる** (2026-07-24 実測)。遮蔽 OBB は blockage 属性由来なので
  collision の有無は通信計算に影響しない。SUV メッシュは論外。
- 掃引つまみ: `headway_s.mean` (密度)、`vehicle_mix` の大型車比率。遮蔽発生率 数%〜17% を
  カバーするプリセットを sweep cases に置く。

---

## 4. チャネルモデル

合成式は現行のまま: `RSSI = TxPower + G_tx + G_rx − PL(d) − L_blockage − X_shadow (− L_fading)`

### 4.1 成分別の扱い

| 成分 | 実装 | 作業 |
|---|---|---|
| 距離減衰 | 既存 `LogDistancePathLossModel` (60GHz, d0=1m 自由空間基準)。大気吸収は exponent に畳込み (初期値 n=2.0、【実測】で校正) | 設定のみ |
| 双方向指向性 | 既存。両端の e/h plane 実測パターン (ピーク22dBi) をオフボアサイト角で適用済み | 検証のみ (§4.4) |
| 動的遮蔽 | 既存 `BinaryObbBlockageModel` + §3.3 の交通流 | 車種カタログ追加 |
| 持続シャドウ | **新規 `FrozenFieldShadowingModel`** (§4.2) | 新規 |
| フェージング | `fading.enabled: false` (コードは保持) | 設定のみ |
| 測定雑音 | 既存 `measurement_report.noise_std_db` = 1〜2dB (run毎シード維持) | 設定のみ |
| 雑音床・レート | **新規 `IRateModel`** (§4.3) | 新規 |

送信電力 −7dBm / 60GHz / BW 100MHz。リンクバジェット目安 (両端ボアサイト時):
d=10m → RSSI≈−51dBm (SNR 44dB)、d=60m → ≈−67dBm (SNR 28dB, ≈0.93Gbps)。
H面は−27dBまで急峻なので、実効的な通信ウィンドウは指向性が決める (これが狙い)。
トラック遮蔽26dBでも近距離では SNR>0 が残る = **NLOS は「切断」でなく「レート劣化」**。
CONNECTED 閾値の設定 (§4.3) が結果を左右するため明示的に管理する。

### 4.2 FrozenFieldShadowingModel (新規, `channel/ShadowingModel.hpp` に追加)

- 責務: **環境シードから決定論生成された固定空間場をサンプルすることのみ**。
  時間・走行回数・リンク履歴に依存しない (Gudmundson との本質的差分)。
- 場の構成: RSU (rxアンテナ) ごとに独立な1次元場 X_j(u)。u = 車両アンテナ位置の道路軸への
  射影 (直線道路では world x。射影軸は config)。格子生成 (間隔 = 相関長/4) + AR(1) 色付け
  + 線形補間。σ = 4dB、相関長 6m【実測バリオグラムで校正】。
- 場のキー: 呼び出しシグネチャを `SampleDb(link_id, tx_pos, rx_pos)` に拡張し、
  **静的な rx アンテナ位置 (量子化) をキー**に場を保持する。link_id からの bs 逆算
  (num_bs の配管) を避け、モデルを構成情報から独立させる。Gudmundson は rx_pos を無視
  (既存動作不変)。
- **シード規約 (重要)**: 生成シードは `channel.shadowing.environment_seed` **のみ**。
  `channel.seed` (run毎に `inject_run_seeds` が上書き) は参照しない。
  → 学習相・評価相の全 run で同一環境が自動的に保たれる。
- 2次元 (s,d) 拡張・複数環境の掃引は environment_seed の変更のみで表現できる (Future Work)。

```yaml
channel:
  shadowing: { enabled: true, type: frozen,          # 既定 "gudmundson" = 後方互換
               sigma_db: 4.0, corr_length_m: 6.0,
               environment_seed: 1, grid_m: 1.5, axis: x }
```

### 4.3 IRateModel (新規, `include/comms_sim_pkg/RateModel.hpp`)

- 責務: **RSSI [dBm] → 到達可能レート [Mbps] の写像のみ**。`CommsCalculator` から
  MCS テーブル参照を抽出し、注入式にする。
  - `McsTableRateModel`: 既存テーブル互換 (**既定** = 旧設定は挙動不変)。
  - `ShannonRateModel`: `rate = η·BW·log2(1 + SNR)`、SNR = RSSI − noise_floor。
    noise_floor = **−95dBm**【実測校正】、BW = 100MHz、η = 1.0 (実装効率、初期値)。
- CONNECTED 判定閾値は rate model が提供する: Shannon では
  `rssi_min = noise_floor + snr_min_db` (snr_min_db 初期値 0.0)。MCS ではテーブル先頭 (現行)。
- **単位規約**: レートは Gbps (プラグインのデータ会計 `MB += rate·1000/8·dt` と
  既存 MCS テーブル値が Gbps 前提のため。Mbps を返すと 1000 倍の水増しになる)。
- T_est = 2ms は既存 `link_establishment_time_ms` (既定値がそのまま 2.0)。
  スロット幅 20ms の「有効時間から T_est 減算」は、現行の連続時間実装
  (確立完了までスループット 0) が等価以上の精度で満たす。**陽なスロット化はしない**。

```yaml
comms_simulator_node:
  ros__parameters:
    tx_power: -7.0
    rate_model: { type: shannon, bandwidth_hz: 100.0e6, noise_floor_dbm: -95.0,
                  efficiency: 1.0, snr_min_db: 0.0 }
```

### 4.4 検証 (実装済み項目の確認テスト)

- 指向性: 静的配置3点 (対向前・正対・通過後) の RSSI を解析値と突合 (両端ゲイン積の回帰)。
- 遮蔽: §3.1 の表どおり「乗用車が遮蔽しない/トラックが遮蔽する」ことを detailed_logs
  (`link_los`, `blockage_loss_dB`) で確認。
- 固定シャドウ: 同一 environment_seed の 2 run で `shadow_dB` が位置の関数として一致、
  異なる run_idx でも一致 (交通だけが変わる) ことを確認。

---

## 5. KKF-REM 第1層の拡張 (kkf_core)

### 5.1 RbfBasis (basis.py に追加)

- φ_i(s) = exp(−(s−c_i)²/2w²)。中心 c_i は道路区間に等間隔 20個 (数・幅・区間は config)。
- 責務: 基底評価のみ。既存 `KrigedKalmanFilter` にそのまま注入できる
  (kkf.py は改変しない = ゴールデンテスト維持)。
- 位置づけ: **まっさら開始の kkf_cold/kkf_conv は事前地図 (PriorProfile) を使わない**。
  RBF 20基底が指向性ピークの非単調構造を自力学習できることが前提
  (LogDistanceBasis 単独では表現不能で大敗した教訓への回答)。
  基底解像度は学習相の収束確認で検証し、不足なら数を増やす (config のみ)。

### 5.2 AleatoricVarianceMap (variance_map.py, 新規)

- 責務: **位置依存の偶然的分散 σ_ν²(s) の保持・更新・照会のみ**。KKF の状態には触れない。
- データ: 格子上の十分統計量 (S2, W)。更新は KKF 更新後残差 r(s) を受けて
  `S2[g] += k(s,g)·r², W[g] += k(s,g)` (k はガウスカーネル、幅 = 格子間隔程度)。
  照会は `σ_ν²(s) = S2/W` (W < w_min の格子は事前値 σ_ν0² を返す)。
- **単純 EMA は使わない** (確率的遮蔽イベントを忘れるため)。割引は run 境界でのみ
  `S2·=γ, W·=γ` (γ = 0.997、§5.3 の carryover 時に適用)。
- 接続規約 (二重計上の整理):
  - Kalman ゲイン・観測ノイズには**使わない**。観測ノイズは固定 R_meas のまま
    (平均場学習を殺さない)。
  - kkf.py の残差クリギングの σ_ν (固定・持続成分の事前分散) はそのまま。
  - 予測分散への加算のみ: `var_total = var_kkf + σ_ν²(s)`。合成は `KkfMapPredictor`
    (ノード側) の責務であり、kkf.py と variance_map.py は互いを知らない。
- 遮蔽トラッカー (第2層) はこの分散マップが代替する: `kkf_tracker_enabled: false` を
  本番既定とする (コード削除はしない)。

### 5.3 RemStore (rem_store.py, 新規)

- 責務: **REM 状態の直列化・復元・忘却適用のみ**。学習も予測もしない。
- 保存単位: RSU ごとに `(α, P, S2, W, meta)`。meta = {basis 設定ハッシュ, bs_id,
  累積走行数, environment_seed}。形式 npz。basis ハッシュ不一致のロードはエラー
  (黙って壊れた状態で走らせない)。
- 忘却は **load 時に適用**: `P += q_forget·I` (q_forget = 0.05)、`S2·=γ, W·=γ` (γ = 0.997)。
- 置き場所は評価ディレクトリ契約に従う: `sim_results/<name>/rem_state/bs<j>.npz`、
  学習曲線用スナップショットは `rem_state_snapshots/after_run_<m>/bs<j>.npz`
  (rem_state/ の中に入れると再帰コピーになるため兄弟ディレクトリ)。
- ノード側 I/O: `kkf_state_dir` (指定時 load)、`kkf_state_save: true` (保存)。
  保存タイミングは「最終レポートから一定無音」検知時+定期上書き (5s 毎)。
  ノード終了時の gz-transport segfault (既知) に保存を依存させない。
- これで「学習プロセスと本番実行プロセスの分離」が成る: 学習相が状態を作り、
  評価相は load して開始する (評価相中の継続更新は既定 ON、`kkf_freeze: true` で凍結可)。

---

## 6. L3: リスク調整効用のマッチング

### 6.1 HungarianAssigner (matching.py, 新規)

- 責務: **効用行列 U (車×BS) → P2P 排他割当のみ** (`scipy.optimize.linear_sum_assignment`)。
  効用の作り方・ヒステリシス・時系列は呼び出し側の責務。
- 割当禁止 (CONNECTED 不能ペア等) は U = −∞ 相当で表現。min(m,n) 対を割当て、
  余剰の車は自然に idle (現行 MASKED_LCB 起因の飢餓構造が原理的に消える)。

### 6.2 スケジューラ統合 (`kkf_assigner: "hungarian" | "priority_dp"`)

- 効用: `U[i][j] = μ_ij − κ·σ_ij` (predictor の lcb と同一。κ 初期値 **1.0**、本番で再掃引)。
- 時系列の扱い (段階実装):
  1. **第1段 (本命)**: 各再計画 (0.2s) で k=0 の行列のみ Hungarian。ping-pong 抑制は
     現割当ペアへの `switch_bonus_db` (既存 switch_cost の効用版) で行う。
  2. **拡張 (必要時)**: ステージ k=0..K を順に Hungarian し、前段割当と異なるペアに
     切替コストを課す貪欲時系列 (horizon 4s / replan 0.2s の MPC 型)。
- 既存の逐次優先度DP + Viterbi は `priority_dp` として保持 (road_5car 回帰用)。
- 公平性の注記: μ−κσ には明示的な公平項がない (簡易検証では κ≈1 が公平性最大)。
  本番で飢餓が再発した場合の保険として、効用変換フック (α-fair 重み `w_i·U[i][j]`) を
  拡張点として設計に含めるが、初期実装はしない。

---

## 7. 比較手法 (arm) と情報制約

predictor 差し替え式 (`make_predictor`) に統一。**arm ごとに観測できる情報を表で固定**し、
sweep cases が該当設定を強制する (揃っていない比較を構成できなくする)。

| arm | control_plane | 予測器 | 観測 (レポート) | 事前知識 | 状態継承 |
|---|---|---|---|---|---|
| trend (下界) | `trend` (新規) | TrendPredictor: 決定論プロファイル参照のみ、σ=0 | 不要 | 決定論成分 (距離+両端指向性) | なし |
| kf | `ts_kf` (既存+変更) | ScalarRssiKF/ペア。**未観測対は trend 値へフォールバック** (現行の無情報事前を置換) | grant 対のみ | trend と同一 | なし |
| kkf_cold | `kkf_mpc` | KkfMapPredictor (RBF, prior なし, σ_ν² マップ) | grant 対のみ | **なし** (まっさら) | なし |
| kkf_conv (本命) | `kkf_mpc` + `kkf_state_dir` | 同上 | grant 対のみ | なし | **学習相の (α,P,S2,W)** |
| oracle (上界) | `oracle` (新規) | OraclePredictor: 全対の最新真値、σ=0 | **全対・雑音0** (`observe_all_pairs: true, noise_std_db: 0`) | — | — |
| (参考) lut | 既存 feedforward_optimal | プラグイン内静的LUT | — | 環境既知 (静的) | — |

- trend の情報量の定義 (本文書で確定): 「距離+公称ボアサイトゲイン」ではなく
  **決定論成分の完全知識** (既存 `export_rssi_profile` の決定論プロファイル = 距離減衰+
  両端実測パターン、シャドウ・遮蔽を含まない) とする。既存機構で実装でき、
  下界としてより強い trend に勝つ方が主張も強くなる。
- 事前地図ハイブリッド (現行 kkf_full = prior+偏差学習) は kkf_cold/conv からは**外す**
  (「まっさら→自己組織化」の物語と矛盾するため)。ただし arm `kkf_prior` として温存し、
  必要なら第6の腕として報告する (設定のみで構成可)。
- 4段分解: trend → kf → kkf_cold → kkf_conv → oracle で「空間予測の価値 / 収束の価値 /
  残存損失」を切り分ける (元仕様§5どおり)。

---

## 8. 二相評価パイプライン

### 8.1 学習相 (`tools/sim.py learn`, 新規サブコマンド)

```bash
python3 tools/sim.py learn --scenario config/scenarios/road_10car.yaml \
    --name road_10car_learn --runs 100 --snapshot-every 10
```

- 内容: kkf arm 単独で N_learn = 80〜120 run を **1 run ずつ直列実行**し、共有
  `rem_state/` を介して状態を受け渡す (run k の保存 → run k+1 の load)。
  base_seed を run ごとに +stride するため、シード列は単一スイープの run 列と同一。
- スナップショット: m 走行ごとに `rem_state_snapshots/after_run_<m>/` を保存。
  → **学習走行数の掃引は、評価相でスナップショットを選ぶだけで済む** (再学習不要)。
- 収束確認: σ_ν²(s) のコントラスト比・平均場係数の run 間変化 (RMS)・trace(P) を
  learn が learning_curve.csv に出力する (飽和 = 収束)。
- シード: 環境 (シャドウ) は environment_seed で固定。交通は run_idx 由来で毎回変わる
  (§3.3)。これが元仕様の学習相の定義そのもの。失敗時は --resume で継続可能。

### 8.2 評価相 (既存 `sim.py run` + 設定)

- 全 arm を同一 sweep で実行。CRN は既存機構 (run_idx のみ依存のシード導出) で成立。
- kkf_conv の `kkf_state_dir` は学習相ディレクトリの rem_state を指す。manifest に
  学習元ディレクトリ・スナップショット名を記録する (再現性)。
- **評価相の run_idx レンジは学習相とずらす** (base_seed を変える) — 学習で見た交通実現を
  評価に再利用しない。
- 試行数 n≈100 (n=30 不足の教訓)。標準誤差併記。**max_concurrency ≤ 4 厳守**
  (閉ループ方式は CPU 競合で崩壊する既知教訓)。
- 指標・統計は実装済み: min_car / Jain (主指標)、総量、NLOS 暴露率、真のHO回数、
  CRN+ペア差分+Wilcoxon+95%CI。

### 8.3 掃引

sweep の `cases:` に `scenario:` キーを新設した (生成前のシナリオ辞書への
ドットパス上書き。traffic 展開より前に適用される)。密度掃引はこれで書く:

```yaml
- name: density
  cases:
    high:
      scenario:
        traffic.vehicle_mix[1].ratio: 0.30
        traffic.lanes[0].headway_s.mean: 2.5
```

| つまみ | 変数 | 備考 |
|---|---|---|
| 遮蔽密度 | `traffic.lanes[].headway_s.mean`, `vehicle_mix` 大型車比率 | 遮蔽発生率 数%〜17% のプリセット数段階。評価に遮蔽ゼロ条件を含めない |
| κ | `kkf_kappa` | 総量×公平性トレードオフ曲線。κ≥2 崩壊の再確認 |
| 学習走行数 | スナップショット選択 | 学習相1回から構成 |
| (保留) シャドウ環境 | `environment_seed` | 単一環境固定で進め、一般性主張が要るときのみ複数化 (元仕様の保留を踏襲) |

---

## 9. 実装フェーズ計画

| フェーズ | 内容 | 元仕様チェックリスト対応 | 完了条件 (検証) |
|---|---|---|---|
| **P1 チャネル** | FrozenFieldShadowingModel / IRateModel 分離 / fading off / tx −7dBm 設定 | 3b, (2,3 は検証のみ) | §4.4 の3検証。既存シナリオが旧設定で挙動不変 (後方互換) |
| **P2 シナリオ/交通** | scenario_gen + traffic_gen + sweep 統合 (run毎展開)、road_10car.yaml | 1, 3', 9の一部 | 車種比・遮蔽発生率がレンジ通り / 同一 run_idx で方式間の blocker 列一致 (CRN) / RTF 維持 |
| **P3 kkf_core** | RbfBasis / AleatoricVarianceMap / RemStore / HungarianAssigner + 単体テスト | 4, 5, 一部6 | tools/tests/ に各単体テスト。既存ゴールデンテスト無傷 |
| **P4 ノード統合** | assigner 切替 / trend・oracle predictor / kf フォールバック / state I/O | 6, 7 | 各 arm 1run スモーク (閉ループ動作・情報制約どおりのレポート内容) |
| **P5 二相評価** | sim.py learn / スナップショット / 学習曲線 / 本番掃引 | 8, 9 | 学習相で σ_ν² コントラスト飽和を確認 → n≈100 評価 |
| **P6 高速化** | comms計算の間引き (`comms_update_period_s`)。必要ならゲインLUT | 10 | 学習相 100 run が実用時間内 |

**P6 実施済み (2026-07-24)**: プロファイルの結果、RTF ボトルネックは collision でも
交通台数でもなく **物理ステップ (1kHz) ごとの全リンク チャネル評価** (指向性補間 +
遮蔽OBB、Eigen Vector3d 演算) だった。`comms_update_period_s` (既定0=毎ステップ=
後方互換) で comms tick を間引き、実効dt (前回comms更新からの経過sim時間) で
データ会計・リンク確立の総量を保つ。road_10car は 0.005s (200Hz、report 0.05s の
1/10なので観測レポート不変)。**RTF 0.048 → 0.22 (約4.5倍)**。回帰テスト
(pipeline_regression) は間引きOFF既定で不変。GhostBox (collision除去) は交通重なりの
保険として残すが RTF への寄与は小さい。さらなる高速化 (ゲインLUT) は必要時のみ。

依存: P1・P2 は独立に進められる。P3 は独立。P4 は P1〜P3 に依存。P5 は P4 に依存。

---

## 10. 本文書で確定した設計判断と残論点

確定 (元仕様からの具体化・変更):

1. 視線遮蔽は既存 3D OBB 交差で実現し、1D 射影式は実装しない (§3.1)。
2. 交通・幾何のランダム生成は sweep の config 生成段階で決定論展開する。ランタイム不変 (§3.3)。
3. 持続シャドウは rx 位置キーの1次元固定場、シードは environment_seed のみ (§4.2)。
4. レート写像は IRateModel に分離、Shannon は snr_min_db で CONNECTED 閾値を定義 (§4.3)。
5. スロット 20ms は現行連続時間実装で満たし、陽なスロット化はしない (§4.3)。
6. σ_ν²(s) は KKF 本体・観測ノイズに触れず、予測分散への加算のみ。合成はノード側 (§5.2)。
7. 忘却は load 時適用。basis 設定ハッシュ不一致 load はエラー (§5.3)。
8. Hungarian は k=0 + 現割当ボーナスから始め、必要時にステージ貪欲へ拡張 (§6.2)。
9. trend の情報量 = 決定論プロファイル完全知識 (公称ゲインより強い下界) (§7)。
10. kkf_cold/conv は prior なし。現行ハイブリッドは kkf_prior arm として温存 (§7)。
11. トラッカー・Viterbi・優先度DP・Gudmundson・Rician は削除せず設定で無効化 (§0)。

残論点【要決定 / 実装中に判断】:

- RBF 20 基底で指向性ウィンドウ (H面急峻) の平均場を表現しきれるか — 学習相の残差で判定、
  不足なら基底数を増やす (config のみで対応可)。
- 交通到着分布の形 (指数 vs 対数正規) と車線構成【実測】。
- Hungarian 第1段 (k=0) で切替粒度が粗くなる場合の MPC 拡張 (§6.2 の2段目) の要否。
- 公平性が μ−κσ だけで出るか (出ない場合の α-fair フックは設計済み・実装保留)。
- 学習相 100 run の壁時計 (10台+交通で RTF 低下懸念 → P6 の発動判断)。
