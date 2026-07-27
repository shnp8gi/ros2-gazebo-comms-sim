# 都市部2車線シナリオ 実装計画

作成: 2026-07-27 / 対象ブランチ: `feat/urban-2lane` (`feat/production-sim-two-phase` から分岐)
仕様: [urban_2lane_scenario_spec.md](urban_2lane_scenario_spec.md)

## 0. 全体像と方針

| | 内容 |
|---|---|
| **新規 C++** | **2 箇所のみ** (自己遮蔽除外・コリドーゲーティング)。いずれも既存クラスへの引数追加 |
| **新規 Python (生成)** | `gen_road_scenario.py` の幾何一般化、`traffic_gen.py` の対象車生成 |
| **新規 Python (集計)** | **すべて既存 `detailed_logs` の後処理**。データプレーンの変更は不要 |
| **既存のまま使うもの** | `greedy_fcfs` / `oracle_inst` / `feedforward_optimal` / BsOccupancyRegistry / FrozenField / Shannon レートモデル / 評価ディレクトリ契約 |

方針は本番仕様 §0 を踏襲する: **既存の抽象の上に実装し、既存機能は削除せず設定で無効化する**。
後方互換 (road_10car / road_5car / pipeline_regression) を全フェーズで維持する。

---

## P0. 準備 (0.5 h)

1. 作業ツリーの未コミット差分を整理してコミットする。
   - `TxControllerPlugin.cc` の `greedy_fcfs` + `BsOccupancyRegistry::Available` → **仕様 §4.1 の下界 arm 本体**。
   - `tools/sim.py` (learn 関連)、`tools/sweep_progress.py` (状態ファイル修正) → 別コミット。
   - `src/comms_sim_pkg/models/SimpleBox/` が未追跡のままなら**必ず含める** (過去に消えてスイープ全滅した経緯あり)。
2. `feat/urban-2lane` を切る。

**完了条件**: `git status` がクリーン。既存 `road_10car` の 1 run が通る。

---

## P1. 幾何・シナリオ生成 (A1, A2) — **完了 (2026-07-27)**

> **計画からの逸脱**: 当初は `gen_road_scenario.py` を改修する計画だったが、
> **新規 `tools/gen_urban_scenario.py` を追加し、既存生成器は変更しない**方針に変えた。
> 理由: `road_10car.yaml` のヘッダは `gen_road_scenario.py --out ...` を再生成コマンドとして
> 明記しており、引数や既定値を変えると**既存本番シナリオの再現性契約が壊れる**。
> また両者は構造が違う (road_10car は対象車を固定生成、urban は生成しない) ため、
> 1 つの生成器に排他的な 2 モードを持たせるより分けた方が単純。
> 共通の幾何計算は **`tools/lib/road_geometry.py` に純関数として抽出**した。

### 成果物

| ファイル | 内容 |
|---|---|
| `tools/lib/road_geometry.py` | 道路断面・アンテナ角・相互ボアサイト・視線高さの純関数 (stdlib のみ) |
| `tools/gen_urban_scenario.py` | シナリオ yaml の生成 (対象車は生成せず traffic に委ねる) |
| `config/scenarios/road_urban_2lane.yaml` | 生成物 (真実はこちら) |
| `tools/tests/road_geometry_test.py` | 41 件、全緑 |

### 実装中に判明したこと

**傾きの符号は走行方向の担当を決めていた** (仕様 §1.3 に追記済み)。相互ボアサイト条件を
両端で解くと、`−δ` (上流へ振る) の RSU は**下り車**と、`+δ` (下流へ振る) は**上り車**と
しか対向が成立しない。「2 台ずつ上り側と下り側に傾ける」という設置方針は
**上り・下りそれぞれに 2 台を割り当てること**と等価であり、空間ダイバーシチではない。

**仕様書の視線高さ 1 箇所が誤っていた** (y=3.0 で 2.06 → 正しくは 2.05)。
乗用車の余裕も、大型車の車幅 (2.5 m) の端で評価していたため 0.18 m としていたが、
乗用車自身の車幅 (1.8 m) の端で評価するのが正しく **0.24 m**。いずれも修正済み。

<details>
<summary>当初計画 (参考)</summary>

### 変更: `tools/gen_road_scenario.py`

道路断面から積み上げる方式に変える。追加するヘルパ (いずれも純関数、テスト対象):

```python
def lane_layout(lanes_near, lanes_far, lane_width):
    """車線中心の y 座標。近車線 = RSU 側 (+y, direction +1)、奥車線 = 対向 (−y, −1)"""
    near = [(k + 0.5) * lane_width for k in range(lanes_near)]
    far  = [-(k + 0.5) * lane_width for k in range(lanes_far)]
    return near, far

def rsu_y_from_section(lanes_near, lane_width, shoulder_m, sidewalk_m):
    """歩道の外側端 = 車線帯 + 路肩 + 歩道 (仕様 §1.1)"""
    return lanes_near * lane_width + shoulder_m + sidewalk_m

def rsu_tilts(num_rsu, tilt_deg, pattern):
    """2台ずつ上流/下流へ振る。alternate = [−δ,+δ,−δ,+δ] / grouped = [−δ,−δ,+δ,+δ]"""

def veh_relative_yaw(direction, veh_tilt_deg):
    """車体座標での相対 yaw。world +y (RSU側) へ傾けるため direction で符号反転 (仕様 §1.4)"""
    return math.radians(veh_tilt_deg if direction > 0 else -veh_tilt_deg)
```

新しい引数 (既定値は仕様の確定値):

| 引数 | 既定 | 備考 |
|---|---|---|
| `--lanes-near` / `--lanes-far` | 1 / 1 | 将来 2 / 2 |
| `--lane-width` | 3.5 | |
| `--shoulder-m` / `--sidewalk-m` | 0.5 / 2.0 | |
| `--rsu-y` | `auto` | `auto` = 上式。車線数掃引時は固定値を明示指定 (仕様 §1.1) |
| `--rsu-h` | 2.5 | **【実測】確認待ち。3.5〜4.0 推奨 (仕様 §1.2)** |
| `--num-rsu` / `--rsu-spacing` | 4 / 10.0 | |
| `--rsu-tilt-deg` / `--rsu-tilt-pattern` | 45.0 / `alternate` | |
| `--veh-tilt-deg` | `auto` | `auto` = `90 − rsu_tilt` (相互ボアサイト条件) |
| `--corridor-margin-m` | 40.0 | RSU 列の外側の助走・退出 |

**削除する責務**: 対象車 (`car_1..car_N`) の固定生成をやめる。対象車は P2 で `traffic_gen` が
交通流から生成する (仕様 §2.1)。`--num-cars` / `--car-gap` は廃止。

`model_catalog` に **ミニバンクラスを追加** (仕様 §1.2 の (b) 案を config だけで選べるようにする):

```yaml
SedanBlocker:   { size: [4.5, 1.8, 1.5], loss_db:  8.0 }   # 低車高乗用車
MinivanBlocker: { size: [4.8, 1.8, 1.9], loss_db: 12.0 }   # ミニバン/SUV。既定 ratio 0
TruckBlocker:   { size: [12.0, 2.5, 3.8], loss_db: 26.0 }
BusBlocker:     { size: [11.0, 2.5, 3.2], loss_db: 22.0 }
```

出力: `config/scenarios/road_urban_2lane.yaml`。

### 新規テスト: `tools/tests/gen_road_scenario_test.py`

| # | 検証 |
|---|---|
| 1 | 1+1 で車線 y = ±1.75、RSU y = 6.0。2+2 で ±1.75/±5.25、RSU y = 9.5 |
| 2 | RSU x = −15,−5,+5,+15、tilt が `alternate` で `[−45,+45,−45,+45]` |
| 3 | `--veh-tilt-deg auto` で `rsu_tilt + veh_tilt == 90` |
| 4 | **相互ボアサイト対向の検算**: δ=45° のとき Δx = L·tan δ の位置で、RSU→車 と 車→RSU の視線ベクトルが双方のボアサイトと一致する (数値で角度差 < 1e-6) |
| 5 | `--rsu-y 8.0` の明示指定が `auto` を上書きする |

**完了条件**: 上記が緑。生成 yaml を `scenario_loader` が読めて sim_launch が起動する (エンティティ 0 台でも可)。

</details>

---

## P2. 交通生成 — 対象車の混在 (A3, A5) — 6 h

### 変更: `tools/lib/traffic_gen.py`

責務は「`traffic:` 定義 + seed → エンティティ列への決定論展開のみ」を維持する。
**幾何の知識を持たせない**ため、対象車のアンテナ諸元は生成側がテンプレートとして書き込む:

```yaml
traffic:
  target_ratio: 0.30
  prefill: auto                 # auto = (コリドー長 + margin) / speed
  target_template:              # gen_road_scenario が書く。traffic_gen は引き写すだけ
    model: Car
    antenna_offset: [0.0, 0.0, 1.35]
    relative_yaw_by_direction: { "1": 0.785398, "-1": -0.785398 }
  vehicle_mix:
    - { model: SedanBlocker,   ratio: 0.80, can_be_target: true  }
    - { model: MinivanBlocker, ratio: 0.00, can_be_target: true  }
    - { model: TruckBlocker,   ratio: 0.15, can_be_target: false }
    - { model: BusBlocker,     ratio: 0.05, can_be_target: false }
  lanes:
    - { y:  1.75, direction:  1, speed_mps: 16.7, headway_s: {...} }
    - { y: -1.75, direction: -1, speed_mps: 16.7, headway_s: {...} }
```

展開手順 (既存の `expand_traffic` を作り替える):

```
1. 車線ごとに到着時刻列を引く。開始を t = −prefill とする  ← 新規
2. 1台ごとに (車種, is_target) を引く
     車種は vehicle_mix から。is_target は can_be_target: true の車のみ対象に、
     全交通に占める割合が target_ratio になるよう条件付き確率で判定
3. 全車線の車を **到着時刻 t でソート**してから entities に積む  ← 新規 (A5)
4. is_target → role: tx + antennas + waypoints、それ以外 → role: blocker
```

#### `prefill` (新規) — 長い warmup を避けるための負の到着時刻

現行は `t ≥ 0` から生成するため **t=0 のコリドーは空**で、定常状態に達するまで
`コリドー長 / v` かかる。`load: over` (5 m/s) では 24 s を超え、60 s 窓の 4 割が無駄になる。

負の到着時刻を許すと、既存の配置式 `x_start = c0 − v·t` がそのまま
「t=0 時点でコリドー内にいる車」を表現する。`prefill: auto` は
`(corridor長 + margin) / speed` とし、**コリドー出口を通り過ぎる車は生成しない**ようクランプする。

→ `warmup_s` は「制御プレーンの立ち上がり」ぶんの短い値 (5 s) で足りるようになる。

### 変更: `tools/lib/scenario_loader.py`

`role: tx` でも `model_catalog[...].blockage` があれば `blockers` にも登録する (仕様 §7 B1)。
現状は `role == 'blocker'` の分岐内でしか登録しないため、**遮蔽登録を role 判定から独立させる**。

> **なぜ必要か**: 対象車 (乗用車 1.5 m) は RSU への視線 (1.68〜2.06 m) の下を通るので
> 横断的には遮蔽しない。しかし **同一車線で前方近傍にいる車**は、視線が車線と平行に近い
> 区間 (Δx が小さい領域) で交差する。対象車が奥車線の 30〜60% を占めるため、
> これを登録しないと**同一車線の遮蔽が系統的に欠落して楽観側に偏る**。
> 特に `load: over` (渋滞・車間が詰まる) で効く。

### テスト: `tools/tests/traffic_gen_test.py` (既存を拡張)

| # | 検証 |
|---|---|
| 1 | 同一 seed で完全に同一のエンティティ列 (CRN) |
| 2 | 車種構成が `vehicle_mix` の比率に収束する (大数の検定、n=5000) |
| 3 | 対象車比率が `target_ratio` に収束し、`can_be_target: false` の車種が tx にならない |
| 4 | **エンティティ列が到着時刻で単調** (A5 のバイアス除去) |
| 5 | 上り車線 (direction −1) の yaw = π、waypoint が −x 方向、relative yaw が負 |
| 6 | `prefill` で t=0 にコリドー内に車が存在する。コリドー出口を越える車は生成されない |
| 7 | 生成された tx が `scenario_loader` で `vehicles` と `blockers` の**両方**に載る |

**完了条件**: 上記が緑。1 run 起動して Gazebo 上で両車線に車が流れる (GUI で目視 1 回)。

---

## P3. 遮蔽の正しさ (B2) — 2 h

### 変更: `BlockageEnvironment::Refresh` に除外名を追加

P2 で対象車が遮蔽体になると、**自車の OBB が自分のリンクを必ず遮る**。
最小の変更で済ませるため、障害物リストを組む段階で自分を落とす:

```cpp
// BlockageEnvironment.hpp
void Refresh(gz::sim::EntityComponentManager& _ecm,
             const std::string& exclude_name = "")   // 既定 "" = 従来通り
{
    this->obstacles.clear();
    for (auto& entry : this->entries) {
        if (!exclude_name.empty() && entry.box.name == exclude_name) continue;
        ...
    }
}
```

```cpp
// TxControllerPlugin.cc:722
this->blockage_env.Refresh(_ecm, this->model_name);
```

**この方式を選ぶ理由**: `IBlockageModel::Evaluate` / `ChannelModel::Evaluate` の
シグネチャを変えずに済み (両者は「リンクの所有者」を知るべきでない)、
毎 tick のコピーも発生しない。除外は「障害物リストの構築」の責務として自然。

### テスト

- `tools/tests/channel_ext_test.cc` に追加: 自分の OBB を含む障害物リストで
  `exclude_name` を与えると LOS が保たれ、与えないと NLOS になる。
- 実機 1 run: `detailed_logs` の `link_los` が、対象車自身の位置と無関係に振る舞う
  (自己遮蔽していれば常時 NLOS になるので一目で分かる)。

**完了条件**: C++ テスト緑。1 run の NLOS 率が 100% でない。

---

## P4. 閾値と性能 (§3.1, B3) — 3 h

### 4-1. 接続閾値 (設定のみ)

`rate_model.snr_min_db: 26.5` → `rssi_min = −68.5 dBm` (MCS 表下限と等価、仕様 §3.1)。

- テスト `F7`: `ShannonRateModel::MinRssiDbm()` が −68.5 を返す (既存 `channel_ext_test.cc` に追加)。

### 4-2. コリドーゲーティング (B3)

`CommsEnvironment::CalculateMetrics` に `link_eval_radius_m` を追加し、
距離が半径を超えるペアは**指向性補間もチャネル評価もせず** `best_rssi = −999` で埋める。

```cpp
// 既定 0.0 = 無効 = 後方互換
if (eval_radius_m > 0.0 &&
    (ant_pos_world - bs_antenna_pos).norm() > eval_radius_m) {
    bs_metrics[bs_idx] = AntennaMetrics{};   // best_rssi = −999.0 (既定値)
    continue;
}
```

**必要性の根拠 (仕様 §2.3 の台数見積りより)**:

| 条件 | コリドー内対象車 | 全対象車 (60s窓) | リンク評価数/tick |
|---|---|---|---|
| 現行 road_10car | 10 | 10 | 30 |
| `load: under` | ≈ 0.4 | ≈ 7 | 28 → **ゲーティング不要** |
| `load: over` | ≈ 5.8 | ≈ 29 | 116 → **ゲーティング必須** |

`over` は待機中の車も含めて全 tick で 4 RSU 分を評価するため約 4 倍。RTF が 0.22 → 0.06 まで
落ちると 1 run 17 分・全体 34 時間になる。半径 40 m のゲートで実効 7 台程度に落ちる。

- 注意: `observe_all_pairs: true` (oracle_inst) では、ゲートされた対のレポートが
  −999 になる。**ゲート半径は「その距離では閾値を割っている」ことが保証される値**に
  取ること (閾値 −68.5 dBm ⇒ ボアサイトでも 65 m 相当なので 40 m は安全側)。

### スモーク

`load: over` で 1 run 実行し、**RTF・壁時計・対象車パス数**を実測する。
ここで §6 の `window_s` / `N` を確定する。

**完了条件**: `over` 1 run が 10 分以内。`pipeline_regression_test` が緑 (ゲート既定 OFF)。

---

## P5. 指標と★判断ポイント (E1, E2, E5) — 5 h

**すべて `detailed_logs` の後処理で実装できる** (C++ 変更なし)。使う列:

`vehicle_name, time_s, tx_x_m, tx_y_m, has_link_grant, rssi_dBm, bs_x_m, link_state, link_los, total_data_MB`

### 変更: `tools/lib/eval_metrics.py`

| 指標 | 導出 |
|---|---|
| **パス単位配信量** | `tx_x_m` がコリドー区間に入ってから出るまでを 1 パスとし、その間の `total_data_MB` の増分。`warmup_s` 以前に開始したパスは除外 |
| **車線ラベル** | `tx_y_m` の符号 (`near` / `far`)。**記述用。利得の主張には使わない** (仕様 §1.3 の交絡) |
| **RSU 占有時間分布** | `has_link_grant == True` の行を `bs_x_m` でグループ化し、(車, RSU) の連続区間長を集める |
| **飢餓率** | `has_link_grant == False` かつ `rssi_dBm ≥ rssi_min` の時間 / コリドー内滞在時間。**未接続時の `rssi_dBm` は全 RSU の最良値が入る** (TxControllerPlugin:780 のフォールバック) ので、そのまま「圏内なのに繋がっていない」を意味する |
| **HO / 再アソシ** | `has_link_grant` が真の区間で `bs_x_m` が変化 = **能動的切替**。偽を挟んで変化 = **再アソシ**。**全 arm で同一定義**になる |
| **PF 効用** | パス単位配信量 `R_i` に対する `Σ log(R_i)` (R_i = 0 のパスの扱いは下記) |

> **`ho_count` を events から取るのをやめる理由**: 既存の `handover_count` は
> 方式ごとに意味が違う (feedforward は常に 0、external_schedule はプローブ切替も数える)。
> `bs_x_m` の変化から導けば **全 arm で同じ定義**になり、比較が成立する。

> **PF の 0 除算**: 1 パスの配信量が 0 の車が出る (飢餓)。`log(0) = −∞` を避けるため
> `log(R_i + ε)`、ε = 1 MB とし、**ε の値を報告に明記する**。あわせて
> **配信量 0 のパス比率**を独立した指標として出す (こちらの方が飢餓の説明として直接的)。

### ★ 判断ポイント

`greedy_fcfs` を `load` 3 条件で N=5 だけ回し、次を確認する:

1. **RSU 占有時間分布に長い尾があるか** (1 台が長時間 1 RSU を握っているか)
2. **飢餓率が `over` で有意に立つか** (圏内なのに繋がれない時間が存在するか)
3. `under` で 1・2 がほぼ 0 になるか (対照として機能するか)

**ここで独占が観測できなければ先へ進まない。** 調整するつまみは順に
`load` (§2.3) → 接続閾値 (§3.1) → RSU 間隔 (§8-1)。

---

## P6. 掃引と本番 (E3, E4) — 3 h + 実行 9 h

- `config/sweep/urban_2lane_eval.yaml`: arm 4 本 (`greedy_fcfs` / `kkf_conv` / `oracle_inst` / `ff_lut`)
  × `load` 3 条件 × N。`analysis:` に新指標を登録。
- `execution.max_concurrency: 4` **厳守** (閉ループ方式が CPU 競合で崩壊する既知教訓)。
- 逐次停止 (E3): N=15 で回して主要指標の 95% CI 幅を見てから N=40 まで積む。
- `kkf_conv` を入れるなら、**新シナリオで学習相をやり直す** (`sim.py learn`)。
  REM 状態は幾何・環境に固有なので road_10car の状態は使えない。

---

## 工数と依存

```
P0 準備      0.5h  ─┐
P1 幾何      4h    ─┼─→ P2 交通 6h ─→ P3 遮蔽 2h ─→ P4 閾値/性能 3h ─→ P5 指標 5h ★ ─→ P6 掃引 3h
                    ┘                                                              └→ 実行 9h
```

実装 ≈ **23.5 h**、本番実行 ≈ **9 h**。P1 と P3 (C++) は独立に着手できる。

## 各フェーズの後方互換チェック (毎回走らせる)

```bash
python3 tools/tests/pipeline_regression_test.py
```

```bash
python3 tools/sim.py run --sweep-config scratch/pipeline_smoke_sweep.yaml
```

## 未決定のまま進める項目 (仕様 §8)

| 項目 | 進め方 |
|---|---|
| RSU 高 2.5 m vs 3.5〜4.0 m | **パラメータ化して既定 2.5 のまま進める**。【実測】の根拠が判明した時点で既定を変える。P5 の判断ポイントで遮蔽発生率が低すぎたらここも疑う |
| 乗用車の車高クラス分割 | `MinivanBlocker` を **ratio 0 でカタログに置く**。yaml の比率変更だけで (b) 案に移れる |
| RSU 間隔 10 m | 生成引数のままにし、P5 の結果を見て掃引に加えるか決める |
