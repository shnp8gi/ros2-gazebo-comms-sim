# KKF予測ハンドオーバー × 拡張チャネルモデル 統合設計書 (v2)

作成: 2026-07-07 / 対象: ros2-gazebo-comms-sim (Gazebo Harmonic 8.11 + ROS 2 Humble)

---

## 1. 目的

1. KKF 3層システム(電波環境地図・遮蔽トラッカー・予測スケジューラ)を、制御プレーン/データプレーン分離型でシミュレータに統合する。
2. チャネルモデルを「距離減衰+白色ガウス雑音」から、**幾何遮蔽による LOS/NLOS・空間相関シャドウイング・小規模フェージング**を含む合成モデルへ拡張する。
3. トラック・歩行者等の**遮蔽エンティティ(blocker)** を追加し、その座標・体積からリンクごとの遮蔽状態を幾何計算する。

評価軸は「移動遮蔽が存在する環境」における3方式比較:
`feedforward_optimal`(真値LUT=理論上限) / `kkf_predictive`(提案) / `sequential`(リアクティブ下限)。

> **重要な前提**: 遮蔽もフェージングもない現状環境では、KKF方式は原理的に神託LUTを超えられない(学習した地図の上限が真値地図)。**提案方式の優位性は「LUT作成後に環境が変わる/動的遮蔽がある」状況で初めて現れる**ため、チャネル拡張(2.)と blocker(3.)は KKF評価の前提条件であり、実装順として先行させる。

---

## 2. 現状ベースライン(確認済みの事実)

| 項目 | 現状 |
|---|---|
| データプレーン | `TxControllerPlugin.cc`(gz-sim システムプラグイン、gz-transport使用、ROSノードではない) |
| チャネル | `PropagationModel` 抽象+Factory([comms_calculator.hpp](../src/comms_sim_pkg/include/comms_sim_pkg/comms_calculator.hpp)): LogDistance / TwoRay。雑音は対称ガウス(白色・空間相関なし)。**遮蔽・フェージングなし** |
| スケジューラ | `HandoverScheduler` + `ISchedulingStrategy`(戦略登録・委譲機構は導入済み) |
| C++組込KKF | `include/comms_sim_pkg/kkf/`(RoadCoordinate/KKF/Planner/Strategy)実装済み・ビルド可・実機未検証 |
| エンティティ | role: rx(基地局)/tx(車両)のみ。遮蔽体 role なし |
| ブリッジ | コンテナ内で `gz.transport13` Python バインディング利用可(確認済み)。`ros_gz_bridge` も有 |
| 並列実行 | スイープは worker ごとに ROS_DOMAIN_ID / GZ(IGN)_PARTITION 分離 |

---

## 3. ご提示設計(制御/データプレーン分離案)への評価

### 3.1 採用する点

- **制御プレーン(Python ROS 2ノード)とデータプレーン(C++プラグイン)の分離** — 文書6.3節に忠実で、実機のエッジサーバ/RSU群の構成を模倣できる。数理部の研究速度も高い。
- **P2P制約の厳密化** — 観測レポートを「grant中ペア(+測定割当ペア)のみ」に制限し、神託の混入を通信路レベルで遮断する。
- **スケジュール購読+直近決定のみ実行(MPC)** — 遅延・欠落に構造的に頑健。

### 3.2 修正・補強する点(ブラッシュアップ)

1. **通信経路の具体化**: プラグインは ROS ノードではないため「ROS 2トピックで直結」はそのままでは成立しない。
   - **案A(推奨)**: 制御プレーンが `gz.transport13` Python API で gz トピックを直接 pub/sub する。ブリッジ不要、GZ_PARTITION による worker 分離をそのまま継承。
   - 案B: `ros_gz_bridge` を挟み ROS 2 トピック化する。実機構成には近いが、スイープ時に worker ごとのブリッジプロセスが増える。
   - フェーズBの冒頭で案Aの partition 分離を実測確認し、問題があれば案Bへ切替(インターフェースは同一に保つ)。
2. **メッセージ形式**: 【決定済 2026-07-07】最初から厳密型とする。gz-transport 上のカスタム protobuf メッセージ(`proto/comms_sim_msgs.proto`)を定義し、CMake の protoc 生成で C++ 側、`protoc --python_out` で制御プレーン側の型を共有する(§7)。
3. **時刻規約**: 全メッセージ sim time 基準。スケジュールは「(開始時刻, 割当)列+有効期限」を持ち、期限切れ時はフェイルセーフ(現割当維持)。physics step は 1e-4 s なので**毎ステップ publish は禁止**、レポートは 20 Hz(sim)、スケジュールは 5 Hz(sim) に律速。
4. **再現性**: 非同期ノード化により完全決定性は失われる。乱数シードの決定的導出(worker/run/リンク別)+周期の離散化で統計的再現性を担保。厳密検証には §6.3 のゴールデンテストを用いる。
5. **既存C++組込KKFの扱い**: 同期・決定的な**リファレンス実装**として保持し、Python版の数値一致検証(ゴールデンテスト)に使う。二重実装の恒久維持はせず、Python版安定後に「削除」か「組込高速モードとして残す」かを再判断。
6. **探索割り当て(文書5.4節)**: 単一車両では地図鮮度が自車履歴に限られ効果が限定的。機構(MEASURINGモード)はフェーズBで先に用意し、活用はフェーズE。

### 3.3 ご提示の Open Questions への回答

1. **制御プレーンは Python でよいか** → 賛成。ただし数理コアは ROS/gz 非依存の純粋ライブラリ(`kkf_core/`)として分離し、ノードは I/O の薄皮に徹する(§6.1)。
2. **軌道 s(t) の取得** → **経路形状(waypoint の幾何)は既知、速度は観測から推定**のハイブリッド。鉄道では路線形状が事前に既知であることは物理的に正当。速度まで既知とすると遅延・非定常への頑健性が評価できなくなる。
3. **強制測定モード** → 賛成。リンク状態機械に `MEASURING`(データ会計なし・観測レポートのみ)を追加し、スケジュールの `mode: measure` で発動する。

---

## 4. チャネルモデル拡張設計

### 4.1 合成モデルの構成

```
RSSI(link, t) = TxPower + G_ant(link)
              − PL(d)                    # 既存: PropagationModel
              − L_blockage(link, t)      # 新規: 幾何遮蔽 (LOS/NLOS)
              − X_shadow(s, t)           # 新規: 空間相関シャドウイング
              − L_fading(link, t)        # 新規: 小規模フェージング
```

責務分割(すべて `comms_sim` 名前空間、ヘッダオンリー、Eigen+標準ライブラリのみ):

| モジュール | 責務(変更理由) | モデル |
|---|---|---|
| `PropagationModel`(既存) | 距離減衰のみ | LogDistance / TwoRay |
| `IBlockageModel`(新規) | リンク線分と遮蔽体群の幾何交差 → LOS判定と超過損失 | §4.2 |
| `IShadowingModel`(新規) | 位置に紐づく相関シャドウイング | Gudmundson(相関距離 d_corr, σ_sh)。現行の白色 `noise_variance` はこの特殊ケース(d_corr→0)として統合 |
| `IFadingModel`(新規) | 時間方向の小規模変動 | LOS: Rician(K高) / NLOS: Rayleigh(=Rician K=0)。dB損失に変換して合成 |
| `ChannelModel`(新規) | 上記の合成のみ(ファサード) | — |

- `CommsCalculator` は `ChannelModel` を注入されて使う形に変更(パスロス直呼びを置換)。
- 生成は `ChannelModelFactory`(既存 `PropagationModelFactory` を包含)が yaml `channel:` セクションから組み立てる。**旧設定(path_loss + noise_variance のみ)はそのまま動く後方互換**とする。
- シードは (base_seed, run_idx, link_id) から決定的に導出。

### 4.2 幾何遮蔽 (LOS/NLOS 判定)

- 各 blocker は **OBB(中心pose + 寸法 size[x,y,z])** で表現。60 GHz では第1フレネル半径が数 cm(λ≈5 mm)のため、**フェーズ1はリンク線分 vs OBB の交差判定で二値 LOS/NLOS + 遮蔽体種別ごとの固定超過損失**(例: 人 15 dB / トラック 30 dB、3GPP TR 38.901 blockage 相当のオーダ)で十分な近似となる。
- 交差計算は slab 法(線分×OBB)。複数遮蔽体は損失を加算(上限クリップ)。
- フェーズ2(任意)で knife-edge 回折による連続値(遮蔽横断長・フレネルゾーン侵入率→損失)へ拡張できるよう、`IBlockageModel` の戻り値は最初から `{is_los, excess_loss_db, blocker_names}` の構造体とする。

### 4.3 ログ拡張

`detailed_logs` に `link_los, blockage_loss_dB, shadow_dB, fading_dB` 列を追加(第2層トラッカーの検証と分析の基盤)。

---

## 5. エンティティ拡張設計 (blocker)

1. **シナリオ定義**: `model_catalog` に遮蔽属性を追加:
   ```yaml
   Truck:
     sdf_path: "models://Truck"
     blockage: { size: [12.0, 2.5, 3.8], loss_db: 30.0 }
   Person:
     sdf_path: "models://Person"
     blockage: { size: [0.5, 0.5, 1.7], loss_db: 15.0 }
   ```
   `entities` に `role: "blocker"` を追加(static / waypoints 付き移動の両対応)。
2. **移動**: 通信プラグインと分離した軽量プラグイン `WaypointMoverPlugin`(新規)を blocker に付与。既存 `VehicleMotionController` を再利用し、責務は「waypoint 追従移動のみ」。
3. **データプレーン**: `BlockageEnvironment`(新規, C++)が ECM から blocker の pose を毎ステップ取得し、`IBlockageModel` に遮蔽体リストを供給。`CommsEnvironment::CalculateMetrics` はこれを注入されて per-link 遮蔽を評価。
4. **scenario_loader(Python)**: role blocker の spawn・プラグイン付与・blockage 属性の sim_params への伝搬を追加。

---

## 6. 制御プレーン設計 (Python)

### 6.1 パッケージ構成(SRP)

```
src/comms_sim_pkg/comms_sim_pkg/
  kkf_core/                  # 純粋数理層: ROS/gz 非依存、numpy のみ。単体テスト可能
    road_coordinate.py       #   弧長座標 (C++ RoadCoordinate と同一仕様)
    basis.py                 #   基底関数 φ(s) (対数距離基底)
    kkf.py                   #   第1層: KF更新+残差クリギング (式(1)〜(8))
    blockage_tracker.py      #   第2層: 残差しきい値抽出→クラスタ→等速KF追跡 (式(9)〜(11))
    planner.py               #   第3層: LCB(式(12))+ビタビDP(式(13))
    frk.py                   #   (フェーズE) Fixed-Rank Kriging (式(14),(15))
  kkf_scheduler_node.py      # I/O層: gz topic 購読/配信・周期制御のみ。数式を持たない
```

### 6.2 処理フロー

- **観測受信(20 Hz sim)** → L1 の該当 RSU マップ更新。
- **L2(観測受信ごと)**: 更新後残差 < ν_th の位置を抽出 → 1次元クラスタリング → トラッカー(等速KF)更新 → 未来遮蔽区間 B(t+Δ) を外挿 → L1 の観測雑音 R(s,t) を先回り設定(式(11))。
- **再計画(5 Hz sim)**: 各(車載アンテナ×RSU)ペアの予測LCB系列を構成し、遮蔽区間ペナルティを加算 → ビタビDP → スケジュール(ホライズン全体)を配信。

### 6.3 ゴールデンテスト(C++リファレンス活用)

同一の観測系列(CSVフィクスチャ)を C++ `kkf/KrigedKalmanFilter.hpp` と Python `kkf_core/kkf.py` に与え、予測平均・分散の一致(許容 1e-9)を CI 的に確認する。移植ミス・回帰の防止と、非同期化前の数値検証を分離できる。

---

## 7. トピック / メッセージ設計 (gz-transport, カスタム protobuf)

`src/comms_sim_pkg/proto/comms_sim_msgs.proto` に以下を定義し、C++(CMake protoc)と Python(生成 pb2 を `comms_sim_pkg` にインストール)で型を共有する:

```proto
message MeasurementEntry { int32 ant = 1; int32 bs = 2; double rssi_dbm = 3;
                           string link_state = 4; Mode mode = 5; }
message MeasurementReport { double t_sim = 1; Vector3 vehicle_pos = 2;
                            repeated MeasurementEntry reports = 3; }
message ScheduleEntry { double t_start = 1; int32 ant = 2; int32 bs = 3; Mode mode = 4; }
message HoSchedule { double t_issued = 1; double valid_until = 2;
                     repeated ScheduleEntry plan = 3; }
enum Mode { DATA = 0; MEASURE = 1; }
```

| トピック | 向き | 周期(sim) | 型 |
|---|---|---|---|
| `/comms/measurement_report` | plugin → node | 20 Hz | `MeasurementReport` |
| `/comms/ho_schedule` | node → plugin | 5 Hz | `HoSchedule` |

- レポートに含まれるのは **grant 中ペア + MEASURING ペアのみ**(観測合成は既存 `KkfMeasurementModel` の役割を data plane 側の publish 判定に移設。真値+観測雑音の付与はプラグイン側=物理の責務とする)。
- プラグイン側実行は **`ExternalScheduleStrategy`(新規, `ISchedulingStrategy` 実装)**: 受信済みスケジュールから現在時刻の割当を適用。未受信・期限切れ時は現割当維持(フェイルセーフ)。§3.2-5 の組込KKF戦略と同じ登録機構に載る。

---

## 8. 実装フェーズ計画

| フェーズ | 内容 | 主な成果物 | 検証 |
|---|---|---|---|
| **A. チャネル拡張+blocker** | §4, §5 全体 | ChannelModel系, BlockageEnvironment, WaypointMoverPlugin, blocker シナリオ, ログ列 | 静止トラック横で LOS→NLOS 遷移がログに現れる/横断トラックで時間窓の遮蔽が再現される |
| **B. データプレーンI/O** | レポートPub・スケジュールSub・ExternalScheduleStrategy・MEASURING | gzトピック疎通, フェイルセーフ | 手書き固定スケジュールをノードから流し、プラグインが追従・切替すること |
| **C. 制御プレーン L1+L3** | kkf_core(移植+ゴールデンテスト), MPC閉ループ | kkf_scheduler_node | ゴールデンテスト一致。遮蔽なし環境で feedforward_optimal に近い総データ量に漸近 |
| **D. L2+評価** | 遮蔽トラッカー, 3方式比較スイープ | blockage_tracker, 評価シナリオ | 横断トラックの B(t+Δ) 予測が実遮蔽窓と一致。移動遮蔽環境での瞬断時間・スループット比較 |
| **E. 任意拡張** | FRK, 探索割当, ビーム地図(文書6.2節) | frk.py 他 | — |

---

## 9. リスクと対処

1. **gz.transport13 の partition 分離が Python から効くか** — フェーズB冒頭に単体で実測。不可なら ros_gz_bridge(案B)へ切替(トピック仕様は不変)。
2. **制御ノードが RTF 5 に追従できない** — 周期の離散化・numpy 化で通常十分。不足時は FRK 前倒し or 該当シナリオのみ RTF 引下げ。
3. **スイープ資源** — worker ごとに制御ノードが1つ増える。sweep_sim.py のタスク起動に制御ノードの起動/終了を組み込む(フェーズC)。
4. **KKFの評価が神託に届かない問題** — 期待どおりの結果(上限は神託)。論点は「動的遮蔽下で神託(静的LUT)が崩れ、KKF が反応・先読みできること」であり、フェーズAを先行させる根拠。

---

## 9.5 実装進捗 (2026-07-08 時点)

- **フェーズA〜D 完了**。ゴールデンテスト(`tools/tests/golden_kkf_test.py`)・トラッカー単体テスト(`tools/tests/blockage_tracker_test.py`)・E2E閉ループすべて検証済み。
- 動的遮蔽環境での評価(`config/scenarios/kkf_blockage_eval.yaml`): grant中ペアの NLOS 暴露 **KKF-MPC 5.7% vs 静的LUT神託 58.7%**。第2層トラッカーが移動遮蔽帯を検出・追跡し、式(11)(12)の還流が機能。
- 残フェーズE(任意): FRK(式(14))、探索割当(5.4節)、ビーム地図(6.2節)、sequentialのペアコミット化(公平比較用)。

## 10. 決定事項(2026-07-07 確定)

1. **メッセージ形式**: 最初から厳密型(カスタム protobuf、§7)
2. **C++組込KKF の扱い**: ゴールデンテスト用リファレンスとして保持
3. **フェーズAの遮蔽粒度**: 二値 LOS/NLOS + 種別別固定損失(knife-edge は拡張余地として IF に確保)
4. **着手順**: フェーズA(チャネル拡張+blocker)から
