# ドローンすれ違い通信モデル仕様

## 目的と実装範囲

ドローン2機の接近・すれ違い・離脱に伴う、接続失敗と500 msタイムアウトの条件を比較する。
Gazebo Harmonicのworldプラグイン `DroneEncounterPlugin` が2機の軌道とアンテナ姿勢を更新し、独立したC++通信コア `drone_link` を進める。ROS 2 Humbleへ `/clock` と `/drone_a/pose`、`/drone_b/pose`（`PoseStamped`）をbridgeする。

接続手順はIEEE 802.15.3eを参考にした抽象モデル。PHYの設計目標はA→B（272 GHz）が802.15.3d、B→A（61 GHz）が802.15.3eである。規格本文との条項単位の適合確認は未実施で、規定PHY波形・フレーム形式、payload FEC、MAC再送・再送上限、チャネル競合、ビーム探索の完全な実装ではない。異帯域を跨ぐ共通接続手順も本実験固有の構成である。

参考資料：[IEEE TG3e資料一覧](https://www.ieee802.org/15/pub/TG3e/3e_table_3.html)、[ITU-R M.2227-2](https://www.itu.int/dms_pub/itu-r/opb/rep/R-REP-M.2227-2-2017-PDF-E.pdf)。これらを根拠に、下記の暫定タイミングを規格値とは扱わない。

## 設定別の適用範囲

本文の64 byte制御フレーム、20 µs／100 µs送信時間、理論BER/PER抽選は、既定の [drone_params.yaml](src/comms_sim_pkg/config/drone_params.yaml) による統計AWGNモードの仕様である。軌道、電力・熱雑音計算、半二重、140 µs周期Beacon、500 ms待ち、接続前タイムアウトと接続後切断の区別は、試作PHYにも共通する。

| モード | 設定 | 受信判定 |
|---|---|---|
| 統計AWGN（既定） | `lab_phy.enabled` 無効／省略、`awgn.enabled: true` | 電力・任意SNR閾値を満たすフレームに理論PER抽選 |
| 試作I/Q PHY | `lab_phy.enabled: true`、`awgn.enabled: true`、`awgn.bit_rate_bps: 3.52e9` | 電力・任意SNR閾値と半二重を確認後、I/QへのAWGN付加・復調・CRC判定 |
| 互換モード | `lab_phy.enabled` 無効／省略、`awgn.enabled: false` | 電力・任意SNR閾値、既存MCS表によるレート選択 |

試作PHYの詳細は末尾の[試作I/Q PHY](#試作iq-phy)と [実装説明](specification_review/runtime_lab_phy_ja.txt) を参照。`specification_review/` の適応MCSやLDPCを含む提案は、そのまま実装済みの仕様を意味しない。

## 軌道と座標

ワールドはZ上向き。Aが低い機体、Bが高い機体。`v_a`、`v_b` は設定速度km/hを3.6で割ったm/s、`t_c` はすれ違い時刻。

- A: `(v_a × (t − t_c), 0, h_a)`、機首は+X。
- 並行B: `(-v_b × (t − t_c), d_y, h_b)`、機首は−X。
- 十字B: `(0, v_b × (t − t_c), h_b)`、機首は+Y。

初期値は `h_a=20 m`、`h_b=30 m`、`d_y=10 m`、両機50 km/h、`t_c=20 s`、終了40 s。
最接近距離は並行で√200 m、十字で10 m。速度を個別に変えても、各機は`t_c`に各経路の中央へ到達する。

FuelのParrot Bebop 2モデルを2個配置し、`Model::SetWorldPoseCmd`で理想軌道を指定する。機体とアンテナの原点を通信位置とし、機体形状の遮蔽は計算しない。`gazebo_poses.csv`はPhysics更新後の実位置であり、解析式から再生成した位置ではない。

固定アンテナはAの仰角+45°、Bの仰角−45°。並行ではAが+Y側（方位角+90°）、Bが−Y側（−90°）を向くため、高度差・横間隔が各10 mの最接近時に正対する。十字も横向きの固定搭載とし、Aは機首の左90°（世界方位+90°）、Bは機首の右90°（世界方位0°）を向く。これは交差後に相手のいる側を向く配置。十字の最接近時には相手が真上／真下に来るため、仰角±45°の固定方式では正対しない。追従アンテナは相手方向へ瞬時に向く。機体姿勢とアンテナ姿勢を分離し、Gazeboのpitch符号と仰角の符号を反転して扱う。`off_axis_*_deg` はアンテナ正面軸と相手方向のなす3次元角度。E/H各面の角度とは異なる。

## 無線計算

参照資料はユーザー提供の `260828_リンクバジェット試算03_5.xlsx`。
「リンク_4m_1」の構成・送信電力・周波数を採用することをユーザーが選択した。
受信判定の電力閾値は指定により両方向とも−68.5 dBmを維持する。
Excelの入力・式・計算済み値と原本SHA256は `src/comms_sim_pkg/test/fixtures/link_budget_workbook.json` に保存。
4 mは資料の照合距離であり、ドローンの高度差・軌道・最接近距離を4 mに変更するものではない。

| 項目 | A→B | B→A |
|---|---:|---:|
| 周波数 | 272 GHz | 61 GHz |
| 送信電力 | −1 dBm ≈ 0.794328 mW | −8 dBm ≈ 0.158489 mW |
| 送信側挿入損失 | 0 dB | 3 dB |
| 送信／受信アンテナ正面利得 | 各24 dBi | 各23 dBi |
| 受信LNA利得 | 25 dB | 0 dB |
| 受信変換利得 | 3 dB | 0 dB |
| 受信側挿入損失 | 3 dB | 3 dB |
| TJX入力端の電力閾値 | −68.5 dBm | −68.5 dBm |

電力の計算は次の順序で行う。電力はdBm、利得・損失はdB。
Excelの挿入損失は負値を加える形式だが、YAMLでは正値を指定して減算する。

```text
Pt = 10 × log10(power_mw)
EIRP = Pt − Ltx + Gtx
PL(d) = 20 × log10(4 × π × f × 1m / c)
        + 10 × n × log10(max(d, 1m) / 1m)
受信アンテナ端 = EIRP − PL(d) + Grx
LNA出力 = 受信アンテナ端 + GLNA
Mixer出力 = LNA出力 + Gconversion
TJX入力 = Mixer出力 − Lrx
```

`c=299792458 m/s`、初期指数 `n=2`。Excelは `c=300000000 m/s` の近似値を使うため、
シミュレータのPLは約0.006011 dB大きく、受信電力は同じだけ小さい。
正対時のTJX入力は A→Bが `72 − PL272(d)`、B→Aが `32 − PL61(d)`。
LNA等は線形・一定利得のモデルであり、飽和・圧縮・AGCは未実装。

アンテナの角度特性には従来の `e_plane.csv`、`h_plane.csv` を使用する。
`G方向 = 指定した正面利得 + CSV合成利得(角度) − CSV合成利得(0,0)` とし、
絶対利得を二重加算しない。E/H補間・合成・減衰上限の処理は従来と同じ。
24/23 dBiは資料由来だが、両帯域の角度特性は共通CSVを使う仮定であり実測校正ではない。
旧YAMLで正面利得が未指定ならCSVの絶対利得を使用し、回路の利得・損失は0で従来計算を維持する。

`rssi_ab_dbm` / `rssi_ba_dbm` は現在はTJX入力端の**信号電力**を表す。
実機のRSSI検出器が示す信号＋雑音の総電力と同一とは限らない。
`gain_a_db` / `gain_b_db` は互換性のため旧CSVの合成利得を保持し、
実際に適用した方向別利得は `ab_tx_gain_dbi` 等を見る。
`timeseries.csv` に両方向のPL・EIRP・アンテナ端・LNA後・Mixer後・TJX入力を記録する。

### 熱雑音

資料の300 K、1.76 Gbaud、ロールオフ0.23から、帯域幅を2.1648 GHzとする。
ここではこの帯域を雑音等価帯域として近似する。受信フィルタの実測等価帯域が分かれば変更する。

```text
B = 1.76e9 × (1 + 0.23) = 2.1648e9 Hz
Nthermal = 10 × log10(k × T × B) + 30  [dBm]
k = 1.380649e−23 J/K
Ninput = Nthermal + NF
Grx_chain = GLNA + Gconversion − Lrx
Ntjx = Ninput + Grx_chain
SNR = Ptjx − Ntjx = Pantenna − Ninput  [dB]
```

`Nthermal ≈ −80.473777 dBm`。NF=10 dBなら入力換算雑音は約−70.473777 dBm。
272 GHzのNF=10 dBはExcelから採用し、アンテナ出力を基準とした受信系全体の等価NFと解釈する。
61 GHz側のNFはExcelに記載がなく、ユーザー指定で0 dBの理想受信機とする。実測値が分かれば変更する。
0 dB設定では挿入損失等による実機の雑音劣化まで再現するものではない。
各段個別のNF・温度によるFriis雑音指数の合成は行わない。

`required_snr_db: null` は固定SNR閾値を追加しない設定。現在は後述のAWGNフレーム誤り判定を有効化している。
数値を設定すると「TJX入力が−68.5 dBm以上、かつSNRが指定値以上」の両方を要求する。
SNR不足は `SNR_BELOW_REQUIRED` として記録し、`ab_reachable` / `ba_reachable` に反映する。
`noise_bandwidth_hz: 0` は旧設定との互換用で雑音計算を無効化する。
Excelの9.8 dBはEb/N0表記であり、ビットレートと帯域の換算条件が未確定なので必要SNRへ自動転用しない。
Excelの「TJX sensitivity + NF」はそのまま閾値へ転用せず、電力・雑音を同一基準点で比較する。

kTBは平均雑音電力を計算する。現在は以下のAWGNモデルで確率的なフレーム誤りを生成する。
`awgn.enabled: false` の互換モードでは従来の `MCStable.csv` をTJX入力信号電力へ適用する。
AWGN有効時はMCS表を使わず、QPSKの固定レート3.52Gbpsを使う。
フェージング、反射・マルチパス、大気吸収、雨、干渉、偏波ずれ、ドップラー、機体遮蔽も未実装。

## 接続と待ち時間

1. Bが61 GHzでBeaconを送信。Aが受信した場合、IFS後に272 GHzでRequest。
2. BがRequestを受信した場合、IFS後に61 GHzでResponse。
3. AがResponseを受信した場合、IFS後に272 GHzでACK。
4. BのACK受信が成功すると論理接続が成立し、A→BのDATAとB→AのDATA_ACKを交互に送る。
5. DATA_ACKを受信できたデータのみ確認済み転送量に加算する。

| 設定 | 初期値 | 意味 |
|---|---:|---|
| `response_timeout_ms` | 500 ms | Requestの受信完了予定時刻からResponseを待つ期限 |
| `ack_timeout_ms` | 500 ms | Responseの受信完了予定時刻から接続ACKを待つ期限 |
| `inactivity_timeout_ms` | 500 ms | 最後のDATA_ACK成功、または接続成立からの無応答期限 |
| `beacon_interval_ms` | 0.14 ms（140 µs） | BがRequestを受信するまで継続する送信開始間隔。ユーザー指定 |
| `control_frame_us` | 20 µs | 全制御フレームの送信時間。モデル仮定 |
| `ifs_us` | 10 µs | フレーム間隔。モデル仮定 |
| `data_frame_us` | 100 µs | データブロックの送信時間。モデル仮定 |

フレーム受信完了は送信時間に開始時の距離/cを加えた時刻。フレーム開始・中間・完了、および `channel_sample_s`（現在はphysics_stepと同値）ごとにリンクを評価し、どれかが感度を下回ればフレーム全体を失敗とする。連続時間で全区間を厳密積分するモデルではない。

タイムアウトはイベント時刻で判定する。Gazeboの更新刻みに20 µsや10 µsを丸めない。通信中の短いフレームをphysics_stepだけで見落とさないよう、開始・中間・完了も評価する。データレートはフレーム内のサンプル最小値を用い、`rate[Gbps] × data_frame_s × 125` をMBとしてDATA_ACK成功後に加算する。MAC/PHYヘッダーや符号化の追加オーバーヘッドは含めない。

**現在は失敗したフレームの再送を行わず、期限まで待つ。** 期限中にRSSIが回復しても、再送による救済はしない。タイムアウトまたは接続後切断の後に次の周期のBeaconから接続をやり直す。このため再送機能のある実機の発生回数と直接同一視できない。`both_directions_reachable_at_timeout` は判定時の電力/SNR固定閾値が両方向で成立するかを表す。AWGNを含む実際のフレーム受信成功を保証する値ではない。

接続手順本体とBのBeacon周期送信を別々にスケジュールする（`indexed_beacon_until_request_half_duplex_v1`）。BはRequest受信成功まで140 µs周期で送信し、AのBeacon受信成功だけでは停止しない。AはRequest送信後、最初のResponse待ち500 msを維持し、新しいBeaconを無視する。Request消失時はBのBeacon送信だけが継続する。Response/ACK消失時はRequestを受信済みなのでBeaconは停止したまま、従来の期限到達後に次の周期格子から再探索する。個別端末の全ローカルタイマーを完全に再現するモデルではない。Beaconの受信失敗だけでは待ちタイマーを開始しない。

A/Bとも半二重として、受信区間（送信開始＋伝搬時間から受信完了まで）と自機の送信区間が一部でも重なると、フレーム全体を `RECEIVER_TRANSMITTING` で失敗させる。異なるBeaconインデックス間でも同様。同時刻の送信終了と受信開始は重複しない。Request受信完了と次Beacon予定時刻が一致する場合は受信完了を先に処理してBeaconを停止する。既に送信済みのBeaconの受信完了イベントは削除せず処理する。

`beacon_index` は実験内で1始まりの送信連番で、再探索・再接続時も増加し続ける。Request/Response/ACKと接続後DATA/DATA_ACK、待ち・タイムアウト・切断にも起点番号を保持する。`events.csv` と `frames.csv` に記録し、解析で `beacon_requests.csv` と `connections.csv` に対応を出力する。制御64 byteの誤り判定対象長は維持し、インデックスはフレーム内の管理情報に含まれる想定（実際のビット配列は生成しない）。

状態図は主接続手順を表し、並行Beaconの成功だけではResponse待ちなどを上書きしない。応答に採用した `BEACON_SELECTED` から手順を復元する。

集計定義 `setup_only_v2`: `timeouts` は接続前のResponse/ACK待ちのみ。接続後の無応答期限到達は `DISCONNECTED` / `INACTIVITY_DISCONNECT` として記録し、`disconnections` に数える。初回DATA失敗や初回DATA_ACK失敗で転送量が0 MBの場合も切断側に数える。設定キー `inactivity_timeout_ms` は互換性のため保持する。旧ログの `TIMEOUT / INACTIVITY_TIMEOUT` は同時刻の `DISCONNECTED` と重複するため再解析で除外し、`classification_summary.csv` に新定義を保存する。

接続後、無応答を待つ500 msも論理接続時間に含める。`connected_time_s`はデータを送れていた時間とは異なる。終了時点で期限未到達なら `pending_waits.csv` に残し、タイムアウトとして数えない。

## 原因分析のデータ

`events.csv`のRX_FAILEDは受信完了予定時刻に発生する。`failure_sample_s`はフレーム内で最初に受信不能を検出したサンプル時刻。半二重による失敗は受信完了時に確定し、`failure_sample_s` も受信完了時刻とする。RX_FAILED行の観測列はその時刻、TIMEOUT行の観測列はタイムアウト時刻の値である。

- `time_s`：イベント確定時刻。
- `wait_start_s`、`deadline_s`：タイマー起点と期限。未設定は−1。
- `observation_time_s`：距離・RSSI・角度の評価時刻。
- `rssi_dbm`：イベント確定時刻における当該フレーム方向のRSSI。
- `rssi_ab_dbm` / `rssi_ba_dbm`：観測時刻の両方向RSSI。
- `margin_ab_db` / `margin_ba_db`：RSSI − 各方向の受信感度。負値は感度不足。
- `off_axis_a_deg` / `off_axis_b_deg`：各アンテナ正面と相手方向のずれ。

成功したDATA/DATA_ACKはイベントCSVのサイズを抑えるため個別記録せず、ACK済みデータフレーム数・時系列の累積量へ反映する。失敗、待ち開始、タイムアウトは記録する。

比較の順序は、まず経路×アンテナ方式の4条件、次にA/B速度・高度差・横間隔・送信電力・受信感度などを一つずつ変更する。回数だけで判断せず、試行数、成立数、Beacon未受信、終了時の未判定待ちを併記する。現在のモデルで扱える原因には、距離・指向性による感度不足、任意SNR閾値不足、半二重の送受信重複、モード別のPHY受信失敗、互換モードのMCS利用不可、および設定した応答期限への到達がある。

## 再現性と検証

実行設定・入力CSV・モデルの出典情報を保存する。同じ出力先への再実行は拒否する。終了コード0だけで成功と判断せず、`completed`とCSVを検証する。各実験は40 sの同じ観測窓で比較する。

既存の車両・固定基地局シナリオはこのブランチから削除した。共通のアンテナ・伝搬計算とCSVは継承し、旧結果ファイルは保存している。新しいビルド成果物は `build/drone` / `install/drone` を使用する。

## リンク確立時間の集計

解析ツールは `connections.csv` に接続成立ごとの時刻を保存する。
- `setup_ms`: 成功したBeaconの送信開始から接続用ACK受信完了まで。探索時間を含まない。
- `after_beacon_rx_ms`: Beacon受信完了から接続用ACK受信完了まで。
- `since_sim_start_s`: シミュレーション開始（0秒）から接続成立まで。初回は探索待ちを含む。
未成立なら接続行なし、比較表の平均確立時間・初回成立時刻は空欄（0にしない）。
既定の制御フレーム20µs×4、IFS10µs×3に各フレームの伝搬遅延を加えるため、
成功時のsetup_msは約0.11ms。実機の処理遅延・再送・ビーム探索を再現した測定値ではない。

## AWGNによるフレーム誤り

ユーザー了承済みの初期仮定: Gray QPSK、同期は理想、誤り訂正なし、3.52Gbps（1.76Gbaud×2）。
情報ビット列・I/Q波形は生成せず、AWGN下の理論BERからフレーム誤り率を求め、フレームごとに抽選する。
これはPHYの統計的な抽象モデルであり、系列・同期捕捉・周波数ずれ・実CRC計算は含まない。

```text
Eb/N0[dB] = SNR[dB] + 10 log10(B / Rb)
BER = 0.5 × erfc(sqrt(10^(Eb/N0[dB]/10)))
PER = 1 − (1 − BER)^N
U = 一様乱数[0,1)
フレーム受信失敗 = U < PER
```

式の参照: MathWorks「Analytical Expressions Used in berawgn」
https://www.mathworks.com/help/comm/ug/analytical-expressions-used-in-berawgn-function-and-bit-error-rate-analysis-app.html

Bは各方向の雑音帯域2.1648GHz、Rbは3.52e9bit/s。NFを含むSNRを使い、NFを二重加算しない。
フレーム内でサンプル評価した最小SNRをフレーム全体へ適用する保守的な近似。独立なビット誤りを仮定。
PER計算は桁落ちを避けるため `-expm1(N * log1p(-BER))` を使う。

制御フレーム（Beacon/Request/Response/ACK/DATA_ACK）の誤り判定対象長は64byte=512bit。
送信時間20µsは維持し、残り時間のプリアンブル等は誤り判定対象として個別には扱わない。
DATAは100µs×3.52Gbps=352000bit=44000byteで、ヘッダ・誤り訂正等の追加オーバーヘッドは未モデル化。
1bit以上の誤りでフレーム全体を失敗とする。ACKで確認できた44000byteだけを転送量へ加算する。

電力閾値-68.5dBmと任意の固定SNR閾値を先に判定し、満たすフレームに対してAWGN抽選を1回行う。
RSSIそのものにGaussian乱数を加える処理ではない。`ab_reachable`等は電力/SNR固定閾値の判定であり、AWGN抽選結果ではない。
乱数シードは `awgn.seed`（既定20261027）。mt19937_64の上位53bitからUを生成する。
ログやRadioModel::Evaluateの呼び出し回数によって乱数系列が変わらないよう、抽選は受信完了処理だけで行う。

`frames.csv`は全フレームの開始/完了、信号電力、最小SNR、Eb/N0、N、BER、PER、U、成否、原因を保存する。
電力不足等で抽選しなかった場合はU=-1。原因 `AWGN_PACKET_ERROR` がAWGNによる受信失敗。
この場合のfailure_sample_sは実際のビット誤り時刻ではなく、フレーム受信完了時刻。
`awgn_trials`は感度等の条件を満たして抽選した回数、`awgn_errors`はその抽選で失敗した回数。

再送なし・500ms待ちは維持するため、AWGNによる単発の失敗でもResponse待ち/ACK待ち/無応答待ちへ移る。
1つのシードの結果を平均成功率や実機の性能保証として扱わない。旧結果とはPHYレートも異なるため、
転送量の差すべてをAWGNだけの影響と解釈しない。

## 試作I/Q PHY

[drone_lab_phy.yaml](src/comms_sim_pkg/config/drone_lab_phy.yaml) は40秒、[drone_lab_phy_smoke.yaml](src/comms_sim_pkg/config/drone_lab_phy_smoke.yaml) は1.1秒の試作PHY実行設定。出力設定の `phy_error_model` は `lab_iq_rect_v0_uncoded`、`protocol_revision` は `indexed_beacon_lab_mac80_v0` となる。

独自のMAC／PHYヘッダー、ヘッダー誤り訂正、CRC、preambleを含むビット系列を生成する。A→BはQPSK、B→Aはπ/2回転QPSK、1.76 Gbaud・2 sample/symbolの矩形パルスを使う。受信フレーム窓に実際のGaussian雑音を加算し、相関・復調・ヘッダー検査・body CRCから成否を決める。理論PERによる抽選は行わない。

Beacon／Request／Responseのbodyは16 byte、ACK／DATA_ACKは0 byte、DATAは `lab_phy.data_bytes`（既定2048 byte）。送信時間は実際のサンプル数から算出し、ACK済みのDATA bodyだけを転送量へ加算する。`awgn.control_frame_bytes`、`protocol.control_frame_us`、`protocol.data_frame_us` はこのモードの長さ・時間には使わない。

`frames.csv` には `phy_model`、送受信サンプル時刻、処理サンプル数、bit誤り数、相関値、実際の16 bit通番と参照番号を追加する。`frame_bits` はbody情報bit数、`per` はNaN、`random_draw` は−1。`ber` は比較用の理論値であり、受信成否を決める抽選確率ではない。感度不足や半二重で先に失敗が決まる場合、`processed_samples` は0となる。

payload FECは `none` のみ。LDPC、RRCパルス整形、連続待受け探索、検出時刻ジッタ、CFO、クロック偏差、正式な規格系列・フレーム形式は未実装または未照合。受信窓とシンボル位相を既知とし、フレーム内SNRを一定とする近似がある。I/Q波形自体は保存しない。実装の詳細・試験範囲は [runtime_lab_phy_ja.txt](specification_review/runtime_lab_phy_ja.txt) に記載する。

## 未実装の設計案

[provisional_design_ja.txt](specification_review/provisional_design_ja.txt) と [adaptive_mcs_design_ja.txt](specification_review/adaptive_mcs_design_ja.txt) は規格解説資料を基にした仮設計。適応変調とpayload LDPCは実行コードへ未接続であり、現行の統計AWGN／試作PHYはいずれも固定QPSKを使う。

`MCStable.csv` の各行とMCSの対応、速度がPHYレートか実効速度か、RSSIの参照面は未確定。試作PHYのgoodputと表の値を参考比較しても、適応変調の性能を検証したことにはならない。提案YAMLは `config_file` に渡す実行設定ではない。
