# TJX基準の理論PHYモデル

開発ブランチ: `feature/tjx-theoretical-phy`。モデル識別子: `tjx_ldpc_awgn_v1`。

この文書は開発・再現用の内部メモ。実装上の仮定やLUTの統計上の制約を発表資料へ自動転記しない。
資料には実験条件、モデル名、指標の定義、試行数・平均・分散とアンテナ利得の計算詳細を残す。

## 既存モデルとの関係

`empirical_phy` は実機の受信特性、`theoretical_phy` は通信シミュレーションから生成したLUTを使う。
同時に有効にはできない。既存の実測モデル・詳細I/Qモデル・受信電力閾値モデルは維持する。
理論モデルは `MCStable.csv` と実測BER/PER表を読み込まない。
アンテナのE面・H面パターンCSV、位置・姿勢・RF収支・熱雑音計算は共通。
「実機受信特性が不要」はアンテナパターンまで理論化したという意味ではない。

入力例: `src/comms_sim_pkg/config/scenarios/drone_dual60_cross_fixed_theoretical.yaml`。
各機1 m/s、十字固定、roll=0、進行方向基準のpitch/yawはA=-45/-45度、B=+45/+45度。
交錯点手前7.071067812 mで正対する。90秒、両方向60 GHz、DATAはB→A、本文2048 bytes。

## MCSと送信時間

TJX資料の対応（IEEEのMCS番号とは異なる）をコード中の `tjx_mcs()` に定義する。

| TJX MCS | 変調 | 符号化率 | 繰り返し |
|---|---|---|---|
| 0 | BPSK | 11/15 | 2 |
| 3 | BPSK | 11/15 | 1 |
| 6 | QPSK | 11/15 | 1 |
| 8 | QPSK | 14/15 | 1 |
| 9 | 16QAM | 11/15 | 1 |
| 11 | 16QAM | 14/15 | 1 |

シンボルレート1.76 Gsymbol/s。PHY速度はシンボルレート×変調bit数×符号化率÷繰り返し数。
pilot有効なら56/64倍。MCS0は各反復のシンボル電力を維持し、受信LLRを軟合成する。

送信時間は平均PHY速度で本文を割るだけではなく、MACが作った本文（サブヘッダ・FCSを含む）の長さから、
短縮LDPCの実送信bit数、繰り返し、64chipブロックへの端数埋め、pilotを算出する。
setupのプリアンブル5120chip、通常3328chip、EHヘッダー1216chip、末尾pilot8chipを加える。
DATA2048 bytesはMAC本文2056 bytesになる。header-only ACKのLDPC本文は0 bytes。
時刻は既存HRCPの3.52 GHz整数sample時計で処理する。

MCSはACKフィードバックで0→3→6→8→9→11の順に上がる。10回確認で1段上げ、未確認/回復で1段下げる。
相手の真のSNRを送信側へ通知する方式や、旧受信電力閾値表は使わない。
接続、ACK、SIFS/RIFS、再送、ATP、重複排除は共通の `HrcpPairnet` を使う。

## LUT生成

既存のLDPC(1440,1056)/(1440,1344)、layered normalized min-sum（50反復・係数0.8）を再利用する。
各セルは `(TJX MCS, 実際の情報bit数, Es/N0)`。全長符号と、設定パケットから生じる短縮符号を個別に生成する。
ランダム情報bit→LDPC符号化→I/Qシンボル→複素AWGN→軟復調→LDPC復号→正解bit比較を行う。
I/Q雑音は各分散1/2、シンボル平均電力1、信号振幅はsqrt(Es/N0)。符号化利得の定数補正は入れない。
復号非収束または情報bit不一致をblock errorとする。復号後のbit errorsも別に保存する。
ヘッダーはEH(8,4)・4chip拡散の軟合成・HCSを別セル（mcs=-1、132情報bit）で評価する。

同梱LUTはEs/N0=-12〜20 dB、1 dB刻み、561セル×256試行。セルごとの試行数・block errors・bit errorsを保存する。
`trials_per_point=256`はLUT推定用であり、飛行30試行とは別。
生成はOpenMP並列化するが、セルの乱数seedは実行スレッド数に依存しない。

## 飛行時の受信判定

受信電力と雑音電力の差をSNRとし、`Es/N0[dB] = SNR[dB] + 10 log10(B / 1.76e9)` でLUTの軸へ変換する。
ここでBはSNRを算出した雑音帯域幅。符号化率を二重に適用しない。
隣接Es/N0のblock error確率を線形補間し、独立な符号語の成功確率の積から本文PERを計算する。
例えば同じ全長符号語n個と短縮符号語1個なら `PER_body = 1 - (1-P_full)^n * (1-P_short)`。
残留BERから `1-(1-BER)^L` に変換する方式ではない。情報bit長の別曲線がなければ起動時にエラーにする。

ヘッダーと本文を別の乱数で抽選する。正常ヘッダーは本文失敗でもMACへ渡し、ACK情報や回復処理を維持する。
飛行時のDATA内容はseed・世代番号から生成する疑似乱数byte列。波形は生成せず、復号成功時の内容をMACへ渡す。
受信済み通番の重複を除いた有効ペイロードだけを通信量へ加算する。ACK確認数とは区別する。

Gazeboのframes.csvにはモデル名、TJX MCS、送信sample数、SNR、Es/N0、BER、総PER、header/body PER、
header/bodyの乱数とLUT範囲外フラグを保存する。BERは復号後情報bitの期待誤り率で、当該抽選で観測したBERではない。
従来互換の `minimum_snr_db` 列には本モードでは送信開始時SNRを保存する。
`random_draw` はbodyの抽選、`header_random_draw` はheaderの抽選で、総PERと単一の乱数を直接比較しない。

## 内部に記録する仮定と制約

- TJXの変調・符号化率の対応は資料に基づく。BPSKにもHRCPのLDPC行列・短縮方法・共通EHヘッダーを適用するのは本モデルの仮定。
- 同期時刻・チャネル係数は既知。搬送波、連続時間波形、パルス整形、同期探索の失敗、CFO、位相雑音、マルチパス、Dopplerは未評価。
- ヘッダー誤りモデルは共通EHの代表的な内容を使用。誤ったbitが偶然CRCを通過する事象は受信成功として数えない。
- SNRはフレーム開始時に評価して保持。符号語・header/body・フレームの雑音は独立。
- LUT下限より低いEs/N0はPER1、上限より高い場合は最終点の確率を保持する。範囲外利用をログへ残す。
- 確率は試行数に対する誤り数の比。平滑化・校正補正なし。256試行で0 errorsは真の誤り率0を保証しない。
- 飛行30試行の分散は同じLUTを条件としたパケット抽選とMAC乱数の分散。LUT自体の推定誤差は含まない。
- NF・帯域等のRF設定は既存条件を保持する。TJX実機の性能保証や校正済み再現モデルとは扱わない。
- 接続後ATP10秒と接続待ち500 msを区別する。転送時間は10 msごとの受領量増加binの合計と転送spanを併記する。

## ネイティブ実行・再生成

リポジトリのルートで実行する。Eigen3、yaml-cpp、CMake、C++17、Python3/PyYAMLを使用。
Gazeboと同じ通信・位置計算コアを使うネイティブランナーなので、GUIのインストールは不要。

```bash
cmake -S tools/tjx_theory -B build/tjx_theory -DCMAKE_BUILD_TYPE=Release
cmake --build build/tjx_theory -j 6
ctest --test-dir build/tjx_theory --output-on-failure -j 4
python3 tools/run_tjx_theory_trials.py --trials 30 --jobs 6
```

結果には `aggregate.json`（平均・母分散・不偏分散・標準偏差）、`trials.csv`、試行別時系列とMCS集計、
設定・LUT・アンテナ入力のコピー、source snapshotとhash、内部のreporting notesを保存する。
結果ディレクトリは毎回新規作成する。

`timeouts`・`connections`・`disconnections` も試行別と平均・母分散・不偏分散を保存する。
`timeouts` は接続確立前の端点ごとの待機期限満了。A/B両端が満了した場合はそれぞれ1回と数える。
Beaconが届かず探索を続ける時間、個々のパケット失敗、再送はこの回数に含めない。
接続後のATP満了による切断は `disconnections` として別に記録する。

LUTを高精度化、またはDATA長や制御MCSを変える場合は別名で再生成し、設定の `theoretical_phy.table` を変更する。

```bash
OMP_NUM_THREADS=6 build/tjx_theory/tjx_generate_lut /tmp/tjx_lut_1000.yaml 1000 20261102 2048 3 -12 20 1
```

引数は出力先、各セル試行数、seed、DATA byte数、制御TJX MCS、Es/N0下限、上限、刻み。
既存LUTは上書きしない。生成完了後に検証してから.partialを本名へ変更する。

ROS/Gazebo側は通常のパッケージビルド後、同じ設定を `sim_launch.py` に渡す。

```bash
ros2 launch comms_sim_pkg sim_launch.py headless:=true \
  config_file:="$(pwd)/src/comms_sim_pkg/config/scenarios/drone_dual60_cross_fixed_theoretical.yaml"
```
