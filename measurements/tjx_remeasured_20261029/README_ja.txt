コネクター接続方法修正後の実測（2026-10-29）

今回を使用対象とし、旧measurements/tjx_downlink_mcs8_20261029は使用対象外。
旧データの誤りの原因はコネクターの接続方法（ユーザー申告）。
旧ファイルは履歴として保持し、INVALIDATED_ja.txtとメタデータで明示した。

ユーザー確認済み条件:
 下り60.48 GHz、100 kB、clear write。MCS間の送信電力・配線は同じ。
 上り下りの機器は入れ替わっている。同型だが個体IDは不明。
 現設定の周波数により論理方向Q_A_TO_Bへ対応付ける。
 今回の機器の組み合わせに対する測定であり、同型機器間の個体差ゼロは仮定しない。
 100 kBはTJXの表記に合わせ100000 bytesとして扱う。

84条件、9495行:
 MCS 3: 13条件（45〜60 dB）
 MCS 6: 9条件（45〜55 dB）
 MCS 8: 8条件（45〜50 dB）
 MCS 9: 16条件（35〜45 dB）
 MCS 11: 38条件（20〜40 dB）
 MCS 0は今回含まれない。
 全行のMCS番号は格納先フォルダと一致。全数値は有限、各ファイル内時刻は増加。
 元CSVをraw/へ内容変更なしでコピーし、SHA256の一致を確認。
 MCS6/48.0d.csvはファイル名から48.0 dBと解釈した。

集計:
 clear writeの全行を使い、BERを検出パケット数で重み付け。
 PERも検出数で重み付けするが、本来の分母に含まれるロスト数がCSVにないため近似。
 BERのFEC前後は不明。BER→PERの独立ビット誤り仮定の正しさは未検証。
 per_relative.csv / ber_relative.csvは減衰量から求めた相対電力表。
 relative_power_dbはdBmではない。シミュレーターへ直接渡さない。

結果の例（検出数重み付き近似PER）:
 MCS 3: 55.0 dBで約20.4%、56.3 dBで100%。
 MCS 6: 51.3 dBで約3.60%、52.5 dBで100%。
 MCS 8: 48.0 dBで約37.6%、48.6 dBで約99.7%。
 MCS 9: 41.5 dBで約55.5%、42.0 dBで約94.8%。
 MCS 11: 34.0 dBで約51.5%、35.5 dBで約97.4%。
 MCS 11は20〜27.5 dBでもPER約3.55〜4.45%あり、ゼロへ補正していない。
 原因はこのデータだけでは断定しない。RSSIとSNRもグラフに残す。
 MCS 8の最小減衰45 dBでも僅かな誤りがあり、完全なゼロ点と扱わない。

保存物:
 manifest.csv: 方向・長さ・減衰量・MCS・元CSVを対応付けた条件表。
 measurement_summary.csv: 各測定点の集計と表示値範囲。
 measurement_conditions.json: 確定条件、機器入れ替え、出典とハッシュ。
 per_relative.csv / ber_relative.csv: 相対曲線と各provenance.json。
 receiver_curves.png / .pdf: 5つのMCSのPER/BER/RSSI/SNR。
 analyze.py: 元データの監査・コピー・集計・作図。曲線は既存なら上書き拒否。

残る調整:
 絶対受信電力の基準点は不明。分かった時点でmanifestのreference_rssi_dbmを
 記入し、tools/prepare_receiver_curve.pyを--relative-powerなしで別名出力する。
 上り、MCS 0、短い制御パケットは今回の測定で校正済みとはしない。
 ②のデモ設定は人工データのまま保持。実測への全面差し替え・Gazebo実行はしていない。
