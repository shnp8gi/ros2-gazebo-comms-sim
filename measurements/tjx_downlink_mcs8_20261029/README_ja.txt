【使用対象外】ユーザーが前回測定は誤りと申告し再測定したため、校正・シミュレーションへの利用対象から除外。原本コピーと旧集計は履歴として保存。

実測データの確定条件と取り込み（2026-10-29）

ユーザー確認済み:
 下り60.48 GHz、ペイロード100 kB、clear write。
 全CSVのMCS列は8。
 現在のradio_ab.frequency_hz=60.48e9によりQ_A_TO_Bへ対応付ける。
 100 kBはTJXの十進表記に合わせ100000 bytesと解釈して保存。
 DATA分類はPHY試験ペイロード用。実MACのDATAパケットを測定したという意味ではない。

raw/: ダウンロード原本34ファイルの内容をそのままコピー。
manifest.csv: 確認した条件。絶対電力reference_rssi_dbmは未記入。
measurement_conditions.json: 条件、ファイル名解釈、出典。
per_relative.csv: 全行のPERをpacket（検出数）で重み付けした近似。
ber_relative.csv: 全行のBERをpacket（検出数）で重み付け。
各provenance.json: 元CSVのSHA256と採用行数、重み付け方法。
receiver_curves.png/.pdf: 確定条件でのグラフ。plot.pyで再描画できる。

clear writeでは最後の行だけを採用しない。全34条件4900行を使用。
BERは一定長であるため検出数による重み付けで統合できる（CSVの丸め精度内）。
FEC前後は不明のまま残す。BERから独立誤りを仮定したPERの妥当性は未検証。
PERの本来の分母は検出数+ロスト数。標準7列CSVにロスト数がないため、
検出数を重みとする近似を明示。厳密な合算PERとは呼ばない。

全区間を用いた近似PER:
 減衰12 dB: 約0.1931%
 減衰12.5 dB: 約2.1543%
 減衰13 dB: 約24.8585%
 減衰13.5 dB: 約98.4439%
 減衰14 dB以降: 100%
 最初の中央値による確認と、今回の重み付き集計は別の統計量。

相対電力は基準減衰0 dBからの追加減衰に負号を付けた値。
relative_power_dbをrssi_dbmとして使わない。生RSSIもdBmとして使わない。
基準位置での受信電力が決まった後、manifest.csvのreference_rssi_dbmへ
記入して--relative-powerなしで変換する。仮の基準値を使う場合は仮定と明記する。
今回の表を他方向・他MCS・異なる長さへ無条件に流用しない。
既存デモの設定・実測未校正という表示は変更していない。

再生成（リポジトリルートから。既存出力は上書きしないため別名を指定）:
 python3 tools/prepare_receiver_curve.py measurements/tjx_downlink_mcs8_20261029/manifest.csv 任意の新規PER表.csv --metric per --relative-power --allow-detect-weighted-per
 python3 tools/prepare_receiver_curve.py measurements/tjx_downlink_mcs8_20261029/manifest.csv 任意の新規BER表.csv --metric ber --relative-power --allow-unknown-ber

変換器は従来のaveragedモードを維持。clear_writeのPER厳密合算には
packet_lost列を追加した入力も扱える。近似PERはフラグ指定時だけ許可する。
テスト: python3 tools/test_prepare_receiver_curve.py（3件成功）
