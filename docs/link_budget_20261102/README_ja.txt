リンクバジェットから回路要求を検討するための暫定資料
作成日: 2026-11-02

1. この版の対象
対象構成と目標距離が未確定のため、drone_params.yamlの61 GHz／272 GHz構成を採用。
A=PRDEV、B=PRC。A→B=61 GHz、B→A=272 GHz。
specification.mdには逆の方向表記が残っているが、この資料は現行YAMLとテストに従う。
両方向60 GHzの構成を設計する場合には、その構成の利得・NF等で再計算が必要。

link_budget.csv: 既存設定に基づく各段の信号電力、雑音、SNR。Excel等で開ける。
circuit_requirements_example.csv: 必要SNR=10 dB、設計余裕=6 dBの仮定例。
source.txt: 入力設定のパス、SHA256、計算条件。
必要SNR=10 dBと余裕=6 dBは、規格値・実測値・確定要求ではない。
変調、FEC、目標BER/PER、フレーム長が決まったら必要SNRを差し替える。
現在のYAMLはrequired_snr_db=nullであり、この例のSNR閾値はシミュレーションに設定していない。

2. 基準条件
                  A→B                  B→A
周波数            61 GHz               272 GHz
送信出力          -8 dBm               -1 dBm
送信挿入損失      3 dB                 0 dB
送受信利得        各23 dBi             各24 dBi
LNA利得           0 dB                 25 dB
変換利得          0 dB                 3 dB
受信挿入損失      3 dB                 3 dB
受信系総利得      -3 dB                25 dB
受信系等価NF      0 dB（理想仮定）     10 dB（既存仮定）
TJX入力電力閾値   -68.5 dBm            -68.5 dBm
共通: 300 K、雑音帯域2.1648 GHz、自由空間伝搬、アンテナ正対。
送信出力は全距離で固定。原Excelの8/12/16 mシートの272 GHz送信出力-7 dBmとは異なる。
61 GHzのNF=0 dBは実機設計値ではなく、3 dBの受信損失による雑音劣化も再現していない。

3. 基準点と式
送信出力端 → 送信損失 → 送信アンテナ → 伝搬 → 受信アンテナ出力端
           → LNA → Mixer → 受信損失 → TJX入力端
これは現行モデルの計算順序。実機でのフィルタ・スイッチの配置は別途確定する。

PL = 20 log10(4πfd/c), c=299792458 m/s
P_ant = P_tx - L_tx + G_tx - PL + G_rx
G_chain = G_LNA + G_conversion - L_rx
P_TJX = P_ant + G_chain
N_thermal = 10 log10(kTB) + 30 = -80.473776833 dBm
N_ant = N_thermal + NF_total
N_TJX = N_ant + G_chain
SNR = P_ant - N_ant = P_TJX - N_TJX
電力閾値との差 = P_TJX - (-68.5)

信号と雑音は同一基準点で比較する。アンテナ端雑音と増幅後の信号を直接比較しない。
電力閾値との差が正でも、目標BER/PERを満たすとは限らない。
等価NFを一定にしたまま受信利得だけ増やしてもSNRは改善しない。
実際にはLNAの利得・NF変更で後段の雑音寄与も変わるため、回路段ごとの再評価が必要。

4. 回路要求の逆算
S: 復調器の実装劣化を含む必要SNR、M: 別途確保する設計余裕。
NF_max = P_ant - N_thermal - S - M
G_chain_min = P_TJX_threshold + M - P_ant
P_tx_min = P_tx + max(S + M - SNR, M - (P_TJX - P_TJX_threshold))
各式は他条件を現状に固定した個別の制約。NF上限と最小利得を同時に満たす必要がある。
P_tx_minはNF・受信利得を維持したときの送信出力端の値であり、EIRPではない。
G_chain_minは電力閾値だけから求めた下限であり、適切なLNA利得を単独では決定できない。
Mは両制約に対して一度ずつ適用し、合算して12 dBの余裕とはしない。

16 m、S=10 dB、M=6 dBという仮定例:
・61 GHz: NF_total <= 7.237 dB。現行NF=0 dBのまま部品を決めない。
・272 GHz: 現行SNR=12.252 dB、必要16 dBに対して3.748 dB不足。
  他条件一定なら、送信出力 >= +2.748 dBm、またはNF_total <= 6.252 dB。
  アンテナ利得の改善や帯域・変調方式の変更も候補だが、要求を再計算する。
  NF改善と送信出力改善を分担する案もある。
・272 GHzの4 mでTJX入力は-21.180 dBm、16 mで-33.222 dBm。
  電力下限に加え、TJX許容最大入力、各段の圧縮、AGC範囲を確認する。
  最短距離が4 m未満なら、最大入力についてその距離も評価する。

5. 回路設計へ展開する順序
(1) 対象構成、最大・最小距離、正味データレート、変調/FEC、BER/PERを確定する。
    PERにはフレーム長と再送条件も必要。3.52 Gbit/s等のPHYレートと正味速度を区別する。
    Eb/N0から換算する場合はSNR=Eb/N0+10log10(Rb/B)。情報ビット/符号化ビットの定義を揃える。
    原Excelの9.8 dBを必要SNRとして直接転用しない。
(2) 姿勢によるアンテナ利得低下、偏波、実装損失、大気・伝搬損失を見積もる。
    正対時の結果を移動中の保証値として扱わず、角度別・軌道別の最悪条件を評価する。
    個別に計上した劣化を設計余裕へ重複加算しない。
(3) 送信側を、変調器/変換器 → PA → フィルタ・給電 → アンテナに分解する。
    各段の入出力、帯域、P1dB、変調波のバックオフ、EVM、消費電力を割り当てる。
    必要平均出力だけではPAのP1dBを決められない。波形のピークと線形性が必要。
(4) 受信側を、アンテナ → 前置損失 → LNA → 周波数変換 → IF/VGA → TJXに分解する。
    各段の利得、NF、圧縮点、帯域、インピーダンスを記入する。
    上記は検討用の構成例であり、特定周波数で部品が実現可能と確認した回路図ではない。
(5) LO周波数計画、位相雑音、イメージ、漏洩、フィルタ、温度・製造ばらつきを評価する。
    部品選定後、Sパラメータや雑音モデルで検証し、各基準点の測定でモデルを更新する。

6. NFの扱い
周波数変換を含まない適切に整合した多段系では、線形値で
F_total = F1 + (F2-1)/G1 + (F3-1)/(G1*G2) + ...
F=10^(NF/10)、G=10^(gain_dB/10)として各段の寄与を評価する。
基準温度と同温度の受動損失ではNF[dB]=挿入損失[dB]。
Mixerを含む場合にはSSB/DSBの定義、イメージ帯雑音、フィルタを合わせて扱う。
カタログのMixer NFを無条件に上の式へ代入しない。
参考: Analog Devices, System Noise-Figure Analysis for Modern Radio Receivers
https://www.analog.com/en/resources/technical-articles/system-noisefigure-analysis-for-modern-radio-receivers.html

CSVは既存コードとの一致のため300 KでkTB+NFを用いる。
一般的な部品NFの基準温度T0=290 Kとアンテナ雑音温度Taを分離する設計では、
入力換算雑音はkB[Ta + T0(F_total-1)]で扱う。CSVの等価NFと定義を混同しない。

7. 再生成
リポジトリ直下で以下を実行する。出力先は新しいディレクトリを指定する。
python3 tools/create_circuit_link_budget.py --output-dir /tmp/circuit_budget_new
距離変更例: --distances 4 8 12 16 30
仮定変更例: --example-required-snr-db 12 --example-design-margin-db 3
別設定: --config 設定.yaml
この補助ツールは正面アンテナ利得をYAMLに明示した設定専用。
CSV指向性の合成、軌道、PHY/MAC成否は扱わない。
