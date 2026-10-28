#!/usr/bin/env python3
"""仮フレームの長さから送信時間を見積もる。PHY波形・規格準拠の受信性能は生成しない。"""
import argparse,csv,math
from pathlib import Path
import yaml

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',type=Path,default=Path(__file__).with_name('provisional_profile.yaml'))
    parser.add_argument('--output',type=Path,default=Path(__file__).with_name('timing_budget.csv'))
    args=parser.parse_args()
    c=yaml.safe_load(args.profile.read_text());p=c['phy_targets'];a=c['lab_timing_assumptions']
    assert sum(p['ab']['phy_header_fields_bits'].values())==p['ab']['phy_header_bits']
    assert sum(c['lab_mac_layout']['fields_bits'].values())==c['lab_mac_layout']['total_bits']
    for name,fields in c['lab_body_layouts'].items():
        assert sum(fields.values())==a['control_body_bytes'][name]*8
    rows=[]
    for mode in a['timing_budget_modes']:
        for frame,direction,phase in [('BEACON','ba','setup'),('REQUEST','ab','setup'),('RESPONSE','ba','setup'),('ACK','ab','setup'),('DATA','ab','data'),('DATA_ACK','ba','data')]:
            body=a['data_body_bytes'] if frame=='DATA' else a['control_body_bytes'][frame]
            raw=body*8+(a['payload_crc_bits_if_body_present'] if body else 0)
            n,k=p['ab']['payload_ldpc_n'],p['ab']['payload_ldpc_k']
            blocks=math.ceil(raw/k) if mode=='ldpc_length_budget_only' else 0
            coded=blocks*n if mode=='ldpc_length_budget_only' else raw
            h=a[f'header_information_bits_{direction}']
            hb=math.ceil(h/a['header_code_k'])*a['header_code_n']
            pre=a['preamble_chips'][f'{direction}_{phase}']
            symbols=pre+math.ceil(hb/a['header_bits_per_symbol'])+math.ceil(coded/a['data_payload_bits_per_symbol'])+a['tx_extra_symbols']
            samples=symbols*p['samples_per_symbol']
            seconds=symbols/p['symbol_rate_hz']
            rows.append(dict(mode=mode,frame=frame,direction=direction,body_bytes=body,
                header_information_bits=h,header_coded_bits=hb,body_with_crc_bits=raw,
                ldpc_blocks=blocks,payload_coded_bits=coded,preamble_chips=pre,
                total_symbols=symbols,total_samples=samples,tx_us=seconds*1e6,
                status='仮定プロファイルの算術結果・規格値でも現行実装値でもない'))
    with args.output.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
    for mode in a['timing_budget_modes']:
        setup=sum(r['tx_us'] for r in rows if r['mode']==mode and r['frame'] in ('BEACON','REQUEST','RESPONSE','ACK'))+3*a['ifs_us']
        print(f'{mode}: 接続手順（伝搬・追加処理遅延なし）{setup:.6f} us')
    print(f'保存: {args.output}（受信PERや符号化利得は計算していない）')

if __name__=='__main__':main()
