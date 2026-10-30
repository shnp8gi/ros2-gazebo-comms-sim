"""モニターのイベント記録を、指定したシミュレーション時刻の表示へ復元する。"""
from bisect import bisect_left, bisect_right
import json
import math
from pathlib import Path


def format_event_time(seconds):
    """時刻の大きさで単位を選び、ns精度までの不要な末尾0を省く。"""
    magnitude=abs(seconds)
    for scale,unit,decimals in ((1,'s',9),(1e3,'ms',6),(1e6,'µs',3),(1e9,'ns',0)):
        if magnitude==0 or magnitude*scale>=1 or unit=='ns':
            value=f'{seconds*scale:.{decimals}f}'
            if '.' in value:value=value.rstrip('0').rstrip('.')
            return f'{value} {unit}'


def advance_playback(seconds, elapsed, segments, duration):
    """区間ごとの速度で再生時刻を進める。区間境界を飛ばさず実経過を配分する。"""
    for segment in segments:
        end=min(float(segment['until_s']),duration)
        speed=float(segment['speed'])
        if speed<=0:
            raise ValueError('再生速度は正数で指定してください')
        if seconds>=end:
            continue
        needed=(end-seconds)/speed
        if elapsed<needed:
            return seconds+elapsed*speed
        seconds=end
        elapsed-=needed
    return min(seconds,duration)


class MonitorReplay:
    def __init__(self, snapshot):
        snapshot=Path(snapshot)
        self.final=json.loads(snapshot.read_text())
        self.hz=self.final['sample_rate_hz']
        self.duration=(self.final['sample']+1)/self.hz
        self.rows={name:[] for name in ('setup','stage','rssi','status')}
        with (snapshot.parent/'monitor_replay.jsonl').open() as stream:
            for line in stream:
                row=json.loads(line)
                if row[0] in self.rows:
                    self.rows[row[0]].append(row[1:])
        self.times={}
        for name,rows in self.rows.items():
            rows.sort(key=lambda row:row[0])
            self.times[name]=[row[0] for row in rows]

    def after_event(self, seconds, sample=None):
        """指定イベントまで処理済みになる再生時刻。次のサンプルは含めない。"""
        if sample is None:
            # TimeoutのCSV時刻は小数9桁。丸めでイベントより1〜2サンプル
            # 手前になるため、同じ丸め区間の精密な状態遷移sampleを優先する。
            estimate=seconds*self.hz
            tolerance=self.hz*0.5e-9+1e-4
            candidates=self.times['stage'][bisect_left(self.times['stage'],estimate-tolerance):
                                           bisect_right(self.times['stage'],estimate+tolerance)]
            sample=candidates[-1] if candidates else math.floor(estimate+tolerance)
        return min(self.duration,max(0,(sample+1)/self.hz))

    def at(self, seconds):
        seconds=max(0,min(seconds,self.duration))
        sample=max(0,int(round(seconds*self.hz))-1)
        data=dict(self.final)
        data.update(sample=sample,complete=seconds>=self.duration,replay=True,
                    setup_history=[],stage_history=[],rssi_history=[],predicted_signal_history=[],failures=[],power=[],iq=[],iq_last_a=[],iq_last_b=[],
                    state_a='SCANNING',state_b='SCANNING',protocol_stage='BEACON',
                    rssi_a_dbm=None,rssi_b_dbm=None,delivered_mb=0,connections=0,timeouts=0,disconnections=0)
        known={name:self.rows[name][:bisect_right(self.times[name],sample)] for name in self.rows}
        data['setup_history']=known['setup']
        data['stage_history']=known['stage']
        data['rssi_history']=[row[:3] for row in known['rssi'][-10000:]]
        data['rssi_valid_chips']=[0,0]
        # 未来の最終値を現在時刻へ混入させない。
        data['admission']={'known':False,'enabled':self.final.get('admission',{}).get('enabled',True)}
        for key in ('tx_a','tx_b','rx_a','rx_b'):data[key]='未収録'
        data['predicted_signal_a_dbm']=data['predicted_signal_b_dbm']=None
        for key in ('reception_ab','reception_ba','setup_ab','setup_ba'):
            data[key]={'known':False,'ok':False,'sample':0,'frame':'','reason':''}
        if known['status']:
            row=known['status'][-1]
            data.update(delivered_mb=row[4],connections=row[5],timeouts=row[6],disconnections=row[7])
            if len(row)>9:data.update(row[9])
        if known['setup']:
            row=known['setup'][-1]
            for key,value in zip(('setup_ab','setup_ba'),row[1:3]):
                data[key]={'known':value!=0,'ok':value==1,'sample':row[0],'frame':'記録済み判定','reason':''}
            if row[3]:data['state_a']=data['state_b']='ASSOCIATED'
            # statusは実時間で間引かれる。接続・切断はsample精度のsetup遷移から復元する。
            was_connected=False
            data['connections']=data['disconnections']=0
            for entry in known['setup']:
                connected=bool(entry[3])
                data['connections']+=connected and not was_connected
                data['disconnections']+=was_connected and not connected
                was_connected=connected
        if known['stage']:
            stage=known['stage'][-1][1];data['protocol_stage']=stage
            if stage=='DATA':data['state_a']=data['state_b']='ASSOCIATED'
            elif stage=='ACK':data['state_a'],data['state_b']='ASSOCIATED','WAIT_ACK'
            elif stage=='RESPONSE':data['state_a'],data['state_b']='WAIT_RESPONSE','WAIT_ACK'
        if known['status'] and (not known['stage'] or known['status'][-1][0]>=known['stage'][-1][0]):
            data['state_a'],data['state_b']=known['status'][-1][1:3]
        if known['rssi']:
            row=known['rssi'][-1]
            data['rssi_a_dbm'],data['rssi_b_dbm']=row[1:3]
            if self.final.get('rssi_source')!='rx_iq_matched_filter':
                data['predicted_signal_history']=data['rssi_history']
                data['rssi_history']=[]
                data['predicted_signal_a_dbm'],data['predicted_signal_b_dbm']=row[1:3]
                data['rssi_a_dbm']=data['rssi_b_dbm']=None
            if len(row)>=5:data['rssi_valid_chips']=row[3:5]
        return data
