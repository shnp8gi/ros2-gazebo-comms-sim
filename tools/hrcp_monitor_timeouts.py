"""TIMEOUTの正確なイベント時刻と、再生時の短時間ハイライト。"""
from bisect import bisect_right
import csv
import os
from pathlib import Path


class TimeoutEvents:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.rows = []
        self.offset = 0
        self.header = None
        self.finished = False
        cache = self.directory / 'timeout_events.csv'
        if cache.exists() and (self.directory/'completed').exists():
            with cache.open() as stream:
                self.rows = [{**r, 'time_s': float(r['time_s'])} for r in csv.DictReader(stream)]
            self.rows.sort(key=lambda r: r['time_s'])
            self.finished = True

    def refresh(self):
        if self.finished:return
        source = self.directory/'events.csv'
        if not source.exists():return
        with source.open('rb') as stream:
            stream.seek(self.offset)
            while True:
                start = self.offset
                line = stream.readline()
                if not line or not line.endswith(b'\n'):
                    self.offset = start
                    break
                self.offset += len(line)
                if self.header is None:
                    self.header = next(csv.reader([line.decode('utf-8-sig')]))
                elif b',TIMEOUT,' in line:
                    row = dict(zip(self.header, next(csv.reader([line.decode('utf-8')]))))
                    if row.get('event') == 'TIMEOUT':
                        self.rows.append(dict(time_s=float(row['time_s']), frame=row.get('frame',''),
                                              direction=row.get('direction',''), reason=row.get('reason','')))
        self.rows.sort(key=lambda r: r['time_s'])
        if (self.directory/'completed').exists():
            summary=self.directory/'summary.csv'
            if summary.exists():
                with summary.open() as stream:expected=int(next(csv.DictReader(stream))['timeouts'])
                if len(self.rows)!=expected:raise ValueError('Timeout索引とサマリーの件数が一致しません')
            # 完了済み記録は一度抽出したコンパクトな索引を以後の再生でも使う。
            temp = self.directory/f'.timeout_events.{os.getpid()}.tmp'
            with temp.open('w', newline='') as stream:
                writer=csv.DictWriter(stream,fieldnames=['time_s','frame','direction','reason'])
                writer.writeheader();writer.writerows(self.rows)
            temp.replace(self.directory/'timeout_events.csv')
            self.finished=True


class TimeoutHighlight:
    def __init__(self, hold_seconds=2.0):
        self.hold_seconds=hold_seconds
        self.previous=None
        self.until=0.0
        self.index=None

    def update(self, times, seconds, wall, playing):
        count=bisect_right(times,seconds+1e-9)
        index=count-1
        exact=index>=0 and abs(times[index]-seconds)<1e-6
        crossing=(playing and self.previous is not None and seconds>self.previous and
                  index>=0 and times[index]>self.previous+1e-10)
        moved=self.previous is None or seconds!=self.previous
        if self.previous is not None and seconds<self.previous:
            self.index=None;self.until=0.0
        if crossing or (exact and moved):
            self.index=index;self.until=wall+self.hold_seconds
        elif moved and not playing and not exact:
            self.index=None;self.until=0.0
        self.previous=seconds
        return self.index if wall<self.until else None
