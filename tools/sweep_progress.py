#!/usr/bin/env python3
"""
sweep_progress.py
-----------------
スイープシミュレーションの進捗を確認するスクリプト。
sweep_sim.py が出力するプログレスログとCSV結果から進捗を可視化します。
高負荷時の更新遅延に耐性を持たせ、SOLID原則に基づいた設計にリファクタリングされています。
"""

import argparse
import datetime
import glob
import json
import os
import re
import shutil
import subprocess
import sys
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

# モジュール読み込み用のパス設定
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(script_dir)
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

# 総タスク数は「今走っている sweep が状態ファイルに書いた total_tasks」を唯一の
# 真実とする。実行前 (状態ファイルなし) は総数を推定せず「未実行」扱いにする。
# 以前はレガシー lib/sweep_config の静的既定 (Y_POSITIONS×ANGLES や 760) を
# フォールバックにしていたが、シナリオ非追従で誤解を招くため撤去した。
TOTAL_TASKS = 0  # 0 = 不明 (状態ファイルが total_tasks を持つまで表示しない)

try:
    import lib.system_monitor as sysmon
    HAS_SYSMON = True
except ImportError:
    HAS_SYSMON = False

# ==========================================
# 1. Domain Models (データ構造)
# ==========================================
@dataclass
class TaskProgress:
    task_no: int
    params_str: str
    started_at: Optional[datetime.datetime]
    worker_id: Optional[int]
    progress_val: float = 0.0
    run_sec: int = 0

@dataclass
class SweepState:
    completed: int = 0
    total_tasks: int = 0
    concurrency: Optional[int] = None
    start_time: Optional[datetime.datetime] = None
    last_updated: Optional[datetime.datetime] = None
    running_tasks: Dict[int, TaskProgress] = field(default_factory=dict)
    task_durations: List[float] = field(default_factory=list)
    sweep_dir_name: str = ""
    scenario_name: str = ""
    is_missing: bool = False
    completed_runs_fallback: int = 0
    latest_csv_fallback: Optional[str] = None

@dataclass
class SystemUsage:
    cpu_pct: float = 0.0
    mem_used_gb: float = 0.0
    mem_total_gb: float = 0.0
    mem_pct: float = 0.0
    gpus: List[Dict[str, Any]] = field(default_factory=list)

@dataclass
class AnalysisResult:
    state: SweepState
    is_stale: bool
    is_zombie: bool
    is_lagging: bool
    delay_sec: float
    progress_pct: float
    remaining_tasks: int
    elapsed_sec: float
    tasks_per_min: float
    eta_sec: float
    avg_duration: float

# ==========================================
# 2. Interfaces (抽象層)
# ==========================================
class IStateRepository(ABC):
    @abstractmethod
    def fetch_state(self) -> SweepState:
        """シミュレーションの状態を取得する"""
        pass

class ISystemMonitor(ABC):
    @abstractmethod
    def get_usage(self) -> SystemUsage:
        """システムリソースの使用状況を取得する"""
        pass

class IWorkerProgressReader(ABC):
    @abstractmethod
    def read(self, worker_id: int, sweep_dir: str) -> Tuple[Optional[float], Optional[float]]:
        """ワーカーの進捗率[%]とログ更新時刻(mtime)を返す。取得できない項目は None。"""
        pass

# ==========================================
# 3. Implementations (実装層)
# ==========================================
class LocalSystemMonitor(ISystemMonitor):
    def get_usage(self) -> SystemUsage:
        usage = SystemUsage()
        if HAS_SYSMON:
            usage.cpu_pct = sysmon.get_cpu_usage()
            usage.mem_used_gb, usage.mem_total_gb, usage.mem_pct = sysmon.get_memory_usage()
        
        usage.gpus = self._get_gpu_usage()
        return usage

    def _get_gpu_usage(self) -> List[Dict[str, Any]]:
        if not shutil.which("nvidia-smi"):
            return []
        try:
            res = subprocess.run(
                ["nvidia-smi", "--query-gpu=name,utilization.gpu,utilization.memory,memory.used,memory.total", "--format=csv,noheader,nounits"],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=1
            )
            gpus = []
            if res.returncode == 0:
                for line in res.stdout.strip().split("\n"):
                    if not line.strip(): continue
                    parts = [p.strip() for p in line.split(",")]
                    if len(parts) >= 5:
                        gpus.append({
                            "name": parts[0],
                            "gpu_util": float(parts[1]),
                            "mem_util": float(parts[2]),
                            "mem_used": float(parts[3]),
                            "mem_total": float(parts[4])
                        })
            return gpus
        except Exception:
            return []

class WorkerLogProgressReader(IWorkerProgressReader):
    """ワーカーの stdout ログ末尾から進捗率を解析する。責務: ログの読取・解析のみ。"""

    PROGRESS_PATTERN = re.compile(r'PROGRESS:\s*([\d\.]+)%')
    TAIL_READ_BYTES = 4096

    def __init__(self, log_base_dir: str):
        self.log_base_dir = log_base_dir

    def read(self, worker_id: int, sweep_dir: str) -> Tuple[Optional[float], Optional[float]]:
        if worker_id is None or not sweep_dir:
            return None, None
        stdout_file = os.path.join(self.log_base_dir, sweep_dir, f"stdout_worker_{worker_id}.log")
        if not os.path.exists(stdout_file):
            return None, None

        progress_val = None
        mtime = None
        try:
            mtime = os.path.getmtime(stdout_file)
            with open(stdout_file, 'rb') as f:
                f.seek(0, 2)
                filesize = f.tell()
                read_size = min(self.TAIL_READ_BYTES, filesize)
                f.seek(-read_size, 2)
                lines = f.read().decode('utf-8', errors='ignore').splitlines()
            for line in reversed(lines):
                m = self.PROGRESS_PATTERN.search(line)
                if m:
                    progress_val = float(m.group(1))
                    break
        except Exception:
            pass
        return progress_val, mtime

class JsonStateRepository(IStateRepository):
    def __init__(self, log_path: str, project_root_path: str,
                 progress_reader: IWorkerProgressReader = None,
                 default_total_tasks: int = 0):
        self.log_path = log_path
        self.project_root = project_root_path
        self.progress_reader = progress_reader
        self.default_total_tasks = default_total_tasks

    def _make_missing_state(self) -> SweepState:
        completed_fb, csv_fb = self._count_csv_rows()
        return SweepState(is_missing=True, total_tasks=self.default_total_tasks,
                          completed_runs_fallback=completed_fb, latest_csv_fallback=csv_fb)

    def _parse_dt(self, dt_str: Optional[str]) -> Optional[datetime.datetime]:
        if not dt_str: return None
        try:
            return datetime.datetime.fromisoformat(dt_str)
        except ValueError:
            return None

    def _count_csv_rows(self) -> Tuple[int, Optional[str]]:
        sim_results_dir = os.path.join(self.project_root, "sim_results")
        csvs = sorted(glob.glob(os.path.join(sim_results_dir, "sweep_*", "sweep_summary_run*.csv")))
        if not csvs:
            csvs = sorted(glob.glob(os.path.join(sim_results_dir, "sweep_summary_*.csv")))
        if not csvs:
            return 0, None
        latest = csvs[-1]
        run_ids = set()
        try:
            with open(latest, encoding="utf-8") as f:
                for i, line in enumerate(f):
                    if i == 0: continue
                    cols = line.strip().split(",")
                    if cols and cols[0]:
                        run_ids.add(cols[0])
        except Exception:
            pass
        return len(run_ids), latest

    def fetch_state(self) -> SweepState:
        if not os.path.exists(self.log_path):
            return self._make_missing_state()

        try:
            with open(self.log_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except Exception:
            return self._make_missing_state()

        state = SweepState()
        state.completed = data.get("completed", 0)
        state.total_tasks = data.get("total_tasks") or self.default_total_tasks
        state.concurrency = data.get("concurrency")
        state.start_time = self._parse_dt(data.get("start_time"))
        state.last_updated = self._parse_dt(data.get("last_updated"))
        state.task_durations = data.get("task_durations", [])
        state.sweep_dir_name = data.get("sweep_dir_name", "")
        state.scenario_name = data.get("scenario_name", "")

        raw_tasks = data.get("running_tasks", {})
        max_worker_mtime = None
        for k, v in raw_tasks.items():
            try:
                task_id = int(k)
                tp = TaskProgress(
                    task_no=v.get("task_no", 0),
                    params_str=v.get("params_str", ""),
                    started_at=self._parse_dt(v.get("started_at")),
                    worker_id=v.get("worker_id")
                )
                if self.progress_reader is not None:
                    progress_val, mtime = self.progress_reader.read(tp.worker_id, state.sweep_dir_name)
                    if progress_val is not None:
                        tp.progress_val = progress_val
                    if mtime:
                        dt_mtime = datetime.datetime.fromtimestamp(mtime)
                        if max_worker_mtime is None or dt_mtime > max_worker_mtime:
                            max_worker_mtime = dt_mtime

                state.running_tasks[task_id] = tp
            except ValueError:
                pass

        if max_worker_mtime:
            if state.last_updated is None or max_worker_mtime > state.last_updated:
                state.last_updated = max_worker_mtime

        return state

# ==========================================
# 4. Business Logic (ドメイン層)
# ==========================================
class ProgressAnalyzer:
    def __init__(self, zombie_timeout_sec: float = 900.0, lag_threshold_sec: float = 30.0):
        # SRP: タイムアウトの判定閾値の管理と、それに基づく状態の決定
        self.zombie_timeout_sec = zombie_timeout_sec
        self.lag_threshold_sec = lag_threshold_sec

    def analyze(self, state: SweepState) -> AnalysisResult:
        now = datetime.datetime.now()
        is_lagging = False
        is_stale = False
        is_zombie = False
        delay_sec = 0.0

        # 実行経過秒は「現在時刻に依存する導出値」なのでここで計算する(Repositoryは取得のみ)
        for tp in state.running_tasks.values():
            if tp.started_at:
                tp.run_sec = int((now - tp.started_at).total_seconds())

        if state.last_updated:
            delay_sec = (now - state.last_updated).total_seconds()
            if delay_sec > self.zombie_timeout_sec:
                if state.running_tasks:
                    is_zombie = True
                else:
                    is_stale = True
            elif delay_sec > self.lag_threshold_sec:
                is_lagging = True

        total = state.total_tasks if state.total_tasks > 0 else 1
        pct = state.completed / total * 100
        remaining = max(0, state.total_tasks - state.completed)

        elapsed_sec = 0.0
        if state.start_time:
            if state.completed == state.total_tasks and state.last_updated:
                elapsed_sec = (state.last_updated - state.start_time).total_seconds()
            else:
                elapsed_sec = (now - state.start_time).total_seconds()

        tasks_per_min = 0.0
        eta_sec = 0.0
        if state.completed > 0:
            spt = elapsed_sec / state.completed
            eta_sec = spt * remaining
            if spt > 0:
                tasks_per_min = 60.0 / spt

        avg_duration = 0.0
        if state.task_durations:
            avg_duration = sum(state.task_durations) / len(state.task_durations)

        return AnalysisResult(
            state=state,
            is_stale=is_stale,
            is_zombie=is_zombie,
            is_lagging=is_lagging,
            delay_sec=delay_sec,
            progress_pct=pct,
            remaining_tasks=remaining,
            elapsed_sec=elapsed_sec,
            tasks_per_min=tasks_per_min,
            eta_sec=eta_sec,
            avg_duration=avg_duration
        )

# ==========================================
# 5. UI/Presentation Layer (描画層)
# ==========================================
class ConsoleRenderer:
    def _clear_screen(self):
        if os.name == "nt":
            os.system("cls")
        else:
            sys.stdout.write("\033[H\033[2J")
            sys.stdout.flush()

    def _make_bar(self, percent: float, width: int = 40) -> str:
        filled = int(width * percent / 100)
        filled = max(0, min(width, filled))
        return "█" * filled + "░" * (width - filled)

    def render(self, sys_usage: SystemUsage, analysis: AnalysisResult):
        self._clear_screen()
        now = datetime.datetime.now()
        print("=" * 75)
        print(f"       🚀 Sweep Sim Progress Monitor  [{now.strftime('%Y-%m-%d %H:%M:%S')}]")
        print("=" * 75)

        self._render_system(sys_usage)
        
        state = analysis.state
        if state.is_missing:
            print(f"\n  💤 未実行 — sweep はまだ開始されていません (進捗状態ファイルなし)。")
            if state.completed_runs_fallback:
                print(f"      参考: 直近CSVに {state.completed_runs_fallback} run_id 分の結果あり")
            if state.latest_csv_fallback:
                print(f"      最新CSV: {os.path.basename(state.latest_csv_fallback)}")
            print("\n  sweep を開始すると総タスク数と進捗がここに表示されます。")
            print("=" * 75)
            return

        if analysis.is_stale:
            print(f"\n  💤 現在実行中のシミュレーションはありません。")
            if state.last_updated:
                print(f"      (前回のスイープ終了/中断時刻: {state.last_updated.strftime('%Y-%m-%d %H:%M:%S')})")
            print("\n" + "=" * 75)
            return

        self._render_progress(analysis)
        self._render_eta(analysis, now)
        self._render_tasks(analysis)

        print("\n" + "=" * 75)

    def _render_system(self, sys_usage: SystemUsage):
        print("\n  💻 システムリソース状況:")
        cpu_bar = self._make_bar(sys_usage.cpu_pct, width=20)
        print(f"    CPU 使用率:  [{cpu_bar}] {sys_usage.cpu_pct:5.1f}%")
        
        if sys_usage.mem_total_gb > 0:
            mem_bar = self._make_bar(sys_usage.mem_pct, width=20)
            print(f"    メモリ使用量: [{mem_bar}] {sys_usage.mem_used_gb:5.1f} / {sys_usage.mem_total_gb:.1f} GiB ({sys_usage.mem_pct:.1f}%)")
        else:
            print("    メモリ使用量: 取得できませんでした")

        if sys_usage.gpus:
            print("\n    GPU 状況:")
            for idx, gpu in enumerate(sys_usage.gpus):
                gpu_bar = self._make_bar(gpu['gpu_util'], width=15)
                gpu_mem_pct = (gpu['mem_used'] / gpu['mem_total'] * 100) if gpu['mem_total'] > 0 else 0
                gpu_mem_bar = self._make_bar(gpu_mem_pct, width=15)
                print(f"      GPU #{idx} [{gpu['name']}]:")
                print(f"        負荷: [{gpu_bar}] {gpu['gpu_util']:5.1f}%")
                print(f"        メモリ: [{gpu_mem_bar}] {gpu['mem_used']:.0f} / {gpu['mem_total']:.0f} MiB ({gpu_mem_pct:.1f}%)")

    def _render_progress(self, analysis: AnalysisResult):
        state = analysis.state
        if state.scenario_name:
            print(f"\n  🗺️ シナリオ: {state.scenario_name}")
        if state.total_tasks > 0:
            bar = self._make_bar(analysis.progress_pct, width=40)
            print(f"\n  📊 進捗: {state.completed:4d} / {state.total_tasks} タスク  ({analysis.progress_pct:.1f}%)")
            print(f"  [{bar}]")
            print(f"  残り: {analysis.remaining_tasks} タスク")
        else:
            print(f"\n  📊 進捗: {state.completed} タスク完了 (総数不明)")
        
        if analysis.is_lagging:
            print(f"  ⚠️ [遅延検知] プログレスの更新が滞っています ({int(analysis.delay_sec)}秒間更新なし)")
            print(f"      (システム高負荷によりロガーの書き込みが遅れている可能性があります)")

    def _render_eta(self, analysis: AnalysisResult, now: datetime.datetime):
        state = analysis.state
        print("\n  ⏱️ 時間計測 & 進捗速度:")
        if state.start_time:
            elapsed_td = datetime.timedelta(seconds=int(analysis.elapsed_sec))
            print(f"    開始時刻 : {state.start_time.strftime('%Y-%m-%d %H:%M:%S')}")
            print(f"    経過時間 : {str(elapsed_td)}")
            
            if state.completed > 0:
                print(f"    処理速度 : 全体スループット {analysis.tasks_per_min:.1f} タスク/分")
                if analysis.avg_duration > 0:
                    print(f"    シミュレーション平均実行時間: {analysis.avg_duration:.1f} 秒 (1インスタンス単体)")
                
                if state.completed == state.total_tasks and state.last_updated:
                    print(f"    推定残り時間: 0:00:00")
                    print(f"    完了時刻: {state.last_updated.strftime('%Y-%m-%d %H:%M:%S')}")
                else:
                    eta_td = datetime.timedelta(seconds=int(analysis.eta_sec))
                    eta_dt = now + datetime.timedelta(seconds=analysis.eta_sec)
                    print(f"    推定残り時間: {str(eta_td)}")
                    print(f"    推定完了時刻: {eta_dt.strftime('%Y-%m-%d %H:%M:%S')}")
            else:
                print("    処理速度 : 算出中 (最初のタスク完了待ち)...")
        else:
            print("    開始時刻 : ログに記録されていません")

    def _render_tasks(self, analysis: AnalysisResult):
        state = analysis.state
        if analysis.is_zombie:
            print(f"\n  ❌ 異常終了 (ABORTED) を検知しました。シミュレーションプロセスが停止した可能性があります。")
            print(f"      ({int(analysis.delay_sec)}秒以上更新がありません)")
            return

        limit_str = f" / 最大並列数: {state.concurrency}" if state.concurrency else ""
        print(f"\n  🔄 実行中の並列タスク ({len(state.running_tasks)} 個のアクティブワーカー{limit_str}):")
        
        if state.running_tasks:
            for t_id in sorted(state.running_tasks.keys()):
                tp = state.running_tasks[t_id]
                prog_bar = self._make_bar(tp.progress_val, width=20)
                print(f"    ▶ タスク #{tp.task_no:3d} | {tp.params_str} | 実行時間: {tp.run_sec:3d}秒 | 進捗: [{prog_bar}] {tp.progress_val:5.1f}%")
        elif state.completed == state.total_tasks:
            print("    ✅ 全タスク完了しました!")
        else:
            print("    💤 現在実行中のタスクはありません（待機中または終了中）")

# ==========================================
# 6. Application Orchestration (結合層)
# ==========================================
class MonitorApplication:
    def __init__(self, repo: IStateRepository, monitor: ISystemMonitor, analyzer: ProgressAnalyzer, renderer: ConsoleRenderer):
        self.repo = repo
        self.monitor = monitor
        self.analyzer = analyzer
        self.renderer = renderer

    def run_once(self):
        state = self.repo.fetch_state()
        sys_usage = self.monitor.get_usage()
        analysis = self.analyzer.analyze(state)
        self.renderer.render(sys_usage, analysis)

    def watch(self, interval: float):
        try:
            while True:
                self.run_once()
                print(f"  (Ctrl+C で終了。{interval}秒ごとに更新)")
                time.sleep(interval)
        except KeyboardInterrupt:
            print("\n進捗監視を終了します。")

def main():
    parser = argparse.ArgumentParser(description="Sweep sim progress monitor")
    parser.add_argument("--watch", action="store_true", help="自動更新モード")
    parser.add_argument("--interval", type=float, default=5.0, help="--watch 時の更新間隔 (秒, デフォルト 5)")
    parser.add_argument("--log-file", default=None, help="進捗ログファイルのパス (デフォルト: 最新のログファイル)")
    parser.add_argument("--timeout", type=float, default=900.0, help="ゾンビ判定までのタイムアウト秒数 (高負荷時は大きくする)")
    args = parser.parse_args()

    log_file = args.log_file or os.path.join(project_root, "tools", "log", ".latest_sweep_state.json")

    # Dependency Injection
    progress_reader = WorkerLogProgressReader(log_base_dir=os.path.join(project_root, "tools", "log"))
    repo = JsonStateRepository(log_path=log_file, project_root_path=project_root,
                               progress_reader=progress_reader,
                               default_total_tasks=TOTAL_TASKS)
    monitor = LocalSystemMonitor()
    analyzer = ProgressAnalyzer(zombie_timeout_sec=args.timeout)
    renderer = ConsoleRenderer()
    
    app = MonitorApplication(repo, monitor, analyzer, renderer)

    if args.watch:
        app.watch(args.interval)
    else:
        app.run_once()

if __name__ == "__main__":
    main()
