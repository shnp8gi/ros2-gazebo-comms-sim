import os
import signal
import time
try:
    import psutil
except ImportError:
    psutil = None

class ProcessManager:
    """
    Encapsulates logic for finding, tracking, and cleanly terminating 
    worker processes or lingering zombie processes in the system.
    """
    
    @staticmethod
    def get_worker_processes(proc, worker_id: int) -> list:
        """Finds all processes associated with a worker, using child-tree and command line matching."""
        procs = []
        if proc:
            try:
                if psutil:
                    parent = psutil.Process(proc.pid)
                    procs.append(parent)
                    procs.extend(parent.children(recursive=True))
                else:
                    return procs
            except Exception:
                pass
                
        if not psutil:
            return procs

        cfg_keyword = f"sim_params_tmp_{worker_id}.yaml"
        part_keyword = f"comms_sim_partition_{worker_id}"
        gz_port_keyword = str(11345 + worker_id)
        
        for p in psutil.process_iter(['pid', 'cmdline', 'environ']):
            try:
                if any(x.pid == p.pid for x in procs):
                    continue
                
                cmdline = p.info.get('cmdline')
                if cmdline:
                    cmdline_str = " ".join(cmdline)
                    if (cfg_keyword in cmdline_str or 
                        part_keyword in cmdline_str or 
                        gz_port_keyword in cmdline_str):
                        procs.append(p)
                        continue
                        
                env = p.info.get('environ')
                if env:
                    if env.get('ROS_DOMAIN_ID') == str(10 + worker_id) or env.get('GZ_PARTITION') == part_keyword:
                        procs.append(p)
                        continue
            except Exception:
                pass
                
        return procs

    @staticmethod
    def send_sigint_to_worker(proc, worker_id: int):
        if proc:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except Exception:
                pass
                
        procs = ProcessManager.get_worker_processes(proc, worker_id)
        for p in procs:
            try:
                p.send_signal(signal.SIGINT)
            except Exception:
                pass

    @staticmethod
    def send_sigkill_to_worker(proc, worker_id: int):
        if proc:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
                
        procs = ProcessManager.get_worker_processes(proc, worker_id)
        for p in procs:
            try:
                p.kill()
            except Exception:
                pass

    @staticmethod
    def terminate_process_cleanly(proc, worker_id: int, timeout: float = 2.0):
        if not proc and worker_id is None:
            return

        ProcessManager.send_sigint_to_worker(proc, worker_id)

        t_start = time.time()
        while time.time() - t_start < timeout:
            if proc and proc.poll() is not None:
                if not ProcessManager.get_worker_processes(None, worker_id):
                    return
            time.sleep(0.1)

        ProcessManager.send_sigkill_to_worker(proc, worker_id)

    @staticmethod
    def kill_all_simulation_zombies():
        """
        Global sweep of all remaining simulation processes in the container.
        This is an aggressive final defense line to guarantee a clean state.
        """
        if not psutil:
            return

        target_proc_names = ['gz', 'ruby', 'ros2', 'comms_node', 'mission_coordinator_node.py', 'progress_logger_node.py', 'tx_controller']
        
        killed_count = 0
        for p in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                name = p.info.get('name', '')
                cmdline = p.info.get('cmdline') or []
                cmdline_str = " ".join(cmdline)
                
                # Check if process name matches exactly, or if it's running a known python node
                should_kill = False
                for target in target_proc_names:
                    if name == target or target in name or target in cmdline_str:
                        should_kill = True
                        break
                        
                # Ensure we don't kill the sweep_sim.py process itself
                if should_kill and "sweep_sim.py" not in cmdline_str:
                    p.kill()
                    killed_count += 1
            except Exception:
                pass
                
        if killed_count > 0:
            print(f"[ProcessManager] Aggressive Cleanup: Forcibly eliminated {killed_count} zombie processes.")

