import os

HAS_PSUTIL = False
try:
    import psutil
    HAS_PSUTIL = True
except ImportError:
    pass

def get_cpu_usage():
    """
    Returns the overall CPU usage as a percentage.
    If psutil is unavailable, attempts to read from /proc/stat on Linux.
    Returns 0.0 if unable to determine.
    """
    if HAS_PSUTIL:
        return psutil.cpu_percent(interval=None)
    
    # Fallback for Linux without psutil
    try:
        with open('/proc/stat', 'r') as f:
            lines = f.readlines()
            for line in lines:
                if line.startswith('cpu '):
                    parts = line.split()
                    # parts: cpu user nice system idle iowait irq softirq steal guest guest_nice
                    idle = float(parts[4]) + float(parts[5])
                    non_idle = float(parts[1]) + float(parts[2]) + float(parts[3]) + \
                               float(parts[6]) + float(parts[7]) + float(parts[8])
                    total = idle + non_idle
                    
                    # For a single snapshot, we can't get exact usage without delta.
                    # We return a dummy 0.0 or a rough estimate if we cache previous values.
                    # To keep it simple and stateless without psutil, we just return 0.0
                    # since delta is required.
                    return 0.0
    except Exception:
        pass
        
    return 0.0

def get_memory_usage():
    """
    Returns memory usage as (used_gb, total_gb, percent).
    Returns (0.0, 0.0, 0.0) if unable to determine.
    """
    if HAS_PSUTIL:
        mem = psutil.virtual_memory()
        used_gb = mem.used / (1024 ** 3)
        total_gb = mem.total / (1024 ** 3)
        return used_gb, total_gb, mem.percent
        
    # Fallback for Linux without psutil
    try:
        with open('/proc/meminfo', 'r') as f:
            lines = f.readlines()
            mem_total = 0
            mem_free = 0
            mem_available = 0
            for line in lines:
                if line.startswith('MemTotal:'):
                    mem_total = int(line.split()[1]) * 1024
                elif line.startswith('MemAvailable:'):
                    mem_available = int(line.split()[1]) * 1024
                elif line.startswith('MemFree:'):
                    mem_free = int(line.split()[1]) * 1024
                    
            if mem_total > 0:
                avail = mem_available if mem_available > 0 else mem_free
                used = mem_total - avail
                used_gb = used / (1024 ** 3)
                total_gb = mem_total / (1024 ** 3)
                percent = (used / mem_total) * 100.0
                return used_gb, total_gb, percent
    except Exception:
        pass
        
    return 0.0, 0.0, 0.0
