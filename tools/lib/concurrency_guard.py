import os
import sys
import fcntl

class ConcurrencyGuard:
    """
    Ensures exclusive execution of the sweep simulation within a container
    using an OS-level file lock.
    """
    def __init__(self, lock_file: str = "/tmp/sweep_sim.lock"):
        self.lock_file = lock_file
        self._fd = None

    def __enter__(self):
        self._fd = os.open(self.lock_file, os.O_CREAT | os.O_RDWR)
        try:
            # LOCK_EX: Exclusive lock, LOCK_NB: Non-blocking
            fcntl.flock(self._fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print("\n[CRITICAL ERROR] Another instance of sweep_sim.py or an independent run is currently executing in this container.", file=sys.stderr)
            print("Concurrent execution will cause port collisions, extreme CPU load, and undefined behavior.", file=sys.stderr)
            print("Aborting to protect system integrity.\n", file=sys.stderr)
            os.close(self._fd)
            sys.exit(1)
        except Exception as e:
            print(f"[ConcurrencyGuard] Warning: Could not acquire lock due to error: {e}", file=sys.stderr)
        
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self._fd is not None:
            try:
                fcntl.flock(self._fd, fcntl.LOCK_UN)
            except Exception:
                pass
            os.close(self._fd)
