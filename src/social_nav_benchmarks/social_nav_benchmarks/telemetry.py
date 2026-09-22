"""System telemetry captured during a benchmark run.

Records CPU / RAM (psutil, or a /proc fallback if psutil is missing), GPU utilisation and
memory (parsed from ``nvidia-smi``), and the simulator real-time factor (from /clock rate
vs walltime). Every metric that cannot be measured on this host is reported as ``None`` /
``{"available": false}`` -- it is NEVER fabricated or zero-filled.

The parsing and aggregation helpers are pure standard-library so they unit-test without a
GPU, without psutil and without ROS. Only the sampler thread and the nvidia-smi call touch
the machine.
"""

import subprocess
import threading
import time

try:
    import psutil  # noqa: F401

    _HAVE_PSUTIL = True
except Exception:  # noqa: BLE001 - fall back to /proc parsing
    _HAVE_PSUTIL = False


# --------------------------------------------------------------------------------------
# Pure parsing / aggregation (unit-tested without hardware).
# --------------------------------------------------------------------------------------

def _to_float(token):
    """Parse one nvidia-smi CSV token; return None for '[N/A]' / '[Not Supported]'."""
    token = token.strip()
    if not token or token.lower().startswith("[n") or token.lower() == "n/a":
        return None
    try:
        return float(token)
    except ValueError:
        return None


def parse_nvidia_smi_line(line):
    """Parse one CSV line from
    ``nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total
    --format=csv,noheader,nounits`` into a dict. Unavailable fields become None.
    """
    parts = line.split(",")
    if len(parts) < 3:
        return None
    return {
        "gpu_util_percent": _to_float(parts[0]),
        "gpu_mem_used_mb": _to_float(parts[1]),
        "gpu_mem_total_mb": _to_float(parts[2]),
    }


def parse_proc_meminfo(text):
    """Return used RAM in MB from /proc/meminfo text (MemTotal - MemAvailable)."""
    total_kb = avail_kb = None
    for raw in text.splitlines():
        if raw.startswith("MemTotal:"):
            total_kb = float(raw.split()[1])
        elif raw.startswith("MemAvailable:"):
            avail_kb = float(raw.split()[1])
    if total_kb is None or avail_kb is None:
        return None
    return round((total_kb - avail_kb) / 1024.0, 1)


def _stat_busy_total(line):
    """(busy, total) jiffies from a /proc/stat 'cpu ...' aggregate line."""
    fields = [float(x) for x in line.split()[1:]]
    idle = fields[3] + (fields[4] if len(fields) > 4 else 0.0)  # idle + iowait
    total = sum(fields)
    return total - idle, total


def cpu_percent_from_stat(prev_line, cur_line):
    """CPU utilisation percent between two /proc/stat 'cpu' snapshots."""
    b0, t0 = _stat_busy_total(prev_line)
    b1, t1 = _stat_busy_total(cur_line)
    dt = t1 - t0
    if dt <= 0:
        return None
    return round(100.0 * (b1 - b0) / dt, 1)


def compute_rtf(clock_samples):
    """Sim real-time factor from [(wall_s, sim_s), ...] samples.

    RTF = elapsed sim time / elapsed wall time. Needs >= 2 samples and positive wall span;
    otherwise None (not measurable).
    """
    pts = [p for p in clock_samples if p is not None]
    if len(pts) < 2:
        return None
    wall0, sim0 = pts[0]
    wall1, sim1 = pts[-1]
    dwall = wall1 - wall0
    if dwall <= 0:
        return None
    return round((sim1 - sim0) / dwall, 3)


def summarize(values):
    """{'mean', 'peak'} over the non-None numbers, or None if there are none."""
    nums = [float(v) for v in values if isinstance(v, (int, float))]
    if not nums:
        return None
    return {"mean": round(sum(nums) / len(nums), 1), "peak": round(max(nums), 1)}


def aggregate_samples(cpu, ram, gpu_util, gpu_mem, gpu_available, gpu_name=None):
    """Assemble the telemetry summary dict from collected sample lists."""
    return {
        "sampler": "psutil" if _HAVE_PSUTIL else "proc",
        "cpu_percent": summarize(cpu),
        "ram_used_mb": summarize(ram),
        "gpu": {
            "available": bool(gpu_available),
            "name": gpu_name,
            "util_percent": summarize(gpu_util) if gpu_available else None,
            "mem_used_mb": summarize(gpu_mem) if gpu_available else None,
        },
    }


# --------------------------------------------------------------------------------------
# Hardware access (nvidia-smi, host info, live sampler).
# --------------------------------------------------------------------------------------

def _run_nvidia_smi(query):
    try:
        out = subprocess.run(
            ["nvidia-smi", f"--query-gpu={query}",
             "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=4)
    except (OSError, subprocess.SubprocessError):
        return None
    if out.returncode != 0:
        return None
    return [ln for ln in out.stdout.splitlines() if ln.strip()]


def nvidia_smi_available():
    return _run_nvidia_smi("utilization.gpu") is not None


def gpu_name():
    lines = _run_nvidia_smi("name")
    return lines[0].strip() if lines else None


def sample_gpu():
    """One GPU sample (first GPU) as a parsed dict, or None if nvidia-smi is unavailable."""
    lines = _run_nvidia_smi("utilization.gpu,memory.used,memory.total")
    if not lines:
        return None
    return parse_nvidia_smi_line(lines[0])


def host_info():
    """Static host metadata for experiment_metadata.json (measured, no fabrication)."""
    import os

    info = {
        "psutil_available": _HAVE_PSUTIL,
        "nvidia_smi_available": nvidia_smi_available(),
        "cpu_logical_cores": os.cpu_count(),
        "ram_total_mb": None,
        "gpu_name": None,
    }
    if _HAVE_PSUTIL:
        info["ram_total_mb"] = round(psutil.virtual_memory().total / 1e6, 1)
    else:
        try:
            with open("/proc/meminfo") as f:
                for raw in f:
                    if raw.startswith("MemTotal:"):
                        info["ram_total_mb"] = round(float(raw.split()[1]) / 1024.0, 1)
                        break
        except OSError:
            pass
    if info["nvidia_smi_available"]:
        info["gpu_name"] = gpu_name()
    return info


class TelemetrySampler:
    """Samples CPU / RAM / GPU on a background thread while a run executes.

    Start before the goal is sent, stop after it finishes; ``stop()`` returns the summary
    dict. RTF is added separately by the caller (it comes from /clock, not from here).
    """

    def __init__(self, interval_s=0.5):
        self.interval_s = interval_s
        self._stop = threading.Event()
        self._thread = None
        self._cpu = []
        self._ram = []
        self._gpu_util = []
        self._gpu_mem = []
        self._gpu_available = nvidia_smi_available()
        self._gpu_name = gpu_name() if self._gpu_available else None
        self._t0 = None

    def start(self):
        self._t0 = time.time()
        if _HAVE_PSUTIL:
            psutil.cpu_percent(interval=None)  # prime the delta baseline
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        return self

    def _loop(self):
        prev_stat = self._read_stat_line() if not _HAVE_PSUTIL else None
        while not self._stop.is_set():
            if _HAVE_PSUTIL:
                self._cpu.append(psutil.cpu_percent(interval=None))
                self._ram.append(round(psutil.virtual_memory().used / 1e6, 1))
            else:
                cur_stat = self._read_stat_line()
                if prev_stat and cur_stat:
                    val = cpu_percent_from_stat(prev_stat, cur_stat)
                    if val is not None:
                        self._cpu.append(val)
                prev_stat = cur_stat or prev_stat
                mem = self._read_meminfo()
                if mem is not None:
                    self._ram.append(mem)
            if self._gpu_available:
                g = sample_gpu()
                if g:
                    if g["gpu_util_percent"] is not None:
                        self._gpu_util.append(g["gpu_util_percent"])
                    if g["gpu_mem_used_mb"] is not None:
                        self._gpu_mem.append(g["gpu_mem_used_mb"])
            self._stop.wait(self.interval_s)

    @staticmethod
    def _read_stat_line():
        try:
            with open("/proc/stat") as f:
                return f.readline()
        except OSError:
            return None

    @staticmethod
    def _read_meminfo():
        try:
            with open("/proc/meminfo") as f:
                return parse_proc_meminfo(f.read())
        except OSError:
            return None

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        summary = aggregate_samples(
            self._cpu, self._ram, self._gpu_util, self._gpu_mem,
            self._gpu_available, self._gpu_name)
        summary["samples"] = len(self._cpu)
        summary["duration_s"] = round(time.time() - self._t0, 1) if self._t0 else None
        return summary
