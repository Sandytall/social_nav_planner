"""Pure parsing / aggregation tests for system telemetry (no GPU / psutil / ROS needed)."""

from social_nav_benchmarks import telemetry


def test_parse_nvidia_smi_normal_line():
    d = telemetry.parse_nvidia_smi_line("12, 1024, 8192")
    assert d == {"gpu_util_percent": 12.0, "gpu_mem_used_mb": 1024.0,
                 "gpu_mem_total_mb": 8192.0}


def test_parse_nvidia_smi_na_fields_are_none():
    d = telemetry.parse_nvidia_smi_line("[N/A], [Not Supported], 8192")
    assert d["gpu_util_percent"] is None
    assert d["gpu_mem_used_mb"] is None
    assert d["gpu_mem_total_mb"] == 8192.0


def test_parse_nvidia_smi_bad_line_is_none():
    assert telemetry.parse_nvidia_smi_line("garbage") is None


def test_parse_proc_meminfo_used_mb():
    text = "MemTotal:       16000000 kB\nMemFree: 100 kB\nMemAvailable:    8000000 kB\n"
    # used = (16000000 - 8000000) kB = 8000000 kB = 7812.5 MB
    assert telemetry.parse_proc_meminfo(text) == 7812.5


def test_cpu_percent_from_stat():
    # fields after 'cpu' are user nice system idle iowait ...; busy = total - (idle+iowait).
    prev = "cpu 100 0 0 100 0 0 0 0 0 0"   # busy=100, total=200
    cur = "cpu 150 0 0 150 0 0 0 0 0 0"    # busy=150, total=300 -> dbusy=50, dtotal=100
    assert telemetry.cpu_percent_from_stat(prev, cur) == 50.0


def test_compute_rtf():
    # 2 s of sim time over 4 s of walltime -> 0.5x real time.
    samples = [(1000.0, 10.0), (1002.0, 11.0), (1004.0, 12.0)]
    assert telemetry.compute_rtf(samples) == 0.5


def test_compute_rtf_insufficient_samples_is_none():
    assert telemetry.compute_rtf([(1.0, 1.0)]) is None


def test_summarize_and_aggregate():
    assert telemetry.summarize([]) is None
    assert telemetry.summarize([10, 20, 30]) == {"mean": 20.0, "peak": 30.0}
    agg = telemetry.aggregate_samples(
        cpu=[10, 20], ram=[100, 200], gpu_util=[5, 15], gpu_mem=[500, 700],
        gpu_available=True, gpu_name="Test GPU")
    assert agg["cpu_percent"]["peak"] == 20.0
    assert agg["gpu"]["available"] is True
    assert agg["gpu"]["util_percent"]["mean"] == 10.0


def test_aggregate_marks_absent_gpu_null():
    agg = telemetry.aggregate_samples([1], [1], [], [], gpu_available=False)
    assert agg["gpu"]["available"] is False
    assert agg["gpu"]["util_percent"] is None
    assert agg["gpu"]["mem_used_mb"] is None
