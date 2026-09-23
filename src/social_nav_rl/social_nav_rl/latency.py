"""Inference-latency meter: P50/P95/P99/max over recorded call times (ms).

Real-time claims must be measured, not asserted - wrap each policy inference in `measure()`.
"""
import time

import numpy as np


class LatencyMeter:
    def __init__(self):
        self.samples_ms = []

    def record(self, ms: float):
        self.samples_ms.append(float(ms))

    class _Timer:
        def __init__(self, meter):
            self.meter = meter

        def __enter__(self):
            self._t0 = time.perf_counter()
            return self

        def __exit__(self, *exc):
            self.meter.record((time.perf_counter() - self._t0) * 1000.0)
            return False

    def measure(self):
        return LatencyMeter._Timer(self)

    def summary(self) -> dict:
        if not self.samples_ms:
            return {"count": 0, "p50": None, "p95": None, "p99": None, "max": None, "mean": None}
        a = np.asarray(self.samples_ms)
        return {"count": int(a.size), "mean": float(a.mean()), "max": float(a.max()),
                "p50": float(np.percentile(a, 50)), "p95": float(np.percentile(a, 95)),
                "p99": float(np.percentile(a, 99))}
