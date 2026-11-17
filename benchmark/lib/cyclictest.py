"""cyclictest invocation and summary parser (rt-tests).

The histogram form (-h) prints the latency counts per microsecond bucket and a
summary of "# Min/Avg/Max Latencies" and "# Histogram Overflows" lines. The
percentiles come from the histogram; latencies above the histogram range are
counted as overflows and are reported as a lower bound, not dropped.
"""
from __future__ import annotations
import re

# Fixed parameters of the lane, in cyclictest's own flags.
#   -m   lock memory           -p 80  SCHED_FIFO priority 80
#   -i   wake-up interval, microseconds
#   -t 1 one measurement thread, pinned with -a <cpu>
#   -h   histogram range in microseconds (also the overflow limit)
#   -q   print the summary only
HISTOGRAM_US = 2000
INTERVAL_US = 1000
PRIORITY = 80


def build_args(duration_s, cpu):
    return ["cyclictest", "-m", "-p", str(PRIORITY), "-i", str(INTERVAL_US),
            "-t", "1", "-a", str(cpu), "-h", str(HISTOGRAM_US), "-q",
            "-D", f"{int(duration_s)}s"]


_ROW = re.compile(r"^(\d+)\s+(\d+)\s*$")
_FIELD = re.compile(r"^#\s*(Min|Avg|Max) Latencies:\s*(\d+)")
_OVERFLOW = re.compile(r"^#\s*Histogram Overflows:\s*(\d+)")
_TOTAL = re.compile(r"^#\s*Total:\s*(\d+)")


def _percentile_us(counts, overflow, q):
    """Latency in us at quantile ``q``; (value, lower_bound) where lower_bound
    is True when the quantile falls in the overflow."""
    total = sum(counts) + overflow
    if total == 0:
        return None, False
    target = q * total
    seen = 0
    for us, c in enumerate(counts):
        seen += c
        if seen >= target:
            return us, False
    return len(counts), True


def parse_summary(text):
    """Parse cyclictest -h output of one thread into min/avg/p99/max in microseconds.

    ``p99_lower_bound`` is True when more than 1% of the samples exceeded the
    histogram range, so p99 is only known to be at least the range limit.
    Raises ValueError when the output holds no summary.
    """
    counts = {}
    fields = {}
    overflow = 0
    total = None
    for line in text.splitlines():
        m = _ROW.match(line.strip())
        if m:
            counts[int(m.group(1))] = int(m.group(2))
            continue
        for rx, key in ((_FIELD, None), (_OVERFLOW, "overflow"), (_TOTAL, "total")):
            m = rx.match(line.strip())
            if not m:
                continue
            if key is None:
                fields[m.group(1).lower()] = int(m.group(2))
            elif key == "overflow":
                overflow = int(m.group(1))
            else:
                total = int(m.group(1))
    if not {"min", "avg", "max"} <= fields.keys():
        raise ValueError("no cyclictest summary (Min/Avg/Max Latencies) in output")
    buckets = [counts.get(i, 0) for i in range(max(counts) + 1)] if counts else []
    p99, lower = _percentile_us(buckets, overflow, 0.99)
    return {
        "min_us": fields["min"], "avg_us": fields["avg"], "max_us": fields["max"],
        "p99_us": p99, "p99_lower_bound": lower,
        "samples": total if total is not None else sum(buckets) + overflow,
        "overflow": overflow, "histogram_us": HISTOGRAM_US,
    }
