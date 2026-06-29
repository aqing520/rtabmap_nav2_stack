#!/usr/bin/env python3
"""Summarize one or more controller benchmark trial CSV pairs."""

import argparse
import csv
import math
from pathlib import Path


def percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return 0.0
    index = (len(ordered) - 1) * fraction
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


def mean(values):
    return sum(values) / len(values) if values else 0.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "runtime_csv",
        nargs="+",
        type=Path,
        help="Runtime CSV files; matching *_compute.csv files are discovered automatically",
    )
    parser.add_argument("--controller-frequency", type=float, default=15.0)
    args = parser.parse_args()

    grouped = {}
    for runtime_path in args.runtime_csv:
        with runtime_path.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        active_rows = [row for row in rows if row["active"] == "1"]
        if not active_rows:
            print(f"warning: no active samples in {runtime_path}")
            continue

        label = active_rows[0]["label"]
        compute_path = runtime_path.with_name(runtime_path.stem + "_compute.csv")
        with compute_path.open(newline="", encoding="utf-8") as stream:
            compute = [
                float(row["compute_time_ms"])
                for row in csv.DictReader(stream)
            ]
        if not compute:
            print(f"warning: no raw compute samples in {compute_path}")
            continue

        bucket = grouped.setdefault(
            label,
            {"cpu": [], "pss": [], "rss": [], "cmd_p95": [], "compute": [], "trials": 0},
        )
        bucket["trials"] += 1
        bucket["cpu"].extend(float(row["cpu_percent_one_core"]) for row in active_rows)
        bucket["pss"].extend(float(row["pss_mb"]) for row in active_rows)
        bucket["rss"].extend(float(row["rss_mb"]) for row in active_rows)
        bucket["cmd_p95"].extend(float(row["cmd_period_p95_ms"]) for row in active_rows)
        bucket["compute"].extend(compute)

    deadline_ms = 1000.0 / args.controller_frequency
    header = (
        "label,trials,calls,compute_mean_ms,compute_p50_ms,compute_p95_ms,"
        "compute_p99_ms,compute_max_ms,deadline_miss_pct,cpu_mean_pct,"
        "pss_mean_mb,rss_mean_mb,cmd_period_p95_mean_ms"
    )
    print(header)
    for label, values in sorted(grouped.items()):
        compute = values["compute"]
        miss_pct = 100.0 * sum(v > deadline_ms for v in compute) / len(compute)
        print(
            f"{label},{values['trials']},{len(compute)},{mean(compute):.3f},"
            f"{percentile(compute, 0.50):.3f},{percentile(compute, 0.95):.3f},"
            f"{percentile(compute, 0.99):.3f},{max(compute):.3f},{miss_pct:.3f},"
            f"{mean(values['cpu']):.2f},{mean(values['pss']):.2f},"
            f"{mean(values['rss']):.2f},{mean(values['cmd_p95']):.3f}"
        )


if __name__ == "__main__":
    main()
