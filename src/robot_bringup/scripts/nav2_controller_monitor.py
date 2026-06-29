#!/usr/bin/env python3
"""Monitor Nav2 controller_server memory and CPU usage."""

import argparse
import csv
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path


RUNNING = True


def handle_signal(_signum, _frame):
    global RUNNING
    RUNNING = False


def run_command(args, timeout=1.0):
    try:
        return subprocess.check_output(
            args,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=timeout,
        ).strip()
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired, FileNotFoundError):
        return ""


def find_pid(pattern):
    output = run_command(["pgrep", "-f", pattern])
    for line in output.splitlines():
        pid = line.strip()
        if pid and pid != str(os.getpid()):
            return pid
    return ""


def read_process_sample(pid):
    output = run_command(
        ["ps", "-p", str(pid), "-o", "rss=,vsz=,pmem=,pcpu=,nlwp="],
        timeout=0.5,
    )
    fields = output.split()
    if len(fields) != 5:
        return None
    return {
        "rss_kb": fields[0],
        "vsz_kb": fields[1],
        "pmem": fields[2],
        "pcpu": fields[3],
        "threads": fields[4],
    }


def lifecycle_state(node_name):
    output = run_command(["ros2", "lifecycle", "get", node_name], timeout=0.8)
    if not output:
        return ""
    return output.splitlines()[-1].strip()


def topic_hz(topic_name):
    output = run_command(["ros2", "topic", "hz", topic_name, "-w", "3"], timeout=2.5)
    for line in output.splitlines():
        line = line.strip()
        if line.startswith("average rate:"):
            return line.replace("average rate:", "").strip()
    return ""


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record controller_server RSS/VSZ/CPU/thread usage to CSV."
    )
    parser.add_argument(
        "--pid",
        default="",
        help="controller_server PID. If omitted, the script auto-detects it.",
    )
    parser.add_argument(
        "--pattern",
        default="/nav2_controller/.*/controller_server",
        help="pgrep -f pattern used when --pid is omitted.",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="Sampling interval in seconds.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Duration in seconds. Use 0 to run until Ctrl-C.",
    )
    parser.add_argument(
        "--output",
        default="",
        help="CSV output path. Defaults to /tmp/nav2_controller_monitor_TIMESTAMP.csv.",
    )
    parser.add_argument(
        "--with-lifecycle",
        action="store_true",
        help="Also record lifecycle states for controller/planner/bt_navigator.",
    )
    parser.add_argument(
        "--with-topic-hz",
        action="store_true",
        help="Also sample /cloud_registered_body and costmap topic hz. This is slower.",
    )
    parser.add_argument(
        "--print",
        action="store_true",
        help="Print samples to the terminal while recording.",
    )
    return parser.parse_args()


def main():
    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    args = parse_args()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = Path(args.output or f"/tmp/nav2_controller_monitor_{timestamp}.csv")
    output.parent.mkdir(parents=True, exist_ok=True)

    pid = args.pid or find_pid(args.pattern)
    if not pid:
        print(
            "controller_server not found. Start navigation first, or pass --pid.",
            file=sys.stderr,
        )
        return 2

    fields = [
        "timestamp",
        "elapsed_s",
        "pid",
        "rss_kb",
        "rss_mb",
        "vsz_kb",
        "vsz_mb",
        "pmem",
        "pcpu",
        "threads",
    ]
    if args.with_lifecycle:
        fields.extend(["controller_state", "planner_state", "bt_navigator_state"])
    if args.with_topic_hz:
        fields.extend(["cloud_hz", "local_costmap_hz", "global_costmap_hz"])

    start = time.monotonic()
    max_rss = 0
    samples = 0

    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        stream.flush()

        print(f"monitoring controller_server pid={pid}")
        print(f"writing CSV: {output}")

        while RUNNING:
            elapsed = time.monotonic() - start
            if args.duration > 0 and elapsed > args.duration:
                break

            sample = read_process_sample(pid)
            if sample is None:
                print(f"process {pid} is no longer available")
                break

            rss_kb = int(float(sample["rss_kb"]))
            vsz_kb = int(float(sample["vsz_kb"]))
            max_rss = max(max_rss, rss_kb)
            samples += 1

            row = {
                "timestamp": datetime.now().isoformat(timespec="seconds"),
                "elapsed_s": f"{elapsed:.1f}",
                "pid": pid,
                "rss_kb": rss_kb,
                "rss_mb": f"{rss_kb / 1024.0:.2f}",
                "vsz_kb": vsz_kb,
                "vsz_mb": f"{vsz_kb / 1024.0:.2f}",
                "pmem": sample["pmem"],
                "pcpu": sample["pcpu"],
                "threads": sample["threads"],
            }

            if args.with_lifecycle:
                row["controller_state"] = lifecycle_state("/controller_server")
                row["planner_state"] = lifecycle_state("/planner_server")
                row["bt_navigator_state"] = lifecycle_state("/bt_navigator")

            if args.with_topic_hz:
                row["cloud_hz"] = topic_hz("/cloud_registered_body")
                row["local_costmap_hz"] = topic_hz("/local_costmap/costmap")
                row["global_costmap_hz"] = topic_hz("/global_costmap/costmap")

            writer.writerow(row)
            stream.flush()

            if args.print:
                print(
                    f"{row['elapsed_s']}s rss={row['rss_mb']}MB "
                    f"vsz={row['vsz_mb']}MB cpu={row['pcpu']}% "
                    f"threads={row['threads']}"
                )

            time.sleep(max(args.interval, 0.1))

    print(f"samples: {samples}")
    print(f"max_rss_mb: {max_rss / 1024.0:.2f}")
    print(f"csv: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
