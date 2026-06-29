#!/usr/bin/env python3
"""Record Nav2 controller process resources, output period, and direct compute times."""

import argparse
import csv
import math
import os
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from std_msgs.msg import Float64


CLK_TCK = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
PAGE_SIZE = os.sysconf(os.sysconf_names["SC_PAGE_SIZE"])


def percentile(values, fraction):
    if not values:
        return 0.0
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


def find_pid(pattern):
    matches = []
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit():
            continue
        try:
            cmdline = (proc / "cmdline").read_bytes().replace(b"\x00", b" ").decode(
                errors="ignore"
            )
        except OSError:
            continue
        if pattern in cmdline and "controller_runtime_monitor.py" not in cmdline:
            matches.append((int(proc.name), cmdline.strip()))
    if len(matches) == 1:
        return matches[0][0]
    if not matches:
        return None
    details = "\n".join(f"  pid={pid}: {cmd}" for pid, cmd in matches)
    raise RuntimeError(f"Multiple processes match {pattern!r}; pass --pid:\n{details}")


def read_proc_sample(pid):
    try:
        raw_stat = Path(f"/proc/{pid}/stat").read_text()
        stat = raw_stat[raw_stat.rfind(")") + 2 :].split()
        statm = Path(f"/proc/{pid}/statm").read_text().split()
    except OSError:
        return None

    # Fields after "comm": state is index 0, utime/stime are indexes 11/12.
    sample = {
        "cpu_ticks": int(stat[11]) + int(stat[12]),
        "rss_mb": int(statm[1]) * PAGE_SIZE / (1024.0 * 1024.0),
        "pss_mb": 0.0,
    }
    try:
        for line in Path(f"/proc/{pid}/smaps_rollup").read_text().splitlines():
            if line.startswith("Pss:"):
                sample["pss_mb"] = float(line.split()[1]) / 1024.0
                break
    except OSError:
        pass
    return sample


class ControllerRuntimeMonitor(Node):
    FIELDNAMES = [
        "wall_time",
        "label",
        "pid",
        "active",
        "cpu_percent_one_core",
        "cpu_percent_machine",
        "rss_mb",
        "pss_mb",
        "cmd_count",
        "cmd_period_mean_ms",
        "cmd_period_p95_ms",
        "cmd_period_max_ms",
        "last_cmd_age_ms",
        "compute_count",
        "compute_mean_ms",
        "compute_p50_ms",
        "compute_p95_ms",
        "compute_p99_ms",
        "compute_max_ms",
        "deadline_miss_count",
    ]

    def __init__(self, args):
        super().__init__("controller_runtime_monitor")
        self.args = args
        self.pid = args.pid or find_pid(args.process_pattern)
        if self.pid is None:
            raise RuntimeError(f"Cannot find process matching {args.process_pattern!r}")

        self.last_wall_time = time.monotonic()
        self.last_proc = read_proc_sample(self.pid)
        if self.last_proc is None:
            raise RuntimeError(f"Cannot read /proc/{self.pid}")

        self.last_cmd_time = None
        self.cmd_periods = []
        self.compute_times = []
        self.deadline_ms = 1000.0 / args.controller_frequency

        self.csv_file = open(args.output, "w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.FIELDNAMES)
        self.writer.writeheader()
        timing_output = args.timing_output or str(
            Path(args.output).with_name(Path(args.output).stem + "_compute.csv")
        )
        self.timing_file = open(timing_output, "w", newline="", encoding="utf-8")
        self.timing_writer = csv.DictWriter(
            self.timing_file, fieldnames=["wall_time", "label", "compute_time_ms"]
        )
        self.timing_writer.writeheader()

        msg_type = TwistStamped if args.twist_stamped else Twist
        self.cmd_sub = self.create_subscription(msg_type, args.topic, self.on_cmd, 100)
        self.timing_sub = self.create_subscription(
            Float64, args.timing_topic, self.on_compute_time, 1000
        )
        self.timer = self.create_timer(args.sample_period, self.on_timer)
        self.get_logger().info(
            f"label={args.label} pid={self.pid} cmd={args.topic} "
            f"timing={args.timing_topic} output={args.output} raw={timing_output}"
        )

    def on_cmd(self, _msg):
        now = time.monotonic()
        if self.last_cmd_time is not None:
            self.cmd_periods.append((now - self.last_cmd_time) * 1000.0)
        self.last_cmd_time = now

    def on_compute_time(self, msg):
        value = float(msg.data)
        self.compute_times.append(value)
        self.timing_writer.writerow(
            {
                "wall_time": f"{time.time():.6f}",
                "label": self.args.label,
                "compute_time_ms": f"{value:.6f}",
            }
        )

    def on_timer(self):
        now = time.monotonic()
        proc = read_proc_sample(self.pid)
        if proc is None:
            self.get_logger().error(f"process {self.pid} disappeared")
            rclpy.shutdown()
            return

        dt = max(now - self.last_wall_time, 1.0e-9)
        cpu = 100.0 * ((proc["cpu_ticks"] - self.last_proc["cpu_ticks"]) / CLK_TCK) / dt
        cmd = self.cmd_periods
        compute = self.compute_times
        deadline_misses = sum(value > self.deadline_ms for value in compute)
        active = bool(compute)

        row = {
            "wall_time": f"{time.time():.3f}",
            "label": self.args.label,
            "pid": self.pid,
            "active": int(active),
            "cpu_percent_one_core": f"{cpu:.2f}",
            "cpu_percent_machine": f"{cpu / (os.cpu_count() or 1):.2f}",
            "rss_mb": f"{proc['rss_mb']:.2f}",
            "pss_mb": f"{proc['pss_mb']:.2f}",
            "cmd_count": len(cmd),
            "cmd_period_mean_ms": f"{sum(cmd) / len(cmd):.3f}" if cmd else "0.000",
            "cmd_period_p95_ms": f"{percentile(cmd, 0.95):.3f}",
            "cmd_period_max_ms": f"{max(cmd):.3f}" if cmd else "0.000",
            "last_cmd_age_ms": (
                f"{(now - self.last_cmd_time) * 1000.0:.3f}"
                if self.last_cmd_time is not None
                else "0.000"
            ),
            "compute_count": len(compute),
            "compute_mean_ms": (
                f"{sum(compute) / len(compute):.3f}" if compute else "0.000"
            ),
            "compute_p50_ms": f"{percentile(compute, 0.50):.3f}",
            "compute_p95_ms": f"{percentile(compute, 0.95):.3f}",
            "compute_p99_ms": f"{percentile(compute, 0.99):.3f}",
            "compute_max_ms": f"{max(compute):.3f}" if compute else "0.000",
            "deadline_miss_count": deadline_misses,
        }
        self.writer.writerow(row)
        self.csv_file.flush()
        self.timing_file.flush()

        if active:
            self.get_logger().info(
                f"{self.args.label}: compute mean={row['compute_mean_ms']}ms "
                f"p95={row['compute_p95_ms']}ms max={row['compute_max_ms']}ms "
                f"cpu={row['cpu_percent_one_core']}% PSS={row['pss_mb']}MB"
            )

        self.cmd_periods = []
        self.compute_times = []
        self.last_wall_time = now
        self.last_proc = proc


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", required=True, help="e.g. mppi_run1 or dwb_run1")
    parser.add_argument("--topic", default="/cmd_vel_nav")
    parser.add_argument(
        "--timing-topic", default="/controller_benchmark/compute_time_ms"
    )
    parser.add_argument("--output", required=True)
    parser.add_argument(
        "--timing-output",
        help="Raw per-call CSV; defaults to <output stem>_compute.csv",
    )
    parser.add_argument("--pid", type=int)
    parser.add_argument("--process-pattern", default="controller_server")
    parser.add_argument("--sample-period", type=float, default=1.0)
    parser.add_argument("--controller-frequency", type=float, default=15.0)
    parser.add_argument("--twist-stamped", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = ControllerRuntimeMonitor(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.csv_file.close()
        node.timing_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
