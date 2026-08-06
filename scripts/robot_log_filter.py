#!/usr/bin/env python3
"""Keep robot.sh terminal output concise while preserving the raw log via tee."""

import argparse
import re
import signal
import sys
import time


ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
DELAY_RE = re.compile(r"(?:delay=|differ on )([0-9.]+)")


class RobotLogFilter:
    def __init__(self, mode: str) -> None:
        self.mode = mode
        if mode == "map":
            self.stack_name = "建图栈"
        elif mode == "rel":
            self.stack_name = "重定位导航栈"
        else:
            self.stack_name = "导航栈"
        self.shutdown = False
        self.once_keys = set()
        self.last_emit = {}
        self.collision_stale_total = 0
        self.collision_stale_last_reported = 0
        self.collision_stale_delay = 0.0
        self.tf_timeout_total = 0

    @staticmethod
    def clean(line: str) -> str:
        return ANSI_RE.sub("", line).replace("\r", "").strip()

    @staticmethod
    def emit(message: str) -> None:
        print(message, flush=True)

    def emit_once(self, key: str, message: str) -> None:
        if key not in self.once_keys:
            self.once_keys.add(key)
            self.emit(message)

    def emit_throttled(self, key: str, message: str, interval: float = 5.0) -> None:
        now = time.monotonic()
        if now - self.last_emit.get(key, -interval) >= interval:
            self.last_emit[key] = now
            self.emit(message)

    def process(self, raw_line: str) -> None:
        line = self.clean(raw_line)
        if not line:
            return

        # Normal shutdown is noisy in ROS 2 Humble. Keep one concise status line.
        if "user interrupted with ctrl-c" in line:
            if not self.shutdown:
                self.shutdown = True
                self.emit(f"[INFO] 正在停止机器人{self.stack_name}...")
            return

        if self.shutdown and any(
            text in line
            for text in (
                "process has died",
                "process has finished cleanly",
                "signal_handler(signum=2)",
                "Attempting to unload library",
                "at line 127 in ./src/class_loader.cpp",
                "terminate called without an active exception",
                "catch sig",
                "Rebuild thread terminated normally",
                "SDK Deinit completely",
                "lddc destory",
                "Destroying",
            )
        ):
            return

        # Startup milestones.
        if "Init lds lidar success" in line:
            self.emit_once("livox", "[OK] Livox MID360 驱动初始化完成")
            return
        if "Node init finished." in line and "laser_mapping" in line:
            self.emit_once("fastlio", "[OK] FAST-LIO 节点初始化完成")
            return
        if "IMU Initial Done" in line:
            self.emit_once("imu", "[OK] FAST-LIO IMU 初始化完成")
            return
        if "Initialize the map kdtree" in line:
            self.emit_once("kdtree", "[OK] FAST-LIO 地图 KD-Tree 初始化完成")
            return
        if 'Using database from "' in line:
            match = re.search(r'Using database from "([^"]+)"', line)
            path = match.group(1) if match else "RTAB-Map数据库"
            self.emit_once("database", f"[OK] RTAB-Map 已加载数据库：{path}")
            return
        if "Localization mode (Mem/IncrementalMemory=false)" in line:
            self.emit_once("localization", "[OK] RTAB-Map 激光定位模式已启动")
            return
        if "last localization pose is ignored" in line:
            self.emit_once(
                "start_at_origin",
                "[INFO] RTAB-Map 使用数据库起点作为初始位姿",
            )
            return
        if "Configured CudaMppiController" in line:
            details = line.split("Configured CudaMppiController", 1)[1].strip()
            self.emit_once("cuda_mppi", f"[OK] CUDA MPPI 控制器已加载 {details}")
            return
        if "stvl_layer initialization complete" in line:
            if "local_costmap" in line:
                self.emit_once("local_stvl", "[OK] 局部代价地图 STVL 已初始化")
            elif "global_costmap" in line:
                self.emit_once("global_stvl", "[OK] 全局代价地图 STVL 已初始化")
            return
        if "Managed nodes are active" in line:
            if "lifecycle_manager_navigation" in line:
                self.emit_once("nav_ready", "[READY] Nav2 导航节点已全部激活")
            elif "lifecycle_manager_collision_monitor" in line:
                self.emit_once("collision_ready", "[OK] Collision Monitor 已激活")
            return

        # Keep map/database save results.
        if "Saving database/long-term memory...done" in line:
            self.emit_once("db_saved", "[OK] RTAB-Map 数据库保存完成")
            return
        if "2D occupancy grid map saved" in line:
            self.emit_once("map_saved", "[OK] RTAB-Map 二维栅格地图保存完成")
            return

        # Rate-limit Collision Monitor stale-cloud warnings.
        if "Ignoring the source" in line and "timestamps differ on" in line:
            self.collision_stale_total += 1
            match = DELAY_RE.search(line)
            if match:
                self.collision_stale_delay = float(match.group(1))
            now = time.monotonic()
            if now - self.last_emit.get("collision_stale", -5.0) >= 5.0:
                self.last_emit["collision_stale"] = now
                self.collision_stale_last_reported = self.collision_stale_total
                self.emit(
                    "[WARN] Collision Monitor 点云过期："
                    f"延迟 {self.collision_stale_delay:.2f}s，累计忽略 "
                    f"{self.collision_stale_total} 次"
                )
            return

        # Hide known pure-LiDAR and startup noise.
        if any(
            text in line
            for text in (
                "Missing visual features or missing raw data",
                "Old-style arguments are deprecated",
                "There is no image subscription",
                "No HW_ID was set",
                "Parameter migration from",
                'Setting "RGBD/ProximityPathMaxNeighbors"',
                "sync diagnostics disabled",
                "No point, skip this scan",
                "Update map correction based on last localization saved",
                "Passing new path to controller",
                "lifecycle node launched",
                "Waiting on external lifecycle transitions",
                "See https://design.ros2.org/articles/node_lifecycle",
            )
        ):
            return

        # RTAB-Map timing is useful only when delay becomes abnormal.
        if "Rate=" in line and "RTAB-Map=" in line and "delay=" in line:
            match = DELAY_RE.search(line)
            if match and float(match.group(1)) >= 3.0:
                delay = float(match.group(1))
                self.emit_throttled(
                    "rtab_delay",
                    f"[WARN] RTAB-Map 数据处理延迟达到 {delay:.2f}s",
                )
            return

        # TF startup waits can repeat quickly. Keep a throttled warning.
        if "Timed out waiting for transform" in line:
            self.tf_timeout_total += 1
            if self.tf_timeout_total >= 10:
                self.emit_throttled(
                    "tf_timeout",
                    f"[WARN] TF持续不可用，累计等待 {self.tf_timeout_total} 次",
                    5.0,
                )
            return

        # Navigation and safety events.
        if any(
            text in line
            for text in (
                "Begin navigating",
                "Received a goal, begin computing control effort",
                "Navigation succeeded",
                "Navigation failed",
                "Goal canceled",
                "Goal cancelled",
                "Failed to make progress",
                "Failed to create plan",
                "Planner loop missed",
                "Controller loop missed",
                "No valid trajectories",
                "Robot to stop",
                "Robot to slowdown",
                "Robot to continue normal operation",
            )
        ):
            self.emit(line)
            return

        # Keep unexpected warnings and runtime errors. Expected shutdown errors
        # were filtered above after the SIGINT marker.
        if re.search(r"\[(?:\s*WARN(?:ING)?|ERROR|FATAL)\]", line):
            self.emit(line)
            return

    def finish(self) -> None:
        if self.collision_stale_total > self.collision_stale_last_reported:
            self.emit(
                "[WARN] Collision Monitor 点云过期汇总："
                f"最后延迟 {self.collision_stale_delay:.2f}s，"
                f"累计忽略 {self.collision_stale_total} 次"
            )
        if self.shutdown:
            self.emit(f"[OK] 机器人{self.stack_name}已停止")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("map", "nav", "rel"), required=True)
    args = parser.parse_args()

    # ros2 launch must receive Ctrl+C, while the log consumer should keep
    # draining shutdown output until the launch process closes the pipe.
    signal.signal(signal.SIGINT, signal.SIG_IGN)

    log_filter = RobotLogFilter(args.mode)
    for line in sys.stdin:
        log_filter.process(line)
    log_filter.finish()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
