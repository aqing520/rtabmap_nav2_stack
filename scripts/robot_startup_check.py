#!/usr/bin/env python3
"""FAST-LIO data freshness gate used by robot.sh nav/rel.

Before this checker runs, robot.sh has already executed ``pkill -f ros2``.
This node verifies that the newly started FAST-LIO instance has exactly one
publisher for cloud and odometry, that both streams are fresh, and that the
odom -> base_footprint transform is fresh before Nav2 is activated.
"""

import argparse
import os
import signal
import sys
import time
from typing import Dict, Iterable, List, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _endpoint_names(endpoints: Iterable) -> List[str]:
    names = set()
    for endpoint in endpoints:
        namespace = endpoint.node_namespace.rstrip("/")
        if not namespace:
            namespace = ""
        names.add(f"{namespace}/{endpoint.node_name}" or "/")
    return sorted(names)


def _format_endpoint_names(endpoints: Iterable) -> str:
    names = _endpoint_names(endpoints)
    return ", ".join(names) if names else "none"


def _init_rclpy() -> None:
    # Do not pass this script's CLI arguments into ROS argument parsing.
    rclpy.init(args=[])


class SensorFreshnessCheck(Node):
    def __init__(
        self,
        cloud_topic: str,
        odom_topic: str,
        odom_frame: str,
        base_frame: str,
    ):
        super().__init__(f"robot_sensor_check_{os.getpid()}")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.cloud_topic = cloud_topic
        self.odom_topic = odom_topic
        self.odom_frame = odom_frame
        self.base_frame = base_frame
        self.cloud_msg: Optional[PointCloud2] = None
        self.odom_msg: Optional[Odometry] = None
        self.cloud_generation = 0
        self.odom_generation = 0
        self.create_subscription(
            PointCloud2, cloud_topic, self._cloud_cb, qos
        )
        self.create_subscription(
            Odometry, odom_topic, self._odom_cb, qos
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=False
        )

    def _cloud_cb(self, msg: PointCloud2) -> None:
        self.cloud_msg = msg
        self.cloud_generation += 1

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom_msg = msg
        self.odom_generation += 1

    def publisher_state(self, topic: str) -> Tuple[int, str]:
        endpoints = self.get_publishers_info_by_topic(topic)
        return len(endpoints), _format_endpoint_names(endpoints)

    def age_sec(self, stamp) -> float:
        stamp_ns = _stamp_ns(stamp)
        if stamp_ns <= 0:
            return float("inf")
        return (self.get_clock().now().nanoseconds - stamp_ns) / 1.0e9

    def latest_tf_age(self) -> Tuple[Optional[float], str]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                Time(),
            )
        except TransformException as exc:
            return None, str(exc)
        age = self.age_sec(transform.header.stamp)
        return age, "ok"


def _age_is_valid(age: float, max_age: float, max_future: float) -> bool:
    return -max_future <= age <= max_age


def run_sensor_check(args) -> int:
    _init_rclpy()
    node = SensorFreshnessCheck(
        args.cloud_topic,
        args.odom_topic,
        args.odom_frame,
        args.base_frame,
    )
    deadline = time.monotonic() + args.timeout
    streak = 0
    evaluated_cloud_generation = 0
    evaluated_odom_generation = 0
    last_reason = "等待 FAST-LIO publisher 和数据"
    next_log = 0.0
    latest_metrics: Dict[str, float] = {}

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

            cloud_count, cloud_publishers = node.publisher_state(
                args.cloud_topic
            )
            odom_count, odom_publishers = node.publisher_state(
                args.odom_topic
            )
            if cloud_count != 1 or odom_count != 1:
                streak = 0
                last_reason = (
                    f"publisher 数量不正确：{args.cloud_topic}={cloud_count} "
                    f"({cloud_publishers})，{args.odom_topic}={odom_count} "
                    f"({odom_publishers})"
                )
            elif node.cloud_msg is None or node.odom_msg is None:
                streak = 0
                last_reason = "publisher 已发现，等待点云和里程计消息"
            elif (
                node.cloud_generation > evaluated_cloud_generation
                and node.odom_generation > evaluated_odom_generation
            ):
                evaluated_cloud_generation = node.cloud_generation
                evaluated_odom_generation = node.odom_generation
                cloud_age = node.age_sec(node.cloud_msg.header.stamp)
                odom_age = node.age_sec(node.odom_msg.header.stamp)
                tf_age, tf_reason = node.latest_tf_age()
                latest_metrics = {
                    "cloud_age": cloud_age,
                    "odom_age": odom_age,
                    "tf_age": tf_age if tf_age is not None else float("inf"),
                }

                invalid = []
                if not _age_is_valid(
                    cloud_age, args.max_age, args.max_future
                ):
                    invalid.append(f"cloud age={cloud_age:.3f}s")
                if not _age_is_valid(
                    odom_age, args.max_age, args.max_future
                ):
                    invalid.append(f"odom age={odom_age:.3f}s")
                if tf_age is None:
                    invalid.append(
                        f"无 {args.odom_frame}->{args.base_frame} TF: "
                        f"{tf_reason}"
                    )
                elif not _age_is_valid(
                    tf_age, args.max_age, args.max_future
                ):
                    invalid.append(f"TF age={tf_age:.3f}s")

                if invalid:
                    streak = 0
                    last_reason = "；".join(invalid)
                else:
                    streak += 1
                    last_reason = (
                        f"新鲜样本 {streak}/{args.required_samples}："
                        f"cloud={cloud_age:.3f}s odom={odom_age:.3f}s "
                        f"TF={tf_age:.3f}s"
                    )
                    print(f"[INFO] {last_reason}")
                    if streak >= args.required_samples:
                        print(
                            "[OK] 传感器启动检查通过："
                            f"{args.cloud_topic} publisher={cloud_publishers}；"
                            f"{args.odom_topic} publisher={odom_publishers}；"
                            f"连续 {streak} 组点云/里程计/TF 时间戳新鲜。"
                        )
                        return 0

            now = time.monotonic()
            if now >= next_log and streak == 0:
                print(f"[INFO] 启动检查：{last_reason}")
                next_log = now + 2.0
    except KeyboardInterrupt:
        print("[WARN] 传感器启动检查被中断。", file=sys.stderr)
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    metric_text = ""
    if latest_metrics:
        metric_text = (
            "；最后观测 "
            f"cloud={latest_metrics['cloud_age']:.3f}s "
            f"odom={latest_metrics['odom_age']:.3f}s "
            f"TF={latest_metrics['tf_age']:.3f}s"
        )
    print(
        f"[ERROR] 传感器启动检查在 {args.timeout:.1f}s 内未通过："
        f"{last_reason}{metric_text}",
        file=sys.stderr,
    )
    print(
        "[ACTION] Nav2 将保持 inactive。请检查是否有重复 publisher、"
        "FAST-LIO 是否积压，以及系统时间和 TF 时间戳。",
        file=sys.stderr,
    )
    return 1


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sensors", action="store_true")
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--max-age", type=float, default=0.5)
    parser.add_argument("--max-future", type=float, default=0.2)
    parser.add_argument("--required-samples", type=int, default=5)
    parser.add_argument(
        "--cloud-topic", default="/cloud_registered_body"
    )
    parser.add_argument("--odom-topic", default="/Odometry")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_footprint")
    args = parser.parse_args()
    args.timeout = max(1.0, args.timeout)
    args.max_age = max(0.0, args.max_age)
    args.max_future = max(0.0, args.max_future)
    args.required_samples = max(1, args.required_samples)
    return args


def main() -> None:
    args = parse_args()

    # Let Python raise KeyboardInterrupt promptly instead of leaving robot.sh
    # waiting for the checker timeout after Ctrl+C.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    status = run_sensor_check(args)
    raise SystemExit(status)


if __name__ == "__main__":
    main()
