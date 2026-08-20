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
    endpoint_list = list(endpoints)
    names = _endpoint_names(endpoint_list)
    if len(names) == len(endpoint_list):
        return ", ".join(names) if names else "none"

    labels = []
    for endpoint in endpoint_list:
        namespace = endpoint.node_namespace.rstrip("/")
        name = f"{namespace}/{endpoint.node_name}" or "/"
        gid = ".".join(f"{byte:02x}" for byte in endpoint.endpoint_gid)
        labels.append(f"{name}[gid={gid[-11:]}]")
    return ", ".join(labels) if labels else "none"


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
        self.cloud_received_monotonic: Optional[float] = None
        self.odom_received_monotonic: Optional[float] = None
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
        self.cloud_received_monotonic = time.monotonic()
        self.cloud_generation += 1

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom_msg = msg
        self.odom_received_monotonic = time.monotonic()
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


def _format_delta(value: Optional[float]) -> str:
    return "n/a" if value is None else f"{value:+.3f}s"


def _classify_timestamp_progress(
    cloud_stamp_delta: Optional[float],
    cloud_receive_delta: Optional[float],
    max_catchup_age_step: float,
) -> str:
    if cloud_stamp_delta is None or cloud_receive_delta is None:
        return "收集首个时间戳间隔"
    if cloud_stamp_delta <= 0.0:
        return "点云时间戳非单调"
    if cloud_stamp_delta - cloud_receive_delta > max_catchup_age_step:
        return "旧帧正在追赶"
    return "实时稳定"


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
    previous_cloud_stamp_ns: Optional[int] = None
    previous_odom_stamp_ns: Optional[int] = None
    previous_cloud_received: Optional[float] = None
    previous_odom_received: Optional[float] = None
    previous_cloud_age: Optional[float] = None
    latest_timestamp_state = "尚未收到成对点云/里程计"

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
                cloud_stamp_ns = _stamp_ns(node.cloud_msg.header.stamp)
                odom_stamp_ns = _stamp_ns(node.odom_msg.header.stamp)
                cloud_stamp_delta = (
                    None
                    if previous_cloud_stamp_ns is None
                    else (cloud_stamp_ns - previous_cloud_stamp_ns) / 1.0e9
                )
                odom_stamp_delta = (
                    None
                    if previous_odom_stamp_ns is None
                    else (odom_stamp_ns - previous_odom_stamp_ns) / 1.0e9
                )
                cloud_receive_delta = (
                    None
                    if previous_cloud_received is None
                    or node.cloud_received_monotonic is None
                    else node.cloud_received_monotonic - previous_cloud_received
                )
                odom_receive_delta = (
                    None
                    if previous_odom_received is None
                    or node.odom_received_monotonic is None
                    else node.odom_received_monotonic - previous_odom_received
                )
                cloud_age_change = (
                    None
                    if previous_cloud_age is None
                    else cloud_age - previous_cloud_age
                )
                latest_timestamp_state = _classify_timestamp_progress(
                    cloud_stamp_delta,
                    cloud_receive_delta,
                    args.max_catchup_age_step,
                )
                previous_cloud_stamp_ns = cloud_stamp_ns
                previous_odom_stamp_ns = odom_stamp_ns
                previous_cloud_received = node.cloud_received_monotonic
                previous_odom_received = node.odom_received_monotonic
                previous_cloud_age = cloud_age
                latest_metrics = {
                    "cloud_age": cloud_age,
                    "odom_age": odom_age,
                    "tf_age": tf_age if tf_age is not None else float("inf"),
                }
                diagnostics = (
                    "时间戳诊断："
                    f"cloud_stamp_step={_format_delta(cloud_stamp_delta)} "
                    f"cloud_receive_step={_format_delta(cloud_receive_delta)} "
                    f"cloud_age_step={_format_delta(cloud_age_change)}；"
                    f"odom_stamp_step={_format_delta(odom_stamp_delta)} "
                    f"odom_receive_step={_format_delta(odom_receive_delta)}；"
                    f"cloud-odom_stamp={_format_delta((cloud_stamp_ns - odom_stamp_ns) / 1.0e9)}；"
                    f"状态={latest_timestamp_state}"
                )

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

                not_ready = []
                if cloud_age > args.ready_max_age:
                    not_ready.append(
                        f"cloud 尚未追到实时 age={cloud_age:.3f}s"
                    )
                if odom_age > args.ready_max_age:
                    not_ready.append(
                        f"odom 尚未追到实时 age={odom_age:.3f}s"
                    )
                if tf_age is not None and tf_age > args.ready_max_age:
                    not_ready.append(
                        f"TF 尚未追到实时 age={tf_age:.3f}s"
                    )
                if cloud_stamp_delta is None or odom_stamp_delta is None:
                    not_ready.append("等待时间戳推进样本")
                elif cloud_stamp_delta <= 0.0 or odom_stamp_delta <= 0.0:
                    not_ready.append("点云或里程计时间戳非单调")
                elif latest_timestamp_state != "实时稳定":
                    not_ready.append(f"FAST-LIO {latest_timestamp_state}")

                if invalid:
                    streak = 0
                    last_reason = "；".join(invalid) + "；" + diagnostics
                elif not_ready:
                    streak = 0
                    last_reason = "；".join(not_ready) + "；" + diagnostics
                else:
                    streak += 1
                    last_reason = (
                        f"实时稳定样本 {streak}/{args.required_samples}："
                        f"cloud={cloud_age:.3f}s odom={odom_age:.3f}s "
                        f"TF={tf_age:.3f}s；{diagnostics}"
                    )
                    print(f"[INFO] {last_reason}")
                    if streak >= args.required_samples:
                        print(
                            "[OK] 传感器启动检查通过："
                            f"{args.cloud_topic} publisher={cloud_publishers}；"
                            f"{args.odom_topic} publisher={odom_publishers}；"
                            f"FAST-LIO 已追到实时，连续 {streak} 组点云/里程计/TF 时间戳稳定。"
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
            f"TF={latest_metrics['tf_age']:.3f}s；"
            f"时间戳状态={latest_timestamp_state}"
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
    parser.add_argument("--ready-max-age", type=float, default=0.3)
    parser.add_argument("--max-future", type=float, default=0.2)
    parser.add_argument("--max-catchup-age-step", type=float, default=0.03)
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
    args.ready_max_age = min(
        max(0.0, args.ready_max_age), args.max_age
    )
    args.max_future = max(0.0, args.max_future)
    args.max_catchup_age_step = max(0.0, args.max_catchup_age_step)
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
