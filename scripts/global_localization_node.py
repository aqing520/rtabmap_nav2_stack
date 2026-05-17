#!/usr/bin/env python3
"""
Global localization client node.

Workflow:
  1. Load global map PCD → call /set_global_map
  2. Switch engine via /set_engine
  3. Subscribe to /cloud_registered_body, wait for one scan
  4. Call /query
  5. Publish best result to /initialpose, then exit

Prerequisites:
  - hdl_global_localization_node running
  - FAST-LIO running (publishing /cloud_registered_body)

Usage:
  source install/setup.bash
  python3 scripts/global_localization_node.py [map.pcd] [--engine FPFH_RANSAC|BBS]
"""

import sys
import os
import glob
import math
import argparse
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

from hdl_global_localization.srv import SetGlobalMap, SetGlobalLocalizationEngine, QueryGlobalLocalization

CLOUD_MAP_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "cloud_map"
)

_COVARIANCE = [0.0] * 36
_COVARIANCE[0]  = 0.25   # x
_COVARIANCE[7]  = 0.25   # y
_COVARIANCE[35] = 0.068  # yaw


def find_latest_pcd(directory: str) -> str:
    files = sorted(glob.glob(os.path.join(directory, "*.pcd")))
    if not files:
        raise FileNotFoundError(f"No .pcd files in {directory}")
    return files[-1]


def load_pcd_as_cloud2(pcd_path: str) -> PointCloud2:
    """Read binary PCD (x y z [intensity]) → sensor_msgs/PointCloud2."""
    with open(pcd_path, "rb") as f:
        raw = f.read()

    header_end = raw.find(b"\nDATA ")
    header_text = raw[:header_end].decode("ascii", errors="ignore")
    data_line_end = raw.find(b"\n", header_end + 1)
    data_start = data_line_end + 1

    fields, sizes, types_str, counts = [], [], [], []
    n_points = 0
    for line in header_text.splitlines():
        parts = line.split()
        if not parts:
            continue
        key = parts[0].upper()
        if key == "FIELDS":   fields    = parts[1:]
        elif key == "SIZE":   sizes     = [int(x) for x in parts[1:]]
        elif key == "TYPE":   types_str = parts[1:]
        elif key == "COUNT":  counts    = [int(x) for x in parts[1:]]
        elif key == "POINTS": n_points  = int(parts[1])

    dtype_map = {"F": {4: np.float32, 8: np.float64},
                 "I": {1: np.int8,   2: np.int16,   4: np.int32},
                 "U": {1: np.uint8,  2: np.uint16,  4: np.uint32}}
    ros_dtype = {"F": {4: PointField.FLOAT32, 8: PointField.FLOAT64},
                 "I": {1: PointField.INT8,    2: PointField.INT16,   4: PointField.INT32},
                 "U": {1: PointField.UINT8,   2: PointField.UINT16,  4: PointField.UINT32}}

    msg = PointCloud2()
    msg.header.frame_id = "map"
    msg.height = 1
    msg.width  = n_points
    msg.is_bigendian = False
    msg.is_dense     = True

    offset = 0
    ros_fields = []
    for name, size, t, count in zip(fields, sizes, types_str, counts):
        for c in range(count):
            fname = name if count == 1 else f"{name}_{c}"
            pf = PointField()
            pf.name     = fname
            pf.offset   = offset
            pf.datatype = ros_dtype[t][size]
            pf.count    = 1
            ros_fields.append(pf)
            offset += size

    msg.fields     = ros_fields
    msg.point_step = offset
    msg.row_step   = offset * n_points
    msg.data       = list(raw[data_start: data_start + offset * n_points])
    return msg


class GlobalLocalizationClient(Node):
    def __init__(self, pcd_path: str, engine: str):
        super().__init__("global_localization_client")
        self._pcd_path = pcd_path
        self._engine   = engine
        self._result_event = threading.Event()
        self._scan_msg     = None
        self._done         = False

        self._pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        self._set_engine_cli = self.create_client(SetGlobalLocalizationEngine, "/set_engine")
        self._set_map_cli    = self.create_client(SetGlobalMap,                "/set_global_map")
        self._query_cli      = self.create_client(QueryGlobalLocalization,     "/query")

        # 同时兼容 RELIABLE 和 BEST_EFFORT 发布者
        self._sub = self.create_subscription(
            PointCloud2, "/cloud_registered_body", self._scan_cb,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE),
        )

    # ── called from main thread (spin runs in background thread) ──

    def setup(self):
        """Wait for services, set engine and global map. Blocking."""
        for cli in (self._set_map_cli, self._set_engine_cli, self._query_cli):
            self.get_logger().info(f"Waiting for {cli.srv_name} ...")
            if not cli.wait_for_service(timeout_sec=15.0):
                self.get_logger().error(f"Service {cli.srv_name} not available")
                raise RuntimeError(f"Service not available: {cli.srv_name}")

        self.get_logger().info(f"Loading map: {self._pcd_path}")
        cloud_msg = load_pcd_as_cloud2(self._pcd_path)
        self.get_logger().info(f"Sending map ({cloud_msg.width} pts) ...")
        map_req = SetGlobalMap.Request()
        map_req.global_map = cloud_msg
        self._call_sync(self._set_map_cli, map_req)
        self.get_logger().info("Map loaded. Waiting for scan on /cloud_registered_body ...")

    def wait_and_query(self, max_retries: int = 3, scan_timeout: float = 30.0) -> bool:
        """Block until scan arrives, run query, publish result. Returns success."""
        for attempt in range(1, max_retries + 1):
            arrived = self._result_event.wait(timeout=scan_timeout)
            if not arrived:
                self.get_logger().error(
                    f"Timed out waiting {scan_timeout:.0f}s for /cloud_registered_body. "
                    "Is FAST-LIO running?"
                )
                return False

            msg = self._scan_msg
            self.get_logger().info(
                f"[{attempt}/{max_retries}] Scan received ({msg.width} pts). Querying ..."
            )

            req = QueryGlobalLocalization.Request()
            req.cloud = msg
            req.max_num_candidates = 1
            res = self._call_sync(self._query_cli, req)

            if res.poses:
                break

            self.get_logger().warn(f"No pose candidates (attempt {attempt}), retrying with next scan ...")
            # 重置，等下一帧扫描
            self._done = False
            self._result_event.clear()
        else:
            self.get_logger().error(f"No pose candidates after {max_retries} attempts")
            return False

        pose   = res.poses[0]
        inlier = res.inlier_fractions[0]
        error  = res.errors[0]

        q = pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

        self.get_logger().info(
            f"x={pose.position.x:.3f}  y={pose.position.y:.3f}  "
            f"yaw={math.degrees(yaw):.1f}°  inlier={inlier:.3f}  err={error:.3f}"
        )
        if inlier < 0.1:
            self.get_logger().warn("Low inlier fraction — result may be unreliable")

        # 地面机器人只取 x, y, yaw，强制 z=0 / roll=0 / pitch=0
        qx, qy, qz, qw = 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)

        out = PoseWithCovarianceStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"
        out.pose.pose.position.x = float(pose.position.x)
        out.pose.pose.position.y = float(pose.position.y)
        out.pose.pose.position.z = 0.0
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        out.pose.covariance = _COVARIANCE[:]
        self._pub.publish(out)
        self.get_logger().info("/initialpose published")
        return True

    # ── internal helpers ──

    def _scan_cb(self, msg: PointCloud2):
        if self._done:
            return
        self._done = True
        self._scan_msg = msg
        self._result_event.set()

    def _call_sync(self, client, request):
        """Call a service synchronously from a non-spin thread."""
        future = client.call_async(request)
        # Busy-wait on the future; spin() in background thread delivers the response
        import time
        while not future.done():
            time.sleep(0.01)
        return future.result()


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("pcd", nargs="?", default=None)
    p.add_argument("--engine", default="FPFH_RANSAC", choices=["FPFH_RANSAC", "BBS"])
    known, _ = p.parse_known_args()
    return known


def main():
    args = parse_args()
    pcd_path = args.pcd or find_latest_pcd(CLOUD_MAP_DIR)
    if not os.path.isfile(pcd_path):
        print(f"[ERROR] PCD not found: {pcd_path}", file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    node = GlobalLocalizationClient(pcd_path, args.engine)

    # spin在后台线程运行，主线程做阻塞式service调用
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.setup()
        success = node.wait_and_query()
    except KeyboardInterrupt:
        success = False
    except Exception as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        success = False
    finally:
        rclpy.shutdown()          # 先让 spin() 退出
        spin_thread.join(timeout=3.0)  # 等 spin 线程结束
        node.destroy_node()       # 再销毁节点

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
