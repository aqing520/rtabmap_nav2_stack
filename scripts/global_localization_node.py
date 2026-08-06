#!/usr/bin/env python3
"""
Global localization client node.

Workflow:
  1. Load global map PCD → call /set_global_map
  2. Switch engine via /set_engine
  3. Subscribe to /cloud_registered_body, wait for one scan
  4. Call /query
  5. Wait for an /initialpose subscriber
  6. Publish the best result once with transient-local durability
  7. Trust the published /initialpose and exit

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
import time
import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

from hdl_global_localization.srv import SetGlobalMap, SetGlobalLocalizationEngine, QueryGlobalLocalization

WORKSPACE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CLOUD_MAP_DIR = os.path.join(WORKSPACE_ROOT, "db", "pcd")

_COVARIANCE = [0.0] * 36
_COVARIANCE[0]  = 0.25   # x
_COVARIANCE[7]  = 0.25   # y
_COVARIANCE[35] = 0.068  # yaw


def _yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1 - 2 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


def _stamp_nanoseconds(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def find_latest_pcd(directory: str) -> str:
    files = sorted(glob.glob(os.path.join(directory, "*.pcd")))
    if not files:
        raise FileNotFoundError(f"No .pcd files in {directory}")
    return files[-1]


def _adaptive_voxel_size(pcd, target: int, tolerance: float = 0.2,
                         voxel_min: float = 0.05, voxel_max: float = 5.0,
                         max_iter: int = 20) -> tuple:
    """Binary-search for voxel size that yields target ± tolerance*target points."""
    # 若最小体素已无法达到目标点数，说明点云太稀疏，直接返回原始点云
    if len(pcd.voxel_down_sample(voxel_min).points) < target * (1 - tolerance):
        return pcd, voxel_min, len(pcd.points)

    lo, hi = voxel_min, voxel_max
    best_pcd, best_voxel = pcd, lo
    for _ in range(max_iter):
        mid = (lo + hi) / 2.0
        down = pcd.voxel_down_sample(mid)
        n = len(down.points)
        best_pcd, best_voxel = down, mid
        if abs(n - target) <= target * tolerance:
            break
        if n > target:
            lo = mid
        else:
            hi = mid
    return best_pcd, best_voxel, len(best_pcd.points)


def load_pcd_as_cloud2(pcd_path: str, voxel_size: float = None,
                       target_points: int = 150_000,
                       z_min: float = 0.1, z_max: float = 2.2) -> PointCloud2:
    """Load PCD, height-filter, pre-downsample with open3d, return xyz-only PointCloud2.

    Pre-downsampling avoids sending millions of raw points over the ROS2
    service — the hdl node would downsample them anyway.
    If voxel_size is None, adaptively searches for a voxel size that yields
    ~target_points (default 150 000, tolerance ±20 %).
    z_min/z_max clip floor reflections and ceiling to keep only structurally
    distinctive points (columns, walls, furniture).
    """
    import open3d as o3d

    pcd = o3d.io.read_point_cloud(pcd_path)
    n_raw = len(pcd.points)

    # height filter: remove floor and ceiling
    pts = np.asarray(pcd.points)
    mask = (pts[:, 2] >= z_min) & (pts[:, 2] <= z_max)
    pcd = pcd.select_by_index(np.where(mask)[0])
    n_filtered = len(pcd.points)
    print(f"[map] height filter z=[{z_min}, {z_max}]: {n_raw} → {n_filtered} pts")

    if voxel_size is not None:
        pcd_down = pcd.voxel_down_sample(voxel_size)
        n_down = len(pcd_down.points)
        print(f"[map] {n_filtered} pts → {n_down} pts (voxel={voxel_size:.3f}m, fixed)")
    elif n_filtered <= target_points:
        pcd_down = pcd
        n_down = n_filtered
        print(f"[map] {n_filtered} pts (already ≤ target={target_points}, no downsampling)")
    else:
        pcd_down, voxel_size, n_down = _adaptive_voxel_size(pcd, target_points)
        print(f"[map] {n_filtered} pts → {n_down} pts (voxel={voxel_size:.3f}m, adaptive target={target_points})")

    xyz = np.asarray(pcd_down.points, dtype=np.float32)

    msg = PointCloud2()
    msg.header.frame_id = "map"
    msg.height = 1
    msg.width  = n_down
    msg.is_bigendian = False
    msg.is_dense     = True

    fields = []
    for i, name in enumerate(["x", "y", "z"]):
        pf = PointField()
        pf.name     = name
        pf.offset   = i * 4
        pf.datatype = PointField.FLOAT32
        pf.count    = 1
        fields.append(pf)

    msg.fields     = fields
    msg.point_step = 12  # 3 × float32
    msg.row_step   = 12 * n_down
    msg.data       = xyz.tobytes()
    return msg


class InitialPoseHandler(Node):
    """Publish or observe one /initialpose without secondary confirmation."""

    def __init__(self, node_name: str, monitor_manual_pose: bool = False):
        super().__init__(node_name)
        self._manual_initialpose_event = threading.Event()

        self._pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._manual_initialpose_sub = None
        if monitor_manual_pose:
            # VOLATILE is intentional: do not consume an old latched automatic
            # pose. Only accept a pose newly published by RViz after fallback
            # mode starts.
            self._manual_initialpose_sub = self.create_subscription(
                PoseWithCovarianceStamped,
                "/initialpose",
                self._manual_initialpose_cb,
                QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                ),
            )

    def wait_for_initialpose_subscriber(self, timeout_sec: float) -> bool:
        """Wait until RTAB-Map (or another consumer) has discovered the publisher."""
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            count = self._pub.get_subscription_count()
            if count > 0:
                self.get_logger().info(
                    f"/initialpose subscriber discovered ({count}); "
                    "publishing one latched pose."
                )
                return True
            time.sleep(0.05)

        self.get_logger().error(
            f"No /initialpose subscriber discovered within {timeout_sec:.1f}s."
        )
        return False

    def publish_once(
        self,
        pose_msg: PoseWithCovarianceStamped,
        subscriber_timeout: float,
        delivery_grace_sec: float = 0.5,
    ) -> bool:
        if not self.wait_for_initialpose_subscriber(subscriber_timeout):
            return False

        pose_msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(pose_msg)
        self.get_logger().info(
            "/initialpose published once (TRANSIENT_LOCAL); "
            "accepting it without localization_pose/TF confirmation."
        )

        deadline = time.monotonic() + max(0.0, delivery_grace_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            time.sleep(0.05)
        return rclpy.ok()

    def wait_for_manual_initialpose(self, manual_pose_timeout: float) -> bool:
        self.get_logger().info(
            "Waiting for a new volatile /initialpose from RViz ..."
        )
        deadline = time.monotonic() + manual_pose_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if self._manual_initialpose_event.wait(
                timeout=min(0.1, max(0.0, remaining))
            ):
                self.get_logger().info(
                    "New manual /initialpose received; accepting it without "
                    "localization_pose/TF confirmation."
                )
                return True

        if rclpy.ok():
            self.get_logger().error(
                f"Timed out after {manual_pose_timeout:.1f}s waiting for "
                "a new manual /initialpose."
            )
        return False

    def _manual_initialpose_cb(self, msg: PoseWithCovarianceStamped):
        del msg
        self._manual_initialpose_event.set()


class GlobalLocalizationClient(InitialPoseHandler):
    def __init__(self, pcd_path: str, engine: str):
        super().__init__("global_localization_client")
        self._pcd_path = pcd_path
        self._engine   = engine
        self._scan_lock = threading.Lock()
        self._result_event = threading.Event()
        self._scan_msg     = None
        self._done         = False

        self._set_engine_cli = self.create_client(SetGlobalLocalizationEngine, "/set_engine")
        self._set_map_cli    = self.create_client(SetGlobalMap,                "/set_global_map")
        self._query_cli      = self.create_client(QueryGlobalLocalization,     "/query")

        # 同时兼容 RELIABLE 和 BEST_EFFORT 发布者
        self._sub = self.create_subscription(
            PointCloud2, "/cloud_registered_body", self._scan_cb,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

    # ── called from main thread (spin runs in background thread) ──

    def setup(self):
        """Wait for services, set engine and global map. Blocking."""
        for cli in (self._set_map_cli, self._set_engine_cli, self._query_cli):
            self.get_logger().info(f"Waiting for {cli.srv_name} ...")
            if not cli.wait_for_service(timeout_sec=15.0):
                self.get_logger().error(f"Service {cli.srv_name} not available")
                raise RuntimeError(f"Service not available: {cli.srv_name}")

        engine_req = SetGlobalLocalizationEngine.Request()
        engine_req.engine_name = String(data=self._engine)
        self._call_sync(self._set_engine_cli, engine_req, timeout_sec=15.0)
        self.get_logger().info(f"Global localization engine set to {self._engine}")

        self.get_logger().info(f"Loading map: {self._pcd_path}")
        cloud_msg = load_pcd_as_cloud2(self._pcd_path)
        self.get_logger().info(f"Sending map ({cloud_msg.width} pts) ...")
        map_req = SetGlobalMap.Request()
        map_req.global_map = cloud_msg
        self._call_sync(self._set_map_cli, map_req, timeout_sec=120.0)
        # FPFH map setup can take tens of seconds. A scan received while the
        # map is loading is no longer representative of the robot's current
        # pose, so explicitly discard it and wait for a post-setup scan.
        self._reset_scan()
        self.get_logger().info(
            "Map loaded. Discarded scans cached during map setup; waiting "
            "for a fresh scan on /cloud_registered_body ..."
        )

    def wait_and_query(self, max_retries: int = 1, scan_timeout: float = 30.0,
                       min_inlier: float = 0.98,
                       max_error: float = math.inf,
                       max_scan_age: float = 0.5,
                       max_future_skew: float = 0.2,
                       subscriber_timeout: float = 15.0,
                       ) -> bool:
        """Block until scan arrives, run query, publish result. Returns success."""
        attempt = 0
        while attempt < max_retries:
            deadline = time.monotonic() + scan_timeout
            msg = None
            while rclpy.ok() and time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                if self._result_event.wait(
                    timeout=min(0.1, max(0.0, remaining))
                ):
                    with self._scan_lock:
                        msg = self._scan_msg
                    if msg is None:
                        self._reset_scan()
                        continue

                    stamp_ns = _stamp_nanoseconds(msg.header.stamp)
                    if stamp_ns <= 0:
                        self.get_logger().warning(
                            "Rejecting scan with zero header timestamp; "
                            "waiting for a fresh scan without consuming a "
                            "matching attempt."
                        )
                        self._reset_scan()
                        msg = None
                        continue

                    scan_age = (
                        self.get_clock().now().nanoseconds - stamp_ns
                    ) / 1.0e9
                    if (
                        scan_age > max_scan_age
                        or scan_age < -max_future_skew
                    ):
                        self.get_logger().warning(
                            f"Rejecting stale/invalid scan age={scan_age:.3f}s "
                            f"(allowed {-max_future_skew:.3f}.."
                            f"{max_scan_age:.3f}s); waiting for a fresh scan "
                            "without consuming a matching attempt."
                        )
                        self._reset_scan()
                        msg = None
                        continue
                    break
            if not rclpy.ok():
                return False
            if msg is None:
                self.get_logger().error(
                    f"Timed out waiting {scan_timeout:.0f}s for a fresh "
                    f"/cloud_registered_body scan (max age "
                    f"{max_scan_age:.3f}s). Is FAST-LIO running without "
                    "backlog?"
                )
                return False

            attempt += 1
            self.get_logger().info(
                f"[matching attempt {attempt}/{max_retries}] "
                f"Fresh scan received ({msg.width} pts). Querying ..."
            )

            req = QueryGlobalLocalization.Request()
            req.cloud = msg
            req.max_num_candidates = 1
            res = self._call_sync(self._query_cli, req, timeout_sec=120.0)

            if not res.poses:
                self.get_logger().warn(f"No pose candidates (attempt {attempt}), retrying with next scan ...")
                self._reset_scan()
                continue

            inlier = res.inlier_fractions[0]
            error  = res.errors[0]

            if inlier < min_inlier:
                self.get_logger().warn(
                    f"[matching attempt {attempt}/{max_retries}] "
                    "Low inlier fraction "
                    f"({inlier:.3f} < {min_inlier}) — rejecting, retrying ..."
                )
                self._reset_scan()
                continue

            if not math.isfinite(error) or error > max_error:
                self.get_logger().warn(
                    f"[matching attempt {attempt}/{max_retries}] "
                    "Matching error too high "
                    f"({error:.3f} > {max_error:.3f}) — rejecting, retrying ..."
                )
                self._reset_scan()
                continue

            # ── 成功：打印结果并发布 ──
            pose = res.poses[0]
            yaw = _yaw_from_quaternion(pose.orientation)

            self.get_logger().info(
                f"x={pose.position.x:.3f}  y={pose.position.y:.3f}  "
                f"yaw={math.degrees(yaw):.1f}°  inlier={inlier:.3f}  err={error:.3f}"
            )

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
            return self.publish_once(
                out,
                subscriber_timeout=subscriber_timeout,
            )

        self.get_logger().warn(
            f"Failed after {max_retries} matching attempts — "
            "no /initialpose published. "
            "Navigation must remain inactive."
        )
        return False

    # ── internal helpers ──

    def _scan_cb(self, msg: PointCloud2):
        with self._scan_lock:
            if self._done:
                return
            self._done = True
            self._scan_msg = msg
            self._result_event.set()

    def _reset_scan(self):
        with self._scan_lock:
            self._done = False
            self._scan_msg = None
            self._result_event.clear()

    def _call_sync(self, client, request, timeout_sec: float):
        """Call a service synchronously from a non-spin thread."""
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while not future.done():
            if not rclpy.ok():
                raise InterruptedError(
                    f"ROS shutdown while calling {client.srv_name}"
                )
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"Service call timed out after {timeout_sec:.0f}s: "
                    f"{client.srv_name}"
                )
            time.sleep(0.01)
        result = future.result()
        if result is None:
            raise RuntimeError(f"Service call failed: {client.srv_name}")
        return result


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("pcd", nargs="?", default=None)
    p.add_argument("--wait-manual", action="store_true")
    p.add_argument("--engine", default="FPFH_RANSAC", choices=["FPFH_RANSAC", "BBS"])
    p.add_argument("--min-inlier", type=float, default=0.98)
    p.add_argument("--max-error", type=float, default=math.inf)
    p.add_argument("--max-retries", type=int, default=1)
    p.add_argument("--scan-timeout", type=float, default=30.0)
    p.add_argument("--max-scan-age", type=float, default=0.5)
    p.add_argument("--max-future-skew", type=float, default=0.2)
    p.add_argument("--subscriber-timeout", type=float, default=15.0)
    p.add_argument("--manual-pose-timeout", type=float, default=86400.0)
    known, _ = p.parse_known_args()
    return known


def _spin_node(node: Node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


def main():
    args = parse_args()

    rclpy.init()
    if args.wait_manual:
        node = InitialPoseHandler(
            "manual_initialpose_monitor",
            monitor_manual_pose=True,
        )
    else:
        pcd_path = args.pcd or find_latest_pcd(CLOUD_MAP_DIR)
        if not os.path.isfile(pcd_path):
            print(f"[ERROR] PCD not found: {pcd_path}", file=sys.stderr)
            rclpy.shutdown()
            sys.exit(1)
        node = GlobalLocalizationClient(pcd_path, args.engine)

    # spin在后台线程运行，主线程做阻塞式service调用
    spin_thread = threading.Thread(target=_spin_node, args=(node,), daemon=True)
    spin_thread.start()

    try:
        if args.wait_manual:
            result = node.wait_for_manual_initialpose(
                manual_pose_timeout=max(1.0, args.manual_pose_timeout),
            )
        else:
            node.setup()
            result = node.wait_and_query(
                max_retries=max(1, args.max_retries),
                scan_timeout=max(1.0, args.scan_timeout),
                min_inlier=max(0.0, min(1.0, args.min_inlier)),
                max_error=args.max_error,
                max_scan_age=max(0.0, args.max_scan_age),
                max_future_skew=max(0.0, args.max_future_skew),
                subscriber_timeout=max(1.0, args.subscriber_timeout),
            )
    except KeyboardInterrupt:
        result = False
    except Exception as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        result = False
    finally:
        if rclpy.ok():
            rclpy.shutdown()      # 先让 spin() 退出
        spin_thread.join(timeout=3.0)  # 等 spin 线程结束
        node.destroy_node()       # 再销毁节点

    sys.exit(0 if result else 1)


if __name__ == "__main__":
    main()
