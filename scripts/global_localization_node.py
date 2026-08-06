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
  7. Keep the publisher alive until RTAB-Map localization_pose and TF confirm it

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
from rclpy.time import Time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformException, TransformListener

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


def _angle_distance(first: float, second: float) -> float:
    return abs(math.atan2(
        math.sin(first - second),
        math.cos(first - second),
    ))


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


class InitialPoseAcceptanceMonitor(Node):
    """Publish or observe /initialpose and wait for RTAB-Map to apply it."""

    def __init__(self, node_name: str, monitor_manual_pose: bool = False):
        super().__init__(node_name)
        self._pose_lock = threading.Lock()
        self._latest_localization_pose = None
        self._latest_localization_received_at = 0.0
        self._manual_initialpose = None
        self._manual_initialpose_received_at = 0.0
        self._manual_initialpose_event = threading.Event()

        self._pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._localization_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/localization_pose",
            self._localization_pose_cb,
            QoSProfile(
                depth=5,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
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

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer,
            self,
            spin_thread=False,
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

    def publish_and_confirm(
        self,
        pose_msg: PoseWithCovarianceStamped,
        subscriber_timeout: float,
        confirmation_timeout: float,
        linear_tolerance: float,
        yaw_tolerance: float,
        max_linear_variance: float,
        max_yaw_variance: float,
        map_frame: str,
        base_frame: str,
    ) -> bool:
        if not self.wait_for_initialpose_subscriber(subscriber_timeout):
            return False

        not_before = time.monotonic()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        published_stamp_ns = _stamp_nanoseconds(pose_msg.header.stamp)
        self._pub.publish(pose_msg)
        self.get_logger().info(
            "/initialpose published once (TRANSIENT_LOCAL); "
            "latch will remain until localization is confirmed."
        )

        return self._wait_for_confirmation(
            pose_msg,
            not_before=not_before,
            published_stamp_ns=published_stamp_ns,
            confirmation_timeout=confirmation_timeout,
            linear_tolerance=linear_tolerance,
            yaw_tolerance=yaw_tolerance,
            max_linear_variance=max_linear_variance,
            max_yaw_variance=max_yaw_variance,
            map_frame=map_frame,
            base_frame=base_frame,
        )

    def wait_for_manual_and_confirm(
        self,
        manual_pose_timeout: float,
        confirmation_timeout: float,
        linear_tolerance: float,
        yaw_tolerance: float,
        max_linear_variance: float,
        max_yaw_variance: float,
        map_frame: str,
        base_frame: str,
    ) -> bool:
        self.get_logger().info(
            "Waiting for a new volatile /initialpose from RViz ..."
        )
        deadline = time.monotonic() + manual_pose_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if self._manual_initialpose_event.wait(
                timeout=min(0.1, max(0.0, remaining))
            ):
                break

        if not rclpy.ok():
            return False
        if not self._manual_initialpose_event.is_set():
            self.get_logger().error(
                f"Timed out after {manual_pose_timeout:.1f}s waiting for "
                "a new manual /initialpose."
            )
            return False

        with self._pose_lock:
            pose_msg = self._manual_initialpose
            not_before = self._manual_initialpose_received_at

        # Use receipt time rather than trusting the RViz header stamp when
        # deciding whether later localization/TF data is fresh.
        published_stamp_ns = self.get_clock().now().nanoseconds
        self.get_logger().info(
            "New manual /initialpose received; waiting for RTAB-Map and TF "
            "confirmation before activating navigation."
        )
        return self._wait_for_confirmation(
            pose_msg,
            not_before=not_before,
            published_stamp_ns=published_stamp_ns,
            confirmation_timeout=confirmation_timeout,
            linear_tolerance=linear_tolerance,
            yaw_tolerance=yaw_tolerance,
            max_linear_variance=max_linear_variance,
            max_yaw_variance=max_yaw_variance,
            map_frame=map_frame,
            base_frame=base_frame,
        )

    def _wait_for_confirmation(
        self,
        expected_msg: PoseWithCovarianceStamped,
        not_before: float,
        published_stamp_ns: int,
        confirmation_timeout: float,
        linear_tolerance: float,
        yaw_tolerance: float,
        max_linear_variance: float,
        max_yaw_variance: float,
        map_frame: str,
        base_frame: str,
    ) -> bool:
        expected_pose = expected_msg.pose.pose
        expected_yaw = _yaw_from_quaternion(expected_pose.orientation)
        deadline = time.monotonic() + confirmation_timeout
        last_reason = "waiting for a new /localization_pose"
        next_progress_log = time.monotonic() + 2.0

        while rclpy.ok() and time.monotonic() < deadline:
            with self._pose_lock:
                localization_msg = self._latest_localization_pose
                localization_received_at = self._latest_localization_received_at

            if (
                localization_msg is None
                or localization_received_at < not_before
            ):
                last_reason = "waiting for a post-publication /localization_pose"
            elif localization_msg.header.frame_id != map_frame:
                last_reason = (
                    "localization_pose frame is "
                    f"'{localization_msg.header.frame_id}', expected '{map_frame}'"
                )
            elif _stamp_nanoseconds(localization_msg.header.stamp) < published_stamp_ns:
                last_reason = "latest /localization_pose still predates /initialpose"
            else:
                covariance_ok, covariance_reason = self._covariance_is_valid(
                    localization_msg,
                    max_linear_variance,
                    max_yaw_variance,
                )
                if not covariance_ok:
                    last_reason = covariance_reason
                else:
                    localization_pose = localization_msg.pose.pose
                    localization_linear_error = math.hypot(
                        localization_pose.position.x - expected_pose.position.x,
                        localization_pose.position.y - expected_pose.position.y,
                    )
                    localization_yaw_error = _angle_distance(
                        _yaw_from_quaternion(localization_pose.orientation),
                        expected_yaw,
                    )
                    if localization_linear_error > linear_tolerance:
                        last_reason = (
                            "RTAB-Map localization differs from initial pose by "
                            f"{localization_linear_error:.3f}m "
                            f"(limit {linear_tolerance:.3f}m)"
                        )
                    elif localization_yaw_error > yaw_tolerance:
                        last_reason = (
                            "RTAB-Map localization yaw differs by "
                            f"{math.degrees(localization_yaw_error):.1f}° "
                            f"(limit {math.degrees(yaw_tolerance):.1f}°)"
                        )
                    else:
                        tf_ok, tf_reason, tf_errors = self._check_tf(
                            expected_pose,
                            expected_yaw,
                            published_stamp_ns,
                            linear_tolerance,
                            yaw_tolerance,
                            map_frame,
                            base_frame,
                        )
                        if tf_ok:
                            tf_linear_error, tf_yaw_error = tf_errors
                            self.get_logger().info(
                                "Relocalization confirmed: RTAB-Map published "
                                "a valid localization_pose and "
                                f"{map_frame}->{base_frame} matches "
                                f"(TF error={tf_linear_error:.3f}m/"
                                f"{math.degrees(tf_yaw_error):.1f}°)."
                            )
                            self.get_logger().info(
                                "Confirmation complete; destroying this node "
                                "will clear the transient-local /initialpose latch."
                            )
                            return True
                        last_reason = tf_reason

            now = time.monotonic()
            if now >= next_progress_log:
                self.get_logger().info(
                    f"Waiting for relocalization confirmation: {last_reason}"
                )
                next_progress_log = now + 2.0
            time.sleep(0.05)

        self.get_logger().error(
            f"Relocalization was not confirmed within "
            f"{confirmation_timeout:.1f}s: {last_reason}. "
            "Navigation must remain inactive."
        )
        return False

    def _check_tf(
        self,
        expected_pose,
        expected_yaw: float,
        published_stamp_ns: int,
        linear_tolerance: float,
        yaw_tolerance: float,
        map_frame: str,
        base_frame: str,
    ):
        try:
            transform = self._tf_buffer.lookup_transform(
                map_frame,
                base_frame,
                Time(),
            )
        except TransformException as exc:
            return False, f"waiting for {map_frame}->{base_frame} TF: {exc}", None

        if _stamp_nanoseconds(transform.header.stamp) < published_stamp_ns:
            return (
                False,
                f"latest {map_frame}->{base_frame} TF still predates /initialpose",
                None,
            )

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        linear_error = math.hypot(
            translation.x - expected_pose.position.x,
            translation.y - expected_pose.position.y,
        )
        yaw_error = _angle_distance(
            _yaw_from_quaternion(rotation),
            expected_yaw,
        )
        if linear_error > linear_tolerance:
            return (
                False,
                f"{map_frame}->{base_frame} TF differs from initial pose by "
                f"{linear_error:.3f}m (limit {linear_tolerance:.3f}m)",
                None,
            )
        if yaw_error > yaw_tolerance:
            return (
                False,
                f"{map_frame}->{base_frame} TF yaw differs by "
                f"{math.degrees(yaw_error):.1f}° "
                f"(limit {math.degrees(yaw_tolerance):.1f}°)",
                None,
            )
        return True, "ok", (linear_error, yaw_error)

    @staticmethod
    def _covariance_is_valid(
        msg: PoseWithCovarianceStamped,
        max_linear_variance: float,
        max_yaw_variance: float,
    ):
        covariance = msg.pose.covariance
        values = (covariance[0], covariance[7], covariance[35])
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            return False, "RTAB-Map localization covariance is invalid"
        if max(covariance[0], covariance[7]) > max_linear_variance:
            return (
                False,
                "RTAB-Map localization linear variance is "
                f"{max(covariance[0], covariance[7]):.3f} "
                f"(limit {max_linear_variance:.3f})",
            )
        if covariance[35] > max_yaw_variance:
            return (
                False,
                "RTAB-Map localization yaw variance is "
                f"{covariance[35]:.3f} (limit {max_yaw_variance:.3f})",
            )
        return True, "ok"

    def _localization_pose_cb(self, msg: PoseWithCovarianceStamped):
        with self._pose_lock:
            self._latest_localization_pose = msg
            self._latest_localization_received_at = time.monotonic()

    def _manual_initialpose_cb(self, msg: PoseWithCovarianceStamped):
        if self._manual_initialpose_event.is_set():
            return
        with self._pose_lock:
            self._manual_initialpose = msg
            self._manual_initialpose_received_at = time.monotonic()
        self._manual_initialpose_event.set()


class GlobalLocalizationClient(InitialPoseAcceptanceMonitor):
    def __init__(self, pcd_path: str, engine: str):
        super().__init__("global_localization_client")
        self._pcd_path = pcd_path
        self._engine   = engine
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
        self.get_logger().info("Map loaded. Waiting for scan on /cloud_registered_body ...")

    def wait_and_query(self, max_retries: int = 3, scan_timeout: float = 30.0,
                       min_inlier: float = 0.98,
                       max_error: float = math.inf,
                       subscriber_timeout: float = 15.0,
                       confirmation_timeout: float = 30.0,
                       linear_tolerance: float = 1.0,
                       yaw_tolerance: float = math.radians(30.0),
                       max_linear_variance: float = 1.0,
                       max_yaw_variance: float = 1.0,
                       map_frame: str = "map",
                       base_frame: str = "base_footprint") -> bool:
        """Block until scan arrives, run query, publish result. Returns success."""
        for attempt in range(1, max_retries + 1):
            deadline = time.monotonic() + scan_timeout
            arrived = False
            while rclpy.ok() and time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                if self._result_event.wait(
                    timeout=min(0.1, max(0.0, remaining))
                ):
                    arrived = True
                    break
            if not rclpy.ok():
                return False
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
            res = self._call_sync(self._query_cli, req, timeout_sec=120.0)

            if not res.poses:
                self.get_logger().warn(f"No pose candidates (attempt {attempt}), retrying with next scan ...")
                self._done = False
                self._result_event.clear()
                continue

            inlier = res.inlier_fractions[0]
            error  = res.errors[0]

            if inlier < min_inlier:
                self.get_logger().warn(
                    f"[{attempt}/{max_retries}] Low inlier fraction "
                    f"({inlier:.3f} < {min_inlier}) — rejecting, retrying ..."
                )
                self._done = False
                self._result_event.clear()
                continue

            if not math.isfinite(error) or error > max_error:
                self.get_logger().warn(
                    f"[{attempt}/{max_retries}] Matching error too high "
                    f"({error:.3f} > {max_error:.3f}) — rejecting, retrying ..."
                )
                self._done = False
                self._result_event.clear()
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
            return self.publish_and_confirm(
                out,
                subscriber_timeout=subscriber_timeout,
                confirmation_timeout=confirmation_timeout,
                linear_tolerance=linear_tolerance,
                yaw_tolerance=yaw_tolerance,
                max_linear_variance=max_linear_variance,
                max_yaw_variance=max_yaw_variance,
                map_frame=map_frame,
                base_frame=base_frame,
            )

        self.get_logger().warn(
            f"Failed after {max_retries} attempts — no /initialpose published. "
            "Navigation must remain inactive."
        )
        return False

    # ── internal helpers ──

    def _scan_cb(self, msg: PointCloud2):
        if self._done:
            return
        self._done = True
        self._scan_msg = msg
        self._result_event.set()

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
    p.add_argument("--max-retries", type=int, default=3)
    p.add_argument("--scan-timeout", type=float, default=30.0)
    p.add_argument("--subscriber-timeout", type=float, default=15.0)
    p.add_argument("--confirmation-timeout", type=float, default=30.0)
    p.add_argument("--manual-pose-timeout", type=float, default=86400.0)
    p.add_argument("--linear-tolerance", type=float, default=1.0)
    p.add_argument("--yaw-tolerance-deg", type=float, default=30.0)
    p.add_argument("--max-linear-variance", type=float, default=1.0)
    p.add_argument("--max-yaw-variance", type=float, default=1.0)
    p.add_argument("--map-frame", default="map")
    p.add_argument("--base-frame", default="base_footprint")
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
        node = InitialPoseAcceptanceMonitor(
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
            result = node.wait_for_manual_and_confirm(
                manual_pose_timeout=max(1.0, args.manual_pose_timeout),
                confirmation_timeout=max(1.0, args.confirmation_timeout),
                linear_tolerance=max(0.0, args.linear_tolerance),
                yaw_tolerance=math.radians(
                    max(0.0, args.yaw_tolerance_deg)
                ),
                max_linear_variance=max(0.0, args.max_linear_variance),
                max_yaw_variance=max(0.0, args.max_yaw_variance),
                map_frame=args.map_frame,
                base_frame=args.base_frame,
            )
        else:
            node.setup()
            result = node.wait_and_query(
                max_retries=max(1, args.max_retries),
                scan_timeout=max(1.0, args.scan_timeout),
                min_inlier=max(0.0, min(1.0, args.min_inlier)),
                max_error=args.max_error,
                subscriber_timeout=max(1.0, args.subscriber_timeout),
                confirmation_timeout=max(1.0, args.confirmation_timeout),
                linear_tolerance=max(0.0, args.linear_tolerance),
                yaw_tolerance=math.radians(
                    max(0.0, args.yaw_tolerance_deg)
                ),
                max_linear_variance=max(0.0, args.max_linear_variance),
                max_yaw_variance=max(0.0, args.max_yaw_variance),
                map_frame=args.map_frame,
                base_frame=args.base_frame,
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
