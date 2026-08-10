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
import json
import signal
import subprocess
import threading
import time
from bisect import bisect_left
from collections import deque
from datetime import datetime
from pathlib import Path
import numpy as np

from hdl_map_preprocessing import preprocess_legacy_pcd

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from hdl_global_localization.srv import (
    LoadGlobalMapCache,
    SetGlobalMap,
    SetGlobalLocalizationEngine,
    QueryGlobalLocalization,
    QueryGlobalLocalizationV2,
)
try:
    from rosbag2_interfaces.srv import Snapshot
except Exception:  # pragma: no cover - optional runtime dependency
    Snapshot = None

WORKSPACE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CLOUD_MAP_DIR = os.path.join(WORKSPACE_ROOT, "db", "pcd")
DEFAULT_RUNS_DIR = os.path.join(WORKSPACE_ROOT, "db", "relocalization_runs")

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


def _quaternion_to_rotation_matrix(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return np.eye(3, dtype=np.float64)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _odom_to_matrix(msg: Odometry) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = _quaternion_to_rotation_matrix(msg.pose.pose.orientation)
    transform[:3, 3] = [
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z,
    ]
    return transform


def _pointcloud2_to_xyzi(msg: PointCloud2) -> np.ndarray:
    field_names = [field.name for field in msg.fields]
    required = ["x", "y", "z"]
    optional = ["intensity"] if "intensity" in field_names else []
    array = pc2.read_points_numpy(
        msg,
        field_names=required + optional,
        skip_nans=True,
    )
    if array.size == 0:
        return np.empty((0, 4), dtype=np.float32)
    array = np.asarray(array, dtype=np.float32)
    if array.ndim == 1:
        array = array.reshape(1, -1)
    if array.shape[1] == 3:
        intensity = np.zeros((array.shape[0], 1), dtype=np.float32)
        array = np.hstack([array, intensity])
    return array[:, :4]


def _estimate_oriented_normals(
    xyz: np.ndarray,
    radius: float,
) -> np.ndarray:
    import open3d as o3d

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=float(radius), max_nn=30
        )
    )
    pcd.orient_normals_towards_camera_location(np.zeros(3))
    normals = np.asarray(pcd.normals, dtype=np.float32)
    finite = np.isfinite(normals).all(axis=1)
    normals[~finite] = np.array([0.0, 0.0, 1.0], dtype=np.float32)
    return normals


def _voxel_merge_xyzin(
    points: np.ndarray,
    normals: np.ndarray,
    voxel_size: float,
    max_points: int,
) -> tuple[np.ndarray, np.ndarray]:
    if points.size == 0:
        return points.reshape(0, 4), normals.reshape(0, 3)

    keys = np.floor(points[:, :3] / float(voxel_size)).astype(np.int64)
    buckets: dict[tuple[int, int, int], list] = {}
    for point, normal, key in zip(points, normals, keys):
        if not np.isfinite(point[:3]).all() or not np.isfinite(normal).all():
            continue
        bucket_key = tuple(int(v) for v in key)
        bucket = buckets.get(bucket_key)
        if bucket is None:
            buckets[bucket_key] = [
                point.astype(np.float64).copy(),
                normal.astype(np.float64).copy(),
                1,
            ]
        else:
            if np.dot(bucket[1], normal) < 0.0:
                normal = -normal
            bucket[0] += point
            bucket[1] += normal
            bucket[2] += 1

    merged_points = []
    merged_normals = []
    for point_sum, normal_sum, count in buckets.values():
        point = point_sum / count
        normal_norm = np.linalg.norm(normal_sum)
        if normal_norm <= 1e-6:
            normal = np.array([0.0, 0.0, 1.0])
        else:
            normal = normal_sum / normal_norm
        merged_points.append(point)
        merged_normals.append(normal)

    if not merged_points:
        return np.empty((0, 4), dtype=np.float32), np.empty((0, 3), dtype=np.float32)
    merged_points = np.asarray(merged_points, dtype=np.float32)
    merged_normals = np.asarray(merged_normals, dtype=np.float32)
    if max_points > 0 and len(merged_points) > max_points:
        # Deterministic stride sampling keeps offline replay reproducible.
        indices = np.linspace(0, len(merged_points) - 1, max_points, dtype=np.int64)
        merged_points = merged_points[indices]
        merged_normals = merged_normals[indices]
    return merged_points, merged_normals


def _xyzin_to_cloud2(
    points: np.ndarray,
    normals: np.ndarray,
    frame_id: str,
    stamp,
) -> PointCloud2:
    points = np.asarray(points, dtype=np.float32)
    normals = np.asarray(normals, dtype=np.float32)
    payload = np.hstack([points[:, :4], normals[:, :3]]).astype(np.float32)
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = int(payload.shape[0])
    msg.is_bigendian = False
    msg.is_dense = True
    names = ["x", "y", "z", "intensity", "normal_x", "normal_y", "normal_z"]
    msg.fields = []
    for index, name in enumerate(names):
        field = PointField()
        field.name = name
        field.offset = index * 4
        field.datatype = PointField.FLOAT32
        field.count = 1
        msg.fields.append(field)
    msg.point_step = 28
    msg.row_step = msg.point_step * msg.width
    msg.data = payload.tobytes()
    return msg


def find_latest_pcd(directory: str) -> str:
    files = sorted(glob.glob(os.path.join(directory, "*.pcd")))
    if not files:
        raise FileNotFoundError(f"No .pcd files in {directory}")
    return files[-1]


def load_pcd_as_cloud2(pcd_path: str, voxel_size: float = None,
                       target_points: int = 0,
                       z_min: float | None = None,
                       z_max: float | None = None) -> PointCloud2:
    """Load a PCD and preserve all points unless filtering is requested."""
    pcd_down, metadata = preprocess_legacy_pcd(
        pcd_path,
        voxel_size=voxel_size,
        target_points=target_points,
        z_min=z_min,
        z_max=z_max,
    )
    n_raw = metadata["raw_points"]
    n_filtered = metadata["height_filtered_points"]
    n_down = metadata["prepared_points"]
    if z_min is None and z_max is None:
        print(f"[map] height filter disabled: {n_raw} pts preserved")
    else:
        print(f"[map] height filter z=[{z_min}, {z_max}]: {n_raw} → {n_filtered} pts")
    if metadata["python_voxel"] is None:
        print(f"[map] {n_filtered} pts preserved without Python downsampling")
    else:
        print(
            f"[map] {n_filtered} pts → {n_down} pts "
            f"(voxel={metadata['python_voxel']:.3f}m)"
        )

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
    def __init__(
        self,
        pcd_path: str,
        engine: str,
        cache_dir: str | None = None,
        cache_source_path: str | None = None,
        allow_online_map_setup: bool = False,
        use_query_v2: bool = False,
        query_accumulation_sec: float = 0.0,
        query_min_frames: int = 5,
        query_max_frames: int = 30,
        query_max_points: int = 30000,
        query_surface_voxel: float = 0.10,
        odom_sync_tolerance: float = 0.02,
        normal_radius: float = 0.5,
        diagnostic_candidates: int = 20,
        save_diagnostics: bool = True,
        capture_query_bag: bool = False,
        runs_dir: str = DEFAULT_RUNS_DIR,
    ):
        super().__init__("global_localization_client")
        self._pcd_path = pcd_path
        self._engine   = engine
        self._cache_dir = cache_dir
        self._cache_source_path = cache_source_path or pcd_path
        self._allow_online_map_setup = allow_online_map_setup
        self._use_query_v2 = use_query_v2
        self._query_accumulation_sec = max(0.0, query_accumulation_sec)
        self._query_min_frames = max(1, query_min_frames)
        self._query_max_frames = max(1, query_max_frames)
        self._query_max_points = max(1, query_max_points)
        self._query_surface_voxel = query_surface_voxel
        self._odom_sync_tolerance_ns = int(max(0.0, odom_sync_tolerance) * 1e9)
        self._normal_radius = normal_radius
        self._diagnostic_candidates = max(1, diagnostic_candidates)
        self._save_diagnostics = save_diagnostics
        self._capture_query_bag = capture_query_bag
        self._run_dir = None
        if self._save_diagnostics or self._capture_query_bag:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            self._run_dir = Path(runs_dir).expanduser().resolve() / ts
            self._run_dir.mkdir(parents=True, exist_ok=True)
        self._bag_process: subprocess.Popen | None = None
        self._snapshot_cli = None
        self._scan_lock = threading.Lock()
        self._odom_lock = threading.Lock()
        self._result_event = threading.Event()
        self._scan_msg     = None
        self._done         = False
        self._cloud_buffer = deque()
        self._odom_buffer = deque()

        self._set_engine_cli = self.create_client(SetGlobalLocalizationEngine, "/set_engine")
        self._set_map_cli    = self.create_client(SetGlobalMap,                "/set_global_map")
        self._load_cache_cli = self.create_client(
            LoadGlobalMapCache, "/load_global_map_cache"
        )
        self._query_cli      = self.create_client(QueryGlobalLocalization,     "/query")
        self._query_v2_cli = self.create_client(QueryGlobalLocalizationV2, "/query_v2")
        if Snapshot is not None:
            self._snapshot_cli = self.create_client(
                Snapshot, "/rosbag2_recorder/snapshot"
            )

        # 同时兼容 RELIABLE 和 BEST_EFFORT 发布者
        self._sub = self.create_subscription(
            PointCloud2, "/cloud_registered_body", self._scan_cb,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self._odom_sub = self.create_subscription(
            Odometry, "/Odometry", self._odom_cb,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        if self._use_query_v2 and self._query_accumulation_sec <= 0.0:
            self.get_logger().warning(
                "/query_v2 requires oriented normals. The live FAST-LIO "
                "single-frame cloud normally does not contain normal_x/y/z, "
                "so enable query accumulation unless a custom normal-bearing "
                "cloud is being supplied."
            )

    # ── called from main thread (spin runs in background thread) ──

    def setup(self):
        """Wait for services, set engine and global map. Blocking."""
        required_clients = [self._set_engine_cli]
        if self._use_query_v2:
            required_clients.append(self._query_v2_cli)
        else:
            required_clients.append(self._query_cli)
        if self._engine == "FPFH_RANSAC" and self._cache_dir:
            required_clients.append(self._load_cache_cli)
        else:
            required_clients.append(self._set_map_cli)
        for cli in required_clients:
            self.get_logger().info(f"Waiting for {cli.srv_name} ...")
            if not cli.wait_for_service(timeout_sec=15.0):
                self.get_logger().error(f"Service {cli.srv_name} not available")
                raise RuntimeError(f"Service not available: {cli.srv_name}")

        engine_req = SetGlobalLocalizationEngine.Request()
        engine_req.engine_name = String(data=self._engine)
        self._call_sync(self._set_engine_cli, engine_req, timeout_sec=15.0)
        self.get_logger().info(f"Global localization engine set to {self._engine}")

        cache_loaded = False
        if self._engine == "FPFH_RANSAC" and self._cache_dir:
            self.get_logger().info(f"Loading offline map cache: {self._cache_dir}")
            cache_req = LoadGlobalMapCache.Request()
            cache_req.source_pcd_path = self._cache_source_path
            cache_req.cache_directory = self._cache_dir
            cache_res = self._call_sync(
                self._load_cache_cli, cache_req, timeout_sec=30.0
            )
            if cache_res.success:
                cache_loaded = True
                self.get_logger().info(
                    "Map cache loaded "
                    f"key={cache_res.cache_key} "
                    f"surface={cache_res.surface_points} "
                    f"keypoints={cache_res.keypoint_points} "
                    f"load={cache_res.cache_load_sec:.3f}s "
                    f"index={cache_res.feature_index_sec:.3f}s"
                )
            elif not self._allow_online_map_setup:
                raise RuntimeError(
                    "Offline FPFH cache rejected: "
                    f"{cache_res.message}. Run './robot.sh cache' first."
                )
            else:
                self.get_logger().warning(
                    "Offline cache rejected; explicit online fallback enabled: "
                    f"{cache_res.message}"
                )

        if not cache_loaded:
            if (
                self._engine == "FPFH_RANSAC"
                and not self._allow_online_map_setup
            ):
                raise RuntimeError(
                    "FPFH_RANSAC requires an offline cache. "
                    "Run './robot.sh cache' or explicitly enable online fallback."
                )
            self.get_logger().info(f"Loading map online: {self._pcd_path}")
            cloud_msg = load_pcd_as_cloud2(self._pcd_path)
            self.get_logger().info(f"Sending map ({cloud_msg.width} pts) ...")
            map_req = SetGlobalMap.Request()
            map_req.global_map = cloud_msg
            self._call_sync(self._set_map_cli, map_req, timeout_sec=120.0)
        # FPFH map setup can take tens of seconds. A scan received while the
        # map is loading is no longer representative of the robot's current
        # pose, so explicitly discard it and wait for a post-setup scan.
        self._reset_scan()
        self._start_snapshot_recorder()
        self.get_logger().info(
            "Map ready. Discarded scans cached during map setup; waiting "
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

            query_msg = msg
            if self._query_accumulation_sec > 0.0:
                try:
                    query_msg = self._build_accumulated_query(msg)
                except Exception as exception:
                    self.get_logger().error(
                        "Query accumulation failed; structural mode will not "
                        f"fall back to a single frame: {exception}"
                    )
                    self._write_rejection_metadata(
                        attempt,
                        "query_accumulation_failed",
                        str(exception),
                    )
                    self._reset_scan()
                    continue

            self._snapshot_query_bag(attempt)

            if self._use_query_v2:
                res = self._query_v2(query_msg)
                poses = res.poses
                inlier = res.coarse_inliers[0] if res.coarse_inliers else 0.0
                error = res.coarse_errors[0] if res.coarse_errors else math.inf
                if not res.success or not poses or not res.candidate_valid[0]:
                    reason = res.rejection_reason or (
                        res.candidate_rejection_reasons[0]
                        if res.candidate_rejection_reasons else "unknown"
                    )
                    self.get_logger().warn(
                        f"/query_v2 rejected attempt {attempt}: {reason}"
                    )
                    self._write_query_v2_metadata(attempt, res, accepted=False)
                    self._reset_scan()
                    continue
                pose = poses[0]
                self._write_query_v2_metadata(attempt, res, accepted=True)
            else:
                req = QueryGlobalLocalization.Request()
                req.cloud = query_msg
                req.max_num_candidates = self._diagnostic_candidates
                res = self._call_sync(self._query_cli, req, timeout_sec=120.0)

                if not res.poses:
                    self.get_logger().warn(f"No pose candidates (attempt {attempt}), retrying with next scan ...")
                    self._write_query_metadata(attempt, res, accepted=False)
                    self._reset_scan()
                    continue

                inlier = res.inlier_fractions[0]
                error  = res.errors[0]
                pose = res.poses[0]

            if not self._use_query_v2:
                self._write_query_metadata(attempt, res, accepted=False)

            if not self._use_query_v2 and inlier < min_inlier:
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
                if self._use_query_v2:
                    self._write_query_v2_metadata(attempt, res, accepted=False)
                self._reset_scan()
                continue

            # ── 成功：打印结果并发布 ──
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
            if not self._use_query_v2:
                self._write_query_metadata(attempt, res, accepted=True)
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
            self._cloud_buffer.append(msg)
            self._trim_cloud_buffer()
            if self._done:
                return
            self._done = True
            self._scan_msg = msg
            self._result_event.set()

    def _odom_cb(self, msg: Odometry):
        with self._odom_lock:
            self._odom_buffer.append(msg)
            cutoff = (
                self.get_clock().now().nanoseconds
                - int(max(10.0, self._query_accumulation_sec + 5.0) * 1e9)
            )
            while self._odom_buffer and _stamp_nanoseconds(
                self._odom_buffer[0].header.stamp
            ) < cutoff:
                self._odom_buffer.popleft()

    def _reset_scan(self):
        with self._scan_lock:
            self._done = False
            self._scan_msg = None
            self._result_event.clear()

    def _trim_cloud_buffer(self):
        cutoff = (
            self.get_clock().now().nanoseconds
            - int(max(10.0, self._query_accumulation_sec + 5.0) * 1e9)
        )
        while self._cloud_buffer and _stamp_nanoseconds(
            self._cloud_buffer[0].header.stamp
        ) < cutoff:
            self._cloud_buffer.popleft()

    def _nearest_odom(self, stamp_ns: int, odoms: list[Odometry]) -> Odometry | None:
        stamps = [_stamp_nanoseconds(odom.header.stamp) for odom in odoms]
        position = bisect_left(stamps, stamp_ns)
        candidates = []
        if position < len(odoms):
            candidates.append(odoms[position])
        if position > 0:
            candidates.append(odoms[position - 1])
        if not candidates:
            return None
        best = min(
            candidates,
            key=lambda odom: abs(_stamp_nanoseconds(odom.header.stamp) - stamp_ns),
        )
        if abs(_stamp_nanoseconds(best.header.stamp) - stamp_ns) > self._odom_sync_tolerance_ns:
            return None
        return best

    def _build_accumulated_query(self, anchor_msg: PointCloud2) -> PointCloud2:
        anchor_stamp_ns = _stamp_nanoseconds(anchor_msg.header.stamp)
        start_ns = anchor_stamp_ns - int(self._query_accumulation_sec * 1e9)
        with self._scan_lock:
            clouds = [
                msg for msg in self._cloud_buffer
                if start_ns <= _stamp_nanoseconds(msg.header.stamp) <= anchor_stamp_ns
            ][-self._query_max_frames:]
        with self._odom_lock:
            odoms = sorted(
                list(self._odom_buffer),
                key=lambda msg: _stamp_nanoseconds(msg.header.stamp),
            )
        anchor_odom = self._nearest_odom(anchor_stamp_ns, odoms)
        if anchor_odom is None:
            raise RuntimeError("no odometry paired with anchor cloud")
        T_anchor_inv = np.linalg.inv(_odom_to_matrix(anchor_odom))

        transformed_points = []
        transformed_normals = []
        paired_frames = 0
        for cloud in clouds:
            cloud_stamp_ns = _stamp_nanoseconds(cloud.header.stamp)
            odom = self._nearest_odom(cloud_stamp_ns, odoms)
            if odom is None:
                continue
            xyzi = _pointcloud2_to_xyzi(cloud)
            if xyzi.shape[0] == 0:
                continue
            normals = _estimate_oriented_normals(xyzi[:, :3], self._normal_radius)
            T_anchor_frame = T_anchor_inv @ _odom_to_matrix(odom)
            xyz_h = np.hstack(
                [xyzi[:, :3].astype(np.float64), np.ones((xyzi.shape[0], 1))]
            )
            xyz_anchor = (T_anchor_frame @ xyz_h.T).T[:, :3]
            normals_anchor = (T_anchor_frame[:3, :3] @ normals.T).T
            transformed_points.append(
                np.hstack([xyz_anchor, xyzi[:, 3:4]]).astype(np.float32)
            )
            transformed_normals.append(normals_anchor.astype(np.float32))
            paired_frames += 1

        if paired_frames < self._query_min_frames:
            raise RuntimeError(
                f"paired frames {paired_frames} < required {self._query_min_frames}"
            )
        points = np.vstack(transformed_points)
        normals = np.vstack(transformed_normals)
        points, normals = _voxel_merge_xyzin(
            points,
            normals,
            voxel_size=self._query_surface_voxel,
            max_points=self._query_max_points,
        )
        if len(points) < 3:
            raise RuntimeError("accumulated query has fewer than 3 points")
        self.get_logger().info(
            "Accumulated query submap: "
            f"frames={paired_frames} points={len(points)} "
            f"voxel={self._query_surface_voxel:.3f}m"
        )
        return _xyzin_to_cloud2(
            points,
            normals,
            frame_id=anchor_msg.header.frame_id,
            stamp=anchor_msg.header.stamp,
        )

    def _query_v2(self, cloud_msg: PointCloud2):
        req = QueryGlobalLocalizationV2.Request()
        req.cloud = cloud_msg
        req.max_num_candidates = 1
        req.matching_deadline_sec = 45.0
        return self._call_sync(self._query_v2_cli, req, timeout_sec=120.0)

    def _start_snapshot_recorder(self):
        if not self._capture_query_bag or self._run_dir is None:
            return
        if Snapshot is None:
            self.get_logger().warning(
                "rosbag2 Snapshot service type unavailable; query bag capture disabled"
            )
            return
        bag_dir = self._run_dir / "raw_query_bag"
        log_path = self._run_dir / "rosbag2_record.log"
        command = [
            "ros2", "bag", "record",
            "--snapshot-mode",
            "--max-cache-size", "536870912",
            "-s", "sqlite3",
            "-o", str(bag_dir),
            "/cloud_registered_body",
            "/Odometry",
        ]
        log_file = log_path.open("w", encoding="utf-8")
        self._bag_process = subprocess.Popen(
            command,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self.get_logger().info(f"Started rosbag2 snapshot recorder: {bag_dir}")

    def _snapshot_query_bag(self, attempt: int):
        if not self._capture_query_bag or self._snapshot_cli is None:
            return
        if not self._snapshot_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning("rosbag2 snapshot service not available")
            return
        try:
            res = self._call_sync(
                self._snapshot_cli,
                Snapshot.Request(),
                timeout_sec=10.0,
            )
            self.get_logger().info(
                f"rosbag2 snapshot attempt={attempt} success={res.success}"
            )
        except Exception as exception:
            self.get_logger().warning(f"rosbag2 snapshot failed: {exception}")

    def stop_capture(self):
        if self._bag_process is None:
            return
        if self._bag_process.poll() is None:
            try:
                os.killpg(self._bag_process.pid, signal.SIGINT)
                self._bag_process.wait(timeout=5.0)
            except Exception:
                try:
                    os.killpg(self._bag_process.pid, signal.SIGTERM)
                except Exception:
                    pass
        self._bag_process = None

    def _pose_to_dict(self, pose) -> dict:
        return {
            "position": {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
            },
            "orientation": {
                "x": pose.orientation.x,
                "y": pose.orientation.y,
                "z": pose.orientation.z,
                "w": pose.orientation.w,
            },
            "yaw_deg": math.degrees(_yaw_from_quaternion(pose.orientation)),
        }

    def _write_json(self, name: str, payload: dict):
        if not self._save_diagnostics or self._run_dir is None:
            return
        path = self._run_dir / name
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        os.replace(temporary, path)

    def _write_query_metadata(self, attempt: int, res, accepted: bool):
        candidates = []
        for i, pose in enumerate(res.poses):
            candidates.append(
                {
                    "rank": i + 1,
                    "inlier": res.inlier_fractions[i],
                    "error": res.errors[i],
                    "pose": self._pose_to_dict(pose),
                }
            )
        self._write_json(
            f"attempt_{attempt:02d}_query.json",
            {
                "schema_version": 1,
                "engine": self._engine,
                "query_interface": "query",
                "pcd_path": self._pcd_path,
                "cache_dir": self._cache_dir,
                "cache_source_path": self._cache_source_path,
                "accepted": accepted,
                "top_candidates": candidates,
            },
        )

    def _write_query_v2_metadata(self, attempt: int, res, accepted: bool):
        candidates = []
        for i, pose in enumerate(res.poses):
            candidates.append(
                {
                    "rank": i + 1,
                    "valid": bool(res.candidate_valid[i]),
                    "rejection_reason": res.candidate_rejection_reasons[i],
                    "refinement_status": res.refinement_status[i],
                    "coarse_inlier": res.coarse_inliers[i],
                    "coarse_error": res.coarse_errors[i],
                    "fine_overlap_q2m": res.fine_overlap_q2m[i],
                    "fine_overlap_m2q": res.fine_overlap_m2q[i],
                    "fine_trimmed_rmse": res.fine_trimmed_rmse[i],
                    "degeneracy_ratio": res.degeneracy_ratios[i],
                    "top1_top2_margin": res.top1_top2_margins[i],
                    "pose": self._pose_to_dict(pose),
                }
            )
        coarse = []
        for i, pose in enumerate(res.diagnostic_coarse_poses):
            coarse.append(
                {
                    "rank": i + 1,
                    "inlier": res.diagnostic_coarse_inliers[i],
                    "error": res.diagnostic_coarse_errors[i],
                    "pose": self._pose_to_dict(pose),
                }
            )
        self._write_json(
            f"attempt_{attempt:02d}_query_v2.json",
            {
                "schema_version": 1,
                "engine": self._engine,
                "query_interface": "query_v2",
                "pcd_path": self._pcd_path,
                "cache_dir": self._cache_dir,
                "cache_source_path": self._cache_source_path,
                "accepted": accepted,
                "success": bool(res.success),
                "score_profile": res.score_profile,
                "rejection_reason": res.rejection_reason,
                "total_time_sec": res.total_time_sec,
                "diagnostic_coarse": coarse,
                "candidates": candidates,
            },
        )

    def _write_rejection_metadata(self, attempt: int, reason: str, detail: str):
        self._write_json(
            f"attempt_{attempt:02d}_rejection.json",
            {
                "schema_version": 1,
                "engine": self._engine,
                "pcd_path": self._pcd_path,
                "cache_dir": self._cache_dir,
                "cache_source_path": self._cache_source_path,
                "accepted": False,
                "rejection_reason": reason,
                "detail": detail,
            },
        )

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
    p.add_argument("--cache-dir", default=None)
    p.add_argument("--cache-source-path", default=None)
    p.add_argument("--allow-online-map-setup", action="store_true")
    p.add_argument("--query-v2", action="store_true")
    p.add_argument("--query-accumulation-sec", type=float, default=0.0)
    p.add_argument("--query-min-frames", type=int, default=5)
    p.add_argument("--query-max-frames", type=int, default=30)
    p.add_argument("--query-max-points", type=int, default=30000)
    p.add_argument("--query-surface-voxel", type=float, default=0.10)
    p.add_argument("--odom-sync-tolerance", type=float, default=0.02)
    p.add_argument("--normal-radius", type=float, default=0.5)
    p.add_argument("--diagnostic-candidates", type=int, default=20)
    p.add_argument("--runs-dir", default=DEFAULT_RUNS_DIR)
    p.add_argument("--no-save-diagnostics", action="store_true")
    p.add_argument("--capture-query-bag", action="store_true")
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
        node = GlobalLocalizationClient(
            pcd_path,
            args.engine,
            cache_dir=args.cache_dir,
            cache_source_path=args.cache_source_path,
            allow_online_map_setup=args.allow_online_map_setup,
            use_query_v2=args.query_v2,
            query_accumulation_sec=args.query_accumulation_sec,
            query_min_frames=args.query_min_frames,
            query_max_frames=args.query_max_frames,
            query_max_points=args.query_max_points,
            query_surface_voxel=args.query_surface_voxel,
            odom_sync_tolerance=args.odom_sync_tolerance,
            normal_radius=args.normal_radius,
            diagnostic_candidates=args.diagnostic_candidates,
            save_diagnostics=not args.no_save_diagnostics,
            capture_query_bag=args.capture_query_bag,
            runs_dir=args.runs_dir,
        )

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
        if hasattr(node, "stop_capture"):
            node.stop_capture()
        if rclpy.ok():
            rclpy.shutdown()      # 先让 spin() 退出
        spin_thread.join(timeout=3.0)  # 等 spin 线程结束
        node.destroy_node()       # 再销毁节点

    sys.exit(0 if result else 1)


if __name__ == "__main__":
    main()
