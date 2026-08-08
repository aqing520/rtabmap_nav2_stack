#!/usr/bin/env python3
"""Replay saved HDL relocalization queries without starting sensors.

This tool is intentionally service-based: it can start only
hdl_global_localization_node, load an offline map cache, replay a saved query
PCD or rosbag2 directory, and write the same Top-K/score diagnostics used by
field runs. It does not start Livox, FAST-LIO, Nav2, or RTAB-Map.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

from hdl_global_localization.srv import (
    LoadGlobalMapCache,
    QueryGlobalLocalization,
    QueryGlobalLocalizationV2,
    SetGlobalLocalizationEngine,
)
from std_msgs.msg import String


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y * q.y + q.z * q.z),
    )


def pose_to_dict(pose) -> dict:
    return {
        "x": pose.position.x,
        "y": pose.position.y,
        "z": pose.position.z,
        "qx": pose.orientation.x,
        "qy": pose.orientation.y,
        "qz": pose.orientation.z,
        "qw": pose.orientation.w,
        "yaw_deg": math.degrees(yaw_from_quaternion(pose.orientation)),
    }


def estimate_normals(xyz: np.ndarray, radius: float) -> np.ndarray:
    import open3d as o3d

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30)
    )
    pcd.orient_normals_towards_camera_location(np.zeros(3))
    normals = np.asarray(pcd.normals, dtype=np.float32)
    finite = np.isfinite(normals).all(axis=1)
    normals[~finite] = np.array([0.0, 0.0, 1.0], dtype=np.float32)
    return normals


def make_cloud2(
    xyz: np.ndarray,
    frame_id: str,
    with_normals: bool = False,
    normal_radius: float = 0.5,
) -> PointCloud2:
    xyz = np.asarray(xyz, dtype=np.float32)
    if xyz.size == 0:
        raise RuntimeError("query cloud is empty")
    if xyz.ndim == 1:
        xyz = xyz.reshape(1, -1)
    xyz = xyz[:, :3]
    if with_normals:
        intensity = np.zeros((xyz.shape[0], 1), dtype=np.float32)
        normals = estimate_normals(xyz, normal_radius)
        payload = np.hstack([xyz, intensity, normals]).astype(np.float32)
        names = ["x", "y", "z", "intensity", "normal_x", "normal_y", "normal_z"]
    else:
        payload = xyz.astype(np.float32)
        names = ["x", "y", "z"]
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = payload.shape[0]
    msg.is_bigendian = False
    msg.is_dense = True
    msg.fields = []
    for i, name in enumerate(names):
        field = PointField()
        field.name = name
        field.offset = i * 4
        field.datatype = PointField.FLOAT32
        field.count = 1
        msg.fields.append(field)
    msg.point_step = payload.shape[1] * 4
    msg.row_step = msg.point_step * msg.width
    msg.data = payload.tobytes()
    return msg


def pcd_to_cloud2(
    path: Path,
    frame_id: str,
    with_normals: bool = False,
    normal_radius: float = 0.5,
) -> PointCloud2:
    import open3d as o3d

    pcd = o3d.io.read_point_cloud(str(path))
    xyz = np.asarray(pcd.points, dtype=np.float32)
    if xyz.size == 0:
        raise RuntimeError(f"query PCD is empty: {path}")
    return make_cloud2(xyz, frame_id, with_normals, normal_radius)


def ensure_normals(cloud: PointCloud2, normal_radius: float = 0.5) -> PointCloud2:
    field_names = [field.name for field in cloud.fields]
    if {"normal_x", "normal_y", "normal_z"}.issubset(field_names):
        return cloud
    xyz = pc2.read_points_numpy(
        cloud, field_names=["x", "y", "z"], skip_nans=True
    )
    output = make_cloud2(
        np.asarray(xyz, dtype=np.float32),
        cloud.header.frame_id,
        with_normals=True,
        normal_radius=normal_radius,
    )
    output.header.stamp = cloud.header.stamp
    return output


def latest_cloud_from_bag(bag_dir: Path, topic: str) -> PointCloud2:
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_dir), storage_id="sqlite3"
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    if topic not in topic_types:
        raise RuntimeError(f"topic {topic} not found in bag {bag_dir}")
    message_type = get_message(topic_types[topic])
    latest = None
    latest_stamp = -1
    while reader.has_next():
        name, data, _timestamp = reader.read_next()
        if name != topic:
            continue
        msg = deserialize_message(data, message_type)
        stamp = stamp_to_ns(msg.header.stamp)
        if stamp >= latest_stamp:
            latest = msg
            latest_stamp = stamp
    if latest is None:
        raise RuntimeError(f"no messages on {topic} in {bag_dir}")
    return latest


class ReplayClient(Node):
    def __init__(self):
        super().__init__("offline_relocalization_replay")
        self.set_engine_cli = self.create_client(
            SetGlobalLocalizationEngine, "/set_engine"
        )
        self.load_cache_cli = self.create_client(
            LoadGlobalMapCache, "/load_global_map_cache"
        )
        self.query_cli = self.create_client(QueryGlobalLocalization, "/query")
        self.query_v2_cli = self.create_client(QueryGlobalLocalizationV2, "/query_v2")

    def call_sync(self, client, request, timeout_sec: float):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise TimeoutError(f"service unavailable: {client.srv_name}")
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.02)
            if time.monotonic() >= deadline:
                raise TimeoutError(f"service timeout: {client.srv_name}")
        result = future.result()
        if result is None:
            raise RuntimeError(f"service call failed: {client.srv_name}")
        return result

    def setup_cache(self, source_pcd: Path, cache_dir: Path):
        engine_req = SetGlobalLocalizationEngine.Request()
        engine_req.engine_name = String(data="FPFH_RANSAC")
        self.call_sync(self.set_engine_cli, engine_req, 15.0)

        cache_req = LoadGlobalMapCache.Request()
        cache_req.source_pcd_path = str(source_pcd)
        cache_req.cache_directory = str(cache_dir)
        res = self.call_sync(self.load_cache_cli, cache_req, 60.0)
        if not res.success:
            raise RuntimeError(f"cache load rejected: {res.message}")
        return res

    def query(self, cloud: PointCloud2, max_candidates: int):
        req = QueryGlobalLocalization.Request()
        req.cloud = cloud
        req.max_num_candidates = max_candidates
        started = time.monotonic()
        res = self.call_sync(self.query_cli, req, 180.0)
        return res, time.monotonic() - started

    def query_v2(self, cloud: PointCloud2, max_candidates: int, deadline: float):
        req = QueryGlobalLocalizationV2.Request()
        req.cloud = cloud
        req.max_num_candidates = max_candidates
        req.matching_deadline_sec = deadline
        started = time.monotonic()
        res = self.call_sync(self.query_v2_cli, req, 180.0)
        return res, time.monotonic() - started


def start_node() -> subprocess.Popen:
    return subprocess.Popen(
        ["ros2", "run", "hdl_global_localization", "hdl_global_localization_node"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-pcd", required=True)
    parser.add_argument("--cache-dir", required=True)
    parser.add_argument("--query-pcd")
    parser.add_argument("--query-bag")
    parser.add_argument("--query-topic", default="/cloud_registered_body")
    parser.add_argument("--query-v2", action="store_true")
    parser.add_argument("--max-candidates", type=int, default=20)
    parser.add_argument("--matching-deadline-sec", type=float, default=45.0)
    parser.add_argument("--normal-radius", type=float, default=0.5)
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--start-node", action="store_true")
    parser.add_argument("--output", default="-")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if bool(args.query_pcd) == bool(args.query_bag):
        print("Specify exactly one of --query-pcd or --query-bag", file=sys.stderr)
        return 2

    node_process = start_node() if args.start_node else None
    try:
        if args.query_pcd:
            query_cloud = pcd_to_cloud2(
                Path(args.query_pcd).resolve(),
                "base_footprint",
                with_normals=args.query_v2,
                normal_radius=args.normal_radius,
            )
        else:
            query_cloud = latest_cloud_from_bag(
                Path(args.query_bag).resolve(), args.query_topic
            )
            if args.query_v2:
                query_cloud = ensure_normals(query_cloud, args.normal_radius)

        rclpy.init()
        client = ReplayClient()
        cache_res = client.setup_cache(
            Path(args.source_pcd).resolve(), Path(args.cache_dir).resolve()
        )
        output = {
            "schema_version": 1,
            "source_pcd": str(Path(args.source_pcd).resolve()),
            "cache_dir": str(Path(args.cache_dir).resolve()),
            "cache_key": cache_res.cache_key,
            "query_interface": "query_v2" if args.query_v2 else "query",
            "repeats": [],
        }
        for repeat in range(max(1, args.repeats)):
            if args.query_v2:
                res, elapsed = client.query_v2(
                    query_cloud, args.max_candidates, args.matching_deadline_sec
                )
                candidates = []
                for i, pose in enumerate(res.poses):
                    candidates.append(
                        {
                            "rank": i + 1,
                            "valid": bool(res.candidate_valid[i]),
                            "rejection_reason": res.candidate_rejection_reasons[i],
                            "coarse_inlier": res.coarse_inliers[i],
                            "coarse_error": res.coarse_errors[i],
                            "fine_overlap_q2m": res.fine_overlap_q2m[i],
                            "fine_overlap_m2q": res.fine_overlap_m2q[i],
                            "fine_trimmed_rmse": res.fine_trimmed_rmse[i],
                            "degeneracy_ratio": res.degeneracy_ratios[i],
                            "pose": pose_to_dict(pose),
                        }
                    )
                output["repeats"].append(
                    {
                        "repeat": repeat + 1,
                        "elapsed_sec": elapsed,
                        "success": bool(res.success),
                        "rejection_reason": res.rejection_reason,
                        "candidates": candidates,
                    }
                )
            else:
                res, elapsed = client.query(query_cloud, args.max_candidates)
                output["repeats"].append(
                    {
                        "repeat": repeat + 1,
                        "elapsed_sec": elapsed,
                        "candidates": [
                            {
                                "rank": i + 1,
                                "inlier": res.inlier_fractions[i],
                                "error": res.errors[i],
                                "pose": pose_to_dict(pose),
                            }
                            for i, pose in enumerate(res.poses)
                        ],
                    }
                )
        if args.output == "-":
            print(json.dumps(output, ensure_ascii=False, indent=2))
        else:
            Path(args.output).write_text(
                json.dumps(output, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
        client.destroy_node()
        rclpy.shutdown()
        return 0
    finally:
        if node_process is not None:
            if node_process.poll() is None:
                try:
                    os.killpg(node_process.pid, signal.SIGINT)
                    node_process.wait(timeout=5.0)
                except Exception:
                    os.killpg(node_process.pid, signal.SIGTERM)


if __name__ == "__main__":
    raise SystemExit(main())
