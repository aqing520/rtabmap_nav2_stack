#!/usr/bin/env python3
"""Export a database-level structural XYZI+normal map for garage profiles.

This is the strict, database-aware exporter used by the non-default
garage_structural_v1 path. It refuses to run without Admin.opt_ids/opt_poses,
processes each RTAB-Map node independently, orients scan normals from the
per-scan sensor viewpoint, then merges points/normals with sign alignment.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sqlite3
import sys

import numpy as np

from extract_pcd_from_db import (
    SCAN_FORMAT_CHANNELS,
    SCAN_FORMAT_NAMES,
    decompress_cv_mat,
    get_admin_optimized_poses,
    parse_scan_info,
    write_pcd_binary,
)


def estimate_normals_sensor_frame(xyz: np.ndarray, radius: float) -> np.ndarray:
    import open3d as o3d

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=float(radius), max_nn=30
        )
    )
    # Raw scan coordinates use the LiDAR origin as viewpoint.
    pcd.orient_normals_towards_camera_location(np.zeros(3))
    normals = np.asarray(pcd.normals, dtype=np.float32)
    finite = np.isfinite(normals).all(axis=1)
    normals[~finite] = np.array([0.0, 0.0, 1.0], dtype=np.float32)
    return normals


def voxel_merge_xyzin(
    points_xyzi: np.ndarray,
    normals: np.ndarray,
    voxel_size: float,
) -> tuple[np.ndarray, np.ndarray]:
    keys = np.floor(points_xyzi[:, :3] / voxel_size).astype(np.int64)
    buckets: dict[tuple[int, int, int], list] = {}
    for point, normal, key in zip(points_xyzi, normals, keys):
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
            continue
        if np.dot(bucket[1], normal) < 0.0:
            normal = -normal
        bucket[0] += point
        bucket[1] += normal
        bucket[2] += 1

    merged_points = []
    merged_normals = []
    for point_sum, normal_sum, count in buckets.values():
        point = point_sum / count
        norm = np.linalg.norm(normal_sum)
        normal = normal_sum / norm if norm > 1e-6 else np.array([0.0, 0.0, 1.0])
        merged_points.append(point)
        merged_normals.append(normal)
    if not merged_points:
        return np.empty((0, 4), np.float32), np.empty((0, 3), np.float32)
    return (
        np.asarray(merged_points, dtype=np.float32),
        np.asarray(merged_normals, dtype=np.float32),
    )


def stack_non_empty(items: list[np.ndarray], width: int) -> np.ndarray:
    arrays = [item for item in items if len(item)]
    if not arrays:
        return np.empty((0, width), dtype=np.float32)
    return np.vstack(arrays)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("database")
    parser.add_argument("output_dir")
    parser.add_argument("--profile", default="garage_structural_v1")
    parser.add_argument("--normal-radius", type=float, default=0.5)
    parser.add_argument("--surface-structural-voxel", type=float, default=0.10)
    parser.add_argument("--surface-large-plane-voxel", type=float, default=0.20)
    parser.add_argument("--keypoint-voxel", type=float, default=0.20)
    parser.add_argument("--horizontal-plane-normal-z", type=float, default=0.94)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    db_path = Path(args.database).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    if not db_path.is_file():
        print(f"[ERROR] database not found: {db_path}", file=sys.stderr)
        return 1

    poses = get_admin_optimized_poses(str(db_path), required=True)
    print(f"[INFO] Loaded {len(poses)} optimized poses from Admin.opt_*")

    con = sqlite3.connect(str(db_path))
    cur = con.cursor()
    cur.execute("SELECT id, scan_info, scan FROM Data WHERE scan IS NOT NULL ORDER BY id")

    structural_points = []
    structural_normals = []
    plane_points = []
    plane_normals = []
    keypoint_points = []
    keypoint_normals = []
    scan_format_name = "unknown"
    used_nodes = 0

    for node_id, scan_info_blob, scan_blob in cur.fetchall():
        if node_id not in poses:
            raise RuntimeError(f"optimized pose missing for node {node_id}")
        info = parse_scan_info(scan_info_blob)
        if info is None:
            continue
        fmt = info["format"]
        scan_format_name = SCAN_FORMAT_NAMES.get(fmt, f"unknown({fmt})")
        if SCAN_FORMAT_CHANNELS.get(fmt, 0) < 3:
            continue

        points, n_points, channels = decompress_cv_mat(scan_blob)
        if points is None or n_points == 0 or points.shape[1] < 3:
            continue
        xyz_sensor = points[:, :3].astype(np.float32)
        intensity = (
            points[:, 3:4].astype(np.float32)
            if points.shape[1] >= 4
            else np.zeros((points.shape[0], 1), dtype=np.float32)
        )
        normals_sensor = estimate_normals_sensor_frame(
            xyz_sensor, args.normal_radius
        )

        local = info["local_transform"]
        R_local = local[:3, :3].astype(np.float32)
        t_local = local[:3, 3].astype(np.float32)
        T_map = poses[node_id].astype(np.float32)
        R_map = T_map[:3, :3]
        t_map = T_map[:3, 3]

        xyz_body = (R_local @ xyz_sensor.T).T + t_local
        normals_body = (R_local @ normals_sensor.T).T
        xyz_map = (R_map @ xyz_body.T).T + t_map
        normals_map = (R_map @ normals_body.T).T
        normal_norm = np.linalg.norm(normals_map, axis=1, keepdims=True)
        normals_map = normals_map / np.maximum(normal_norm, 1e-6)

        xyzi_map = np.hstack([xyz_map, intensity]).astype(np.float32)
        horizontal = np.abs(normals_map[:, 2]) >= args.horizontal_plane_normal_z
        plane_points.append(xyzi_map[horizontal])
        plane_normals.append(normals_map[horizontal])
        structural_points.append(xyzi_map[~horizontal])
        structural_normals.append(normals_map[~horizontal])
        keypoint_points.append(xyzi_map[~horizontal])
        keypoint_normals.append(normals_map[~horizontal])
        used_nodes += 1

    con.close()
    if used_nodes == 0:
        print("[ERROR] no usable scans", file=sys.stderr)
        return 1

    structural_points = stack_non_empty(structural_points, 4)
    structural_normals = stack_non_empty(structural_normals, 3)
    plane_points = stack_non_empty(plane_points, 4)
    plane_normals = stack_non_empty(plane_normals, 3)
    keypoint_points = stack_non_empty(keypoint_points, 4)
    keypoint_normals = stack_non_empty(keypoint_normals, 3)

    if structural_points.size == 0 and plane_points.size == 0:
        print("[ERROR] no structural or plane points collected", file=sys.stderr)
        return 1
    if keypoint_points.size == 0:
        print("[ERROR] no structural keypoints collected", file=sys.stderr)
        return 1

    structural_points, structural_normals = voxel_merge_xyzin(
        structural_points, structural_normals, args.surface_structural_voxel
    )
    plane_points, plane_normals = voxel_merge_xyzin(
        plane_points, plane_normals, args.surface_large_plane_voxel
    )
    keypoint_points, keypoint_normals = voxel_merge_xyzin(
        keypoint_points, keypoint_normals, args.keypoint_voxel
    )

    surface_points = np.vstack([structural_points, plane_points])
    surface_normals = np.vstack([structural_normals, plane_normals])
    surface_curvature = np.zeros((surface_points.shape[0], 1), dtype=np.float32)
    keypoint_curvature = np.zeros((keypoint_points.shape[0], 1), dtype=np.float32)
    surface_xyzin = np.hstack(
        [surface_points, surface_normals, surface_curvature]
    ).astype(np.float32)
    keypoints_xyzin = np.hstack(
        [keypoint_points, keypoint_normals, keypoint_curvature]
    ).astype(np.float32)

    surface_path = output_dir / f"{args.profile}_surface_xyzi_normals.pcd"
    keypoint_path = output_dir / f"{args.profile}_keypoints_xyzi_normals.pcd"
    fields = [
        "x",
        "y",
        "z",
        "intensity",
        "normal_x",
        "normal_y",
        "normal_z",
        "curvature",
    ]
    write_pcd_binary(str(surface_path), surface_xyzin, fields=fields)
    write_pcd_binary(str(keypoint_path), keypoints_xyzin, fields=fields)
    print(f"[INFO] Scan format: {scan_format_name}")
    print(f"[INFO] Used nodes: {used_nodes}")
    print(f"[INFO] Surface points: {surface_xyzin.shape[0]}")
    print(f"[INFO] Keypoints: {keypoints_xyzin.shape[0]}")
    print(f"STRUCTURAL_SURFACE: {surface_path}")
    print(f"STRUCTURAL_KEYPOINTS: {keypoint_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
