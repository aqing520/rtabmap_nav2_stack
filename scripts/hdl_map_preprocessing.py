#!/usr/bin/env python3
"""Shared HDL map preprocessing helpers.

The legacy profile intentionally reproduces the preprocessing previously
embedded in global_localization_node.py.
"""

from __future__ import annotations

import numpy as np


def adaptive_voxel_size(
    pcd,
    target: int,
    tolerance: float = 0.2,
    voxel_min: float = 0.05,
    voxel_max: float = 5.0,
    max_iter: int = 20,
):
    if len(pcd.voxel_down_sample(voxel_min).points) < target * (1 - tolerance):
        return pcd, voxel_min, len(pcd.points)

    lo, hi = voxel_min, voxel_max
    best_pcd, best_voxel = pcd, lo
    for _ in range(max_iter):
        mid = (lo + hi) / 2.0
        down = pcd.voxel_down_sample(mid)
        count = len(down.points)
        best_pcd, best_voxel = down, mid
        if abs(count - target) <= target * tolerance:
            break
        if count > target:
            lo = mid
        else:
            hi = mid
    return best_pcd, best_voxel, len(best_pcd.points)


def preprocess_legacy_pcd(
    pcd_path: str,
    voxel_size: float | None = None,
    target_points: int = 150_000,
    z_min: float = 0.1,
    z_max: float = 2.2,
):
    import open3d as o3d

    pcd = o3d.io.read_point_cloud(pcd_path)
    raw_count = len(pcd.points)
    if raw_count == 0:
        raise RuntimeError(f"PCD is empty or unreadable: {pcd_path}")

    points = np.asarray(pcd.points)
    mask = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
    pcd = pcd.select_by_index(np.where(mask)[0])
    filtered_count = len(pcd.points)
    if filtered_count == 0:
        raise RuntimeError(
            f"Height filter z=[{z_min}, {z_max}] removed all map points"
        )

    if voxel_size is not None:
        output = pcd.voxel_down_sample(voxel_size)
        selected_voxel = voxel_size
    elif filtered_count <= target_points:
        output = pcd
        selected_voxel = None
    else:
        output, selected_voxel, _ = adaptive_voxel_size(pcd, target_points)

    metadata = {
        "raw_points": raw_count,
        "height_filtered_points": filtered_count,
        "prepared_points": len(output.points),
        "z_min": z_min,
        "z_max": z_max,
        "target_points": target_points,
        "python_voxel": selected_voxel,
    }
    return output, metadata
