#!/usr/bin/env python3
"""Extract 3D point cloud from RTAB-Map database and save as PCD/PLY.

Reads compressed laser scan data directly from the SQLite database,
decompresses (zlib), assembles with optimized poses, and writes to PCD.
"""
import sqlite3
import struct
import zlib
import sys
import os
import numpy as np
from pathlib import Path

# OpenCV Mat type constants
CV_8U  = 0
CV_8S  = 1
CV_16U = 2
CV_16S = 3
CV_32S = 4
CV_32F = 5
CV_64F = 6

CV_TYPE_TO_DTYPE = {
    CV_8U:  np.uint8,
    CV_8S:  np.int8,
    CV_16U: np.uint16,
    CV_16S: np.int16,
    CV_32S: np.int32,
    CV_32F: np.float32,
    CV_64F: np.float64,
}

# RTAB-Map LaserScan format enum
SCAN_FORMAT_NAMES = {
    0: "kUnknown",
    1: "kXY",
    2: "kXYZ",
    3: "kXYI",
    4: "kXYZI",
    5: "kXYZRGB",
    6: "kXYNormal",
    7: "kXYZNormal",
    8: "kXYINormal",
    9: "kXYZINormal",
    10: "kXYZRGBNormal",
}

SCAN_FORMAT_CHANNELS = {
    0: 0, 1: 2, 2: 3, 3: 3, 4: 4, 5: 4, 6: 5, 7: 6, 8: 6, 9: 7, 10: 7,
}


def decompress_cv_mat(blob):
    """Decompress a RTAB-Map compressed cv::Mat blob.

    Returns (numpy_array, n_points, n_channels) where numpy_array has shape (n_points, n_channels).
    """
    if blob is None or len(blob) < 12:
        return None, 0, 0

    # Last 12 bytes: rows(int32), cols(int32), type(int32)
    rows = struct.unpack_from('<i', blob, len(blob) - 12)[0]
    cols = struct.unpack_from('<i', blob, len(blob) - 8)[0]
    cv_type = struct.unpack_from('<i', blob, len(blob) - 4)[0]

    depth = cv_type & 7  # base type
    channels = (cv_type >> 3) + 1
    dtype = CV_TYPE_TO_DTYPE.get(depth, np.float32)

    # Decompress zlib payload (everything except last 12 bytes)
    compressed = blob[: len(blob) - 12]
    try:
        raw = zlib.decompress(compressed)
    except zlib.error as e:
        print(f"  [WARN] zlib decompress failed: {e}")
        return None, 0, 0

    total_elements = len(raw) // np.dtype(dtype).itemsize
    n_channels = channels  # e.g. 7 for CV_32FC7
    n_points = cols  # cols = number of points, rows = 1

    expected = rows * cols * channels * np.dtype(dtype).itemsize
    if len(raw) != expected:
        print(f"  [WARN] size mismatch: got {len(raw)}, expected {expected}")
        return None, 0, 0

    # Reshape: (rows, cols, channels) -> (n_points, n_channels)
    arr = np.frombuffer(raw, dtype=dtype).reshape(rows * cols, channels)
    return arr, n_points, n_channels


def parse_scan_info(blob):
    """Parse scan_info blob (version >= 0.18.0): 7 floats header + 12 floats localTransform."""
    if blob is None or len(blob) < 76:
        return None
    vals = struct.unpack('<19f', blob)
    return {
        'format': int(vals[0]),
        'min_range': vals[1],
        'max_range': vals[2],
        'angle_min': vals[3],
        'angle_max': vals[4],
        'angle_inc': vals[5],
        'max_pts': int(vals[6]),
        'local_transform': np.array(vals[7:19]).reshape(3, 4),
    }


def parse_pose_blob(blob):
    """Parse a 3x4 transform from a binary blob (12 floats)."""
    if blob is None or len(blob) < 48:
        return np.eye(4)
    vals = struct.unpack('<12f', blob[:48])
    T = np.eye(4)
    T[:3, :] = np.array(vals).reshape(3, 4)
    return T


def get_optimized_poses(db_path):
    """Get optimized poses from the database link graph using simple pose composition.

    For a quick export, we just use the odometry poses directly.
    """
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()

    # Get all node poses (stored as 3x4 transforms)
    cur.execute("SELECT id, pose FROM Node WHERE pose IS NOT NULL ORDER BY id")
    poses = {}
    for row in cur.fetchall():
        node_id = row[0]
        pose_blob = row[1]
        if pose_blob and len(pose_blob) >= 48:
            vals = struct.unpack(f'<{len(pose_blob)//4}f', pose_blob)
            T = np.eye(4)
            T[:3, :] = np.array(vals[:12]).reshape(3, 4)
            poses[node_id] = T
    conn.close()
    return poses


def write_pcd_binary(filename, points, fields=None):
    """Write points to PCD format (binary).

    Args:
        points: Nx3 or Nx4 numpy float32 array
        fields: list of field names, e.g. ['x','y','z'] or ['x','y','z','intensity']
    """
    n = points.shape[0]
    dim = points.shape[1]
    if fields is None:
        fields = ['x', 'y', 'z', 'intensity', 'normal_x', 'normal_y', 'normal_z'][:dim]

    sizes = ' '.join(['4'] * dim)
    types = ' '.join(['F'] * dim)
    counts = ' '.join(['1'] * dim)

    header = (
        f"# .PCD v0.7 - Point Cloud Data file format\n"
        f"VERSION 0.7\n"
        f"FIELDS {' '.join(fields)}\n"
        f"SIZE {sizes}\n"
        f"TYPE {types}\n"
        f"COUNT {counts}\n"
        f"WIDTH {n}\n"
        f"HEIGHT 1\n"
        f"VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        f"DATA binary\n"
    )
    with open(filename, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(points.astype(np.float32).tobytes())
    print(f"[OK] Wrote {n} points to {filename}")


def write_ply_ascii(filename, points):
    """Write Nx3+ points to PLY ascii format."""
    n = points.shape[0]
    dim = points.shape[1]
    has_intensity = dim >= 4
    has_normals = dim >= 7

    with open(filename, 'w') as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {n}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        if has_intensity:
            f.write("property float intensity\n")
        if has_normals:
            f.write("property float nx\n")
            f.write("property float ny\n")
            f.write("property float nz\n")
        f.write("end_header\n")
        for i in range(n):
            vals = ' '.join(f'{v:.6f}' for v in points[i, :min(dim, 7)])
            f.write(vals + '\n')
    print(f"[OK] Wrote {n} points to {filename}")


def main():
    db_path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/.ros/rtabmap.db")
    output_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "cloud_map")
    os.makedirs(output_dir, exist_ok=True)

    if not os.path.isfile(db_path):
        print(f"[ERROR] Database not found: {db_path}")
        sys.exit(1)

    print(f"[INFO] Database: {db_path}")
    print(f"[INFO] Output dir: {output_dir}")

    # Get poses
    poses = get_optimized_poses(db_path)
    print(f"[INFO] Loaded {len(poses)} poses")

    # Read scan data
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    cur.execute("SELECT id, scan_info, scan FROM Data WHERE scan IS NOT NULL ORDER BY id")

    all_points = []
    total_nodes = 0
    scan_format_name = "unknown"

    for row in cur.fetchall():
        node_id, scan_info_blob, scan_blob = row
        total_nodes += 1

        # Parse scan info
        info = parse_scan_info(scan_info_blob)
        if info is None:
            print(f"  [WARN] Node {node_id}: no scan info")
            continue

        fmt = info['format']
        n_channels = SCAN_FORMAT_CHANNELS.get(fmt, 0)
        scan_format_name = SCAN_FORMAT_NAMES.get(fmt, f"unknown({fmt})")

        if total_nodes == 1:
            print(f"[INFO] Scan format: {scan_format_name} ({n_channels} channels)")
            print(f"[INFO] Max range: {info['max_range']:.2f}m")

        # Decompress scan data
        points, n_points, actual_channels = decompress_cv_mat(scan_blob)
        if points is None or n_points == 0:
            print(f"  [WARN] Node {node_id}: decompression failed")
            continue
        if points.shape[1] < 3:
            continue

        # Extract xyz (first 3 columns)
        xyz = points[:, :3]

        # Apply local transform from scan_info
        lt = info['local_transform']  # 3x4
        R_local = lt[:3, :3]
        t_local = lt[:3, 3]
        xyz_local = (R_local @ xyz.T).T + t_local

        # Apply node pose
        if node_id in poses:
            T = poses[node_id]
            R = T[:3, :3]
            t = T[:3, 3]
            xyz_world = (R @ xyz_local.T).T + t
        else:
            print(f"  [WARN] Node {node_id}: no pose, using local frame")
            xyz_world = xyz_local

        # Keep all channels but replace xyz with world-frame xyz
        out = np.copy(points)
        out[:, :3] = xyz_world
        all_points.append(out)

    conn.close()

    if not all_points:
        print("[ERROR] No point cloud data found in database!")
        sys.exit(1)

    cloud = np.vstack(all_points)
    print(f"[INFO] Total assembled points: {cloud.shape[0]}")
    print(f"[INFO] Channels: {cloud.shape[1]}")

    # Write PCD (xyz + intensity)
    from datetime import datetime
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    basename = f"rtabmap_{ts}"

    # Full cloud as PCD (binary, xyz only or xyzi)
    pcd_path = os.path.join(output_dir, f"{basename}_cloud.pcd")
    write_pcd_binary(pcd_path, cloud[:, :min(cloud.shape[1], 4)])

    # Also write PLY for compatibility
    ply_path = os.path.join(output_dir, f"{basename}_cloud.ply")
    write_ply_ascii(ply_path, cloud[:, :min(cloud.shape[1], 7)])

    print(f"\n[DONE] Exported:")
    print(f"  PCD: {pcd_path}")
    print(f"  PLY: {ply_path}")


if __name__ == "__main__":
    main()
