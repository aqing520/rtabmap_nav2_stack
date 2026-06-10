#!/usr/bin/env python3
import sqlite3
import struct
import zlib
import sys
import os
from datetime import datetime

import numpy as np


CV_TYPE_TO_DTYPE = {
    0: np.uint8,
    1: np.int8,
    2: np.uint16,
    3: np.int16,
    4: np.int32,
    5: np.float32,
    6: np.float64,
}


def decompress_cv_mat(blob):
    if blob is None or len(blob) < 12:
        return None

    rows = struct.unpack_from("<i", blob, len(blob) - 12)[0]
    cols = struct.unpack_from("<i", blob, len(blob) - 8)[0]
    cv_type = struct.unpack_from("<i", blob, len(blob) - 4)[0]

    depth = cv_type & 7
    channels = (cv_type >> 3) + 1
    dtype = CV_TYPE_TO_DTYPE.get(depth, np.float32)

    compressed = blob[:len(blob) - 12]

    try:
        raw = zlib.decompress(compressed)
    except zlib.error as e:
        print(f"[WARN] zlib decompress failed: {e}")
        return None

    expected = rows * cols * channels * np.dtype(dtype).itemsize
    if len(raw) != expected:
        print(f"[WARN] size mismatch: got {len(raw)}, expected {expected}")
        return None

    return np.frombuffer(raw, dtype=dtype).reshape(rows * cols, channels)


def parse_scan_info(blob):
    if blob is None or len(blob) < 76:
        return None

    vals = struct.unpack("<19f", blob)

    return {
        "local_transform": np.array(vals[7:19]).reshape(3, 4)
    }


def parse_pose_blob(blob):
    if blob is None or len(blob) < 48:
        return None

    vals = struct.unpack("<12f", blob[:48])

    T = np.eye(4)
    T[:3, :] = np.array(vals).reshape(3, 4)

    return T


def write_pcd_ascii(path, xyz):
    xyz = xyz.astype(np.float32)

    with open(path, "w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {xyz.shape[0]}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {xyz.shape[0]}\n")
        f.write("DATA ascii\n")

        for p in xyz:
            f.write(f"{p[0]} {p[1]} {p[2]}\n")


def default_output_dir():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.dirname(script_dir)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")

    return os.path.join(
        workspace_dir,
        "cloud_map",
        f"keyframes_{ts}"
    )


def main():
    db_path = sys.argv[1] if len(sys.argv) > 1 else "/data/maps/site_a/rtabmap.db"

    if len(sys.argv) > 2:
        output_dir = sys.argv[2]
    else:
        output_dir = default_output_dir()

    if not os.path.isfile(db_path):
        print(f"[ERROR] Database not found: {db_path}")
        sys.exit(1)

    os.makedirs(output_dir, exist_ok=True)

    poses_txt = os.path.join(output_dir, "poses.txt")

    print(f"[INFO] Database: {db_path}")
    print(f"[INFO] Output dir: {output_dir}")

    conn = sqlite3.connect(db_path)
    cur = conn.cursor()

    cur.execute("""
        SELECT Data.id, Node.pose, Data.scan_info, Data.scan
        FROM Data
        JOIN Node ON Data.id = Node.id
        WHERE Data.scan IS NOT NULL
        ORDER BY Data.id
    """)

    rows = cur.fetchall()

    if not rows:
        print("[ERROR] No scan data found in database")
        conn.close()
        sys.exit(1)

    count = 0

    with open(poses_txt, "w") as pose_file:
        pose_file.write("# id x y z yaw pcd_file\n")

        for node_id, pose_blob, scan_info_blob, scan_blob in rows:
            pose = parse_pose_blob(pose_blob)
            info = parse_scan_info(scan_info_blob)
            scan = decompress_cv_mat(scan_blob)

            if pose is None:
                print(f"[WARN] Node {node_id}: no pose")
                continue

            if info is None:
                print(f"[WARN] Node {node_id}: no scan_info")
                continue

            if scan is None:
                print(f"[WARN] Node {node_id}: scan decompress failed")
                continue

            if scan.shape[1] < 3:
                print(f"[WARN] Node {node_id}: scan channels < 3")
                continue

            xyz = scan[:, :3]

            local_T = info["local_transform"]
            R_local = local_T[:3, :3]
            t_local = local_T[:3, 3]

            xyz_local = (R_local @ xyz.T).T + t_local

            R = pose[:3, :3]
            t = pose[:3, 3]

            xyz_world = (R @ xyz_local.T).T + t

            pcd_name = f"{node_id}.pcd"
            pcd_path = os.path.join(output_dir, pcd_name)

            write_pcd_ascii(pcd_path, xyz_world)

            x = pose[0, 3]
            y = pose[1, 3]
            z = pose[2, 3]
            yaw = np.arctan2(pose[1, 0], pose[0, 0])

            pose_file.write(
                f"{node_id} {x:.6f} {y:.6f} {z:.6f} {yaw:.6f} {pcd_name}\n"
            )

            print(f"[OK] keyframe {node_id}: {xyz_world.shape[0]} pts -> {pcd_name}")
            count += 1

    conn.close()

    print("")
    print(f"[DONE] Exported {count} keyframes")
    print(f"[DONE] Directory: {output_dir}")
    print(f"[DONE] Poses: {poses_txt}")


if __name__ == "__main__":
    main()
