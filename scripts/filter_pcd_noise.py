#!/usr/bin/env python3
"""Lightweight binary PCD denoising for RTAB-Map exported clouds."""

import argparse
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


def read_pcd(path):
    header = []
    with open(path, "rb") as f:
        while True:
            line = f.readline()
            if not line:
                raise ValueError("PCD header is incomplete")
            header.append(line.decode("ascii").strip())
            if line.startswith(b"DATA "):
                break
        data = f.read()

    meta = {}
    for line in header:
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        meta[parts[0]] = parts[1:]

    if meta.get("DATA", [""])[0] != "binary":
        raise ValueError("Only binary PCD is supported")
    if meta.get("TYPE") != ["F"] * len(meta["FIELDS"]):
        raise ValueError("Only float fields are supported")

    fields = meta["FIELDS"]
    sizes = [int(v) for v in meta["SIZE"]]
    points = int(meta["POINTS"][0])
    if any(size != 4 for size in sizes):
        raise ValueError("Only 4-byte float fields are supported")

    arr = np.frombuffer(data, dtype=np.float32).reshape(points, len(fields)).copy()
    return header, meta, arr


def write_pcd(path, header, meta, arr):
    new_header = []
    for line in header:
        key = line.split(maxsplit=1)[0] if line and not line.startswith("#") else ""
        if key == "WIDTH":
            new_header.append(f"WIDTH {arr.shape[0]}")
        elif key == "POINTS":
            new_header.append(f"POINTS {arr.shape[0]}")
        elif key == "HEIGHT":
            new_header.append("HEIGHT 1")
        else:
            new_header.append(line)

    with open(path, "wb") as f:
        f.write(("\n".join(new_header) + "\n").encode("ascii"))
        f.write(arr.astype(np.float32).tobytes())


def radius_outlier_filter(arr, xyz_idx, radius, min_neighbors):
    xyz = arr[:, xyz_idx]
    tree = cKDTree(xyz)
    counts = tree.query_ball_point(xyz, r=radius, return_length=True)
    return arr[counts >= min_neighbors]


def voxel_downsample(arr, xyz_idx, leaf):
    if leaf <= 0:
        return arr

    xyz = arr[:, xyz_idx]
    keys = np.floor(xyz / leaf).astype(np.int64)
    _, unique_idx = np.unique(keys, axis=0, return_index=True)
    return arr[np.sort(unique_idx)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_pcd")
    parser.add_argument("output_pcd")
    parser.add_argument("--radius", type=float, default=0.15)
    parser.add_argument("--min-neighbors", type=int, default=5)
    parser.add_argument("--voxel", type=float, default=0.03)
    args = parser.parse_args()

    input_path = Path(args.input_pcd)
    output_path = Path(args.output_pcd)
    header, meta, arr = read_pcd(input_path)

    fields = meta["FIELDS"]
    xyz_idx = [fields.index("x"), fields.index("y"), fields.index("z")]

    before = arr.shape[0]
    arr = radius_outlier_filter(arr, xyz_idx, args.radius, args.min_neighbors)
    after_radius = arr.shape[0]
    arr = voxel_downsample(arr, xyz_idx, args.voxel)
    after_voxel = arr.shape[0]

    output_path.parent.mkdir(parents=True, exist_ok=True)
    write_pcd(output_path, header, meta, arr)

    print(f"[OK] Input points       : {before}")
    print(f"[OK] After noise filter : {after_radius}")
    print(f"[OK] After voxel filter : {after_voxel}")
    print(f"[OK] Output             : {output_path}")


if __name__ == "__main__":
    main()
