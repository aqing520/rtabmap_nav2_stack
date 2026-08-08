#!/usr/bin/env python3
"""Build or locate an offline HDL FPFH cache."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

from hdl_map_preprocessing import preprocess_legacy_pcd


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CACHE_ROOT = WORKSPACE_ROOT / "db" / "pcd" / "cache" / "fpfh_ransac"
DEFAULT_PROFILE_JSON = (
    WORKSPACE_ROOT
    / "src"
    / "hdl_global_localization-humble"
    / "config"
    / "config_fpfh.json"
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def find_builder() -> list[str]:
    executable = shutil.which("build_fpfh_cache")
    if executable:
        return [executable]
    return ["ros2", "run", "hdl_global_localization", "build_fpfh_cache"]


def cache_builder_identity() -> dict:
    result = subprocess.run(
        find_builder() + ["--print-identity"],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if result.returncode != 0:
        raise RuntimeError(
            "Unable to query the installed cache builder identity. "
            "Build and source hdl_global_localization first.\n"
            + result.stderr.strip()
        )
    try:
        identity = json.loads(result.stdout.strip().splitlines()[-1])
    except Exception as exception:
        raise RuntimeError(
            f"Invalid cache builder identity output: {result.stdout!r}"
        ) from exception
    required = {
        "schema_version",
        "cache_builder_version",
        "pcl_major",
        "pcl_minor",
    }
    if not required.issubset(identity):
        raise RuntimeError(f"Incomplete cache builder identity: {identity}")
    return identity


def canonical_profile_hash(
    profile_json: Path,
    profile_name: str,
    builder_identity: dict,
) -> str:
    data = json.loads(profile_json.read_text(encoding="utf-8"))
    profile = dict(data["profiles"][profile_name])
    # Must mirror the C++ cache-builder identity.
    profile.update(
        {
            "schema_version": builder_identity["schema_version"],
            "cache_builder_version": builder_identity[
                "cache_builder_version"
            ],
            "pcl_major": builder_identity["pcl_major"],
            "pcl_minor": builder_identity["pcl_minor"],
        }
    )
    encoded = json.dumps(
        profile, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def expected_cache_dir(
    source_pcd: Path,
    cache_root: Path,
    profile_json: Path,
    profile_name: str,
    builder_identity: dict,
) -> Path:
    return (
        cache_root
        / sha256_file(source_pcd)
        / canonical_profile_hash(
            profile_json, profile_name, builder_identity
        )
    )


def validate_manifest(
    cache_dir: Path,
    source_pcd: Path,
    profile_json: Path,
    profile_name: str,
    builder_identity: dict,
) -> dict:
    manifest_path = cache_dir / "manifest.json"
    if not manifest_path.is_file():
        raise RuntimeError(f"Cache manifest missing: {manifest_path}")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("schema_version") != 1:
        raise RuntimeError("Unsupported cache schema")
    source_sha = sha256_file(source_pcd)
    source = manifest.get("source", {})
    if source.get("sha256") != source_sha:
        raise RuntimeError("Source PCD hash differs from cache manifest")
    if source.get("size_bytes") != source_pcd.stat().st_size:
        raise RuntimeError("Source PCD size differs from cache manifest")
    if manifest.get("profile_name") != profile_name:
        raise RuntimeError("Cache profile name differs from requested profile")
    expected_profile_hash = canonical_profile_hash(
        profile_json, profile_name, builder_identity
    )
    if manifest.get("profile_hash") != expected_profile_hash:
        raise RuntimeError("Cache profile hash differs from current profile")
    if manifest.get("cache_identity") != {
        key: builder_identity[key]
        for key in (
            "schema_version",
            "cache_builder_version",
            "pcl_major",
            "pcl_minor",
        )
    }:
        raise RuntimeError("Cache builder/PCL identity differs from runtime")
    if manifest.get("cache_key") != f"{source_sha}:{expected_profile_hash}":
        raise RuntimeError("Cache key is inconsistent")
    for name in ("map_surface.pcd", "map_keypoints.pcd", "map_fpfh.pcd"):
        artifact = cache_dir / name
        if not artifact.is_file():
            raise RuntimeError(f"Cache artifact missing: {artifact}")
        expected = manifest.get("artifacts", {}).get(name, {}).get("sha256")
        expected_size = (
            manifest.get("artifacts", {}).get(name, {}).get("size_bytes")
        )
        if expected_size != artifact.stat().st_size:
            raise RuntimeError(f"Cache artifact size mismatch: {name}")
        if not expected or sha256_file(artifact) != expected:
            raise RuntimeError(f"Cache artifact checksum mismatch: {name}")
    return manifest


def validate_cache_with_builder(cache_dir: Path, source_pcd: Path) -> None:
    """Run the C++ cache loader to validate PCD contents and descriptors."""
    result = subprocess.run(
        find_builder()
        + [
            "--validate-cache",
            str(cache_dir),
            "--source-pcd",
            str(source_pcd),
        ],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if result.returncode != 0:
        raise RuntimeError(
            "C++ cache validation rejected the cache.\n"
            + result.stderr.strip()
        )
    print(result.stdout.strip())


def write_active_cache(
    cache_root: Path, source_pcd: Path, cache_dir: Path, manifest: dict
) -> None:
    cache_root.mkdir(parents=True, exist_ok=True)
    active = {
        "schema_version": 1,
        "source_pcd": str(source_pcd.resolve()),
        "cache_directory": str(cache_dir.resolve()),
        "cache_key": manifest.get("cache_key"),
        "profile_name": manifest.get("profile_name"),
        "source_sha256": manifest.get("source", {}).get("sha256"),
    }
    temporary = cache_root / f"active.json.tmp.{os.getpid()}"
    temporary.write_text(
        json.dumps(active, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, cache_root / "active.json")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("source")
    parser.add_argument("--cache-root", default=str(DEFAULT_CACHE_ROOT))
    parser.add_argument("--profile-json", default=str(DEFAULT_PROFILE_JSON))
    parser.add_argument("--profile", default="legacy_fpfh_v1")
    parser.add_argument(
        "--structural-surface",
        help="Prepared XYZI+normal surface PCD for non-legacy profiles.",
    )
    parser.add_argument(
        "--structural-keypoints",
        help="Prepared XYZI+normal keypoint PCD for non-legacy profiles.",
    )
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--locate-only", action="store_true")
    parser.add_argument("--no-active-update", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    source_pcd = Path(args.source).expanduser().resolve()
    cache_root = Path(args.cache_root).expanduser().resolve()
    profile_json = Path(args.profile_json).expanduser().resolve()
    if not source_pcd.is_file():
        print(f"[ERROR] Source file not found: {source_pcd}", file=sys.stderr)
        return 1
    if not profile_json.is_file():
        print(f"[ERROR] Profile JSON not found: {profile_json}", file=sys.stderr)
        return 1

    try:
        builder_identity = cache_builder_identity()
        cache_dir = expected_cache_dir(
            source_pcd,
            cache_root,
            profile_json,
            args.profile,
            builder_identity,
        )
    except Exception as exception:
        print(f"[ERROR] {exception}", file=sys.stderr)
        return 1
    if args.locate_only:
        try:
            manifest = validate_manifest(
                cache_dir,
                source_pcd,
                profile_json,
                args.profile,
                builder_identity,
            )
            validate_cache_with_builder(cache_dir, source_pcd)
        except Exception as exception:
            print(f"[ERROR] {exception}", file=sys.stderr)
            return 1
        print(f"CACHE_DIR: {cache_dir}")
        print(f"CACHE_KEY: {manifest.get('cache_key')}")
        return 0

    if args.profile != "legacy_fpfh_v1":
        if not args.structural_surface or not args.structural_keypoints:
            print(
                "[ERROR] Non-legacy profiles require --structural-surface "
                "and --structural-keypoints generated by "
                "scripts/export_structural_map_from_db.py.",
                file=sys.stderr,
            )
            return 2
        structural_surface = Path(args.structural_surface).expanduser().resolve()
        structural_keypoints = Path(args.structural_keypoints).expanduser().resolve()
        if not structural_surface.is_file():
            print(
                f"[ERROR] Structural surface PCD not found: {structural_surface}",
                file=sys.stderr,
            )
            return 1
        if not structural_keypoints.is_file():
            print(
                f"[ERROR] Structural keypoint PCD not found: {structural_keypoints}",
                file=sys.stderr,
            )
            return 1

        command = find_builder() + [
            "--source-pcd",
            str(source_pcd),
            "--prepared-surface",
            str(structural_surface),
            "--prepared-keypoints",
            str(structural_keypoints),
            "--cache-root",
            str(cache_root),
            "--profile-json",
            str(profile_json),
            "--profile",
            args.profile,
        ]
        if args.force:
            command.append("--force")
        environment = os.environ.copy()
        try:
            environment["HDL_CACHE_GIT_COMMIT"] = subprocess.check_output(
                ["git", "-C", str(WORKSPACE_ROOT), "rev-parse", "HEAD"],
                text=True,
            ).strip()
        except Exception:
            environment["HDL_CACHE_GIT_COMMIT"] = "unknown"

        result = subprocess.run(command, env=environment, text=True)
        if result.returncode != 0:
            return result.returncode

        manifest = validate_manifest(
            cache_dir,
            source_pcd,
            profile_json,
            args.profile,
            builder_identity,
        )
        if not args.no_active_update:
            write_active_cache(cache_root, source_pcd, cache_dir, manifest)
        print(f"CACHE_DIR: {cache_dir}")
        print(f"CACHE_KEY: {manifest.get('cache_key')}")
        print(
            "CACHE_POINTS: "
            f"surface={manifest['counts']['surface_points']} "
            f"keypoints={manifest['counts']['keypoint_points']} "
            f"features={manifest['counts']['feature_points']}"
        )
        return 0

    if not args.force and (cache_dir / "manifest.json").is_file():
        try:
            manifest = validate_manifest(
                cache_dir,
                source_pcd,
                profile_json,
                args.profile,
                builder_identity,
            )
            validate_cache_with_builder(cache_dir, source_pcd)
            if not args.no_active_update:
                write_active_cache(cache_root, source_pcd, cache_dir, manifest)
            print(f"CACHE_DIR: {cache_dir}")
            print(f"CACHE_KEY: {manifest.get('cache_key')}")
            print(
                "CACHE_POINTS: "
                f"surface={manifest['counts']['surface_points']} "
                f"keypoints={manifest['counts']['keypoint_points']} "
                f"features={manifest['counts']['feature_points']}"
            )
            return 0
        except Exception as exception:
            print(
                f"[WARN] Existing cache is invalid and will be rebuilt: {exception}",
                file=sys.stderr,
            )

    with tempfile.TemporaryDirectory(prefix="hdl_legacy_map_") as temp_dir:
        prepared_path = Path(temp_dir) / "prepared_map.pcd"
        prepared, metadata = preprocess_legacy_pcd(str(source_pcd))
        import open3d as o3d

        if not o3d.io.write_point_cloud(
            str(prepared_path), prepared, write_ascii=False, compressed=False
        ):
            raise RuntimeError("Failed to write temporary prepared map")
        print(
            "[map] "
            f"raw={metadata['raw_points']} "
            f"height_filtered={metadata['height_filtered_points']} "
            f"prepared={metadata['prepared_points']}"
        )

        command = find_builder() + [
            "--source-pcd",
            str(source_pcd),
            "--prepared-map",
            str(prepared_path),
            "--cache-root",
            str(cache_root),
            "--profile-json",
            str(profile_json),
            "--profile",
            args.profile,
        ]
        if args.force:
            command.append("--force")
        environment = os.environ.copy()
        try:
            environment["HDL_CACHE_GIT_COMMIT"] = subprocess.check_output(
                ["git", "-C", str(WORKSPACE_ROOT), "rev-parse", "HEAD"],
                text=True,
            ).strip()
        except Exception:
            environment["HDL_CACHE_GIT_COMMIT"] = "unknown"

        result = subprocess.run(command, env=environment, text=True)
        if result.returncode != 0:
            return result.returncode

    manifest = validate_manifest(
        cache_dir,
        source_pcd,
        profile_json,
        args.profile,
        builder_identity,
    )
    if not args.no_active_update:
        write_active_cache(cache_root, source_pcd, cache_dir, manifest)
    print(f"CACHE_DIR: {cache_dir}")
    print(f"CACHE_KEY: {manifest.get('cache_key')}")
    print(
        "CACHE_POINTS: "
        f"surface={manifest['counts']['surface_points']} "
        f"keypoints={manifest['counts']['keypoint_points']} "
        f"features={manifest['counts']['feature_points']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
