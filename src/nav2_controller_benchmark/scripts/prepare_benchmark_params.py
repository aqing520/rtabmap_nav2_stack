#!/usr/bin/env python3
"""Generate an explicit MPPI or DWB benchmark parameter file from an existing Nav2 YAML."""

import argparse
from pathlib import Path

import yaml


WRAPPER = "nav2_controller_benchmark::TimingController"
CONTROLLERS = {
    "mppi": "nav2_mppi_controller::MPPIController",
    "dwb": "dwb_core::DWBLocalPlanner",
}


def default_dwb_path():
    source_candidate = Path(__file__).resolve().parent.parent / "config" / "dwb_defaults.yaml"
    if source_candidate.exists():
        return source_candidate
    return (
        Path(__file__).resolve().parents[2]
        / "share"
        / "nav2_controller_benchmark"
        / "config"
        / "dwb_defaults.yaml"
    )


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", choices=sorted(CONTROLLERS), required=True)
    parser.add_argument("--base", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--dwb-defaults",
        type=Path,
        default=default_dwb_path(),
    )
    return parser.parse_args()


def main():
    args = parse_args()
    with args.base.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    params = config["controller_server"]["ros__parameters"]
    follow_path = params["FollowPath"]
    follow_path["plugin"] = WRAPPER
    follow_path["wrapped_plugin"] = CONTROLLERS[args.controller]
    follow_path["timing_topic"] = "controller_benchmark/compute_time_ms"

    if args.controller == "dwb":
        with args.dwb_defaults.open(encoding="utf-8") as stream:
            follow_path.update(yaml.safe_load(stream))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream, sort_keys=False, allow_unicode=True)

    print(f"generated {args.controller.upper()} benchmark config: {args.output}")


if __name__ == "__main__":
    main()
