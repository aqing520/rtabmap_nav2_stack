from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from typing import Any, Iterable

from .nmea import parse_nmea_line
from .serial_reader import SerialStream, candidate_ports


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="BT-468 RTK GNSS serial reader")
    parser.add_argument(
        "--port",
        default="auto",
        help="Serial device, e.g. /dev/ttyCH343USB0. Use 'auto' to probe common ports.",
    )
    parser.add_argument("--baud", type=int, default=38400, help="Serial baud rate")
    parser.add_argument("--timeout", type=float, default=1.0, help="Serial read timeout in seconds")
    parser.add_argument(
        "--max-lines",
        type=int,
        default=None,
        help="Stop after reading this many serial lines.",
    )
    parser.add_argument(
        "--print-raw",
        action="store_true",
        help="Print every raw line before parsing",
    )
    return parser


def _emit(parsed) -> None:
    print(json.dumps(asdict(parsed), ensure_ascii=False, sort_keys=True))


def _format_coordinate(value: Any) -> str:
    if value is None:
        return "-"
    return f"{value:.9f}" if isinstance(value, float) else str(value)


def _emit_position_summary(parsed) -> None:
    if parsed.type != "GGA":
        return
    summary = {
        "定位状态": parsed.fields.get("fix_status", "未知"),
        "纬度": _format_coordinate(parsed.fields.get("lat")),
        "经度": _format_coordinate(parsed.fields.get("lon")),
    }
    print(json.dumps(summary, ensure_ascii=False, sort_keys=True))


def resolve_port(port: str) -> str:
    if port != "auto":
        return port
    ports = candidate_ports()
    if not ports:
        raise RuntimeError("No candidate serial ports found.")
    return ports[0]


def run(port: str, baud: int, timeout: float, print_raw: bool, max_lines: int | None) -> int:
    resolved_port = resolve_port(port)
    print(json.dumps({"port": resolved_port, "baud": baud}, ensure_ascii=False))
    stream = SerialStream(port=resolved_port, baudrate=baud, timeout=timeout, max_lines=max_lines)
    for line in stream.read_lines():
        if print_raw:
            print(line)
        parsed = parse_nmea_line(line)
        if parsed is None:
            continue
        _emit(parsed)
        _emit_position_summary(parsed)
    return 0


def main(argv: Iterable[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    return run(args.port, args.baud, args.timeout, args.print_raw, args.max_lines)


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
