from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Optional

FIX_QUALITY_LABELS: dict[int, str] = {
    0: "没定位",
    1: "普通定位",
    2: "DGPS",
    3: "PPS",
    4: "RTK Fixed",
    5: "RTK Float",
    6: "估算定位",
    7: "手动输入",
    8: "模拟模式",
}


@dataclass(frozen=True)
class NMEASentence:
    raw: str
    talker: str
    sentence_type: str
    fields: list[str]
    checksum: Optional[str]
    checksum_valid: Optional[bool]


@dataclass(frozen=True)
class ParsedSentence:
    type: str
    talker: str
    fields: dict[str, Any]


def fix_quality_to_label(fix_quality: Optional[int]) -> str:
    if fix_quality is None:
        return "未知"
    return FIX_QUALITY_LABELS.get(fix_quality, f"未知({fix_quality})")


def _strip_sentence(line: str) -> str:
    return line.strip().lstrip("\ufeff")


def _split_checksum(body: str) -> tuple[str, Optional[str]]:
    if "*" not in body:
        return body, None
    payload, checksum = body.split("*", 1)
    return payload, checksum.upper()


def _compute_checksum(payload: str) -> str:
    checksum = 0
    for char in payload:
        checksum ^= ord(char)
    return f"{checksum:02X}"


def _parse_float(value: str) -> Optional[float]:
    if value == "":
        return None
    try:
        return float(value)
    except ValueError:
        return None


def _parse_int(value: str) -> Optional[int]:
    if value == "":
        return None
    try:
        return int(value)
    except ValueError:
        return None


def _parse_lat_lon(value: str, hemi: str, width: int) -> Optional[float]:
    if not value or not hemi:
        return None
    if len(value) < width:
        return None
    degrees = value[: width - 2]
    minutes = value[width - 2 :]
    try:
        decimal = float(degrees) + float(minutes) / 60.0
    except ValueError:
        return None
    if hemi in {"S", "W"}:
        decimal = -decimal
    return decimal


def parse_nmea_line(line: str) -> Optional[ParsedSentence]:
    raw = _strip_sentence(line)
    if not raw.startswith("$"):
        return None

    body = raw[1:]
    payload, checksum = _split_checksum(body)
    checksum_valid: Optional[bool] = None
    if checksum is not None:
        checksum_valid = _compute_checksum(payload) == checksum

    parts = payload.split(",")
    if not parts or len(parts[0]) < 3:
        return None

    header = parts[0]
    talker = header[:2]
    sentence_type = header[2:]
    fields = parts[1:]

    parser = _PARSERS.get(sentence_type)
    parsed_fields = parser(fields) if parser else {"raw_fields": fields}
    if checksum_valid is not None:
        parsed_fields["checksum_valid"] = checksum_valid

    return ParsedSentence(type=sentence_type, talker=talker, fields=parsed_fields)


def _parse_gga(fields: list[str]) -> dict[str, Any]:
    lat = _parse_lat_lon(fields[1] if len(fields) > 1 else "", fields[2] if len(fields) > 2 else "", 4)
    lon = _parse_lat_lon(fields[3] if len(fields) > 3 else "", fields[4] if len(fields) > 4 else "", 5)
    fix_quality = _parse_int(fields[5] if len(fields) > 5 else "")
    return {
        "time": fields[0] if len(fields) > 0 else None,
        "lat": lat,
        "lon": lon,
        "fix_quality": fix_quality,
        "fix_status": fix_quality_to_label(fix_quality),
        "num_sv": _parse_int(fields[6] if len(fields) > 6 else ""),
        "hdop": _parse_float(fields[7] if len(fields) > 7 else ""),
        "altitude_m": _parse_float(fields[8] if len(fields) > 8 else ""),
        "geoid_separation_m": _parse_float(fields[10] if len(fields) > 10 else ""),
    }


def _parse_rmc(fields: list[str]) -> dict[str, Any]:
    lat = _parse_lat_lon(fields[2] if len(fields) > 2 else "", fields[3] if len(fields) > 3 else "", 4)
    lon = _parse_lat_lon(fields[4] if len(fields) > 4 else "", fields[5] if len(fields) > 5 else "", 5)
    return {
        "time": fields[0] if len(fields) > 0 else None,
        "status": fields[1] if len(fields) > 1 else None,
        "lat": lat,
        "lon": lon,
        "speed_knots": _parse_float(fields[6] if len(fields) > 6 else ""),
        "track_angle_deg": _parse_float(fields[7] if len(fields) > 7 else ""),
        "date": fields[8] if len(fields) > 8 else None,
        "magnetic_variation": _parse_float(fields[9] if len(fields) > 9 else ""),
        "variation_direction": fields[10] if len(fields) > 10 else None,
        "mode": fields[11] if len(fields) > 11 else None,
    }


def _parse_vtg(fields: list[str]) -> dict[str, Any]:
    return {
        "true_track_deg": _parse_float(fields[0] if len(fields) > 0 else ""),
        "magnetic_track_deg": _parse_float(fields[2] if len(fields) > 2 else ""),
        "speed_knots": _parse_float(fields[4] if len(fields) > 4 else ""),
        "speed_kmh": _parse_float(fields[6] if len(fields) > 6 else ""),
        "mode": fields[8] if len(fields) > 8 else None,
    }


_PARSERS: dict[str, Callable[[list[str]], dict[str, Any]]] = {
    "GGA": _parse_gga,
    "RMC": _parse_rmc,
    "VTG": _parse_vtg,
}
