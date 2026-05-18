from __future__ import annotations

from glob import glob
from dataclasses import dataclass
from typing import Iterator, Optional

from .nmea import ParsedSentence, parse_nmea_line


try:
    import serial  # type: ignore
except ModuleNotFoundError as exc:  # pragma: no cover - runtime dependency guard
    serial = None  # type: ignore[assignment]
    _SERIAL_IMPORT_ERROR = exc
else:
    _SERIAL_IMPORT_ERROR = None


def open_serial(port: str, baudrate: int = 38400, timeout: float = 1.0):
    if serial is None:  # pragma: no cover - runtime dependency guard
        raise RuntimeError(
            "pyserial is required. Install it with `pip install -e .` or `pip install pyserial`."
        ) from _SERIAL_IMPORT_ERROR
    return serial.Serial(port=port, baudrate=baudrate, timeout=timeout)


def candidate_ports() -> list[str]:
    candidates: list[str] = []
    for pattern in (
        "/dev/ttyCH343USB*",
        "/dev/ttyUSB*",
        "/dev/ttyACM*",
        "/dev/ttyTHS*",
    ):
        for path in sorted(glob(pattern)):
            if path not in candidates:
                candidates.append(path)
    return candidates


@dataclass
class SerialStream:
    port: str
    baudrate: int = 38400
    timeout: float = 1.0
    max_lines: Optional[int] = None

    def read_lines(self) -> Iterator[str]:
        with open_serial(self.port, self.baudrate, self.timeout) as device:
            emitted = 0
            while True:
                raw = device.readline()
                if not raw:
                    continue
                yield raw.decode("ascii", errors="replace").strip()
                emitted += 1
                if self.max_lines is not None and emitted >= self.max_lines:
                    return

    def read_parsed(self) -> Iterator[ParsedSentence]:
        for line in self.read_lines():
            parsed = parse_nmea_line(line)
            if parsed is not None:
                yield parsed
