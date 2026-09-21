#!/usr/bin/env python3
"""Capture an unknown serial sensor stream byte-for-byte with a manifest.

The script never writes protocol commands to the serial device.  It is intended
for first-contact capture of the mmWave radar (or another serial sensor) before
a vendor-specific decoder is trusted.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import time


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description="Capture a serial sensor as raw bytes")
    result.add_argument("--device", required=True, help="Serial device, for example /dev/ttyUSB0")
    result.add_argument("--baud", required=True, type=int)
    result.add_argument("--duration-sec", required=True, type=float)
    result.add_argument("--output-dir", required=True)
    result.add_argument("--sensor", default="unknown_serial_sensor")
    result.add_argument("--chunk-size", type=int, default=65536)
    return result


def main() -> int:
    arguments = parser().parse_args()
    if arguments.baud <= 0 or arguments.duration_sec <= 0.0 or arguments.chunk_size <= 0:
        raise SystemExit("baud, duration-sec and chunk-size must be positive")
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is required: install the ROS/OS python3-serial package") from exc

    output_dir = Path(arguments.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=False)
    raw_path = output_dir / "serial_raw.bin"
    started_utc = utc_now()
    started = time.monotonic()
    byte_count = 0
    interrupted = False

    try:
        with serial.Serial(
            port=arguments.device,
            baudrate=arguments.baud,
            timeout=0.2,
            write_timeout=0.2,
        ) as device, raw_path.open("wb") as output:
            while time.monotonic() - started < arguments.duration_sec:
                chunk = device.read(arguments.chunk_size)
                if chunk:
                    output.write(chunk)
                    byte_count += len(chunk)
            output.flush()
    except KeyboardInterrupt:
        interrupted = True

    elapsed = time.monotonic() - started
    digest = file_sha256(raw_path)
    manifest = {
        "schema_version": 1,
        "sensor": str(arguments.sensor),
        "transport": "serial",
        "device": str(arguments.device),
        "baud": int(arguments.baud),
        "requested_duration_sec": float(arguments.duration_sec),
        "elapsed_sec": elapsed,
        "started_utc": started_utc,
        "ended_utc": utc_now(),
        "byte_count": byte_count,
        "sha256": digest,
        "raw_file": raw_path.name,
        "interrupted": interrupted,
        "protocol_commands_sent": False,
        "host": platform.node(),
        "decoder_status": "NOT_APPLIED",
    }
    (output_dir / "capture_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(manifest, ensure_ascii=False, indent=2))
    return 0 if byte_count > 0 else 3


if __name__ == "__main__":
    raise SystemExit(main())
