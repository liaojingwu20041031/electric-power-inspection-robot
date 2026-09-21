#!/usr/bin/env python3
"""Parse the measured UWB stream and build de-duplicated logical inputs."""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Iterable

from uwb_data_model import (
    DEFAULT_LOGICAL_PHYSICAL_MAPPING,
    UwbAnchorConfig,
    UwbRangeObservation,
)


# Survey used for the 2026-09-05 data capture:
# A2=(0, 0), A3=(2.40, 0), A4=(1.20, 1.80), metres.
MEASURED_ANCHOR_COORDINATES = (
    UwbAnchorConfig(physical_id=0, x=0.0, y=0.0, z=0.0),
    UwbAnchorConfig(physical_id=1, x=2.40, y=0.0, z=0.0),
    UwbAnchorConfig(physical_id=2, x=1.20, y=1.80, z=0.0),
)


def _optional_range(value: object) -> float | None:
    text = str(value or "").strip()
    if not text or text.upper() == "NULL":
        return None
    try:
        result = float(text)
    except ValueError:
        return None
    return result if math.isfinite(result) else None


def parse_kt1_ranges(line: str) -> tuple[float | None, float | None, float | None] | None:
    """Return A2/A3/A4 ranges from one device line, preserving missing anchors."""
    text = str(line or "").strip()
    if not text.startswith("$KT1,"):
        return None
    parts = text.split(",")
    if len(parts) < 6:
        return None
    return tuple(_optional_range(value) for value in parts[3:6])  # type: ignore[return-value]


def build_logical_observations(
    timestamp: float,
    physical_ranges: Iterable[float | None],
    *,
    six_logical_slots: bool = True,
) -> list[UwbRangeObservation]:
    """Map three physical ranges to either 3 or 6 logical slots.

    Mirror slots carry the same physical_source_id and correlation_group, so
    the estimator and quality gate count three independent sources only.
    """
    ranges = tuple(physical_ranges)
    if len(ranges) != 3:
        raise ValueError("exactly three physical UWB ranges are required")
    logical_ids = tuple(DEFAULT_LOGICAL_PHYSICAL_MAPPING) if six_logical_slots else ("A0", "A1", "A2")
    observations: list[UwbRangeObservation] = []
    for logical_id in logical_ids:
        physical_id, correlation_group = DEFAULT_LOGICAL_PHYSICAL_MAPPING[logical_id]
        value = ranges[physical_id]
        valid = value is not None and math.isfinite(float(value))
        observations.append(
            UwbRangeObservation(
                timestamp=float(timestamp),
                logical_anchor_id=logical_id,
                physical_source_id=physical_id,
                correlation_group=correlation_group,
                range_m=float(value) if valid else 0.0,
                valid=valid,
            )
        )
    return observations


def load_replay_frames(path: str | Path) -> list[tuple[float | None, float | None, float | None]]:
    """Load measured frames from the saved CSV files or raw serial logs."""
    source = Path(path).expanduser()
    if not source.is_file():
        raise FileNotFoundError(f"UWB replay file does not exist: {source}")
    frames: list[tuple[float | None, float | None, float | None]] = []
    if source.suffix.lower() == ".csv":
        with source.open("r", encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            required = {"a2_m", "a3_m", "a4_m"}
            if not required.issubset(reader.fieldnames or ()):
                raise ValueError(f"UWB CSV must contain {sorted(required)}: {source}")
            for row in reader:
                frames.append(tuple(_optional_range(row[key]) for key in ("a2_m", "a3_m", "a4_m")))
    else:
        with source.open("r", encoding="utf-8", errors="ignore") as handle:
            for line in handle:
                ranges = parse_kt1_ranges(line)
                if ranges is not None:
                    frames.append(ranges)
    if not frames:
        raise ValueError(f"UWB replay file contains no usable frames: {source}")
    return frames
