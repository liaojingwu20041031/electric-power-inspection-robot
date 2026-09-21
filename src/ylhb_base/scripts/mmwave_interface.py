#!/usr/bin/env python3
"""Vendor-neutral mmWave input contract for the optional robot-side adapter.

This module deliberately stops at the boundary between a vendor UART/CAN
decoder and the normalized DG contract.  It does not open a device, send
commands, create ROS publishers, or affect ``/cmd_vel``.  A real driver should
decode its vendor frame first, then call :func:`normalize_mmwave_detection`
with an explicit field map when the vendor names differ.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
import math
from typing import Any, Iterator

from auxiliary_sensor_data import MmwaveDetection


MMWAVE_INTERFACE_VERSION = "1.0"
MMWAVE_DATA_TOPIC = "/dg/mmwave/points"
MMWAVE_QUALITY_TOPIC = "/dg/mmwave/quality"
MMWAVE_FRAME_ID = "mmwave_link"

_FIELDS = (
    "timestamp_us",
    "frame_id",
    "track_id",
    "x_m",
    "y_m",
    "z_m",
    "radial_velocity_mps",
    "snr_db",
    "confidence",
)
_INTEGER_FIELDS = {"timestamp_us", "frame_id", "track_id"}
_FLOAT_FIELDS = set(_FIELDS) - _INTEGER_FIELDS


class MmwaveInterfaceError(ValueError):
    """Raised when a vendor decoder violates the normalized input contract."""


def _value(record: Mapping[str, Any], key: str, field_map: Mapping[str, str]) -> Any:
    source_key = str(field_map.get(key, key))
    if source_key not in record:
        raise MmwaveInterfaceError(f"missing normalized field: {key} (source key: {source_key})")
    return record[source_key]


def _integer(value: Any, field: str) -> int:
    if isinstance(value, bool):
        raise MmwaveInterfaceError(f"{field} must be an integer, not boolean")
    try:
        result = int(str(value).strip())
    except (TypeError, ValueError) as exc:
        raise MmwaveInterfaceError(f"{field} must be an integer") from exc
    if str(value).strip() != str(result):
        raise MmwaveInterfaceError(f"{field} must be an integer")
    return result


def _number(value: Any, field: str) -> float:
    if isinstance(value, bool):
        raise MmwaveInterfaceError(f"{field} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise MmwaveInterfaceError(f"{field} must be a finite number") from exc
    if not math.isfinite(result):
        raise MmwaveInterfaceError(f"{field} must be a finite number")
    return result


def normalize_mmwave_detection(
    record: Mapping[str, Any],
    *,
    field_map: Mapping[str, str] | None = None,
) -> MmwaveDetection:
    """Convert one vendor-decoded record to the robot's mmWave contract.

    ``field_map`` maps canonical names to vendor keys.  No implicit unit or
    coordinate conversion is performed: callers must supply values already in
    microseconds, meters, meters/second, dB, and a confidence in ``[0, 1]``.
    This prevents an unknown vendor protocol from being presented as a valid
    live driver by accident.
    """

    if not isinstance(record, Mapping):
        raise MmwaveInterfaceError("vendor record must be a mapping")
    mapping = field_map or {}
    unknown = set(mapping) - set(_FIELDS)
    if unknown:
        raise MmwaveInterfaceError("field_map contains unknown fields: " + ",".join(sorted(unknown)))

    values: dict[str, int | float] = {}
    for field in _FIELDS:
        raw = _value(record, field, mapping)
        values[field] = _integer(raw, field) if field in _INTEGER_FIELDS else _number(raw, field)

    if any(int(values[field]) < 0 for field in _INTEGER_FIELDS):
        raise MmwaveInterfaceError("timestamp and identifiers must be non-negative")
    confidence = float(values["confidence"])
    if not 0.0 <= confidence <= 1.0:
        raise MmwaveInterfaceError("confidence outside [0, 1]")
    return MmwaveDetection(**values)


def iter_normalized_mmwave_detections(
    records: Iterable[Mapping[str, Any]],
    *,
    field_map: Mapping[str, str] | None = None,
) -> Iterator[MmwaveDetection]:
    """Normalize a decoded stream while enforcing timestamp/frame ordering."""

    previous_timestamp: int | None = None
    previous_frame: int | None = None
    closed_frames: set[int] = set()
    for record in records:
        detection = normalize_mmwave_detection(record, field_map=field_map)
        if previous_timestamp is not None and detection.timestamp_us < previous_timestamp:
            raise MmwaveInterfaceError("mmWave timestamps are not monotonic")
        if previous_frame is not None and detection.frame_id != previous_frame:
            closed_frames.add(previous_frame)
        if detection.frame_id in closed_frames:
            raise MmwaveInterfaceError(f"frame_id {detection.frame_id} is not contiguous")
        previous_timestamp = detection.timestamp_us
        previous_frame = detection.frame_id
        yield detection


def interface_contract() -> dict[str, object]:
    """Return the stable ROS-facing contract for driver introspection."""

    return {
        "version": MMWAVE_INTERFACE_VERSION,
        "data_topic": MMWAVE_DATA_TOPIC,
        "quality_topic": MMWAVE_QUALITY_TOPIC,
        "frame_id": MMWAVE_FRAME_ID,
        "observation_only": True,
        "publishes_cmd_vel": False,
        "dynamic_mask_default": False,
        "units": {
            "timestamp_us": "microseconds",
            "x_m": "meters",
            "y_m": "meters",
            "z_m": "meters",
            "radial_velocity_mps": "meters_per_second",
            "snr_db": "decibels",
            "confidence": "unit_interval",
        },
    }
