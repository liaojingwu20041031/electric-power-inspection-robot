#!/usr/bin/env python3
"""Strict, ROS-independent schemas for measured event-camera and mmWave data.

The event schema matches Shimeta's documented EVT2-to-CSV output.  The mmWave
schema is an internal interchange contract: a vendor-specific UART/CAN decoder
must produce it before the radar data can enter the ROS graph.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Iterable, Iterator, TextIO


EVENT_COLUMNS = ("x", "y", "polarity", "timestamp")
MMWAVE_COLUMNS = (
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


@dataclass(frozen=True)
class EventSample:
    x: int
    y: int
    polarity: int
    timestamp_us: int


@dataclass(frozen=True)
class EventPacket:
    start_timestamp_us: int
    window_us: int
    samples: tuple[EventSample, ...]


@dataclass(frozen=True)
class MmwaveDetection:
    timestamp_us: int
    frame_id: int
    track_id: int
    x_m: float
    y_m: float
    z_m: float
    radial_velocity_mps: float
    snr_db: float
    confidence: float


@dataclass(frozen=True)
class MmwaveFrame:
    timestamp_us: int
    frame_id: int
    detections: tuple[MmwaveDetection, ...]


def _clean_header(fieldnames: Iterable[str | None] | None) -> tuple[str, ...]:
    if fieldnames is None:
        return ()
    return tuple(str(name or "").strip().lstrip("\ufeff") for name in fieldnames)


def _require_columns(reader: csv.DictReader, required: tuple[str, ...], source: str) -> None:
    actual = _clean_header(reader.fieldnames)
    missing = [name for name in required if name not in actual]
    if missing:
        raise ValueError(f"{source}: missing CSV columns: {', '.join(missing)}")
    reader.fieldnames = list(actual)


def _integer(row: dict[str, str], key: str, source: str, line: int) -> int:
    try:
        value = int(str(row[key]).strip())
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"{source}:{line}: {key} must be an integer") from exc
    return value


def _number(row: dict[str, str], key: str, source: str, line: int) -> float:
    try:
        value = float(str(row[key]).strip())
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"{source}:{line}: {key} must be a number") from exc
    if not math.isfinite(value):
        raise ValueError(f"{source}:{line}: {key} must be finite")
    return value


def _event_rows(
    handle: TextIO,
    source: str,
    width: int,
    height: int,
) -> Iterator[EventSample]:
    reader = csv.DictReader(handle)
    _require_columns(reader, EVENT_COLUMNS, source)
    previous_timestamp: int | None = None
    for line, row in enumerate(reader, start=2):
        x = _integer(row, "x", source, line)
        y = _integer(row, "y", source, line)
        polarity = _integer(row, "polarity", source, line)
        timestamp = _integer(row, "timestamp", source, line)
        if not 0 <= x < width:
            raise ValueError(f"{source}:{line}: x outside [0, {width - 1}]")
        if not 0 <= y < height:
            raise ValueError(f"{source}:{line}: y outside [0, {height - 1}]")
        if polarity not in (0, 1):
            raise ValueError(f"{source}:{line}: polarity must be 0 or 1")
        if timestamp < 0:
            raise ValueError(f"{source}:{line}: timestamp must be non-negative")
        if previous_timestamp is not None and timestamp < previous_timestamp:
            raise ValueError(f"{source}:{line}: event timestamps are not monotonic")
        previous_timestamp = timestamp
        yield EventSample(x, y, polarity, timestamp)


def iter_event_samples(
    path: str | Path,
    *,
    width: int = 768,
    height: int = 608,
) -> Iterator[EventSample]:
    if width <= 0 or height <= 0:
        raise ValueError("event sensor dimensions must be positive")
    source_path = Path(path)
    with source_path.open("r", newline="", encoding="utf-8-sig") as handle:
        yield from _event_rows(handle, str(source_path), width, height)


def iter_event_packets(
    path: str | Path,
    *,
    window_us: int = 20_000,
    width: int = 768,
    height: int = 608,
) -> Iterator[EventPacket]:
    if window_us <= 0:
        raise ValueError("window_us must be positive")
    packet: list[EventSample] = []
    start: int | None = None
    for sample in iter_event_samples(path, width=width, height=height):
        if start is None:
            start = sample.timestamp_us
        if sample.timestamp_us >= start + window_us:
            if packet:
                yield EventPacket(start, window_us, tuple(packet))
                packet = []
            elapsed_windows = (sample.timestamp_us - start) // window_us
            start += elapsed_windows * window_us
        packet.append(sample)
    if packet and start is not None:
        yield EventPacket(start, window_us, tuple(packet))


def summarize_event_packet(
    packet: EventPacket,
    *,
    width: int = 768,
    height: int = 608,
) -> dict[str, float | int | bool]:
    count = len(packet.samples)
    if not count:
        return {
            "event_count": 0,
            "event_rate_hz": 0.0,
            "positive_fraction": 0.0,
            "pixel_coverage": 0.0,
            "timestamp_monotonic": True,
        }
    positive = sum(sample.polarity for sample in packet.samples)
    occupied = {(sample.x, sample.y) for sample in packet.samples}
    return {
        "event_count": count,
        "event_rate_hz": count * 1_000_000.0 / packet.window_us,
        "positive_fraction": positive / count,
        "pixel_coverage": len(occupied) / float(width * height),
        "timestamp_monotonic": all(
            left.timestamp_us <= right.timestamp_us
            for left, right in zip(packet.samples, packet.samples[1:])
        ),
    }


def _mmwave_rows(handle: TextIO, source: str) -> Iterator[MmwaveDetection]:
    reader = csv.DictReader(handle)
    _require_columns(reader, MMWAVE_COLUMNS, source)
    previous_timestamp: int | None = None
    previous_frame: int | None = None
    closed_frames: set[int] = set()
    for line, row in enumerate(reader, start=2):
        timestamp = _integer(row, "timestamp_us", source, line)
        frame_id = _integer(row, "frame_id", source, line)
        track_id = _integer(row, "track_id", source, line)
        if timestamp < 0 or frame_id < 0 or track_id < 0:
            raise ValueError(f"{source}:{line}: timestamp and identifiers must be non-negative")
        if previous_timestamp is not None and timestamp < previous_timestamp:
            raise ValueError(f"{source}:{line}: mmWave timestamps are not monotonic")
        if previous_frame is not None and frame_id != previous_frame:
            closed_frames.add(previous_frame)
        if frame_id in closed_frames:
            raise ValueError(f"{source}:{line}: frame_id {frame_id} is not contiguous")
        confidence = _number(row, "confidence", source, line)
        if not 0.0 <= confidence <= 1.0:
            raise ValueError(f"{source}:{line}: confidence outside [0, 1]")
        previous_timestamp = timestamp
        previous_frame = frame_id
        yield MmwaveDetection(
            timestamp_us=timestamp,
            frame_id=frame_id,
            track_id=track_id,
            x_m=_number(row, "x_m", source, line),
            y_m=_number(row, "y_m", source, line),
            z_m=_number(row, "z_m", source, line),
            radial_velocity_mps=_number(row, "radial_velocity_mps", source, line),
            snr_db=_number(row, "snr_db", source, line),
            confidence=confidence,
        )


def iter_mmwave_detections(path: str | Path) -> Iterator[MmwaveDetection]:
    source_path = Path(path)
    with source_path.open("r", newline="", encoding="utf-8-sig") as handle:
        yield from _mmwave_rows(handle, str(source_path))


def iter_mmwave_frames(path: str | Path) -> Iterator[MmwaveFrame]:
    detections: list[MmwaveDetection] = []
    frame_id: int | None = None
    timestamp = 0
    for detection in iter_mmwave_detections(path):
        if frame_id is None:
            frame_id = detection.frame_id
            timestamp = detection.timestamp_us
        if detection.frame_id != frame_id:
            yield MmwaveFrame(timestamp, frame_id, tuple(detections))
            detections = []
            frame_id = detection.frame_id
            timestamp = detection.timestamp_us
        detections.append(detection)
    if detections and frame_id is not None:
        yield MmwaveFrame(timestamp, frame_id, tuple(detections))


def summarize_mmwave_frame(
    frame: MmwaveFrame,
    *,
    moving_speed_threshold: float = 0.15,
) -> dict[str, float | int]:
    count = len(frame.detections)
    if not count:
        return {
            "detection_count": 0,
            "unique_track_count": 0,
            "moving_detection_count": 0,
            "mean_confidence": 0.0,
            "max_snr_db": 0.0,
        }
    return {
        "detection_count": count,
        "unique_track_count": len({item.track_id for item in frame.detections}),
        "moving_detection_count": sum(
            abs(item.radial_velocity_mps) >= moving_speed_threshold
            for item in frame.detections
        ),
        "mean_confidence": sum(item.confidence for item in frame.detections) / count,
        "max_snr_db": max(item.snr_db for item in frame.detections),
    }
