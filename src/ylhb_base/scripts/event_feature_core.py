#!/usr/bin/env python3
"""ROS-independent event-camera feature extraction and repeatability metrics."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence


@dataclass(frozen=True)
class EventObservation:
    x: int
    y: int
    polarity: int
    time_offset_us: int


@dataclass(frozen=True)
class EventFeatureConfig:
    width: int = 768
    height: int = 608
    cell_size_px: int = 12
    min_events_per_cell: int = 5
    min_unique_pixels: int = 3
    max_features: int = 512
    match_distance_px: float = 12.0

    def __post_init__(self) -> None:
        integer_values = (
            self.width,
            self.height,
            self.cell_size_px,
            self.min_events_per_cell,
            self.min_unique_pixels,
            self.max_features,
        )
        if any(value <= 0 for value in integer_values):
            raise ValueError("event feature dimensions and thresholds must be positive")
        if not math.isfinite(self.match_distance_px) or self.match_distance_px <= 0.0:
            raise ValueError("match_distance_px must be finite and positive")


@dataclass(frozen=True)
class EventFeature:
    x: float
    y: float
    event_count: int
    unique_pixel_count: int
    positive_fraction: float


def extract_event_features(
    observations: Iterable[EventObservation],
    config: EventFeatureConfig | None = None,
) -> list[EventFeature]:
    """Aggregate active pixels into deterministic grid features.

    Grid aggregation intentionally avoids a learned detector so the result is
    reproducible on both the RDK X5 and the Jetson replay environment.
    """

    config = config or EventFeatureConfig()
    cells: dict[tuple[int, int], list[EventObservation]] = {}
    for observation in observations:
        if not (0 <= observation.x < config.width and 0 <= observation.y < config.height):
            raise ValueError("event coordinate is outside the configured image")
        if observation.polarity not in (0, 1) or observation.time_offset_us < 0:
            raise ValueError("event polarity must be 0/1 and time offset non-negative")
        key = (observation.x // config.cell_size_px, observation.y // config.cell_size_px)
        cells.setdefault(key, []).append(observation)

    candidates: list[tuple[tuple[int, int], EventFeature]] = []
    for key, samples in cells.items():
        unique_pixels = {(sample.x, sample.y) for sample in samples}
        if (
            len(samples) < config.min_events_per_cell
            or len(unique_pixels) < config.min_unique_pixels
        ):
            continue
        count = len(samples)
        feature = EventFeature(
            x=sum(sample.x for sample in samples) / count,
            y=sum(sample.y for sample in samples) / count,
            event_count=count,
            unique_pixel_count=len(unique_pixels),
            positive_fraction=sum(sample.polarity for sample in samples) / count,
        )
        candidates.append((key, feature))

    selected = sorted(
        candidates,
        key=lambda item: (-item[1].event_count, -item[1].unique_pixel_count, item[0]),
    )[: config.max_features]
    return [feature for _, feature in sorted(selected, key=lambda item: item[0])]


def _one_to_one_match_count(
    previous: Sequence[EventFeature],
    current: Sequence[EventFeature],
    match_distance_px: float,
) -> int:
    limit_squared = match_distance_px * match_distance_px
    adjacency: list[list[int]] = []
    for feature in previous:
        matches: list[tuple[float, int]] = []
        for index, candidate in enumerate(current):
            distance_squared = (feature.x - candidate.x) ** 2 + (feature.y - candidate.y) ** 2
            if distance_squared <= limit_squared:
                matches.append((distance_squared, index))
        adjacency.append([index for _, index in sorted(matches)])

    matched_current: dict[int, int] = {}

    def augment(previous_index: int, seen: set[int]) -> bool:
        for current_index in adjacency[previous_index]:
            if current_index in seen:
                continue
            seen.add(current_index)
            displaced = matched_current.get(current_index)
            if displaced is None or augment(displaced, seen):
                matched_current[current_index] = previous_index
                return True
        return False

    return sum(augment(index, set()) for index in range(len(previous)))


def feature_repeatability(
    previous: Sequence[EventFeature],
    current: Sequence[EventFeature],
    match_distance_px: float,
) -> float | None:
    """Return symmetric adjacent-packet repeatability ``2M/(N1+N2)``."""

    if not math.isfinite(match_distance_px) or match_distance_px <= 0.0:
        raise ValueError("match_distance_px must be finite and positive")
    if not previous or not current:
        return None
    matches = _one_to_one_match_count(previous, current, match_distance_px)
    return 2.0 * matches / (len(previous) + len(current))
