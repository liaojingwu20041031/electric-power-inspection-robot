#!/usr/bin/env python3
"""ROS-independent 2D LiDAR robust filtering and feature POC.

This module deliberately contains no ROS imports.  It accepts a small scan
record, performs transparent geometric filtering and temporal comparison, and
returns plain Python data structures suitable for unit tests or a ROS wrapper.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Iterable, Sequence


@dataclass(frozen=True)
class RobustFeatureConfig:
    """All POC thresholds are explicit and can be overridden by the ROS node."""

    min_range: float = 0.05
    max_range: float = 8.0
    self_radius: float = 0.18
    isolation_range_jump: float = 0.35
    temporal_match_distance: float = 0.12
    corner_deviation: float = math.radians(25.0)
    corner_neighbor_distance: float = 0.35
    line_neighbor_distance: float = 0.25
    min_line_points: int = 3

    def __post_init__(self) -> None:
        numeric = (
            self.min_range,
            self.max_range,
            self.self_radius,
            self.isolation_range_jump,
            self.temporal_match_distance,
            self.corner_deviation,
            self.corner_neighbor_distance,
            self.line_neighbor_distance,
        )
        if any(not math.isfinite(value) or value < 0.0 for value in numeric):
            raise ValueError("all distance and angle thresholds must be finite and non-negative")
        if self.max_range <= self.min_range:
            raise ValueError("max_range must be greater than min_range")
        if self.min_line_points < 2:
            raise ValueError("min_line_points must be at least 2")


@dataclass(frozen=True)
class ScanFrame:
    """Minimal LaserScan-like input for the ROS-independent core."""

    ranges: Sequence[float]
    angle_min: float
    angle_increment: float
    range_min: float = 0.0
    range_max: float = 0.0


@dataclass(frozen=True)
class ScanPoint:
    index: int
    angle: float
    x: float
    y: float
    range: float


@dataclass(frozen=True)
class DynamicSector:
    """A range-bearing region independently reported as dynamic."""

    angle: float
    range: float
    angular_half_width: float
    radial_margin: float

    def __post_init__(self) -> None:
        values = (self.angle, self.range, self.angular_half_width, self.radial_margin)
        if any(not _finite(value) for value in values):
            raise ValueError("dynamic sector values must be finite")
        if self.range < 0.0 or self.angular_half_width < 0.0 or self.radial_margin < 0.0:
            raise ValueError("dynamic sector range and margins must be non-negative")


@dataclass(frozen=True)
class DynamicDetection2D:
    """Normalized moving-object observation in an auxiliary sensor frame."""

    x: float
    y: float
    radial_velocity: float
    snr: float
    confidence: float


def dynamic_sector_from_detection(
    detection: DynamicDetection2D,
    sensor_to_lidar: tuple[float, float, float],
    *,
    min_abs_velocity: float,
    min_snr: float,
    min_confidence: float,
    angular_half_width: float,
    radial_margin: float,
) -> DynamicSector | None:
    """Transform and quality-gate one radar detection into the LiDAR frame."""

    values = (
        detection.x,
        detection.y,
        detection.radial_velocity,
        detection.snr,
        detection.confidence,
        *sensor_to_lidar,
        min_abs_velocity,
        min_snr,
        min_confidence,
        angular_half_width,
        radial_margin,
    )
    if any(not _finite(value) for value in values):
        return None
    if (
        abs(detection.radial_velocity) < min_abs_velocity
        or detection.snr < min_snr
        or detection.confidence < min_confidence
    ):
        return None
    offset_x, offset_y, yaw = sensor_to_lidar
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    x = cosine * detection.x - sine * detection.y + offset_x
    y = sine * detection.x + cosine * detection.y + offset_y
    return DynamicSector(
        angle=math.atan2(y, x),
        range=math.hypot(x, y),
        angular_half_width=angular_half_width,
        radial_margin=radial_margin,
    )


@dataclass(frozen=True)
class ClassifiedPoint:
    point: ScanPoint
    stable: bool
    dynamic_candidate: bool
    temporal_distance: float | None


@dataclass
class FrameResult:
    """Structured result used by tests and the optional ROS2 adapter."""

    raw_point_count: int
    points: list[ScanPoint]
    rejected_indices: set[int]
    classifications: list[ClassifiedPoint]
    features: list[dict[str, Any]]
    temporal_match_ratio: float | None
    outlier_count: int
    quality: dict[str, Any] = field(default_factory=dict)

    @property
    def stable_points(self) -> list[ScanPoint]:
        return [item.point for item in self.classifications if item.stable]

    @property
    def dynamic_points(self) -> list[ScanPoint]:
        return [item.point for item in self.classifications if item.dynamic_candidate]


def _finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def scan_to_points(frame: ScanFrame, config: RobustFeatureConfig) -> tuple[list[ScanPoint], set[int], int]:
    """Convert polar ranges to points and reject invalid/self points.

    Returns cleaned points, rejected beam indices, and the count rejected at
    this stage.  The original beam index is retained for deterministic output.
    """

    if not _finite(frame.angle_min) or not _finite(frame.angle_increment):
        raise ValueError("angle_min and angle_increment must be finite")
    if not _finite(frame.range_min) or not _finite(frame.range_max):
        raise ValueError("range_min and range_max must be finite")
    sensor_min = max(config.min_range, float(frame.range_min))
    sensor_max_value = float(frame.range_max)
    sensor_max = min(config.max_range, sensor_max_value) if sensor_max_value > 0.0 else config.max_range
    if sensor_max <= sensor_min:
        raise ValueError("effective scan range is empty")

    points: list[ScanPoint] = []
    rejected: set[int] = set()
    for index, raw_range in enumerate(frame.ranges):
        if not _finite(raw_range):
            rejected.add(index)
            continue
        distance = float(raw_range)
        angle = float(frame.angle_min) + index * float(frame.angle_increment)
        if distance < sensor_min or distance > sensor_max:
            rejected.add(index)
            continue
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        if math.hypot(x, y) <= config.self_radius:
            rejected.add(index)
            continue
        points.append(ScanPoint(index=index, angle=angle, x=x, y=y, range=distance))
    return points, rejected, len(rejected)


def remove_isolated_points(points: Sequence[ScanPoint], config: RobustFeatureConfig) -> tuple[list[ScanPoint], set[int]]:
    """Remove a point whose two scan neighbours both show a large jump."""

    by_index = {point.index: point for point in points}
    isolated: set[int] = set()
    for point in points:
        left = by_index.get(point.index - 1)
        right = by_index.get(point.index + 1)
        if left is None or right is None:
            continue
        left_jump = abs(point.range - left.range)
        right_jump = abs(point.range - right.range)
        if left_jump > config.isolation_range_jump and right_jump > config.isolation_range_jump:
            isolated.add(point.index)
    return [point for point in points if point.index not in isolated], isolated


def compensate_previous_points(points: Iterable[ScanPoint], relative_pose: tuple[float, float, float]) -> list[ScanPoint]:
    """Transform previous-frame points into the current frame.

    Convention: ``relative_pose=(dx, dy, dyaw)`` is the current robot pose
    expressed in the previous robot frame.  Static previous points therefore
    use ``R(-dyaw) * (p_previous - [dx, dy])``.
    """

    dx, dy, dyaw = relative_pose
    if not all(_finite(value) for value in relative_pose):
        raise ValueError("relative pose must contain finite dx, dy and dyaw")
    cosine = math.cos(float(dyaw))
    sine = math.sin(float(dyaw))
    compensated: list[ScanPoint] = []
    for point in points:
        translated_x = point.x - float(dx)
        translated_y = point.y - float(dy)
        x = cosine * translated_x + sine * translated_y
        y = -sine * translated_x + cosine * translated_y
        compensated.append(
            ScanPoint(index=point.index, angle=math.atan2(y, x), x=x, y=y, range=math.hypot(x, y))
        )
    return compensated


def temporal_classify(
    current_points: Sequence[ScanPoint],
    previous_points: Sequence[ScanPoint] | None,
    relative_pose: tuple[float, float, float],
    config: RobustFeatureConfig,
) -> tuple[list[ClassifiedPoint], float | None]:
    """Classify current points by nearest-neighbour temporal consistency."""

    if previous_points is None:
        return [ClassifiedPoint(point, True, False, None) for point in current_points], None
    compensated = compensate_previous_points(previous_points, relative_pose)
    if not compensated:
        return [ClassifiedPoint(point, False, True, None) for point in current_points], 0.0
    classifications: list[ClassifiedPoint] = []
    matched = 0
    for point in current_points:
        nearest = min(math.hypot(point.x - old.x, point.y - old.y) for old in compensated)
        # Keep an exact threshold boundary stable despite binary float rounding.
        stable = nearest <= config.temporal_match_distance + 1e-12
        matched += int(stable)
        classifications.append(ClassifiedPoint(point, stable, not stable, nearest))
    ratio = matched / len(current_points) if current_points else 0.0
    return classifications, ratio


def apply_dynamic_sectors(
    classifications: Sequence[ClassifiedPoint],
    sectors: Sequence[DynamicSector],
    temporal_ratio: float | None,
) -> tuple[list[ClassifiedPoint], set[int], float | None]:
    """Remove independently confirmed moving sectors from temporal features."""

    if not sectors:
        return list(classifications), set(), temporal_ratio
    masked: set[int] = set()
    result: list[ClassifiedPoint] = []
    for item in classifications:
        point = item.point
        external_dynamic = any(
            abs(math.atan2(math.sin(point.angle - sector.angle), math.cos(point.angle - sector.angle)))
            <= sector.angular_half_width + 1e-12
            and abs(point.range - sector.range) <= sector.radial_margin + 1e-12
            for sector in sectors
        )
        if external_dynamic:
            masked.add(point.index)
            result.append(ClassifiedPoint(point, False, True, item.temporal_distance))
        else:
            result.append(item)
    remaining = [item for item in result if item.point.index not in masked]
    ratio = sum(item.stable for item in remaining) / len(remaining) if remaining else 0.0
    return result, masked, ratio


def _interior_angle(previous: ScanPoint, current: ScanPoint, following: ScanPoint) -> float:
    first_x = previous.x - current.x
    first_y = previous.y - current.y
    second_x = following.x - current.x
    second_y = following.y - current.y
    first_norm = math.hypot(first_x, first_y)
    second_norm = math.hypot(second_x, second_y)
    if first_norm == 0.0 or second_norm == 0.0:
        return math.pi
    cosine = (first_x * second_x + first_y * second_y) / (first_norm * second_norm)
    return math.acos(max(-1.0, min(1.0, cosine)))


def extract_stable_features(classifications: Sequence[ClassifiedPoint], config: RobustFeatureConfig) -> list[dict[str, Any]]:
    """Extract deterministic corner and line representative features."""

    stable = [item.point for item in classifications if item.stable]
    by_index = {point.index: point for point in stable}
    features: list[dict[str, Any]] = []
    for point in stable:
        previous = by_index.get(point.index - 1)
        following = by_index.get(point.index + 1)
        if previous is None or following is None:
            continue
        if max(math.hypot(point.x - previous.x, point.y - previous.y), math.hypot(following.x - point.x, following.y - point.y)) > config.corner_neighbor_distance:
            continue
        angle = _interior_angle(previous, point, following)
        deviation = abs(math.pi - angle)
        if deviation >= config.corner_deviation:
            features.append({"type": "corner", "index": point.index, "x": point.x, "y": point.y, "strength": deviation})

    segments: list[list[ScanPoint]] = []
    current_segment: list[ScanPoint] = []
    for point in stable:
        if current_segment and (
            point.index != current_segment[-1].index + 1
            or math.hypot(point.x - current_segment[-1].x, point.y - current_segment[-1].y) > config.line_neighbor_distance
        ):
            if len(current_segment) >= config.min_line_points:
                segments.append(current_segment)
            current_segment = []
        current_segment.append(point)
    if len(current_segment) >= config.min_line_points:
        segments.append(current_segment)
    for segment in segments:
        middle = segment[len(segment) // 2]
        features.append(
            {
                "type": "line",
                "index": middle.index,
                "x": sum(point.x for point in segment) / len(segment),
                "y": sum(point.y for point in segment) / len(segment),
                "strength": float(len(segment)),
                "point_count": len(segment),
            }
        )
    return features


def _angular_coverage(points: Sequence[ScanPoint]) -> float:
    if len(points) < 2:
        return 0.0
    angles = [point.angle for point in points]
    return max(angles) - min(angles)


def process_scan(
    frame: ScanFrame,
    previous_frame: ScanFrame | None = None,
    relative_pose: tuple[float, float, float] = (0.0, 0.0, 0.0),
    config: RobustFeatureConfig | None = None,
    external_dynamic_sectors: Sequence[DynamicSector] = (),
) -> FrameResult:
    """Run the complete POC pipeline for one scan."""

    config = config or RobustFeatureConfig()
    points, rejected, initial_rejected_count = scan_to_points(frame, config)
    points, isolated = remove_isolated_points(points, config)
    rejected = set(rejected) | isolated
    classifications, raw_temporal_ratio = temporal_classify(
        points,
        None if previous_frame is None else _clean_previous_points(previous_frame, config),
        relative_pose,
        config,
    )
    classifications, externally_masked, temporal_ratio = apply_dynamic_sectors(
        classifications,
        external_dynamic_sectors,
        raw_temporal_ratio,
    )
    features = extract_stable_features(classifications, config)
    stable_points = [item.point for item in classifications if item.stable]
    dynamic_count = sum(item.dynamic_candidate for item in classifications)
    angular_coverage = _angular_coverage(stable_points)
    valid_ratio = len(points) / len(frame.ranges) if frame.ranges else 0.0
    geometry_score = (len(stable_points) / len(points)) * min(1.0, angular_coverage / math.pi) if points else 0.0
    quality = {
        "raw_point_count": len(frame.ranges),
        "valid_point_count": len(points),
        "valid_ratio": valid_ratio,
        "outlier_count": initial_rejected_count + len(isolated),
        "dynamic_candidate_count": dynamic_count,
        "temporal_dynamic_candidate_count": sum(
            item.dynamic_candidate and item.point.index not in externally_masked
            for item in classifications
        ),
        "auxiliary_dynamic_sector_count": len(external_dynamic_sectors),
        "auxiliary_dynamic_point_count": len(externally_masked),
        "stable_point_count": len(stable_points),
        "feature_count": len(features),
        "angular_coverage": angular_coverage,
        "temporal_match_ratio": temporal_ratio,
        "raw_temporal_match_ratio": raw_temporal_ratio,
        "geometry_score": geometry_score,
        "temporal_status": "NO_PREVIOUS_FRAME" if previous_frame is None else "COMPARED",
    }
    return FrameResult(
        raw_point_count=len(frame.ranges),
        points=points,
        rejected_indices=rejected,
        classifications=classifications,
        features=features,
        temporal_match_ratio=temporal_ratio,
        outlier_count=initial_rejected_count + len(isolated),
        quality=quality,
    )


def _clean_previous_points(frame: ScanFrame, config: RobustFeatureConfig) -> list[ScanPoint]:
    points, rejected, _ = scan_to_points(frame, config)
    cleaned, isolated = remove_isolated_points(points, config)
    ignored = rejected | isolated
    return [point for point in cleaned if point.index not in ignored]
