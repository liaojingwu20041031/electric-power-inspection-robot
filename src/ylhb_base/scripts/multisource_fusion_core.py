#!/usr/bin/env python3
"""ROS-independent DG-202611 multi-source localization POC.

The core estimates a global map pose from the existing local odometry and
optional absolute corrections.  It deliberately does not own a TF tree or
replace the robot's wheel/IMU EKF.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Iterable


INITIALIZING = "INITIALIZING"
NOMINAL = "NOMINAL"
GNSS_AIDED = "GNSS_AIDED"
LIDAR_AIDED = "LIDAR_AIDED"
UWB_AIDED = "UWB_AIDED"
DEAD_RECKONING = "DEAD_RECKONING"
DEGRADED = "DEGRADED"
REJECTED_UPDATE = "REJECTED_UPDATE"
WAITING_ALIGNMENT = "WAITING_ALIGNMENT"

GOOD = "GOOD"
DEGRADED_QUALITY = "DEGRADED"
REJECTED = "REJECTED"
STALE = "STALE"

WGS84_A = 6378137.0
WGS84_E2 = 6.6943799901413165e-3

LOCAL_ODOM = "LOCAL_ODOM"
GNSS_POSITION = "GNSS_POSITION"
LIDAR_MAP_POSE_CORRECTION = "LIDAR_MAP_POSE_CORRECTION"


def _finite(value: object) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _clamp(value: float, lower: float = 0.0, upper: float = 1.0) -> float:
    return max(lower, min(upper, value))


def _finite_pose(pose: "Pose2D | None") -> bool:
    return pose is not None and all(_finite(value) for value in (pose.x, pose.y, pose.yaw, pose.timestamp))


@dataclass(frozen=True)
class GeodeticPoint:
    latitude: float
    longitude: float
    altitude: float = 0.0


@dataclass(frozen=True)
class ENUPoint:
    east: float
    north: float
    up: float


@dataclass(frozen=True)
class GeodeticReference:
    latitude: float
    longitude: float
    altitude: float = 0.0


def geodetic_to_ecef(point: GeodeticPoint) -> tuple[float, float, float]:
    """Convert WGS84 latitude/longitude/altitude to ECEF metres."""
    if not all(_finite(value) for value in (point.latitude, point.longitude, point.altitude)):
        raise ValueError("geodetic point must be finite")
    latitude = math.radians(point.latitude)
    longitude = math.radians(point.longitude)
    sin_lat = math.sin(latitude)
    cos_lat = math.cos(latitude)
    sin_lon = math.sin(longitude)
    cos_lon = math.cos(longitude)
    radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    return (
        (radius + point.altitude) * cos_lat * cos_lon,
        (radius + point.altitude) * cos_lat * sin_lon,
        (radius * (1.0 - WGS84_E2) + point.altitude) * sin_lat,
    )


def geodetic_to_enu(point: GeodeticPoint, reference: GeodeticReference) -> ENUPoint:
    """Convert WGS84 geodetic coordinates to a local tangent-plane ENU frame."""
    ref = GeodeticPoint(reference.latitude, reference.longitude, reference.altitude)
    x, y, z = geodetic_to_ecef(point)
    x0, y0, z0 = geodetic_to_ecef(ref)
    latitude = math.radians(reference.latitude)
    longitude = math.radians(reference.longitude)
    dx, dy, dz = x - x0, y - y0, z - z0
    sin_lat = math.sin(latitude)
    cos_lat = math.cos(latitude)
    sin_lon = math.sin(longitude)
    cos_lon = math.cos(longitude)
    return ENUPoint(
        -sin_lon * dx + cos_lon * dy,
        -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz,
        cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz,
    )


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float
    timestamp: float


@dataclass(frozen=True)
class Alignment2D:
    """Map pose = R(map_enu_yaw) * ENU position + map offset."""

    map_enu_yaw: float
    offset_x: float
    offset_y: float

    def apply(self, point: ENUPoint) -> tuple[float, float]:
        cosine = math.cos(self.map_enu_yaw)
        sine = math.sin(self.map_enu_yaw)
        return (
            cosine * point.east - sine * point.north + self.offset_x,
            sine * point.east + cosine * point.north + self.offset_y,
        )


def estimate_alignment(pairs: Iterable[tuple[ENUPoint, Pose2D]]) -> Alignment2D | None:
    """Estimate a 2D rigid ENU->map alignment from at least two pairs."""
    samples = [(enu, pose) for enu, pose in pairs if _finite_pose(pose) and all(_finite(v) for v in (enu.east, enu.north))]
    if len(samples) < 2:
        return None
    enu_x = sum(item[0].east for item in samples) / len(samples)
    enu_y = sum(item[0].north for item in samples) / len(samples)
    map_x = sum(item[1].x for item in samples) / len(samples)
    map_y = sum(item[1].y for item in samples) / len(samples)
    cosine_term = 0.0
    sine_term = 0.0
    for enu, pose in samples:
        ex = enu.east - enu_x
        ny = enu.north - enu_y
        mx = pose.x - map_x
        my = pose.y - map_y
        cosine_term += ex * mx + ny * my
        sine_term += ex * my - ny * mx
    if math.hypot(cosine_term, sine_term) <= 1e-9:
        return None
    yaw = math.atan2(sine_term, cosine_term)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return Alignment2D(
        yaw,
        map_x - cosine * enu_x + sine * enu_y,
        map_y - sine * enu_x - cosine * enu_y,
    )


@dataclass(frozen=True)
class GnssPosition:
    latitude: float
    longitude: float
    altitude: float
    timestamp: float
    state: str = GOOD
    fresh: bool = True
    accepted: bool = True
    hdop: float | None = None
    satellites: float | None = None
    differential_age: float | None = None


@dataclass(frozen=True)
class MapPoseMeasurement:
    pose: Pose2D
    timestamp: float
    state: str = GOOD
    fresh: bool = True
    accepted: bool = True
    source: str = "amcl"
    geometry_score: float | None = None
    temporal_match_ratio: float | None = None
    covariance: float | None = None
    match_score: float | None = None
    inlier_ratio: float | None = None
    mean_distance: float | None = None


@dataclass(frozen=True)
class UwbPositionMeasurement:
    """UWB 2D position measurement for MODEL-1 fusion."""
    x: float
    y: float
    timestamp: float
    state: str = GOOD
    fresh: bool = True
    accepted: bool = True
    residual_rms: float | None = None
    geometry_score: float | None = None
    unique_anchor_count: int = 0
    confidence: float = 0.0


@dataclass(frozen=True)
class FusionInput:
    now: float
    local_odom: Pose2D | None = None
    gnss: GnssPosition | None = None
    scan_match_pose: MapPoseMeasurement | None = None
    amcl_pose: MapPoseMeasurement | None = None
    uwb: UwbPositionMeasurement | None = None


@dataclass(frozen=True)
class FusionConfig:
    gnss_reference: GeodeticReference | None = None
    alignment: Alignment2D | None = None
    freshness_timeout: float = 2.0
    gnss_residual_gate: float = 8.0
    lidar_residual_gate: float = 5.0
    yaw_residual_gate: float = math.radians(60.0)
    max_correction_step: float = 0.5
    max_yaw_correction_step: float = math.radians(20.0)
    initial_position_uncertainty: float = 1.0
    initial_yaw_uncertainty: float = math.radians(20.0)
    odom_position_noise_per_m: float = 0.03
    odom_yaw_noise_per_rad: float = 0.05
    # POC heuristic drift terms; these are not statistically calibrated covariance.
    odom_position_noise_per_s: float = 0.005
    odom_yaw_noise_per_s: float = math.radians(0.1)
    min_position_uncertainty: float = 0.05
    min_yaw_uncertainty: float = math.radians(1.0)
    gnss_good_hdop: float = 1.5
    gnss_degraded_hdop: float = 3.0
    gnss_good_satellites: float = 8.0
    gnss_degraded_satellites: float = 4.0
    gnss_max_differential_age: float = 3.0
    amcl_max_covariance: float = 0.5
    adaptive_min_confidence: float = 0.1
    gnss_base_position_sigma: float = 1.0
    lidar_base_position_sigma: float = 0.5
    lidar_base_yaw_sigma: float = math.radians(10.0)
    uwb_residual_gate: float = 3.0
    uwb_base_position_sigma: float = 0.5
    uwb_min_geometry_score: float = 0.1


@dataclass(frozen=True)
class FusionOutput:
    map_pose: Pose2D | None
    map_to_odom: Pose2D | None
    position_uncertainty: float
    yaw_uncertainty: float
    fusion_mode: str
    accepted_source: str | None
    updated: bool
    reasons: tuple[str, ...]
    timestamp: float
    global_anchored: bool
    measurement_confidence: float
    adaptive_position_uncertainty: float
    adaptive_yaw_uncertainty: float


def _threshold_quality(
    value: float | None,
    good: float,
    degraded: float,
    higher_is_better: bool = False,
) -> float | None:
    if value is None or not _finite(value):
        return None
    if higher_is_better:
        if good <= degraded:
            return 1.0 if value >= good else 0.25
        return 1.0 if value >= good else 0.25 if value <= degraded else 0.25 + 0.75 * ((value - degraded) / (good - degraded))
    if degraded <= good:
        return 1.0 if value <= good else 0.25
    return 1.0 if value <= good else 0.25 if value >= degraded else 1.0 - 0.75 * ((value - good) / (degraded - good))


def compute_gnss_confidence(measurement: GnssPosition, config: FusionConfig | None = None) -> float:
    """Return a bounded confidence from the existing GNSS diagnostic fields."""
    config = config or FusionConfig()
    if not measurement.accepted or not measurement.fresh or measurement.state.upper() in {REJECTED, STALE}:
        return 0.0
    state_factor = 0.5 if measurement.state.upper() == DEGRADED_QUALITY else 1.0
    scores = [
        _threshold_quality(measurement.hdop, config.gnss_good_hdop, config.gnss_degraded_hdop),
        _threshold_quality(
            measurement.satellites,
            config.gnss_good_satellites,
            config.gnss_degraded_satellites,
            higher_is_better=True,
        ),
    ]
    if measurement.differential_age is not None and _finite(measurement.differential_age):
        age = max(0.0, float(measurement.differential_age))
        scores.append(1.0 if age <= config.gnss_max_differential_age else 0.25)
    valid_scores = [score for score in scores if score is not None]
    return _clamp(state_factor * (sum(valid_scores) / len(valid_scores) if valid_scores else 1.0))


def compute_lidar_confidence(measurement: MapPoseMeasurement, config: FusionConfig | None = None) -> float:
    """Return a bounded confidence from LiDAR/global-pose quality diagnostics."""
    config = config or FusionConfig()
    if not measurement.accepted or not measurement.fresh or measurement.state.upper() != GOOD:
        return 0.0
    scores: list[float] = []
    for value in (measurement.geometry_score, measurement.temporal_match_ratio, measurement.inlier_ratio):
        if value is not None and _finite(value):
            scores.append(_clamp(float(value)))
    if measurement.covariance is not None and _finite(measurement.covariance) and config.amcl_max_covariance > 0.0:
        scores.append(_clamp(1.0 - float(measurement.covariance) / config.amcl_max_covariance, 0.25, 1.0))
    if measurement.match_score is not None and _finite(measurement.match_score):
        scores.append(_clamp(float(measurement.match_score)))
    if measurement.mean_distance is not None and _finite(measurement.mean_distance):
        scores.append(_clamp(math.exp(-max(0.0, float(measurement.mean_distance)))))
    return _clamp(sum(scores) / len(scores) if scores else 1.0)


def compute_uwb_confidence(measurement: UwbPositionMeasurement, config: FusionConfig | None = None) -> float:
    """Return a bounded confidence from UWB quality diagnostics."""
    config = config or FusionConfig()
    if not measurement.accepted or not measurement.fresh or measurement.state.upper() in {REJECTED, STALE}:
        return 0.0

    state_factor = 0.5 if measurement.state.upper() == DEGRADED_QUALITY else 1.0

    scores: list[float] = []

    # Geometry score
    if measurement.geometry_score is not None and _finite(measurement.geometry_score):
        geo_score = _clamp(float(measurement.geometry_score) / config.uwb_min_geometry_score if config.uwb_min_geometry_score > 0 else 1.0)
        scores.append(geo_score)

    # Residual RMS (lower is better)
    if measurement.residual_rms is not None and _finite(measurement.residual_rms):
        residual_score = _clamp(1.0 - float(measurement.residual_rms) / 2.0, 0.0, 1.0)
        scores.append(residual_score)

    # Anchor count (more is better, but max useful is ~4)
    if measurement.unique_anchor_count > 0:
        anchor_score = min(1.0, measurement.unique_anchor_count / 4.0)
        scores.append(anchor_score)

    # Use provided confidence if available
    if measurement.confidence > 0.0:
        scores.append(_clamp(measurement.confidence))

    return _clamp(state_factor * (sum(scores) / len(scores) if scores else 0.5))


def _compose(first: Pose2D, second: Pose2D, timestamp: float) -> Pose2D:
    cosine = math.cos(first.yaw)
    sine = math.sin(first.yaw)
    return Pose2D(
        first.x + cosine * second.x - sine * second.y,
        first.y + sine * second.x + cosine * second.y,
        wrap_angle(first.yaw + second.yaw),
        timestamp,
    )


def _inverse(pose: Pose2D, timestamp: float) -> Pose2D:
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    return Pose2D(-cosine * pose.x - sine * pose.y, sine * pose.x - cosine * pose.y, -pose.yaw, timestamp)


def _relative(previous: Pose2D, current: Pose2D) -> Pose2D:
    return _compose(_inverse(previous, current.timestamp), current, current.timestamp)


class MultisourceFusionCore:
    """Propagate local odometry and apply guarded absolute map corrections."""

    def __init__(self, config: FusionConfig | None = None) -> None:
        self.config = config or FusionConfig()
        self._validate_config()
        self._alignment = self.config.alignment
        self._alignment_pairs: list[tuple[ENUPoint, Pose2D]] = []
        self._last_local_odom: Pose2D | None = None
        self._last_absolute_timestamps: dict[str, float] = {}
        self._last_absolute_update_time: float | None = None
        self._last_process_time: float | None = None
        self._map_pose: Pose2D | None = None
        self._global_anchored = False
        self._position_uncertainty = self.config.initial_position_uncertainty
        self._yaw_uncertainty = self.config.initial_yaw_uncertainty
        self._last_measurement_confidence = 0.0
        self._adaptive_position_uncertainty = self.config.gnss_base_position_sigma
        self._adaptive_yaw_uncertainty = self.config.lidar_base_yaw_sigma
        self._last_output: FusionOutput | None = None

    def _validate_config(self) -> None:
        nonnegative = (
            self.config.freshness_timeout,
            self.config.gnss_residual_gate,
            self.config.lidar_residual_gate,
            self.config.uwb_residual_gate,
            self.config.max_correction_step,
            self.config.max_yaw_correction_step,
            self.config.odom_position_noise_per_m,
            self.config.odom_yaw_noise_per_rad,
            self.config.odom_position_noise_per_s,
            self.config.odom_yaw_noise_per_s,
            self.config.gnss_good_hdop,
            self.config.gnss_degraded_hdop,
            self.config.gnss_good_satellites,
            self.config.gnss_degraded_satellites,
            self.config.gnss_max_differential_age,
            self.config.amcl_max_covariance,
            self.config.adaptive_min_confidence,
            self.config.gnss_base_position_sigma,
            self.config.lidar_base_position_sigma,
            self.config.lidar_base_yaw_sigma,
            self.config.uwb_base_position_sigma,
            self.config.uwb_min_geometry_score,
        )
        if any(value < 0.0 for value in nonnegative):
            raise ValueError("fusion limits and noise values must be non-negative")
        if self.config.adaptive_min_confidence > 1.0:
            raise ValueError("adaptive_min_confidence must not exceed 1.0")

    @property
    def alignment(self) -> Alignment2D | None:
        return self._alignment

    def set_alignment(self, alignment: Alignment2D) -> None:
        if not all(_finite(value) for value in (alignment.map_enu_yaw, alignment.offset_x, alignment.offset_y)):
            raise ValueError("alignment must be finite")
        self._alignment = alignment

    def add_alignment_pair(self, enu: ENUPoint, map_pose: Pose2D) -> Alignment2D | None:
        self._alignment_pairs.append((enu, map_pose))
        estimated = estimate_alignment(self._alignment_pairs)
        if estimated is not None:
            self._alignment = estimated
        return estimated

    def add_geodetic_alignment_pair(self, point: GeodeticPoint, map_pose: Pose2D) -> Alignment2D | None:
        if self.config.gnss_reference is None:
            raise ValueError("GNSS reference is required for geodetic alignment")
        return self.add_alignment_pair(geodetic_to_enu(point, self.config.gnss_reference), map_pose)

    def process(self, event: FusionInput) -> FusionOutput:
        if not _finite(event.now):
            return self._output(event.now, INITIALIZING, None, False, ("INVALID_NOW",))
        reasons: list[str] = []
        if self._last_process_time is not None:
            if event.now < self._last_process_time:
                reasons.append("FUSION_TIME_REVERSED")
            else:
                dt = event.now - self._last_process_time
                self._position_uncertainty += self.config.odom_position_noise_per_s * dt
                self._yaw_uncertainty += self.config.odom_yaw_noise_per_s * dt
        self._last_process_time = event.now
        reason_count = len(reasons)
        propagated = self._propagate_odom(event.local_odom, event.now, reasons)
        if event.local_odom is not None and not propagated and len(reasons) == reason_count:
            reasons.append("LOCAL_ODOM_REJECTED")

        if event.gnss is not None and not self._gnss_usable(event.gnss, event.now, reasons):
            if event.gnss.state.upper() not in {REJECTED, STALE} and event.gnss.accepted:
                reasons.append("GNSS_ALIGNMENT_UNAVAILABLE" if self._alignment is None else "GNSS_NOT_USABLE")
        gnss_update = self._gnss_update(event.gnss, event.now, reasons)
        uwb_update = self._uwb_update(event.uwb, event.now, reasons)
        lidar_update, lidar_source = self._select_lidar_update(event, event.now, reasons)

        accepted_source: str | None = None
        updated = False
        if not self._global_anchored:
            # First global anchor priority: Scan-to-Map, then AMCL, then UWB, then aligned GNSS.
            if lidar_update is not None:
                lidar_source_name = lidar_source or lidar_update.source.upper()
                updated = self._apply_absolute_update(
                    lidar_update.pose,
                    lidar_source_name,
                    self.config.lidar_residual_gate,
                    event.now,
                    reasons,
                    compute_lidar_confidence(lidar_update, self.config),
                    correct_yaw=True,
                )
                if updated:
                    accepted_source = lidar_source_name
            elif uwb_update is not None:
                uwb_pose, confidence = uwb_update
                updated = self._apply_absolute_update(
                    uwb_pose,
                    "UWB",
                    self.config.uwb_residual_gate,
                    event.now,
                    reasons,
                    confidence,
                    correct_yaw=False,
                )
                if updated:
                    accepted_source = "UWB"
            elif gnss_update is not None:
                gnss_pose, confidence = gnss_update
                updated = self._apply_absolute_update(
                    gnss_pose,
                    "GNSS",
                    self.config.gnss_residual_gate,
                    event.now,
                    reasons,
                    confidence,
                    correct_yaw=False,
                )
                if updated:
                    accepted_source = "GNSS"
        else:
            if gnss_update is not None:
                gnss_pose, confidence = gnss_update
                updated = self._apply_absolute_update(
                    gnss_pose,
                    "GNSS",
                    self.config.gnss_residual_gate,
                    event.now,
                    reasons,
                    confidence,
                    correct_yaw=False,
                )
                if updated:
                    accepted_source = "GNSS"
            if uwb_update is not None:
                uwb_pose, confidence = uwb_update
                uwb_applied = self._apply_absolute_update(
                    uwb_pose,
                    "UWB",
                    self.config.uwb_residual_gate,
                    event.now,
                    reasons,
                    confidence,
                    correct_yaw=False,
                )
                if uwb_applied:
                    updated = True
                    accepted_source = "UWB"
            if lidar_update is not None:
                lidar_source_name = lidar_source or lidar_update.source.upper()
                lidar_applied = self._apply_absolute_update(
                    lidar_update.pose,
                    lidar_source_name,
                    self.config.lidar_residual_gate,
                    event.now,
                    reasons,
                    compute_lidar_confidence(lidar_update, self.config),
                    correct_yaw=True,
                )
                if lidar_applied:
                    updated = True
                    accepted_source = lidar_source_name

        if accepted_source == "GNSS":
            mode = GNSS_AIDED
        elif accepted_source == "UWB":
            mode = UWB_AIDED
        elif accepted_source is not None:
            mode = LIDAR_AIDED
        elif self._map_pose is None:
            mode = WAITING_ALIGNMENT if event.gnss is not None and self._alignment is None else INITIALIZING
        elif any(reason.startswith("GNSS_") or reason.startswith("LIDAR_") or reason.startswith("UWB_") for reason in reasons):
            mode = DEAD_RECKONING
        else:
            mode = INITIALIZING if not self._global_anchored else NOMINAL
            if (
                self._global_anchored
                and self._last_absolute_update_time is not None
                and event.now - self._last_absolute_update_time > self.config.freshness_timeout
            ):
                mode = DEAD_RECKONING
        if not updated and any(
            reason.endswith("_OUTLIER") or reason.endswith("_OLD_MEASUREMENT")
            for reason in reasons
        ):
            mode = REJECTED_UPDATE
        if (
            event.gnss is not None
            and self._alignment is None
            and not self._global_anchored
            and accepted_source is None
            and event.gnss.accepted
            and event.gnss.state.upper() not in {REJECTED, STALE}
        ):
            mode = WAITING_ALIGNMENT
        output = self._output(event.now, mode, accepted_source, updated, reasons)
        self._last_output = output
        return output

    def _propagate_odom(self, odom: Pose2D | None, now: float, reasons: list[str]) -> bool:
        if odom is None:
            return False
        if not _finite_pose(odom) or odom.timestamp > now + 1e-6:
            reasons.append("LOCAL_ODOM_INVALID")
            return False
        if self._last_local_odom is not None and odom.timestamp == self._last_local_odom.timestamp:
            return False
        if self._last_local_odom is not None and odom.timestamp < self._last_local_odom.timestamp:
            reasons.append("LOCAL_ODOM_TIME_REVERSED")
            return False
        if self._map_pose is None:
            self._map_pose = replace(odom, timestamp=now)
            self._last_local_odom = odom
            return True
        delta = _relative(self._last_local_odom, odom)
        self._map_pose = _compose(self._map_pose, delta, now)
        self._position_uncertainty += self.config.odom_position_noise_per_m * math.hypot(delta.x, delta.y)
        self._yaw_uncertainty += self.config.odom_yaw_noise_per_rad * abs(delta.yaw)
        self._last_local_odom = odom
        return True

    def _gnss_usable(self, gnss: GnssPosition, now: float, reasons: list[str]) -> bool:
        if not gnss.accepted:
            reasons.append("GNSS_REJECTED")
            return False
        if not gnss.fresh or gnss.timestamp > now + 1e-6 or now - gnss.timestamp > self.config.freshness_timeout:
            reasons.append("GNSS_STALE")
            return False
        if gnss.state.upper() == REJECTED:
            reasons.append("GNSS_REJECTED")
            return False
        if gnss.state.upper() == STALE:
            reasons.append("GNSS_STALE")
            return False
        if self.config.gnss_reference is None or self._alignment is None:
            return False
        return all(_finite(value) for value in (gnss.latitude, gnss.longitude, gnss.altitude, gnss.timestamp))

    def _gnss_update(self, gnss: GnssPosition | None, now: float, reasons: list[str]) -> tuple[Pose2D, float] | None:
        if gnss is None or not self._gnss_usable(gnss, now, reasons):
            return None
        enu = geodetic_to_enu(
            GeodeticPoint(gnss.latitude, gnss.longitude, gnss.altitude),
            self.config.gnss_reference,  # type: ignore[arg-type]
        )
        x, y = self._alignment.apply(enu)  # type: ignore[union-attr]
        return (
            Pose2D(x, y, self._map_pose.yaw if self._map_pose is not None else 0.0, gnss.timestamp),
            compute_gnss_confidence(gnss, self.config),
        )

    def _uwb_usable(self, uwb: UwbPositionMeasurement, now: float, reasons: list[str]) -> bool:
        """Check if UWB measurement is usable."""
        if not uwb.accepted:
            reasons.append("UWB_REJECTED")
            return False
        if not uwb.fresh or uwb.timestamp > now + 1e-6 or now - uwb.timestamp > self.config.freshness_timeout:
            reasons.append("UWB_STALE")
            return False
        if uwb.state.upper() == REJECTED:
            reasons.append("UWB_REJECTED")
            return False
        if uwb.state.upper() == STALE:
            reasons.append("UWB_STALE")
            return False
        return all(_finite(value) for value in (uwb.x, uwb.y, uwb.timestamp))

    def _uwb_update(self, uwb: UwbPositionMeasurement | None, now: float, reasons: list[str]) -> tuple[Pose2D, float] | None:
        """Process UWB measurement and return (pose, confidence) or None."""
        if uwb is None or not self._uwb_usable(uwb, now, reasons):
            return None
        # UWB provides x/y in map frame; keep current yaw estimate
        return (
            Pose2D(uwb.x, uwb.y, self._map_pose.yaw if self._map_pose is not None else 0.0, uwb.timestamp),
            compute_uwb_confidence(uwb, self.config),
        )

    def _select_lidar_update(
        self, event: FusionInput, now: float, reasons: list[str]
    ) -> tuple[MapPoseMeasurement | None, str | None]:
        scan = event.scan_match_pose
        if scan is not None and self._measurement_usable(scan, now):
            return scan, "SCAN_MATCH"
        if scan is not None and scan.accepted and scan.state.upper() not in {REJECTED, STALE}:
            reasons.append("LIDAR_SCAN_MATCH_NOT_USABLE")
        amcl = event.amcl_pose
        if amcl is not None and self._measurement_usable(amcl, now):
            return amcl, "AMCL"
        if amcl is not None and amcl.state.upper() not in {REJECTED, STALE}:
            reasons.append("LIDAR_AMCL_NOT_USABLE")
        return None, None

    def _measurement_usable(self, measurement: MapPoseMeasurement, now: float) -> bool:
        return (
            measurement.accepted
            and measurement.fresh
            and measurement.state.upper() == GOOD
            and _finite_pose(measurement.pose)
            and measurement.timestamp <= now + 1e-6
            and now - measurement.timestamp <= self.config.freshness_timeout
        )

    def _apply_absolute_update(
        self,
        measurement: Pose2D,
        source: str,
        residual_gate: float,
        now: float,
        reasons: list[str],
        measurement_confidence: float = 1.0,
        correct_yaw: bool = True,
    ) -> bool:
        if not _finite_pose(measurement) or self._map_pose is None:
            reasons.append(f"{source}_INVALID")
            return False
        last_timestamp = self._last_absolute_timestamps.get(source)
        if last_timestamp is not None and measurement.timestamp == last_timestamp:
            return False
        if last_timestamp is not None and measurement.timestamp <= last_timestamp:
            reasons.append(f"{source}_OLD_MEASUREMENT")
            return False
        dx = measurement.x - self._map_pose.x
        dy = measurement.y - self._map_pose.y
        dyaw = wrap_angle(measurement.yaw - self._map_pose.yaw)
        if self._global_anchored and (math.hypot(dx, dy) > residual_gate or abs(dyaw) > self.config.yaw_residual_gate):
            reasons.append(f"{source}_OUTLIER")
            return False
        confidence = _clamp(measurement_confidence)
        effective_confidence = max(self.config.adaptive_min_confidence, confidence)
        self._last_measurement_confidence = confidence
        base_position_sigma = (
            self.config.gnss_base_position_sigma
            if source == "GNSS"
            else self.config.lidar_base_position_sigma
        )
        self._adaptive_position_uncertainty = base_position_sigma / math.sqrt(effective_confidence)
        if correct_yaw:
            self._adaptive_yaw_uncertainty = self.config.lidar_base_yaw_sigma / math.sqrt(effective_confidence)
        if not self._global_anchored:
            self._map_pose = Pose2D(
                measurement.x,
                measurement.y,
                measurement.yaw if correct_yaw else self._map_pose.yaw,
                now,
            )
            self._global_anchored = True
        else:
            distance = math.hypot(dx, dy)
            if distance > self.config.max_correction_step > 0.0:
                scale = self.config.max_correction_step / distance
                dx *= scale
                dy *= scale
            elif self.config.max_correction_step == 0.0:
                dx = dy = 0.0
            if not correct_yaw:
                dyaw = 0.0
            dx *= effective_confidence
            dy *= effective_confidence
            dyaw *= effective_confidence
            dyaw = max(-self.config.max_yaw_correction_step, min(self.config.max_yaw_correction_step, dyaw))
            self._map_pose = Pose2D(self._map_pose.x + dx, self._map_pose.y + dy, wrap_angle(self._map_pose.yaw + dyaw), now)
        self._position_uncertainty = max(
            self.config.min_position_uncertainty,
            self._position_uncertainty * (1.0 - 0.5 * effective_confidence),
        )
        if correct_yaw:
            self._yaw_uncertainty = max(
                self.config.min_yaw_uncertainty,
                self._yaw_uncertainty * (1.0 - 0.5 * effective_confidence),
            )
        self._last_absolute_timestamps[source] = measurement.timestamp
        self._last_absolute_update_time = now
        return True

    def _output(
        self,
        now: float,
        mode: str,
        source: str | None,
        updated: bool,
        reasons: Iterable[str],
    ) -> FusionOutput:
        map_to_odom = None
        if self._map_pose is not None and self._last_local_odom is not None:
            map_to_odom = _compose(self._map_pose, _inverse(self._last_local_odom, now), now)
        return FusionOutput(
            map_pose=self._map_pose,
            map_to_odom=map_to_odom,
            position_uncertainty=self._position_uncertainty,
            yaw_uncertainty=self._yaw_uncertainty,
            fusion_mode=mode,
            accepted_source=source,
            updated=updated,
            reasons=tuple(dict.fromkeys(reasons)),
            timestamp=now,
            global_anchored=self._global_anchored,
            measurement_confidence=self._last_measurement_confidence,
            adaptive_position_uncertainty=self._adaptive_position_uncertainty,
            adaptive_yaw_uncertainty=self._adaptive_yaw_uncertainty,
        )
