"""Small, explicit scenario schema used by the injector and evaluator.

The schema describes sensor/base inputs only.  It contains no knob for
publishing any ``/dg/*`` health, fusion, relocalization or command result, so a
scenario cannot bypass the software under test.

S05-S08 additionally describe the synthetic base plant and the ray-cast scan
generator.  Both model the *robot* (wheels, odometry, AMCL, LiDAR geometry),
never the algorithms under test.  The plant consumes the arbiter's test command
and answers on ``/odom`` and ``/amcl_pose``; it is the synthetic stand-in for
physics plus AMCL, which is exactly the role Gazebo would play later.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class GnssPhase:
    quality: int = 4
    satellites: int = 12
    hdop: float = 0.8
    differential_age: float = 0.5
    fix_available: bool = True
    publish_fix: bool = True
    publish_status: bool = True
    diagnostic_level: int = 0


@dataclass(frozen=True)
class LidarPhase:
    mode: str = "normal"
    valid_fraction: float = 1.0
    angle_min: float = -3.141592653589793
    angle_max: float = 3.141592653589793
    range_m: float = 3.0


@dataclass(frozen=True)
class AmclPhase:
    """Synthetic AMCL pose covariance schedule.

    ``covariance`` maps to ``pose.covariance[0]`` and ``[7]``, which is the
    field the real supervisor reads as ``amcl_covariance``.  Setting
    ``covariance_end`` ramps the value linearly across the phase so a scenario
    can express a *gradual* localization degradation instead of a step.
    """

    publish: bool = True
    covariance: float = 0.05
    covariance_end: float | None = None
    yaw_covariance: float = 0.03

    def covariance_at(self, ratio: float) -> float:
        if self.covariance_end is None:
            return self.covariance
        clamped = 0.0 if ratio < 0.0 else (1.0 if ratio > 1.0 else ratio)
        return self.covariance + (self.covariance_end - self.covariance) * clamped


@dataclass(frozen=True)
class OdomPhase:
    """Reported wheel odometry twist when the plant is disabled.

    The historical S01-S04 default is a constant 0.05 m/s so those recorded
    runs stay reproducible.  Plant-driven scenarios ignore ``linear_x`` and
    report the integrated command instead.
    """

    publish: bool = True
    linear_x: float = 0.05


@dataclass(frozen=True)
class MapConfig:
    """Deterministic occupancy grid; also the ray-cast source of truth."""

    resolution: float = 0.1
    width: int = 80
    height: int = 80
    origin_x: float = -4.0
    origin_y: float = -4.0
    border: bool = True
    interior_wall: bool = True
    interior_wall_x_min: int = 30
    interior_wall_x_max: int = 31
    interior_wall_y_min: int = 15
    interior_wall_y_max: int = 65


@dataclass(frozen=True)
class ScanConfig:
    """LaserScan generation mode.

    ``analytic`` reproduces the original S01-S04 waveform exactly.  ``raycast``
    casts the beams against the very occupancy grid published on ``/map`` so the
    real Scan-to-Map matcher has genuinely matchable geometry.
    """

    generator: str = "analytic"
    beam_count: int = 360
    range_min: float = 0.05
    range_max: float = 8.0
    frame_id: str = "laser"
    raycast_step: float = 0.05


@dataclass(frozen=True)
class PlantConfig:
    """SYNTHETIC_KINEMATIC_PLANT — the synthetic robot *body* only.

    Scope, recorded verbatim for the evidence trail:
      kinematics       planar differential drive, no wheel dynamics
      integration      explicit forward Euler, dt clamped to [0, 0.5] s
      update rate      the scenario ``sensor_hz`` tick (10 Hz by default)
      initial state    ``start_x`` / ``start_y`` / ``start_yaw``
      velocity input   subscribe-only from ``command_topic``
      pose update      yaw += w*dt, then x/y advance along the new yaw
      noise            none (deterministic by design)
      latency          none (``command_latency_sec`` reserved, default 0)

    It answers on ``/odom`` and optionally TF.  It NEVER publishes ``/dg/*``,
    never publishes the real ``/cmd_vel``, and never publishes ``/amcl_pose`` --
    localization observation is a separate concern, see AmclSurrogateConfig.
    This is SYNTHETIC SOFTWARE VALIDATION, not Gazebo physics and not a
    calibrated robot model.
    """

    enabled: bool = False
    command_topic: str = "/dg/test_cmd_vel"
    start_x: float = 0.0
    start_y: float = 0.0
    start_yaw: float = 0.0
    max_linear: float = 0.5
    max_angular: float = 1.0
    kinematics: str = "PLANAR_DIFFERENTIAL_DRIVE"
    integration: str = "EULER_FORWARD"
    command_latency_sec: float = 0.0
    noise_stddev: float = 0.0
    # The synthetic LiDAR is co-located with base_footprint so the ray-cast
    # origin and the published TF stay consistent.  ``publish_tf`` emits only
    # the two formally approved edges: a dynamic odom->base_footprint and a
    # static base_footprint->sensor.  map->odom is deliberately NOT published;
    # no algorithm under test reads it, and injecting a perfect one would risk
    # ground-truth leakage.  See docs/dg202611/MAP_ODOM_TF_JUSTIFICATION.md.
    publish_tf: bool = False
    odom_frame_id: str = "odom"
    base_frame_id: str = "base_footprint"
    sensor_frame_id: str = "laser"


@dataclass(frozen=True)
class AmclSurrogateConfig:
    """SYNTHETIC_AMCL_SURROGATE — a synthetic localization observer.

    This is NOT the real AMCL and must never be reported as such.  Recorded
    behaviour, verbatim for the evidence trail:
      input   synthetic plant ground-truth pose (DIRECT GROUND-TRUTH ACCESS: YES)
      input   real ``/scan_match_pose`` from the Scan-to-Map node under test
      output  ``/amcl_pose`` only -- never ``/dg/*``, never ``/cmd_vel``
      error   a deliberate constant pose bias, so the seed handed to the real
              matcher is genuinely wrong and the matcher must recover the true
              pose from scan geometry instead of confirming a known answer
      uncert. covariance follows the per-phase AmclPhase schedule/ramp
      recover on a real accepted match it converges: bias -> 0 after
              ``convergence_delay_sec`` and covariance -> ``covariance_after_convergence``
      noise   none unless ``noise_stddev`` > 0 (deterministic by default)
      faults  expressed through AmclPhase.publish and the covariance schedule

    Because the surrogate reads synthetic ground truth, S05-S08 evidence covers
    the software state chain, NOT localization accuracy.
    """

    enabled: bool = False
    follows_plant: bool = True
    accesses_plant_ground_truth: bool = True
    bias_x: float = 0.0
    bias_y: float = 0.0
    bias_yaw: float = 0.0
    converge_on_match: bool = False
    convergence_delay_sec: float = 0.0
    covariance_after_convergence: float | None = None
    noise_stddev: float = 0.0
    match_pose_topic: str = "/scan_match_pose"
    map_frame_id: str = "map"


@dataclass(frozen=True)
class Phase:
    phase_id: str
    start_sec: float
    end_sec: float
    gnss: GnssPhase
    lidar: LidarPhase
    amcl: AmclPhase = field(default_factory=AmclPhase)
    odom: OdomPhase = field(default_factory=OdomPhase)

    def contains(self, elapsed: float) -> bool:
        return self.start_sec <= elapsed < self.end_sec

    def ratio(self, elapsed: float) -> float:
        span = self.end_sec - self.start_sec
        if span <= 0.0:
            return 0.0
        return (elapsed - self.start_sec) / span


@dataclass(frozen=True)
class Scenario:
    scenario_id: str
    name: str
    duration_sec: float
    seed: int
    sensor_hz: float
    map_hz: float
    phases: tuple[Phase, ...]
    expected: dict[str, Any]
    map: MapConfig = field(default_factory=MapConfig)
    scan: ScanConfig = field(default_factory=ScanConfig)
    plant: PlantConfig = field(default_factory=PlantConfig)
    amcl_surrogate: AmclSurrogateConfig = field(default_factory=AmclSurrogateConfig)
    # S01-S04 bootstrap the Scan-to-Map node with a periodic coarse
    # ``/initialpose``.  Closed-loop scenarios switch this off so that every
    # match_quality message is attributable to a real supervisor seed request.
    publish_initialpose: bool = True

    def phase_at(self, elapsed: float) -> Phase:
        if not self.phases:
            raise ValueError(f"scenario {self.scenario_id} has no phases")
        for phase in self.phases:
            if phase.contains(elapsed):
                return phase
        # A final sample can land exactly on duration_sec.  It belongs to the
        # last phase for reporting, while publishers stop after the duration.
        return self.phases[-1]


def _mapping(value: Any, label: str) -> dict[str, Any]:
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a mapping")
    return value


def _optional_float(raw: dict[str, Any], key: str) -> float | None:
    value = raw.get(key)
    return None if value is None else float(value)


def _gnss(value: Any) -> GnssPhase:
    raw = _mapping(value, "gnss")
    return GnssPhase(
        quality=int(raw.get("quality", 4)),
        satellites=int(raw.get("satellites", 12)),
        hdop=float(raw.get("hdop", 0.8)),
        differential_age=float(raw.get("differential_age", 0.5)),
        fix_available=bool(raw.get("fix_available", True)),
        publish_fix=bool(raw.get("publish_fix", True)),
        publish_status=bool(raw.get("publish_status", True)),
        diagnostic_level=int(raw.get("diagnostic_level", 0)),
    )


def _lidar(value: Any) -> LidarPhase:
    raw = _mapping(value, "lidar")
    return LidarPhase(
        mode=str(raw.get("mode", "normal")),
        valid_fraction=float(raw.get("valid_fraction", 1.0)),
        angle_min=float(raw.get("angle_min", -3.141592653589793)),
        angle_max=float(raw.get("angle_max", 3.141592653589793)),
        range_m=float(raw.get("range_m", 3.0)),
    )


def _amcl(value: Any) -> AmclPhase:
    raw = _mapping(value, "amcl")
    phase = AmclPhase(
        publish=bool(raw.get("publish", True)),
        covariance=float(raw.get("covariance", 0.05)),
        covariance_end=_optional_float(raw, "covariance_end"),
        yaw_covariance=float(raw.get("yaw_covariance", 0.03)),
    )
    for name, number in (("covariance", phase.covariance), ("yaw_covariance", phase.yaw_covariance)):
        if number < 0.0:
            raise ValueError(f"amcl.{name} must be non-negative")
    if phase.covariance_end is not None and phase.covariance_end < 0.0:
        raise ValueError("amcl.covariance_end must be non-negative")
    return phase


def _odom(value: Any) -> OdomPhase:
    raw = _mapping(value, "odom")
    return OdomPhase(
        publish=bool(raw.get("publish", True)),
        linear_x=float(raw.get("linear_x", 0.05)),
    )


def _map(value: Any) -> MapConfig:
    raw = _mapping(value, "map")
    config = MapConfig(
        resolution=float(raw.get("resolution", 0.1)),
        width=int(raw.get("width", 80)),
        height=int(raw.get("height", 80)),
        origin_x=float(raw.get("origin_x", -4.0)),
        origin_y=float(raw.get("origin_y", -4.0)),
        border=bool(raw.get("border", True)),
        interior_wall=bool(raw.get("interior_wall", True)),
        interior_wall_x_min=int(raw.get("interior_wall_x_min", 30)),
        interior_wall_x_max=int(raw.get("interior_wall_x_max", 31)),
        interior_wall_y_min=int(raw.get("interior_wall_y_min", 15)),
        interior_wall_y_max=int(raw.get("interior_wall_y_max", 65)),
    )
    if config.resolution <= 0.0 or config.width < 3 or config.height < 3:
        raise ValueError("map resolution must be positive and the grid at least 3x3")
    return config


def _scan(value: Any) -> ScanConfig:
    raw = _mapping(value, "scan")
    config = ScanConfig(
        generator=str(raw.get("generator", "analytic")).strip().lower(),
        beam_count=int(raw.get("beam_count", 360)),
        range_min=float(raw.get("range_min", 0.05)),
        range_max=float(raw.get("range_max", 8.0)),
        frame_id=str(raw.get("frame_id", "laser")),
        raycast_step=float(raw.get("raycast_step", 0.05)),
    )
    if config.generator not in {"analytic", "raycast"}:
        raise ValueError("scan.generator must be 'analytic' or 'raycast'")
    if config.beam_count < 8:
        raise ValueError("scan.beam_count must be at least 8")
    if config.raycast_step <= 0.0 or config.range_max <= config.range_min:
        raise ValueError("scan ranges and raycast_step must be positive and ordered")
    return config


def _plant(value: Any) -> PlantConfig:
    raw = _mapping(value, "plant")
    config = PlantConfig(
        enabled=bool(raw.get("enabled", False)),
        command_topic=str(raw.get("command_topic", "/dg/test_cmd_vel")),
        start_x=float(raw.get("start_x", 0.0)),
        start_y=float(raw.get("start_y", 0.0)),
        start_yaw=float(raw.get("start_yaw", 0.0)),
        max_linear=float(raw.get("max_linear", 0.5)),
        max_angular=float(raw.get("max_angular", 1.0)),
        kinematics=str(raw.get("kinematics", "PLANAR_DIFFERENTIAL_DRIVE")),
        integration=str(raw.get("integration", "EULER_FORWARD")),
        command_latency_sec=float(raw.get("command_latency_sec", 0.0)),
        noise_stddev=float(raw.get("noise_stddev", 0.0)),
        publish_tf=bool(raw.get("publish_tf", False)),
        odom_frame_id=str(raw.get("odom_frame_id", "odom")),
        base_frame_id=str(raw.get("base_frame_id", "base_footprint")),
        sensor_frame_id=str(raw.get("sensor_frame_id", "laser")),
    )
    if config.command_topic == "/cmd_vel":
        raise ValueError("plant.command_topic must never be the real robot /cmd_vel")
    if config.command_topic.startswith("/dg/") and config.command_topic != "/dg/test_cmd_vel":
        raise ValueError("plant.command_topic may only consume /dg/test_cmd_vel")
    if config.max_linear < 0.0 or config.max_angular < 0.0:
        raise ValueError("plant limits must be non-negative")
    if config.command_latency_sec < 0.0 or config.noise_stddev < 0.0:
        raise ValueError("plant latency and noise must be non-negative")
    return config


def _amcl_surrogate(value: Any) -> AmclSurrogateConfig:
    raw = _mapping(value, "amcl_surrogate")
    config = AmclSurrogateConfig(
        enabled=bool(raw.get("enabled", False)),
        follows_plant=bool(raw.get("follows_plant", True)),
        accesses_plant_ground_truth=bool(raw.get("accesses_plant_ground_truth", True)),
        bias_x=float(raw.get("bias_x", 0.0)),
        bias_y=float(raw.get("bias_y", 0.0)),
        bias_yaw=float(raw.get("bias_yaw", 0.0)),
        converge_on_match=bool(raw.get("converge_on_match", False)),
        convergence_delay_sec=float(raw.get("convergence_delay_sec", 0.0)),
        covariance_after_convergence=_optional_float(raw, "covariance_after_convergence"),
        noise_stddev=float(raw.get("noise_stddev", 0.0)),
        match_pose_topic=str(raw.get("match_pose_topic", "/scan_match_pose")),
        map_frame_id=str(raw.get("map_frame_id", "map")),
    )
    if config.match_pose_topic.startswith("/dg/"):
        raise ValueError("amcl_surrogate.match_pose_topic must be a real node output, not /dg/*")
    if config.convergence_delay_sec < 0.0 or config.noise_stddev < 0.0:
        raise ValueError("amcl_surrogate delay and noise must be non-negative")
    if config.enabled and not config.follows_plant and (config.bias_x or config.bias_y or config.bias_yaw):
        raise ValueError("a pose bias is only meaningful when the surrogate follows the plant")
    return config


def load_scenario(path: str | Path) -> Scenario:
    """Load and validate one YAML scenario."""
    source = Path(path)
    with source.open("r", encoding="utf-8") as handle:
        raw = _mapping(yaml.safe_load(handle), "scenario")
    scenario_id = str(raw.get("scenario_id", "")).strip()
    if not scenario_id:
        raise ValueError(f"{source}: scenario_id is required")
    duration = float(raw.get("duration_sec", 0.0))
    if duration <= 0.0:
        raise ValueError(f"{source}: duration_sec must be positive")
    phases: list[Phase] = []
    previous_end = 0.0
    for index, item in enumerate(raw.get("phases", [])):
        phase_raw = _mapping(item, f"phases[{index}]")
        start = float(phase_raw.get("start_sec", previous_end))
        end = float(phase_raw.get("end_sec", start))
        phase_id = str(phase_raw.get("id", f"P{index + 1:02d}"))
        if start < previous_end or end <= start:
            raise ValueError(f"{source}: phase {phase_id} is not ordered")
        phases.append(
            Phase(
                phase_id,
                start,
                end,
                _gnss(phase_raw.get("gnss")),
                _lidar(phase_raw.get("lidar")),
                _amcl(phase_raw.get("amcl")),
                _odom(phase_raw.get("odom")),
            )
        )
        previous_end = end
    if not phases or phases[0].start_sec != 0.0 or phases[-1].end_sec < duration:
        raise ValueError(f"{source}: phases must cover [0, duration_sec)")
    rates = _mapping(raw.get("rates"), "rates")
    return Scenario(
        scenario_id=scenario_id,
        name=str(raw.get("name", scenario_id)),
        duration_sec=duration,
        seed=int(raw.get("seed", 202611)),
        sensor_hz=float(rates.get("sensor_hz", 10.0)),
        map_hz=float(rates.get("map_hz", 1.0)),
        phases=tuple(phases),
        expected=_mapping(raw.get("expected"), "expected"),
        map=_map(raw.get("map")),
        scan=_scan(raw.get("scan")),
        plant=_plant(raw.get("plant")),
        amcl_surrogate=_amcl_surrogate(raw.get("amcl_surrogate")),
        publish_initialpose=bool(raw.get("publish_initialpose", True)),
    )
