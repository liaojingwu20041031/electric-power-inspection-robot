"""ROS2 node that publishes only deterministic sensor/base input messages.

No ``/dg/*`` output is published here.  The nodes under test are the only
writers of health, fusion, relocalization, and command-arbitration topics.

For S05-S08 this node also acts as the synthetic *base plant*: it consumes the
arbiter's test command on ``/dg/test_cmd_vel`` and answers on ``/odom`` and
``/amcl_pose`` (optionally TF).  That models the robot body and AMCL, which is
the role a simulator would play; it is not an algorithm result.  The real
``/cmd_vel`` is never published or consumed here.
"""

from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Any

import numpy as np

from .scenario_schema import (
    AmclPhase,
    AmclSurrogateConfig,
    LidarPhase,
    MapConfig,
    Phase,
    PlantConfig,
    Scenario,
    ScanConfig,
    load_scenario,
)


PLANT_ROLE = "SYNTHETIC_KINEMATIC_PLANT_NOT_AN_ALGORITHM_OUTPUT"
SURROGATE_ROLE = "SYNTHETIC_AMCL_SURROGATE_NOT_REAL_AMCL"


def _normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _stamp(node: Any) -> Any:
    return node.get_clock().now().to_msg()


def _header(node: Any, frame_id: str) -> Any:
    from std_msgs.msg import Header

    header = Header()
    header.stamp = _stamp(node)
    header.frame_id = frame_id
    return header


def occupancy_values(config: MapConfig) -> list[int]:
    """Build the deterministic occupancy payload used by /map and ray casting."""
    data = [0] * (config.width * config.height)
    for y in range(config.height):
        for x in range(config.width):
            on_border = config.border and (
                x in {0, config.width - 1} or y in {0, config.height - 1}
            )
            in_wall = config.interior_wall and (
                config.interior_wall_x_min <= x <= config.interior_wall_x_max
                and config.interior_wall_y_min <= y <= config.interior_wall_y_max
            )
            if on_border or in_wall:
                data[y * config.width + x] = 100
    return data


def occupancy_matrix(config: MapConfig, occupied_threshold: int = 50) -> np.ndarray:
    """Return the boolean occupied mask in (row=y, col=x) order."""
    values = np.asarray(occupancy_values(config), dtype=np.int16)
    return (values.reshape((config.height, config.width)) >= occupied_threshold)


def _map_message(node: Any, config: MapConfig) -> Any:
    from nav_msgs.msg import OccupancyGrid

    message = OccupancyGrid()
    message.header = _header(node, "map")
    message.info.resolution = config.resolution
    message.info.width = config.width
    message.info.height = config.height
    message.info.origin.position.x = config.origin_x
    message.info.origin.position.y = config.origin_y
    message.info.origin.orientation.w = 1.0
    # Four deterministic boundary walls plus one interior strip give the
    # scan-to-map node a valid map message without claiming a realistic map.
    message.data = occupancy_values(config)
    return message


def beam_angles(scan: ScanConfig) -> tuple[float, float, list[float]]:
    """Return (angle_min, angle_increment, angles) for the configured scan."""
    angle_min = -math.pi
    angle_increment = 2.0 * math.pi / scan.beam_count
    angles = [angle_min + index * angle_increment for index in range(scan.beam_count)]
    return angle_min, angle_increment, angles


def _beam_kept(phase: LidarPhase, index: int, angle: float) -> bool:
    in_window = phase.angle_min <= angle <= phase.angle_max
    stride = max(1, int(round(1.0 / max(phase.valid_fraction, 1e-6))))
    keep = in_window and (phase.valid_fraction >= 1.0 or index % stride == 0)
    if phase.mode.lower() in {"empty", "none", "all_invalid"}:
        return False
    return keep


def analytic_ranges(phase: LidarPhase, angles: list[float]) -> list[float]:
    """Original S01-S04 waveform; kept bit-for-bit so those runs reproduce."""
    ranges: list[float] = []
    for index, angle in enumerate(angles):
        if _beam_kept(phase, index, angle):
            # A fixed, gently varying range provides repeatable geometry and
            # enough temporal matches while avoiding random noise.
            ranges.append(
                float(phase.range_m + 0.12 * math.sin(3.0 * angle) + 0.04 * math.cos(7.0 * angle))
            )
        else:
            ranges.append(float("inf"))
    return ranges


def raycast_ranges(
    pose: tuple[float, float, float],
    angles: list[float],
    occupied: np.ndarray,
    map_config: MapConfig,
    scan: ScanConfig,
    phase: LidarPhase,
) -> list[float]:
    """Cast every beam against the same grid that is published on /map.

    This is a synthetic LiDAR sensor model.  Because the geometry comes from the
    published map, the real Scan-to-Map matcher sees genuinely matchable data
    instead of an arbitrary waveform.
    """
    x, y, yaw = pose
    steps = np.arange(scan.range_min, scan.range_max, scan.raycast_step, dtype=float)
    if steps.size == 0:
        return [float("inf")] * len(angles)
    world = np.asarray(angles, dtype=float) + yaw
    xs = x + np.cos(world)[:, None] * steps[None, :]
    ys = y + np.sin(world)[:, None] * steps[None, :]
    gx = np.floor((xs - map_config.origin_x) / map_config.resolution).astype(np.int64)
    gy = np.floor((ys - map_config.origin_y) / map_config.resolution).astype(np.int64)
    inside = (
        (gx >= 0) & (gx < map_config.width) & (gy >= 0) & (gy < map_config.height)
    )
    hit_mask = np.zeros(gx.shape, dtype=bool)
    hit_mask[inside] = occupied[gy[inside], gx[inside]]
    any_hit = hit_mask.any(axis=1)
    first = hit_mask.argmax(axis=1)
    distances = np.where(any_hit, steps[first], np.inf)
    ranges: list[float] = []
    for index, angle in enumerate(angles):
        if not _beam_kept(phase, index, angle):
            ranges.append(float("inf"))
            continue
        ranges.append(float(distances[index]))
    return ranges


class PlantState:
    """Integrated synthetic base pose driven by the arbiter test command."""

    def __init__(self, config: PlantConfig) -> None:
        self.config = config
        self.x = config.start_x
        self.y = config.start_y
        self.yaw = config.start_yaw
        self.linear = 0.0
        self.angular = 0.0
        self.commanded_linear = 0.0
        self.commanded_angular = 0.0
        self.command_count = 0
        self.match_corrections = 0
        self._last_time: float | None = None

    def on_command(self, linear: float, angular: float) -> None:
        limit_linear = self.config.max_linear
        limit_angular = self.config.max_angular
        self.commanded_linear = max(-limit_linear, min(limit_linear, float(linear)))
        self.commanded_angular = max(-limit_angular, min(limit_angular, float(angular)))
        self.command_count += 1

    def on_match_correction(self) -> None:
        self.match_corrections += 1

    def integrate(self, now: float) -> None:
        if self._last_time is None:
            self._last_time = now
            return
        dt = max(0.0, min(0.5, now - self._last_time))
        self._last_time = now
        self.linear = self.commanded_linear
        self.angular = self.commanded_angular
        self.yaw = math.atan2(
            math.sin(self.yaw + self.angular * dt), math.cos(self.yaw + self.angular * dt)
        )
        self.x += self.linear * math.cos(self.yaw) * dt
        self.y += self.linear * math.sin(self.yaw) * dt

    @property
    def pose(self) -> tuple[float, float, float]:
        return self.x, self.y, self.yaw


def _make_scan(node: Any, phase: LidarPhase, scenario: Scenario, pose: tuple[float, float, float], occupied: np.ndarray) -> Any:
    from sensor_msgs.msg import LaserScan

    scan = scenario.scan
    angle_min, angle_increment, angles = beam_angles(scan)
    message = LaserScan()
    message.header = _header(node, scan.frame_id)
    message.angle_min = angle_min
    message.angle_max = math.pi - angle_increment
    message.angle_increment = angle_increment
    message.time_increment = 0.0
    message.scan_time = 0.1
    message.range_min = scan.range_min
    message.range_max = scan.range_max
    if scan.generator == "raycast":
        message.ranges = raycast_ranges(pose, angles, occupied, scenario.map, scan, phase)
    else:
        message.ranges = analytic_ranges(phase, angles)
    message.intensities = [0.0] * scan.beam_count
    return message


def _make_fix(node: Any, available: bool) -> Any:
    from sensor_msgs.msg import NavSatFix, NavSatStatus

    message = NavSatFix()
    message.header = _header(node, "gps")
    message.status.status = NavSatStatus.STATUS_FIX if available else NavSatStatus.STATUS_NO_FIX
    message.status.service = NavSatStatus.SERVICE_GPS
    message.latitude = 31.2304
    message.longitude = 121.4737
    message.altitude = 10.0
    message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    message.position_covariance[0] = 0.25
    message.position_covariance[4] = 0.25
    message.position_covariance[8] = 0.5
    return message


def _make_status(node: Any, phase: Any) -> Any:
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

    array = DiagnosticArray()
    array.header = _header(node, "gps")
    status = DiagnosticStatus()
    status.name = "Synthetic WTRTK980 RTK Status"
    # diagnostic_msgs/msg/DiagnosticStatus.level is a uint8 represented as a
    # one-byte ``bytes`` value by the ROS2 Humble Python generator.
    status.level = bytes([max(0, min(255, int(phase.diagnostic_level)))])
    status.message = "synthetic sensor input"
    status.values = [
        KeyValue(key="quality", value=str(phase.quality)),
        KeyValue(key="quality_text", value="RTK_FIXED" if phase.quality in (4, 5) else "RTK_FLOAT"),
        KeyValue(key="satellites", value=str(phase.satellites)),
        KeyValue(key="hdop", value=str(phase.hdop)),
        KeyValue(key="differential_age", value=str(phase.differential_age)),
        KeyValue(key="base_station_id", value="SYNTHETIC"),
    ]
    array.status = [status]
    return array


def _set_yaw(orientation: Any, yaw: float) -> None:
    orientation.x = 0.0
    orientation.y = 0.0
    orientation.z = math.sin(yaw * 0.5)
    orientation.w = math.cos(yaw * 0.5)


def _make_odom(node: Any, phase: Phase, plant: PlantState | None, config: PlantConfig) -> Any:
    from nav_msgs.msg import Odometry

    message = Odometry()
    message.header = _header(node, config.odom_frame_id)
    message.child_frame_id = config.base_frame_id
    if plant is None:
        # Historical S01-S04 behaviour: a static pose and a constant twist.
        message.pose.pose.orientation.w = 1.0
        message.twist.twist.linear.x = phase.odom.linear_x
    else:
        message.pose.pose.position.x = plant.x
        message.pose.pose.position.y = plant.y
        _set_yaw(message.pose.pose.orientation, plant.yaw)
        message.twist.twist.linear.x = plant.linear
        message.twist.twist.angular.z = plant.angular
    message.pose.covariance[0] = 0.02
    message.pose.covariance[7] = 0.02
    message.pose.covariance[35] = 0.01
    message.twist.covariance[0] = 0.02
    return message


class AmclSurrogate:
    """SYNTHETIC_AMCL_SURROGATE -- a synthetic localization observer.

    NOT the real AMCL.  It reads the synthetic plant's ground-truth pose
    (direct ground-truth access: YES), adds a deliberate constant bias so the
    seed handed to the real Scan-to-Map matcher is genuinely wrong, and reports
    the per-phase covariance schedule.  Convergence is triggered by a *real*
    accepted correction on the matcher's ``/scan_match_pose``, never by a timer
    alone; ``convergence_delay_sec`` only delays the response to that real event.
    """

    def __init__(self, config: AmclSurrogateConfig) -> None:
        self.config = config
        self.match_accepted_count = 0
        self.first_match_time: float | None = None
        self.converged = False

    def on_match_accepted(self, now: float) -> None:
        self.match_accepted_count += 1
        if self.first_match_time is None:
            self.first_match_time = now

    def update(self, now: float) -> None:
        if not self.config.converge_on_match or self.first_match_time is None:
            return
        if now - self.first_match_time >= self.config.convergence_delay_sec:
            self.converged = True

    @property
    def mode(self) -> str:
        if not self.config.enabled:
            return "DISABLED"
        return "CONVERGED" if self.converged else "BIASED"

    def observed_pose(self, truth: tuple[float, float, float] | None) -> tuple[float, float, float] | None:
        if truth is None or not self.config.follows_plant:
            return None
        x, y, yaw = truth
        if self.converged:
            return x, y, yaw
        return (
            x + self.config.bias_x,
            y + self.config.bias_y,
            _normalize_angle(yaw + self.config.bias_yaw),
        )

    def covariance(self, phase: Phase, elapsed: float) -> float:
        if self.converged and self.config.covariance_after_convergence is not None:
            return float(self.config.covariance_after_convergence)
        return float(phase.amcl.covariance_at(phase.ratio(elapsed)))


def _make_amcl(
    node: Any,
    amcl: AmclPhase,
    covariance: float,
    pose: tuple[float, float, float] | None,
    map_frame_id: str,
) -> Any:
    from geometry_msgs.msg import PoseWithCovarianceStamped

    message = PoseWithCovarianceStamped()
    message.header = _header(node, map_frame_id)
    if pose is None:
        message.pose.pose.orientation.w = 1.0
    else:
        message.pose.pose.position.x = pose[0]
        message.pose.pose.position.y = pose[1]
        _set_yaw(message.pose.pose.orientation, pose[2])
    message.pose.covariance[0] = covariance
    message.pose.covariance[7] = covariance
    message.pose.covariance[35] = amcl.yaw_covariance
    return message


def create_node(scenario: Scenario, node_name: str = "dg_synthetic_injector") -> Any:
    """Create an injector node; imports ROS types lazily for unit-testability."""
    from rclpy.node import Node

    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import LaserScan, NavSatFix

    class SyntheticInjectorNode(Node):
        def __init__(self) -> None:
            super().__init__(node_name)
            self._scenario = scenario
            self._start = time.monotonic()
            self._last_map = 0.0
            self._occupied = occupancy_matrix(scenario.map)
            self._plant = PlantState(scenario.plant) if scenario.plant.enabled else None
            self._surrogate = (
                AmclSurrogate(scenario.amcl_surrogate) if scenario.amcl_surrogate.enabled else None
            )
            self._tf_broadcaster = None
            self._static_tf_broadcaster = None
            self._fix_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
            self._status_pub = self.create_publisher(DiagnosticArray, "/gps/rtk_status", 10)
            self._scan_pub = self.create_publisher(LaserScan, "/scan", 10)
            self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
            self._amcl_pub = self.create_publisher(PoseWithCovarianceStamped, "/amcl_pose", 10)
            self._map_pub = self.create_publisher(OccupancyGrid, "/map", 1)
            self._initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
            if self._surrogate is not None and scenario.amcl_surrogate.converge_on_match:
                # /scan_match_pose is published only when the REAL Scan-to-Map
                # node accepts a match, so surrogate convergence is caused by a
                # real algorithm output rather than by a scripted timer.
                self.create_subscription(
                    PoseStamped, scenario.amcl_surrogate.match_pose_topic, self._on_match_pose, 10
                )
            if self._plant is not None:
                # Subscribe-only feedback from the arbiter's *test* output.  The
                # plant is the synthetic robot body, never an algorithm result.
                self.create_subscription(Twist, scenario.plant.command_topic, self._on_command, 10)
                if scenario.plant.publish_tf:
                    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

                    self._tf_broadcaster = TransformBroadcaster(self)
                    self._static_tf_broadcaster = StaticTransformBroadcaster(self)
                    self._static_tf_broadcaster.sendTransform(self._static_transforms())
            period = 1.0 / max(1.0, scenario.sensor_hz)
            self._timer = self.create_timer(period, self._tick)
            self.get_logger().info(
                f"synthetic injector active: {scenario.scenario_id}, duration={scenario.duration_sec:.1f}s; "
                f"sensor/base topics only, scan={scenario.scan.generator}, "
                f"plant={'on' if self._plant else 'off'} ({PLANT_ROLE})"
            )

        @property
        def plant(self) -> PlantState | None:
            return self._plant

        def _static_transforms(self) -> list[Any]:
            """base_footprint->sensor only.

            map->odom is deliberately absent.  No algorithm under test reads it
            (the only TF consumer in the chain is the Scan-to-Map node, which
            looks up base_footprint <- laser), and publishing a perfect one
            would risk feeding ground truth to the software under test.  See
            docs/dg202611/MAP_ODOM_TF_JUSTIFICATION.md.
            """
            from geometry_msgs.msg import TransformStamped

            config = self._scenario.plant
            transform = TransformStamped()
            transform.header.stamp = _stamp(self)
            transform.header.frame_id = config.base_frame_id
            transform.child_frame_id = config.sensor_frame_id
            transform.transform.rotation.w = 1.0
            return [transform]

        def _on_command(self, message: Any) -> None:
            if self._plant is not None:
                self._plant.on_command(message.linear.x, message.angular.z)

        def _on_match_pose(self, _message: Any) -> None:
            # /scan_match_pose is only published when the real Scan-to-Map node
            # ACCEPTS a match, so this models the localization surrogate
            # converging on a real algorithm result rather than on a timer.
            if self._surrogate is not None:
                self._surrogate.on_match_accepted(time.monotonic())

        def _publish_tf(self, plant: PlantState) -> None:
            from geometry_msgs.msg import TransformStamped

            config = self._scenario.plant
            transform = TransformStamped()
            transform.header.stamp = _stamp(self)
            transform.header.frame_id = config.odom_frame_id
            transform.child_frame_id = config.base_frame_id
            transform.transform.translation.x = plant.x
            transform.transform.translation.y = plant.y
            _set_yaw(transform.transform.rotation, plant.yaw)
            self._tf_broadcaster.sendTransform(transform)

        def _tick(self) -> None:
            elapsed = time.monotonic() - self._start
            if elapsed > self._scenario.duration_sec:
                return
            phase = self._scenario.phase_at(elapsed)
            plant = self._plant
            if plant is not None:
                plant.integrate(time.monotonic())
            pose = (0.0, 0.0, 0.0) if plant is None else plant.pose
            if phase.gnss.publish_status:
                self._status_pub.publish(_make_status(self, phase.gnss))
            if phase.gnss.publish_fix:
                self._fix_pub.publish(_make_fix(self, phase.gnss.fix_available))
            self._scan_pub.publish(
                _make_scan(self, phase.lidar, self._scenario, pose, self._occupied)
            )
            if phase.odom.publish:
                self._odom_pub.publish(_make_odom(self, phase, plant, self._scenario.plant))
            surrogate = self._surrogate
            if surrogate is not None:
                surrogate.update(time.monotonic())
            if phase.amcl.publish:
                if surrogate is None:
                    # Historical S01-S04 behaviour: static pose, fixed covariance.
                    covariance = phase.amcl.covariance_at(phase.ratio(elapsed))
                    observed = None
                    map_frame = self._scenario.amcl_surrogate.map_frame_id
                else:
                    covariance = surrogate.covariance(phase, elapsed)
                    observed = surrogate.observed_pose(pose)
                    map_frame = surrogate.config.map_frame_id
                self._amcl_pub.publish(
                    _make_amcl(self, phase.amcl, covariance, observed, map_frame)
                )
            if plant is not None and self._tf_broadcaster is not None:
                self._publish_tf(plant)
            if elapsed - self._last_map >= 1.0 / max(0.2, self._scenario.map_hz):
                self._map_pub.publish(_map_message(self, self._scenario.map))
                if self._scenario.publish_initialpose:
                    self._initialpose_pub.publish(
                        _make_amcl(
                            self,
                            phase.amcl,
                            phase.amcl.covariance,
                            None,
                            self._scenario.amcl_surrogate.map_frame_id,
                        )
                    )
                self._last_map = elapsed

    return SyntheticInjectorNode()


def main(args: list[str] | None = None) -> None:
    import argparse
    import rclpy

    parser = argparse.ArgumentParser(description="Publish deterministic S01-S08 sensor/base inputs")
    parser.add_argument("scenario_file", type=Path)
    parser.add_argument("--node-name", default="dg_synthetic_injector")
    parsed = parser.parse_args(args)
    scenario = load_scenario(parsed.scenario_file)
    rclpy.init(args=None)
    node = create_node(scenario, parsed.node_name)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
