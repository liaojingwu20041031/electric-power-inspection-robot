"""ROS2 evaluator/recorder for outputs produced by the real navigation nodes.

Every column recorded here originates from a topic published by the software
under test.  The evaluator publishes nothing and never fills a missing
observation with a guess.
"""

from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Any

from .result_writer import SampleRecorder
from .scenario_schema import Scenario


def _values(status: Any) -> dict[str, str]:
    return {str(item.key): str(item.value) for item in status.values}


def _number(values: dict[str, str], key: str) -> float | None:
    try:
        return float(values[key])
    except (KeyError, TypeError, ValueError):
        return None


def _integer(values: dict[str, str], key: str) -> int | None:
    try:
        return int(float(values[key]))
    except (KeyError, TypeError, ValueError):
        return None


def _boolean(values: dict[str, str], key: str) -> bool | None:
    value = values.get(key)
    if value is None:
        return None
    return value.strip().lower() in {"1", "true", "yes", "accepted", "ok"}


def _text(values: dict[str, str], key: str) -> str | None:
    value = values.get(key)
    if value is None:
        return None
    stripped = value.strip()
    return stripped or None


def _level(status: Any) -> int:
    raw = status.level
    if isinstance(raw, (bytes, bytearray)):
        return int(raw[0]) if raw else 0
    return int(raw)


def quaternion_yaw(orientation: Any) -> float | None:
    try:
        x = float(orientation.x)
        y = float(orientation.y)
        z = float(orientation.z)
        w = float(orientation.w)
        value = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    except (AttributeError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


class InputState:
    """Recorded synthetic INPUT side: the plant body and the AMCL surrogate.

    Kept deliberately separate from OutputState so the evidence trail never
    blurs "what we fed in" with "what the software under test decided".  These
    are recorded because the round-B protocol asks for them; they are never
    used to assert that an algorithm succeeded.
    """

    def __init__(self) -> None:
        self.odom_x: float | None = None
        self.odom_y: float | None = None
        self.odom_yaw: float | None = None
        self.odom_linear_x: float | None = None
        self.odom_angular_z: float | None = None
        self.amcl_x: float | None = None
        self.amcl_y: float | None = None
        self.amcl_yaw: float | None = None
        self.amcl_covariance: float | None = None
        self.tf_base_to_sensor_available: bool | None = None


class OutputState:
    """Latest values observed from each actual software output topic."""

    def __init__(self) -> None:
        self.gnss: dict[str, Any] = {}
        self.lidar: dict[str, Any] = {}
        self.fusion: dict[str, Any] = {}
        self.navigation: dict[str, Any] = {}
        self.relocalization: dict[str, Any] = {}
        self.match: dict[str, Any] = {}
        self.match_sequence = 0
        self.seed_sequence = 0
        self.cmd_linear_x: float | None = None
        self.cmd_angular_z: float | None = None
        self.recovery_linear_x: float | None = None
        self.recovery_angular_z: float | None = None
        self.nav_linear_x: float | None = None
        self.nav_angular_z: float | None = None


def _close(left: float | None, right: float | None, tolerance: float = 1e-6) -> bool:
    if left is None or right is None:
        return False
    return abs(left - right) <= tolerance


def infer_cmd_vel_source(state: OutputState) -> str:
    """Infer which arbiter input the observed test command corresponds to.

    ``geometry_msgs/Twist`` carries no source metadata, so this is an explicit
    numeric inference over the recorded recovery/navigation commands, never a
    claim read from the message itself.  Ambiguous cases stay ``UNKNOWN``.
    """
    if state.cmd_linear_x is None or state.cmd_angular_z is None:
        return ""
    zero_output = _close(state.cmd_linear_x, 0.0) and _close(state.cmd_angular_z, 0.0)
    matches_recovery = _close(state.cmd_linear_x, state.recovery_linear_x) and _close(
        state.cmd_angular_z, state.recovery_angular_z
    )
    matches_nav = _close(state.cmd_linear_x, state.nav_linear_x) and _close(
        state.cmd_angular_z, state.nav_angular_z
    )
    if matches_recovery and not zero_output:
        return "RECOVERY"
    if matches_nav and not zero_output:
        return "NAV"
    if zero_output:
        return "ZERO"
    return "UNKNOWN"


def _parse_diagnostic(message: Any, state: OutputState) -> None:
    for status in message.status:
        name = str(status.name).lower()
        values = _values(status)
        if "scan-to-map" in name or "match quality" in name:
            state.match = {
                "accepted": _boolean(values, "accepted"),
                "score": _number(values, "score"),
                "inlier_ratio": _number(values, "inlier_ratio"),
                "mean_distance": _number(values, "mean_distance"),
                "used_points": _integer(values, "used_points"),
                "candidate_x": _number(values, "candidate_x"),
                "candidate_y": _number(values, "candidate_y"),
                "candidate_yaw": _number(values, "candidate_yaw"),
                "reason": _text(values, "reason") or str(status.message),
            }
            state.match_sequence += 1
        elif "gnss quality" in name or "gnss" in name and "quality" in name:
            state.gnss = {
                "state": values.get("current_state", str(status.message).split("/", 1)[0]),
                "decision": values.get("decision", str(status.message).split("/", 1)[-1]),
                "satellites": _integer(values, "satellites"),
                "hdop": _number(values, "hdop"),
                "differential_age": _number(values, "differential_age"),
                "accepted": _boolean(values, "accepted"),
            }
        elif "lidar/quality" in name or ("lidar" in name and "quality" in name):
            geometry = _number(values, "geometry_score")
            level = _level(status)
            lidar_state = "REJECTED" if level >= 2 or (geometry is not None and geometry <= 0.0) else ("DEGRADED" if geometry is not None and geometry < 0.2 else "GOOD")
            state.lidar = {
                "state": lidar_state,
                "raw_point_count": _integer(values, "raw_point_count"),
                "valid_point_count": _integer(values, "valid_point_count"),
                "stable_point_count": _integer(values, "stable_point_count"),
                "dynamic_candidate_count": _integer(values, "dynamic_candidate_count"),
                "temporal_match_ratio": _number(values, "temporal_match_ratio"),
                "angular_coverage": _number(values, "angular_coverage"),
                "geometry_score": geometry,
            }
        elif "multi-source fusion" in name or "fusion" in name:
            state.fusion = {
                "mode": values.get("fusion_mode", str(status.message)),
                "accepted_source": values.get("accepted_source"),
                "measurement_confidence": _number(values, "measurement_confidence"),
                "position_uncertainty": _number(values, "position_uncertainty"),
                "yaw_uncertainty": _number(values, "yaw_uncertainty"),
                "adaptive_position_uncertainty": _number(values, "adaptive_position_uncertainty"),
                "adaptive_yaw_uncertainty": _number(values, "adaptive_yaw_uncertainty"),
            }
        elif "navigation health" in name or "navigation" in name:
            state.navigation = {"state": values.get("overall_state", str(status.message))}
        elif "active relocalization" in name or "relocalization supervisor" in name:
            state.relocalization = {
                "state": values.get("state", str(status.message)),
                "attempt_id": _integer(values, "attempt_id"),
                "trigger_reason": _text(values, "trigger_reason"),
                "active_segment": _integer(values, "active_segment"),
                "verification_count": _integer(values, "verification_count"),
                "candidate_score": _number(values, "candidate_score"),
                "candidate_inlier_ratio": _number(values, "candidate_inlier_ratio"),
                "candidate_mean_distance": _number(values, "candidate_mean_distance"),
                "lidar_health": _text(values, "lidar_health"),
                "gnss_health": _text(values, "gnss_health"),
                "amcl_health": _text(values, "amcl_health"),
                "failure_reason": _text(values, "failure_reason"),
                "request_candidate": _boolean(values, "request_candidate"),
                "seed_source": _text(values, "seed_source"),
                "health_reasons": _text(values, "health_reasons"),
                "command_angular_z": _number(values, "command_angular_z"),
            }


def create_node(scenario: Scenario, output_dir: Path, node_name: str = "dg_synthetic_evaluator") -> Any:
    """Create the evaluator node; it subscribes to real outputs only."""
    from rclpy.node import Node
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import Odometry

    class SyntheticEvaluatorNode(Node):
        def __init__(self) -> None:
            super().__init__(node_name)
            self._scenario = scenario
            self._start = time.monotonic()
            self._state = OutputState()
            self._inputs = InputState()
            self._recorder = SampleRecorder(scenario, output_dir)
            self._finished = False
            self._tf_buffer = None
            self._tf_listener = None
            if scenario.plant.publish_tf:
                try:
                    from tf2_ros import Buffer, TransformListener

                    self._tf_buffer = Buffer()
                    self._tf_listener = TransformListener(self._tf_buffer, self)
                except ImportError:
                    self._tf_buffer = None
            for topic in (
                "/dg/gnss/quality",
                "/dg/lidar/quality",
                "/dg/fusion/status",
                "/dg/navigation/status",
                "/dg/relocalization/status",
                "/dg/relocalization/match_quality",
            ):
                self.create_subscription(
                    DiagnosticArray, topic, lambda msg: _parse_diagnostic(msg, self._state), 10
                )
            self.create_subscription(Twist, "/dg/test_cmd_vel", self._on_cmd, 10)
            self.create_subscription(Twist, "/cmd_vel_recovery", self._on_recovery, 10)
            self.create_subscription(Twist, "/cmd_vel_nav", self._on_nav, 10)
            # INPUT side: record the stimulus the plant and surrogate actually
            # fed in.  Subscribe-only; these values are evidence of what was
            # injected and are never used to assert that an algorithm succeeded.
            self.create_subscription(Odometry, "/odom", self._on_odom, 10)
            self.create_subscription(
                PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose, 10
            )
            self.create_subscription(
                PoseWithCovarianceStamped, "/dg/relocalization/seed", self._on_seed, 10
            )
            self._timer = self.create_timer(1.0 / 10.0, self._sample)

        @property
        def recorder(self) -> SampleRecorder:
            return self._recorder

        @property
        def elapsed(self) -> float:
            return time.monotonic() - self._start

        def _on_cmd(self, message: Any) -> None:
            self._state.cmd_linear_x = float(message.linear.x)
            self._state.cmd_angular_z = float(message.angular.z)

        def _on_recovery(self, message: Any) -> None:
            self._state.recovery_linear_x = float(message.linear.x)
            self._state.recovery_angular_z = float(message.angular.z)

        def _on_nav(self, message: Any) -> None:
            self._state.nav_linear_x = float(message.linear.x)
            self._state.nav_angular_z = float(message.angular.z)

        def _on_seed(self, _message: Any) -> None:
            self._state.seed_sequence += 1

        def _on_odom(self, message: Any) -> None:
            inputs = self._inputs
            try:
                inputs.odom_x = float(message.pose.pose.position.x)
                inputs.odom_y = float(message.pose.pose.position.y)
                inputs.odom_linear_x = float(message.twist.twist.linear.x)
                inputs.odom_angular_z = float(message.twist.twist.angular.z)
            except (AttributeError, TypeError, ValueError):
                return
            inputs.odom_yaw = quaternion_yaw(message.pose.pose.orientation)

        def _on_amcl_pose(self, message: Any) -> None:
            inputs = self._inputs
            try:
                inputs.amcl_x = float(message.pose.pose.position.x)
                inputs.amcl_y = float(message.pose.pose.position.y)
                inputs.amcl_covariance = float(message.pose.covariance[0])
            except (AttributeError, IndexError, TypeError, ValueError):
                return
            inputs.amcl_yaw = quaternion_yaw(message.pose.pose.orientation)

        def _refresh_tf_availability(self) -> None:
            """Record whether the one TF the real matcher needs is resolvable."""
            if self._tf_buffer is None:
                self._inputs.tf_base_to_sensor_available = None
                return
            plant = self._scenario.plant
            try:
                from rclpy.time import Time

                self._inputs.tf_base_to_sensor_available = bool(
                    self._tf_buffer.can_transform(
                        plant.base_frame_id, plant.sensor_frame_id, Time()
                    )
                )
            except Exception:
                self._inputs.tf_base_to_sensor_available = None

        def _sample(self) -> None:
            elapsed = self.elapsed
            if elapsed > self._scenario.duration_sec:
                return
            phase = self._scenario.phase_at(elapsed)
            self._refresh_tf_availability()
            state = self._state
            inputs = self._inputs
            gnss = state.gnss
            lidar = state.lidar
            fusion = state.fusion
            navigation = state.navigation
            reloc = state.relocalization
            match = state.match
            self._recorder.add({
                "scenario_id": self._scenario.scenario_id,
                "phase": phase.phase_id,
                "elapsed_time": elapsed,
                "ros_timestamp": self.get_clock().now().nanoseconds / 1e9,
                "gnss_state": gnss.get("state"),
                "gnss_decision": gnss.get("decision"),
                "gnss_satellites": gnss.get("satellites"),
                "gnss_hdop": gnss.get("hdop"),
                "gnss_differential_age": gnss.get("differential_age"),
                "gnss_accepted": gnss.get("accepted"),
                **{key: lidar.get(key) for key in (
                    "raw_point_count", "valid_point_count", "stable_point_count",
                    "dynamic_candidate_count", "temporal_match_ratio", "angular_coverage",
                    "geometry_score")},
                "lidar_state": lidar.get("state"),
                "fusion_mode": fusion.get("mode"),
                "accepted_source": fusion.get("accepted_source"),
                "measurement_confidence": fusion.get("measurement_confidence"),
                "position_uncertainty": fusion.get("position_uncertainty"),
                "yaw_uncertainty": fusion.get("yaw_uncertainty"),
                "adaptive_position_uncertainty": fusion.get("adaptive_position_uncertainty"),
                "adaptive_yaw_uncertainty": fusion.get("adaptive_yaw_uncertainty"),
                "navigation_state": navigation.get("state"),
                "relocalization_state": reloc.get("state"),
                "reloc_attempt_id": reloc.get("attempt_id"),
                "reloc_trigger_reason": reloc.get("trigger_reason"),
                "reloc_active_segment": reloc.get("active_segment"),
                "reloc_verification_count": reloc.get("verification_count"),
                "reloc_failure_reason": reloc.get("failure_reason"),
                "reloc_request_candidate": reloc.get("request_candidate"),
                "reloc_seed_source": reloc.get("seed_source"),
                "reloc_amcl_health": reloc.get("amcl_health"),
                "reloc_lidar_health": reloc.get("lidar_health"),
                "reloc_gnss_health": reloc.get("gnss_health"),
                "reloc_health_reasons": reloc.get("health_reasons"),
                "reloc_command_angular_z": reloc.get("command_angular_z"),
                "reloc_candidate_score": reloc.get("candidate_score"),
                "reloc_candidate_inlier_ratio": reloc.get("candidate_inlier_ratio"),
                "reloc_candidate_mean_distance": reloc.get("candidate_mean_distance"),
                "match_accepted": match.get("accepted"),
                "match_score": match.get("score"),
                "match_inlier_ratio": match.get("inlier_ratio"),
                "match_mean_distance": match.get("mean_distance"),
                "match_used_points": match.get("used_points"),
                "match_candidate_x": match.get("candidate_x"),
                "match_candidate_y": match.get("candidate_y"),
                "match_candidate_yaw": match.get("candidate_yaw"),
                "match_reason": match.get("reason"),
                "match_message_count": state.match_sequence,
                "seed_request_count": state.seed_sequence,
                # The Twist message has no source metadata.  Keeping this
                # empty is intentional; the arbiter's source is not guessed.
                "cmd_vel_source": "",
                "cmd_vel_source_inferred": infer_cmd_vel_source(state),
                "cmd_linear_x": state.cmd_linear_x,
                "cmd_angular_z": state.cmd_angular_z,
                "recovery_linear_x": state.recovery_linear_x,
                "recovery_angular_z": state.recovery_angular_z,
                # --- INPUT side: the synthetic stimulus actually injected ---
                "odom_x": inputs.odom_x,
                "odom_y": inputs.odom_y,
                "odom_yaw": inputs.odom_yaw,
                "odom_linear_x": inputs.odom_linear_x,
                "odom_angular_z": inputs.odom_angular_z,
                "amcl_x": inputs.amcl_x,
                "amcl_y": inputs.amcl_y,
                "amcl_yaw": inputs.amcl_yaw,
                "amcl_covariance": inputs.amcl_covariance,
                "phase_gnss_quality": phase.gnss.quality,
                "phase_lidar_mode": phase.lidar.mode,
                "phase_amcl_covariance_scheduled": phase.amcl.covariance_at(phase.ratio(elapsed)),
                "synthetic_plant_mode": (
                    "ENABLED_CLOSED_LOOP" if self._scenario.plant.enabled else "DISABLED"
                ),
                "synthetic_amcl_surrogate_mode": self._surrogate_mode(),
                "tf_base_to_sensor_available": inputs.tf_base_to_sensor_available,
            })

        def _surrogate_mode(self) -> str:
            """Report the surrogate's observable mode.

            Convergence is inferred from the observed covariance rather than
            read from the surrogate's internals, so this column stays an honest
            observation of the injected stimulus.
            """
            surrogate = self._scenario.amcl_surrogate
            if not surrogate.enabled:
                return "DISABLED"
            if not surrogate.converge_on_match:
                return "BIASED_NO_CONVERGENCE_CONFIGURED"
            converged_covariance = surrogate.covariance_after_convergence
            observed = self._inputs.amcl_covariance
            if converged_covariance is None or observed is None:
                return "BIASED"
            return "CONVERGED" if abs(observed - converged_covariance) <= 1e-6 else "BIASED"

        def finish(self) -> None:
            if self._finished:
                return
            self._finished = True
            self._recorder.write_csvs()

    return SyntheticEvaluatorNode()


def main(args: list[str] | None = None) -> None:
    import argparse
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    parser = argparse.ArgumentParser(description="Record actual DG ROS2 output diagnostics")
    parser.add_argument("scenario_file", type=Path)
    parser.add_argument("output_dir", type=Path)
    parsed = parser.parse_args(args)
    from .scenario_schema import load_scenario

    scenario = load_scenario(parsed.scenario_file)
    rclpy.init(args=None)
    node = create_node(scenario, parsed.output_dir)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
