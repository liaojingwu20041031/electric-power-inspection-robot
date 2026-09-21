import math
import sys
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from active_relocalization_core import (  # noqa: E402
    ACTIVE_SCAN,
    FAILED,
    MANUAL_REQUIRED,
    NORMAL,
    RECOVERED,
    STOPPING,
    SUSPECTED,
    TRIGGERED,
    VERIFYING,
    WAITING_CANDIDATE,
    ActiveRelocalizationConfig,
    ActiveRelocalizationController,
    CandidateQuality,
    LocalizationHealth,
    SupervisorInput,
    assess_health,
    recovery_motion_gate,
)
from active_relocalization_node import parse_match_quality_values, parse_required_signals  # noqa: E402


def healthy(**changes: object) -> LocalizationHealth:
    values = dict(
        timestamp=0.0,
        amcl_covariance=0.1,
        lidar_quality=0.9,
        gnss_quality="GOOD",
        scan_match_score=0.9,
        scan_match_inlier_ratio=0.9,
        scan_match_mean_distance=0.05,
        scan_fresh=True,
        odom_fresh=True,
        imu_fresh=True,
        amcl_fresh=True,
        odom_linear_velocity=0.0,
        odom_angular_velocity=0.0,
        pose_x=0.0,
        pose_y=0.0,
        pose_yaw=0.0,
    )
    values.update(changes)
    return LocalizationHealth(**values)


def candidate(**changes: object) -> CandidateQuality:
    values = dict(timestamp=4.1, score=0.8, inlier_ratio=0.8, mean_distance=0.1, x=0.0, y=0.0, yaw=0.0, used_points=30)
    values.update(changes)
    return CandidateQuality(**values)


def config(**changes: object) -> ActiveRelocalizationConfig:
    values = dict(
        suspect_samples=2,
        trigger_samples=2,
        healthy_recovery_samples=2,
        segment_deltas=(0.1, -0.2, 0.1),
        segment_timeout=1.0,
        settle_time=0.1,
        max_attempt_duration=10.0,
        max_attempts=2,
        verification_samples=2,
    )
    values.update(changes)
    return ActiveRelocalizationConfig(**values)


class RecoveryMotionGateTests(unittest.TestCase):
    def test_unarmed_always_blocks_nonzero_motion(self) -> None:
        allowed, reason = recovery_motion_gate(False, healthy())
        self.assertFalse(allowed)
        self.assertEqual(reason, "RECOVERY_MOTION_NOT_ARMED")

    def test_missing_or_stale_required_sensor_blocks_motion(self) -> None:
        for field in ("scan_fresh", "odom_fresh", "imu_fresh"):
            allowed, reason = recovery_motion_gate(True, healthy(**{field: None}))
            self.assertFalse(allowed)
            self.assertEqual(reason, f"{field.upper()}_NOT_FRESH")

    def test_armed_with_all_required_sensor_streams_allows_motion(self) -> None:
        self.assertEqual(recovery_motion_gate(True, healthy()), (True, ""))


class ActiveRelocalizationCoreTests(unittest.TestCase):
    def trigger_to_stopping(self, controller: ActiveRelocalizationController) -> None:
        bad = healthy(amcl_covariance=1.0)
        self.assertEqual(controller.process(SupervisorInput(0.0, bad)).state, NORMAL)
        self.assertEqual(controller.process(SupervisorInput(0.1, bad)).state, SUSPECTED)
        self.assertEqual(controller.process(SupervisorInput(0.2, bad)).state, TRIGGERED)
        output = controller.process(SupervisorInput(0.3, bad))
        self.assertEqual(output.state, STOPPING)
        self.assertEqual(output.command_angular_z, 0.0)

    def enter_active(self, controller: ActiveRelocalizationController) -> None:
        self.trigger_to_stopping(controller)
        output = controller.process(SupervisorInput(0.4, healthy(), current_yaw=0.0))
        self.assertEqual(output.state, ACTIVE_SCAN)
        self.assertEqual(output.command_angular_z, 0.0)

    def request_candidate(self, controller: ActiveRelocalizationController) -> None:
        self.enter_active(controller)
        controller.process(SupervisorInput(0.5, healthy(), current_yaw=0.1))
        controller.process(SupervisorInput(0.7, healthy(), current_yaw=0.1))
        output = controller.process(SupervisorInput(0.8, healthy(), current_yaw=0.1, scan_updated=True))
        self.assertEqual(output.state, WAITING_CANDIDATE)
        self.assertTrue(output.request_candidate)

    def test_health_is_good_with_all_optional_inputs(self) -> None:
        assessment = assess_health(healthy(), config())
        self.assertFalse(assessment.triggerable)
        self.assertEqual(assessment.lidar_health, "GOOD")
        self.assertEqual(assessment.gnss_health, "GOOD")
        self.assertEqual(assessment.amcl_health, "GOOD")

    def test_missing_optional_lidar_gnss_and_scan_match_do_not_trigger(self) -> None:
        assessment = assess_health(healthy(lidar_quality=None, gnss_quality=None, scan_match_score=None, scan_match_inlier_ratio=None, scan_match_mean_distance=None), config())
        self.assertFalse(assessment.triggerable)

    def test_single_amcl_abnormal_sample_does_not_trigger(self) -> None:
        controller = ActiveRelocalizationController(config())
        output = controller.process(SupervisorInput(0.0, healthy(amcl_covariance=1.0)))
        self.assertEqual(output.state, NORMAL)

    def test_gnss_rejection_alone_does_not_trigger(self) -> None:
        assessment = assess_health(healthy(gnss_quality="REJECTED"), config())
        self.assertFalse(assessment.triggerable)
        self.assertTrue(assessment.degraded)
        controller = ActiveRelocalizationController(config())
        self.assertEqual(controller.process(SupervisorInput(0.0, healthy(gnss_quality="REJECTED"))).state, NORMAL)

    def test_lidar_degradation_alone_does_not_trigger(self) -> None:
        assessment = assess_health(healthy(lidar_quality=0.1), config())
        self.assertFalse(assessment.triggerable)
        self.assertTrue(assessment.degraded)

    def test_old_good_lidar_is_marked_stale_and_does_not_trigger_alone(self) -> None:
        assessment = assess_health(healthy(lidar_fresh=False), config())
        self.assertEqual(assessment.lidar_health, "STALE")
        self.assertFalse(assessment.triggerable)
        self.assertIn("LIDAR_STALE", assessment.reasons)

    def test_old_good_gnss_is_marked_stale_and_does_not_trigger_alone(self) -> None:
        assessment = assess_health(healthy(gnss_quality="GOOD", gnss_fresh=False), config())
        self.assertEqual(assessment.gnss_health, "STALE")
        self.assertFalse(assessment.triggerable)
        self.assertIn("GNSS_STALE", assessment.reasons)

    def test_old_good_scan_match_is_marked_stale_and_does_not_trigger_alone(self) -> None:
        assessment = assess_health(healthy(scan_match_fresh=False), config())
        self.assertFalse(assessment.triggerable)
        self.assertFalse(assessment.degraded)
        self.assertIn("SCAN_MATCH_STALE", assessment.reasons)

    def test_required_stale_signal_can_trigger_by_configuration(self) -> None:
        required = config(required_signals=("lidar", "gnss", "scan_match"))
        assessment = assess_health(healthy(lidar_fresh=False, gnss_fresh=False, scan_match_fresh=False), required)
        self.assertTrue(assessment.triggerable)

    def test_optional_stale_signals_do_not_make_overall_health_degraded(self) -> None:
        for stale_field in ("lidar_fresh", "gnss_fresh", "scan_match_fresh"):
            assessment = assess_health(healthy(**{stale_field: False}), config())
            self.assertFalse(assessment.degraded, stale_field)

    def test_suspected_recovers_with_only_optional_stale_signals(self) -> None:
        controller = ActiveRelocalizationController(config(healthy_recovery_samples=2))
        bad = healthy(amcl_covariance=1.0)
        controller.process(SupervisorInput(0.0, bad))
        self.assertEqual(controller.process(SupervisorInput(0.1, bad)).state, SUSPECTED)
        stale = healthy(lidar_fresh=False, gnss_fresh=False, scan_match_fresh=False)
        self.assertEqual(controller.process(SupervisorInput(0.2, stale)).state, SUSPECTED)
        self.assertEqual(controller.process(SupervisorInput(0.3, stale)).state, NORMAL)

    def test_last_trusted_pose_updates_while_optional_signals_are_stale(self) -> None:
        controller = ActiveRelocalizationController(config())
        controller.process(SupervisorInput(0.0, healthy(pose_x=1.0, pose_y=2.0, pose_yaw=0.1)))
        controller.process(SupervisorInput(0.1, healthy(pose_x=3.0, pose_y=4.0, pose_yaw=0.2, lidar_fresh=False, gnss_fresh=False, scan_match_fresh=False)))
        self.assertEqual(controller.last_trusted_pose, (3.0, 4.0, 0.2))

    def test_required_lidar_stale_is_triggerable(self) -> None:
        self.assertTrue(assess_health(healthy(lidar_fresh=False), config(required_signals=("lidar",))).triggerable)

    def test_required_gnss_stale_is_triggerable(self) -> None:
        self.assertTrue(assess_health(healthy(gnss_fresh=False), config(required_signals=("gnss",))).triggerable)

    def test_required_scan_match_stale_is_triggerable(self) -> None:
        self.assertTrue(assess_health(healthy(scan_match_fresh=False), config(required_signals=("scan_match",))).triggerable)

    def test_required_signals_parameter_normalization(self) -> None:
        self.assertEqual(parse_required_signals(["lidar", "gnss", "", "scan_match"]), ("lidar", "gnss", "scan_match"))
        self.assertEqual(parse_required_signals("amcl"), ("amcl",))

    def test_scan_match_or_amcl_bad_health_is_triggerable(self) -> None:
        self.assertTrue(assess_health(healthy(scan_match_score=0.1), config()).triggerable)
        self.assertTrue(assess_health(healthy(amcl_fresh=False), config()).triggerable)

    def test_trigger_hysteresis_requires_configured_samples(self) -> None:
        controller = ActiveRelocalizationController(config(suspect_samples=3, trigger_samples=2))
        bad = healthy(amcl_covariance=1.0)
        self.assertEqual(controller.process(SupervisorInput(0.0, bad)).state, NORMAL)
        self.assertEqual(controller.process(SupervisorInput(0.1, bad)).state, NORMAL)
        self.assertEqual(controller.process(SupervisorInput(0.2, bad)).state, SUSPECTED)
        self.assertEqual(controller.process(SupervisorInput(0.3, bad)).state, TRIGGERED)

    def test_trigger_hysteresis_also_requires_configured_time(self) -> None:
        controller = ActiveRelocalizationController(config(suspect_duration=1.0, trigger_duration=1.0))
        bad = healthy(amcl_covariance=1.0)
        self.assertEqual(controller.process(SupervisorInput(0.0, bad)).state, NORMAL)
        self.assertEqual(controller.process(SupervisorInput(0.5, bad)).state, NORMAL)
        self.assertEqual(controller.process(SupervisorInput(1.0, bad)).state, SUSPECTED)
        self.assertEqual(controller.process(SupervisorInput(1.5, bad)).state, SUSPECTED)
        self.assertEqual(controller.process(SupervisorInput(2.0, bad)).state, TRIGGERED)

    def test_suspected_state_recovers_after_healthy_hysteresis(self) -> None:
        controller = ActiveRelocalizationController(config(healthy_recovery_samples=2))
        bad = healthy(amcl_covariance=1.0)
        controller.process(SupervisorInput(0.0, bad))
        self.assertEqual(controller.process(SupervisorInput(0.1, bad)).state, SUSPECTED)
        self.assertEqual(controller.process(SupervisorInput(0.2, healthy())).state, SUSPECTED)
        self.assertEqual(controller.process(SupervisorInput(0.3, healthy())).state, NORMAL)

    def test_stopping_waits_for_zero_velocity_and_publishes_zero(self) -> None:
        controller = ActiveRelocalizationController(config())
        self.trigger_to_stopping(controller)
        moving = healthy(odom_linear_velocity=0.1)
        output = controller.process(SupervisorInput(0.4, moving, current_yaw=0.0))
        self.assertEqual(output.state, STOPPING)
        self.assertEqual((output.command_linear_x, output.command_angular_z), (0.0, 0.0))

    def test_stopping_with_stale_odom_fails_without_motion(self) -> None:
        controller = ActiveRelocalizationController(config())
        self.trigger_to_stopping(controller)
        output = controller.process(SupervisorInput(0.4, healthy(odom_fresh=False), current_yaw=0.0))
        self.assertEqual(output.state, FAILED)
        self.assertEqual(output.failure_reason, "ODOM_STALE_OR_MISSING")
        self.assertEqual(output.command_angular_z, 0.0)

    def test_active_rotation_is_bounded_and_has_no_linear_motion(self) -> None:
        controller = ActiveRelocalizationController(config())
        self.enter_active(controller)
        output = controller.process(SupervisorInput(0.5, healthy(), current_yaw=0.0))
        self.assertEqual(output.state, ACTIVE_SCAN)
        self.assertGreater(output.command_angular_z, 0.0)
        self.assertEqual(output.command_linear_x, 0.0)

    def test_rotation_segment_reaches_settling_then_waits_for_scan_candidate(self) -> None:
        controller = ActiveRelocalizationController(config(segment_deltas=(0.1,)))
        self.enter_active(controller)
        reached = controller.process(SupervisorInput(0.5, healthy(), current_yaw=0.1))
        self.assertEqual(reached.state, ACTIVE_SCAN)
        self.assertEqual(reached.command_angular_z, 0.0)
        waiting = controller.process(SupervisorInput(0.7, healthy(), current_yaw=0.1))
        self.assertEqual(waiting.state, WAITING_CANDIDATE)
        self.assertEqual(waiting.command_angular_z, 0.0)

    def test_scan_update_without_candidate_advances_to_next_segment(self) -> None:
        controller = ActiveRelocalizationController(config())
        self.enter_active(controller)
        controller.process(SupervisorInput(0.5, healthy(), current_yaw=0.1))
        controller.process(SupervisorInput(0.7, healthy(), current_yaw=0.1))
        output = controller.process(SupervisorInput(0.8, healthy(), current_yaw=0.1, scan_updated=True))
        self.assertEqual(output.state, WAITING_CANDIDATE)
        self.assertTrue(output.request_candidate)
        repeated = controller.process(SupervisorInput(0.9, healthy(), current_yaw=0.1, scan_updated=True))
        self.assertEqual(repeated.state, WAITING_CANDIDATE)
        self.assertFalse(repeated.request_candidate)

    def test_rejected_match_advances_to_next_segment(self) -> None:
        controller = ActiveRelocalizationController(config())
        self.request_candidate(controller)
        output = controller.process(
            SupervisorInput(0.9, healthy(), current_yaw=0.1, candidate=candidate(accepted=False, reason="SCORE_LOW", received_time=0.9))
        )
        self.assertEqual(output.state, ACTIVE_SCAN)
        self.assertEqual(output.active_segment, 2)

    def test_failed_match_feedback_with_infinite_distance_is_preserved(self) -> None:
        parsed = parse_match_quality_values(
            {"accepted": "false", "reason": "NO_MAP", "score": "0", "inlier_ratio": "0", "mean_distance": "inf"},
            received_time=0.9,
        )
        self.assertFalse(parsed.accepted)
        self.assertEqual(parsed.reason, "NO_MAP")
        self.assertTrue(math.isfinite(parsed.mean_distance))

    def test_failure_feedback_immediately_enters_failure_handling(self) -> None:
        for reason in ("NO_MAP", "NO_SCAN", "TF_FAILED", "NO_VALID_POINTS"):
            controller = ActiveRelocalizationController(config())
            self.request_candidate(controller)
            output = controller.process(SupervisorInput(0.9, healthy(), current_yaw=0.1, candidate=candidate(accepted=False, reason=reason, received_time=0.9)))
            self.assertEqual(output.failure_reason, reason)
            self.assertEqual(output.state, ACTIVE_SCAN, reason)

    def test_quality_rejection_feedback_immediately_enters_next_segment(self) -> None:
        for reason in ("SCORE_LOW", "INLIER_LOW", "MEAN_DISTANCE_HIGH"):
            controller = ActiveRelocalizationController(config())
            self.request_candidate(controller)
            output = controller.process(SupervisorInput(0.9, healthy(), current_yaw=0.1, candidate=candidate(accepted=False, reason=reason, received_time=0.9)))
            self.assertEqual(output.state, ACTIVE_SCAN, reason)

    def test_accepted_candidate_with_nonfinite_score_becomes_rejected(self) -> None:
        parsed = parse_match_quality_values(
            {"accepted": "true", "score": "nan", "inlier_ratio": "0.8", "mean_distance": "0.1", "candidate_x": "0", "candidate_y": "0", "candidate_yaw": "0"},
            received_time=0.9,
        )
        self.assertFalse(parsed.accepted)
        self.assertEqual(parsed.reason, "NONFINITE_MATCH_QUALITY")

    def test_active_scan_rejects_stale_scan(self) -> None:
        controller = ActiveRelocalizationController(config())
        self.enter_active(controller)
        output = controller.process(SupervisorInput(0.5, healthy(scan_fresh=False), current_yaw=0.0))
        self.assertEqual(output.state, FAILED)
        self.assertEqual(output.failure_reason, "SCAN_STALE_DURING_ACTIVE_SCAN")

    def test_active_scan_requires_fresh_odom_before_motion(self) -> None:
        controller = ActiveRelocalizationController(config())
        self.enter_active(controller)
        output = controller.process(SupervisorInput(0.5, healthy(odom_fresh=None), current_yaw=0.0))
        self.assertEqual(output.state, FAILED)
        self.assertEqual(output.failure_reason, "ODOM_STALE_DURING_ACTIVE_SCAN")

    def test_segment_timeout_fails_and_next_cycle_requests_retry(self) -> None:
        controller = ActiveRelocalizationController(config(max_attempt_duration=10.0, segment_timeout=1.0))
        self.enter_active(controller)
        failed = controller.process(SupervisorInput(2.0, healthy(), current_yaw=0.0))
        self.assertEqual(failed.state, FAILED)
        self.assertEqual(failed.failure_reason, "SEGMENT_TIMEOUT")
        retry = controller.process(SupervisorInput(2.1, healthy(), current_yaw=0.0))
        self.assertEqual(retry.state, STOPPING)

    def test_attempt_timeout_fails(self) -> None:
        controller = ActiveRelocalizationController(config(max_attempt_duration=0.5, segment_timeout=10.0))
        self.enter_active(controller)
        output = controller.process(SupervisorInput(1.0, healthy(), current_yaw=0.0))
        self.assertEqual(output.state, FAILED)
        self.assertEqual(output.failure_reason, "ATTEMPT_TIMEOUT")

    def test_waiting_for_candidate_timeout_fails(self) -> None:
        controller = ActiveRelocalizationController(config(segment_deltas=(0.1,), segment_timeout=1.0))
        self.enter_active(controller)
        controller.process(SupervisorInput(0.5, healthy(), current_yaw=0.1))
        self.assertEqual(controller.process(SupervisorInput(0.7, healthy(), current_yaw=0.1)).state, WAITING_CANDIDATE)
        output = controller.process(SupervisorInput(1.8, healthy(), current_yaw=0.1))
        self.assertEqual(output.state, FAILED)
        self.assertEqual(output.failure_reason, "WAITING_FOR_CANDIDATE_TIMEOUT")

    def test_total_rotation_limit_blocks_next_segment(self) -> None:
        controller = ActiveRelocalizationController(config(max_total_rotation=0.1))
        self.request_candidate(controller)
        output = controller.process(SupervisorInput(0.9, healthy(), current_yaw=0.1, candidate=candidate(accepted=False, reason="MAX_TOTAL_ROTATION", received_time=0.9)))
        self.assertEqual(output.state, FAILED)
        self.assertEqual(output.failure_reason, "MAX_TOTAL_ROTATION")

    def test_good_candidate_enters_verification(self) -> None:
        controller = ActiveRelocalizationController(config())
        self.request_candidate(controller)
        output = controller.process(SupervisorInput(0.9, healthy(), current_yaw=0.0, candidate=candidate(received_time=0.9)))
        self.assertEqual(output.state, VERIFYING)
        self.assertEqual(output.command_angular_z, 0.0)

    def test_low_candidate_is_not_accepted(self) -> None:
        controller = ActiveRelocalizationController(config())
        self.request_candidate(controller)
        output = controller.process(SupervisorInput(0.9, healthy(), current_yaw=0.0, candidate=candidate(score=0.1, received_time=0.9)))
        self.assertEqual(output.state, ACTIVE_SCAN)

    def test_verification_requires_consecutive_samples_and_recovers(self) -> None:
        controller = ActiveRelocalizationController(config(verification_samples=2))
        self.request_candidate(controller)
        controller.process(SupervisorInput(0.9, healthy(), current_yaw=0.0, candidate=candidate(received_time=0.9)))
        pending = controller.process(SupervisorInput(0.6, healthy(), current_yaw=0.0))
        self.assertEqual(pending.state, VERIFYING)
        recovered = controller.process(SupervisorInput(0.7, healthy(), current_yaw=0.0))
        self.assertEqual(recovered.state, RECOVERED)
        self.assertEqual(recovered.command_angular_z, 0.0)

    def test_verification_pose_jump_fails(self) -> None:
        controller = ActiveRelocalizationController(config(segment_deltas=(0.1,), verification_samples=2))
        self.request_candidate(controller)
        controller.process(SupervisorInput(0.9, healthy(), current_yaw=0.0, candidate=candidate(received_time=0.9)))
        controller.process(SupervisorInput(1.0, healthy(pose_x=0.0), current_yaw=0.0))
        output = controller.process(SupervisorInput(1.1, healthy(pose_x=1.0), current_yaw=0.0))
        self.assertEqual(output.state, FAILED)
        self.assertEqual(output.failure_reason, "VERIFY_POSE_JUMP")

    def test_verification_quality_failure_returns_to_next_active_segment(self) -> None:
        controller = ActiveRelocalizationController(config())
        self.request_candidate(controller)
        controller.process(SupervisorInput(0.9, healthy(), current_yaw=0.0, candidate=candidate(received_time=0.9)))
        output = controller.process(SupervisorInput(1.0, healthy(), current_yaw=0.0, candidate=candidate(score=0.1, received_time=1.0)))
        self.assertEqual(output.state, ACTIVE_SCAN)
        self.assertEqual(output.failure_reason, "CANDIDATE_QUALITY_LOW")

    def test_failed_attempts_end_in_manual_required(self) -> None:
        controller = ActiveRelocalizationController(config(segment_deltas=(0.1,), max_attempts=1, segment_timeout=0.2))
        self.enter_active(controller)
        failed = controller.process(SupervisorInput(1.0, healthy(), current_yaw=0.0))
        self.assertEqual(failed.state, FAILED)
        manual = controller.process(SupervisorInput(1.1, healthy(), current_yaw=0.0))
        self.assertEqual(manual.state, MANUAL_REQUIRED)
        self.assertEqual(manual.command_angular_z, 0.0)

    def test_manual_takeover_latches_and_stops_immediately(self) -> None:
        controller = ActiveRelocalizationController(config())
        output = controller.process(SupervisorInput(0.0, healthy(), manual_takeover=True))
        self.assertEqual(output.state, MANUAL_REQUIRED)
        self.assertTrue(output.manual_takeover)
        self.assertEqual(output.command_angular_z, 0.0)
        self.assertEqual(controller.process(SupervisorInput(0.1, healthy())).state, MANUAL_REQUIRED)

    def test_shutdown_latches_failed_state_and_zero_command(self) -> None:
        controller = ActiveRelocalizationController(config())
        output = controller.process(SupervisorInput(0.0, healthy(), shutdown=True))
        self.assertEqual(output.state, FAILED)
        self.assertEqual(output.failure_reason, "SHUTDOWN")
        self.assertEqual(output.command_angular_z, 0.0)

    def test_threshold_boundaries_are_inclusive(self) -> None:
        cfg = config(max_covariance=0.5, min_scan_match_score=0.45, min_scan_match_inlier_ratio=0.45, max_scan_match_mean_distance=0.3)
        assessment = assess_health(healthy(amcl_covariance=0.5, scan_match_score=0.45, scan_match_inlier_ratio=0.45, scan_match_mean_distance=0.3), cfg)
        self.assertFalse(assessment.triggerable)
        controller = ActiveRelocalizationController(cfg)
        self.request_candidate(controller)
        output = controller.process(SupervisorInput(0.9, healthy(), current_yaw=0.0, candidate=candidate(score=0.45, inlier_ratio=0.45, mean_distance=0.3, received_time=0.9)))
        self.assertEqual(output.state, VERIFYING)

    def test_stale_candidate_before_request_is_ignored(self) -> None:
        controller = ActiveRelocalizationController(config())
        self.request_candidate(controller)
        output = controller.process(SupervisorInput(0.9, healthy(), current_yaw=0.0, candidate=candidate(received_time=0.7)))
        self.assertEqual(output.state, WAITING_CANDIDATE)

    def test_normal_health_saves_last_trusted_pose_and_attempt_prefers_it(self) -> None:
        controller = ActiveRelocalizationController(config())
        controller.process(SupervisorInput(0.0, healthy(pose_x=2.0, pose_y=3.0, pose_yaw=0.4)))
        self.trigger_to_stopping(controller)
        output = controller.process(SupervisorInput(0.4, healthy(pose_x=9.0, pose_y=9.0), current_yaw=0.0))
        self.assertEqual(output.seed_source, "LAST_TRUSTED")
        self.assertEqual(output.seed_pose, (2.0, 3.0, 0.4))

    def test_current_amcl_is_fallback_seed_when_no_trusted_pose_exists(self) -> None:
        controller = ActiveRelocalizationController(config())
        self.trigger_to_stopping(controller)
        output = controller.process(SupervisorInput(0.4, healthy(pose_x=2.0, pose_y=3.0, pose_yaw=0.4), current_yaw=0.0))
        self.assertEqual(output.seed_source, "CURRENT_AMCL")

    def test_manual_and_shutdown_never_request_candidate(self) -> None:
        manual = ActiveRelocalizationController(config()).process(SupervisorInput(0.0, healthy(), manual_takeover=True))
        shutdown = ActiveRelocalizationController(config()).process(SupervisorInput(0.0, healthy(), shutdown=True))
        self.assertFalse(manual.request_candidate)
        self.assertFalse(shutdown.request_candidate)

    def test_controller_sequence_is_deterministic(self) -> None:
        events = [
            SupervisorInput(0.0, healthy(amcl_covariance=1.0)),
            SupervisorInput(0.1, healthy(amcl_covariance=1.0)),
            SupervisorInput(0.2, healthy(amcl_covariance=1.0)),
            SupervisorInput(0.3, healthy(amcl_covariance=1.0)),
            SupervisorInput(0.4, healthy(), current_yaw=0.0),
            SupervisorInput(0.5, healthy(), current_yaw=0.0),
        ]
        first_controller = ActiveRelocalizationController(config())
        first = [first_controller.process(event) for event in events]
        second_controller = ActiveRelocalizationController(config())
        second = [second_controller.process(event) for event in events]
        self.assertEqual(first, second)


if __name__ == "__main__":
    unittest.main()
