"""CSV/timeline/result writers with no ROS dependency.

Keeping this part ROS-free makes the evaluator contract unit-testable and
prevents the report generator from inventing values when a topic is absent.
Missing observations are written as empty cells and reported as warnings.

Every recorded column carries an explicit role in ``FIELD_ROLES``:

  INPUT              synthetic stimulus fed to the software under test
  SUT_OUTPUT         published by the software under test
  DERIVED_ASSERTION  computed by this evaluator, never read from a topic
  METADATA           bookkeeping

No column ever carries two roles.  ``field_roles.csv`` is written next to
``samples.csv`` so a reader can always tell which side of the boundary a value
came from.
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

from .scenario_schema import Scenario


TRUTH_LABELS = {
    "authenticity_marker": "THIS_IS_SYNTHETIC_SOFTWARE_VALIDATION",
    "validation_class": "SYNTHETIC_SOFTWARE_VALIDATION",
    "data_class": "NOT_REAL_ROBOT_DATA",
    "simulator_class": "NOT_GAZEBO_DATA",
    "performance_claim_label": "NOT_COMPETITION_PERFORMANCE_EVIDENCE",
}

# The integration launch is already alive when the evaluator starts, but the
# first diagnostic cycle can still precede the synthetic sensor publishers.
# Ignore only this bounded startup window when asserting that relocalization
# was not unexpectedly activated; all samples remain recorded in the CSV.
RELOCALIZATION_STARTUP_GRACE_SEC = 2.0

# Real defaults read from src/ylhb_base/scripts/active_relocalization_node.py
# and scan_map_relocalization_node.py.  Recorded here for reporting only -- this
# module never enforces or modifies a core threshold.
REAL_VERIFICATION_SAMPLES = 3
REAL_MIN_SCAN_MATCH_SCORE = 0.45
REAL_MIN_SCAN_MATCH_INLIER_RATIO = 0.45
REAL_MAX_SCAN_MATCH_MEAN_DISTANCE = 0.30
REAL_MAX_COVARIANCE = 0.5

VALID_RELOCALIZATION_STATES = (
    "NORMAL", "SUSPECTED", "TRIGGERED", "STOPPING", "ACTIVE_SCAN",
    "WAITING_CANDIDATE", "VERIFYING", "RECOVERED", "FAILED", "MANUAL_REQUIRED",
)

_METADATA_COLUMNS = ["scenario_id", "phase", "elapsed_time", "ros_timestamp"]

_SUT_OUTPUT_COLUMNS = [
    "gnss_state", "gnss_decision", "gnss_satellites", "gnss_hdop",
    "gnss_differential_age", "gnss_accepted", "lidar_state",
    "raw_point_count", "valid_point_count", "stable_point_count",
    "dynamic_candidate_count", "temporal_match_ratio", "angular_coverage",
    "geometry_score", "fusion_mode", "accepted_source",
    "measurement_confidence", "position_uncertainty", "yaw_uncertainty",
    "adaptive_position_uncertainty", "adaptive_yaw_uncertainty",
    "navigation_state", "relocalization_state",
    "reloc_attempt_id", "reloc_trigger_reason", "reloc_active_segment",
    "reloc_verification_count", "reloc_failure_reason", "reloc_request_candidate",
    "reloc_seed_source", "reloc_amcl_health", "reloc_lidar_health",
    "reloc_gnss_health", "reloc_health_reasons", "reloc_command_angular_z",
    "reloc_candidate_score", "reloc_candidate_inlier_ratio",
    "reloc_candidate_mean_distance",
    "match_accepted", "match_score", "match_inlier_ratio", "match_mean_distance",
    "match_used_points", "match_candidate_x", "match_candidate_y",
    "match_candidate_yaw", "match_reason", "match_message_count",
    "seed_request_count", "cmd_vel_source", "cmd_linear_x", "cmd_angular_z",
    "recovery_linear_x", "recovery_angular_z",
]

# INPUT side: what the synthetic plant / AMCL surrogate actually fed in.  These
# are recorded as evidence of the stimulus and are NEVER used to assert that an
# algorithm succeeded.
_INPUT_COLUMNS = [
    "odom_x", "odom_y", "odom_yaw", "odom_linear_x", "odom_angular_z",
    "amcl_x", "amcl_y", "amcl_yaw", "amcl_covariance",
    "phase_gnss_quality", "phase_lidar_mode", "phase_amcl_covariance_scheduled",
    "synthetic_plant_mode", "synthetic_amcl_surrogate_mode",
    "tf_base_to_sensor_available",
]

_DERIVED_ASSERTION_COLUMNS = ["cmd_vel_source_inferred", "safety_cmd_vel_publishers"]

SAMPLE_COLUMNS = (
    _METADATA_COLUMNS + _SUT_OUTPUT_COLUMNS + _INPUT_COLUMNS + _DERIVED_ASSERTION_COLUMNS
)

FIELD_ROLES: dict[str, str] = {
    **{name: "METADATA" for name in _METADATA_COLUMNS},
    **{name: "SUT_OUTPUT" for name in _SUT_OUTPUT_COLUMNS},
    **{name: "INPUT" for name in _INPUT_COLUMNS},
    **{name: "DERIVED_ASSERTION" for name in _DERIVED_ASSERTION_COLUMNS},
}


def _csv_value(value: Any) -> Any:
    if value is None:
        return ""
    if isinstance(value, float):
        return value if math.isfinite(value) else ""
    return value


# The timeline tracks a few non-SUT components (the inferred arbiter source and
# the surrogate mode) because knowing *when* control switched and *when* the
# surrogate converged is essential evidence.  Each row therefore carries its own
# role so the timeline never blurs the input/output boundary either.
TIMELINE_COMPONENT_ROLES = {
    "gnss": "SUT_OUTPUT",
    "lidar": "SUT_OUTPUT",
    "fusion": "SUT_OUTPUT",
    "navigation": "SUT_OUTPUT",
    "relocalization": "SUT_OUTPUT",
    "arbiter_source_inferred": "DERIVED_ASSERTION",
    "amcl_surrogate": "INPUT",
}

TIMELINE_COLUMNS = [
    "elapsed_time", "ros_timestamp", "phase", "component", "component_role",
    "previous_state", "new_state", "reason",
]


@dataclass
class TimelineTracker:
    """Record only state changes, preserving the first observation."""

    rows: list[dict[str, Any]] = field(default_factory=list)
    _last: dict[str, str] = field(default_factory=dict)

    def update(self, elapsed: float, ros_timestamp: float, phase: str, values: dict[str, Any]) -> None:
        for component, raw in values.items():
            state = "" if raw is None else str(raw)
            if self._last.get(component) == state:
                continue
            previous = self._last.get(component, "")
            self._last[component] = state
            self.rows.append({
                "elapsed_time": elapsed,
                "ros_timestamp": ros_timestamp,
                "phase": phase,
                "component": component,
                "component_role": TIMELINE_COMPONENT_ROLES.get(component, "UNCLASSIFIED"),
                "previous_state": previous,
                "new_state": state,
                "reason": "STATE_CHANGE" if previous else "INITIAL_OBSERVATION",
            })


@dataclass
class SampleRecorder:
    scenario: Scenario
    output_dir: Path
    samples: list[dict[str, Any]] = field(default_factory=list)
    timeline: TimelineTracker = field(default_factory=TimelineTracker)

    def add(self, row: dict[str, Any]) -> None:
        normalized = {column: _csv_value(row.get(column)) for column in SAMPLE_COLUMNS}
        self.samples.append(normalized)
        self.timeline.update(
            float(row.get("elapsed_time", 0.0)),
            float(row.get("ros_timestamp", 0.0)),
            str(row.get("phase", "")),
            {
                "gnss": row.get("gnss_state"),
                "lidar": row.get("lidar_state"),
                "fusion": row.get("fusion_mode"),
                "navigation": row.get("navigation_state"),
                "relocalization": row.get("relocalization_state"),
                "arbiter_source_inferred": row.get("cmd_vel_source_inferred"),
                "amcl_surrogate": row.get("synthetic_amcl_surrogate_mode"),
            },
        )

    def write_csvs(self) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        with (self.output_dir / "samples.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=SAMPLE_COLUMNS)
            writer.writeheader()
            writer.writerows(self.samples)
        with (self.output_dir / "timeline.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=TIMELINE_COLUMNS)
            writer.writeheader()
            writer.writerows(self.timeline.rows)
        with (self.output_dir / "field_roles.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["column", "role"])
            for column in SAMPLE_COLUMNS:
                writer.writerow([column, FIELD_ROLES.get(column, "UNCLASSIFIED")])


def _states(samples: Iterable[dict[str, Any]], key: str) -> list[str]:
    return [str(row.get(key, "")).upper() for row in samples if row.get(key) not in (None, "")]


def _states_after(samples: Iterable[dict[str, Any]], key: str, elapsed_min: float) -> list[str]:
    return [
        str(row.get(key, "")).upper()
        for row in samples
        if row.get(key) not in (None, "")
        and float(row.get("elapsed_time", 0.0) or 0.0) >= elapsed_min
    ]


def _has_in_order(values: list[str], sequence: tuple[str, ...]) -> bool:
    index = 0
    for value in values:
        if value == sequence[index]:
            index += 1
            if index == len(sequence):
                return True
    return False


def _numbers(samples: Iterable[dict[str, Any]], key: str) -> list[float]:
    result: list[float] = []
    for row in samples:
        value = row.get(key)
        if value in (None, ""):
            continue
        try:
            number = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(number):
            result.append(number)
    return result


def _truthy(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "accepted", "ok"}


def _rows_where_true(samples: Iterable[dict[str, Any]], key: str) -> list[dict[str, Any]]:
    return [row for row in samples if row.get(key) not in (None, "") and _truthy(row.get(key))]


def _first_elapsed_where(samples: Iterable[dict[str, Any]], predicate: Any) -> float | None:
    for row in samples:
        if predicate(row):
            try:
                return float(row.get("elapsed_time", 0.0) or 0.0)
            except (TypeError, ValueError):
                return None
    return None


def _first_elapsed_at_state(samples: Iterable[dict[str, Any]], key: str, state: str) -> float | None:
    return _first_elapsed_where(samples, lambda row: str(row.get(key, "")).upper() == state)


def _finite_samples(samples: Iterable[dict[str, Any]], keys: tuple[str, ...]) -> bool:
    for row in samples:
        for key in keys:
            value = row.get(key)
            if value in (None, ""):
                continue
            try:
                if not math.isfinite(float(value)):
                    return False
            except (TypeError, ValueError):
                return False
    return True


def _important_transitions(samples: list[dict[str, Any]]) -> list[str]:
    """Summarize observed state changes without inferring absent transitions."""
    transitions: list[str] = []
    previous: dict[str, str] = {}
    for row in samples:
        elapsed = row.get("elapsed_time", "")
        for key, label in (("gnss_state", "GNSS"), ("lidar_state", "LiDAR"),
                           ("navigation_state", "Navigation"), ("relocalization_state", "Relocalization"),
                           ("fusion_mode", "Fusion")):
            value = row.get(key)
            if value in (None, ""):
                continue
            state = str(value)
            if previous.get(key) == state:
                continue
            if key in previous:
                transitions.append(f"{label}: {previous[key]} -> {state} @ {elapsed}s")
            previous[key] = state
    return transitions


CLOSED_LOOP_SCENARIOS = {"S05", "S06", "S07", "S08"}
_EPS = 1e-6


def _row_number(row: dict[str, Any], key: str) -> float | None:
    value = row.get(key)
    if value in (None, ""):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _ground_truth_disclosure(scenario: Scenario) -> dict[str, Any]:
    """State plainly who owns the true pose and who may read it.

    The synthetic rig authors the true pose.  The surrogate reads it to build a
    deliberately biased ``/amcl_pose``.  No software under test can read it: the
    only TF consumer in the chain looks up base_footprint <- laser, and no
    map->odom transform is published at all.
    """
    surrogate = scenario.amcl_surrogate
    return {
        "GROUND_TRUTH_AVAILABLE": "YES",
        "GROUND_TRUTH_OWNER": "SYNTHETIC_TEST_RIG",
        "GROUND_TRUTH_USED_BY_SUT": "NO",
        "GROUND_TRUTH_USED_BY_SURROGATE": (
            "YES" if surrogate.enabled and surrogate.accesses_plant_ground_truth else "NO"
        ),
        "GROUND_TRUTH_USED_FOR_ASSERTION": "NO_ONLY_FOR_INPUT_AND_BIAS_CONSTRUCTION",
        "GROUND_TRUTH_LEAKAGE_TO_MATCHER": "NO",
        "PERFECT_MAP_ODOM_PUBLISHED": "NO",
    }


def _s05_checks(samples: list[dict[str, Any]], states_reloc: list[str], states_nav: list[str]) -> dict[str, bool]:
    """S05 -- a real, detectable localization failure must actually trigger."""
    return {
        "s05_sut_reported_amcl_health_degraded": _has_in_order(
            _states(samples, "reloc_amcl_health"), ("GOOD", "BAD")
        ),
        "s05_normal_suspected_triggered_in_order": _has_in_order(
            states_reloc, ("NORMAL", "SUSPECTED", "TRIGGERED")
        ),
        "s05_trigger_reason_is_a_real_trigger_path": any(
            "AMCL_COVARIANCE_HIGH" in str(row.get("reloc_trigger_reason", "")).upper()
            or "AMCL_STALE" in str(row.get("reloc_trigger_reason", "")).upper()
            or "SCAN_MATCH_QUALITY_LOW" in str(row.get("reloc_trigger_reason", "")).upper()
            or "SCAN_STALE" in str(row.get("reloc_trigger_reason", "")).upper()
            for row in samples
        ),
        "s05_navigation_health_escalated": any(
            state in {"LOCALIZATION_SUSPECT", "RECOVERING"} for state in states_nav
        ),
    }


def _s06_checks(samples: list[dict[str, Any]], states_reloc: list[str], states_nav: list[str]) -> dict[str, bool]:
    """S06 -- recovery motion must be closed-loop, not scripted."""
    odom_yaws = _numbers(samples, "odom_yaw")
    return {
        "s06_stopping_confirmed_then_active_scan": _has_in_order(
            states_reloc, ("STOPPING", "ACTIVE_SCAN")
        ),
        "s06_navigation_health_recovering": "RECOVERING" in states_nav,
        "s06_supervisor_commanded_rotation": any(
            abs(value) > _EPS for value in _numbers(samples, "reloc_command_angular_z")
        ),
        "s06_arbiter_forwarded_recovery": any(
            str(row.get("cmd_vel_source_inferred", "")).upper() == "RECOVERY" for row in samples
        ),
        "s06_test_cmd_vel_carried_rotation": any(
            abs(value) > _EPS for value in _numbers(samples, "cmd_angular_z")
        ),
        "s06_recovery_is_pure_rotation": all(
            abs(value) <= _EPS for value in _numbers(samples, "cmd_linear_x")
        ),
        "s06_plant_yaw_responded_to_command": bool(odom_yaws)
        and (max(odom_yaws) - min(odom_yaws)) > math.radians(1.0),
    }


def _s07_checks(samples: list[dict[str, Any]]) -> dict[str, bool]:
    """S07 -- the candidate must come from the real matcher, after a real seed."""
    accepted_rows = _rows_where_true(samples, "match_accepted")
    first_seed = _first_elapsed_where(
        samples, lambda row: (_row_number(row, "seed_request_count") or 0.0) >= 1.0
    )
    first_accepted = _first_elapsed_where(
        samples, lambda row: row.get("match_accepted") not in (None, "") and _truthy(row.get("match_accepted"))
    )
    meets_real_thresholds = False
    for row in accepted_rows:
        score = _row_number(row, "match_score")
        inlier = _row_number(row, "match_inlier_ratio")
        mean_distance = _row_number(row, "match_mean_distance")
        if None in (score, inlier, mean_distance):
            continue
        if (
            score >= REAL_MIN_SCAN_MATCH_SCORE
            and inlier >= REAL_MIN_SCAN_MATCH_INLIER_RATIO
            and mean_distance <= REAL_MAX_SCAN_MATCH_MEAN_DISTANCE
        ):
            meets_real_thresholds = True
            break
    return {
        "s07_seed_requested_by_real_supervisor": any(
            value >= 1.0 for value in _numbers(samples, "seed_request_count")
        ),
        "s07_real_matcher_published_quality": any(
            value >= 1.0 for value in _numbers(samples, "match_message_count")
        ),
        "s07_candidate_accepted_by_real_matcher": bool(accepted_rows),
        "s07_candidate_meets_unmodified_thresholds": meets_real_thresholds,
        "s07_candidate_used_real_scan_points": any(
            value > 0.0 for value in _numbers(samples, "match_used_points")
        ),
        "s07_candidate_followed_the_seed_request": (
            first_seed is not None and first_accepted is not None and first_accepted >= first_seed
        ),
    }


def _s08_checks(samples: list[dict[str, Any]], states_reloc: list[str], states_nav: list[str]) -> dict[str, bool]:
    """S08 -- multi-frame verification, then a safe control handoff.

    The handoff check deliberately asserts only that recovery released the
    command.  It does NOT assert that navigation resumed: Nav2 is intentionally
    not running and nothing publishes /cmd_vel_nav, so the arbiter's expected
    outcome is NAV selected followed by a zero output.
    """
    verification_counts = _numbers(samples, "reloc_verification_count")
    recovered_at = _first_elapsed_at_state(samples, "relocalization_state", "RECOVERED")
    after_recovery = [
        row
        for row in samples
        if recovered_at is not None and (_row_number(row, "elapsed_time") or 0.0) >= recovered_at
    ]
    handoff_released = bool(after_recovery) and all(
        abs(value) <= _EPS for value in _numbers(after_recovery, "cmd_angular_z")
    ) and all(abs(value) <= _EPS for value in _numbers(after_recovery, "cmd_linear_x"))
    return {
        "s08_waiting_verifying_recovered_in_order": _has_in_order(
            states_reloc, ("WAITING_CANDIDATE", "VERIFYING", "RECOVERED")
        ),
        "s08_multi_frame_verification_completed": bool(verification_counts)
        and max(verification_counts) >= REAL_VERIFICATION_SAMPLES,
        "s08_recovered_observed": "RECOVERED" in states_reloc,
        "s08_navigation_health_reported_recovered": "RECOVERED" in states_nav,
        "s08_safe_control_handoff_observed": handoff_released,
    }


def evaluate_scenario(
    scenario: Scenario,
    samples: list[dict[str, Any]],
    safety_cmd_vel_publishers: int,
    integration_alive: bool,
    run_class: str = "FINAL",
    safety_cmd_vel_publishers_at_start: int = 0,
) -> dict[str, Any]:
    """Apply the conservative software checks appropriate to each scenario."""
    states_gnss = _states(samples, "gnss_state")
    states_lidar = _states(samples, "lidar_state")
    states_nav = _states(samples, "navigation_state")
    states_nav_steady = _states_after(samples, "navigation_state", RELOCALIZATION_STARTUP_GRACE_SEC)
    states_reloc = _states_after(samples, "relocalization_state", RELOCALIZATION_STARTUP_GRACE_SEC)
    states_reloc_all = _states(samples, "relocalization_state")
    errors: list[str] = []
    warnings: list[str] = []
    checks: dict[str, bool] = {}
    closed_loop = scenario.scenario_id in CLOSED_LOOP_SCENARIOS

    checks["samples_present"] = bool(samples)
    checks["integration_alive"] = integration_alive
    checks["real_cmd_vel_is_unpublished"] = safety_cmd_vel_publishers == 0
    checks["real_cmd_vel_unpublished_at_start"] = safety_cmd_vel_publishers_at_start == 0
    checks["finite_fusion_outputs"] = _finite_samples(
        samples,
        ("measurement_confidence", "position_uncertainty", "yaw_uncertainty",
         "adaptive_position_uncertainty", "adaptive_yaw_uncertainty"),
    )
    relocation_active = any(
        state in {"TRIGGERED", "STOPPING", "ACTIVE_SCAN", "WAITING_CANDIDATE", "VERIFYING", "FAILED", "MANUAL_REQUIRED"}
        for state in states_reloc
    )
    checks["relocalization_activity_observed"] = relocation_active
    # S01 is the nominal control case and must not activate recovery.  S02-S04
    # exercise degraded inputs whose recovery loop was out of scope.  S05-S08
    # exercise the recovery loop deliberately, so activity is expected there.
    # S01 is the only scenario where this discriminates: it is the nominal
    # control case and any recovery activity is a genuine failure.  For every
    # other scenario the value was previously hardcoded True and still counted
    # toward the reported check total, which inflated apparent coverage.  It is
    # now reported separately as non-discriminating rather than counted.
    non_discriminating_checks: dict[str, Any] = {}
    if scenario.scenario_id == "S01":
        checks["relocalization_not_unexpectedly_active"] = not relocation_active
    else:
        non_discriminating_checks["relocalization_not_unexpectedly_active"] = {
            "value": True,
            "reason": (
                "NON_DISCRIMINATING_CHECK: hardcoded True outside S01 because "
                "recovery activity is expected there. Cannot fail, so it is not "
                "counted in the check total."
            ),
        }
    unexpected_states = sorted(
        {state for state in states_reloc_all if state not in VALID_RELOCALIZATION_STATES}
    )
    checks["relocalization_states_are_known"] = not unexpected_states

    if not checks["samples_present"]:
        errors.append("NO_EVALUATOR_SAMPLES")
    if not integration_alive:
        errors.append("INTEGRATION_LAUNCH_EXITED_EARLY")
    if safety_cmd_vel_publishers != 0:
        errors.append(f"UNSAFE_REAL_CMD_VEL_PUBLISHERS={safety_cmd_vel_publishers}")
    if not checks["finite_fusion_outputs"]:
        errors.append("NONFINITE_FUSION_OUTPUT")
    if unexpected_states:
        errors.append("UNKNOWN_RELOCALIZATION_STATE=" + ";".join(unexpected_states))
    if scenario.scenario_id == "S01" and not checks["relocalization_not_unexpectedly_active"]:
        errors.append("UNEXPECTED_RELOCALIZATION_STATE")
    elif scenario.scenario_id not in CLOSED_LOOP_SCENARIOS and scenario.scenario_id != "S01" and relocation_active:
        warnings.append("RELOCALIZATION_ACTIVITY_OBSERVED_OUT_OF_SCOPE")

    if scenario.scenario_id == "S01":
        checks["gnss_nominal_seen"] = "GOOD" in states_gnss
        checks["lidar_nominal_seen"] = "GOOD" in states_lidar
        checks["navigation_not_failed"] = not any(
            state in {"FAILED", "MANUAL_REQUIRED"} for state in states_nav_steady
        )
        for name in ("gnss_nominal_seen", "lidar_nominal_seen", "navigation_not_failed"):
            if not checks[name]:
                errors.append(name.upper())
    elif scenario.scenario_id == "S02":
        checks["gnss_good_to_degraded"] = _has_in_order(states_gnss, ("GOOD", "DEGRADED"))
        checks["gnss_rejection_seen"] = any(state in {"REJECTED", "STALE"} for state in states_gnss)
        checks["gnss_rejected_fix_not_accepted"] = all(
            str(row.get("gnss_accepted", "")).lower() not in {"true", "1"}
            for row in samples
            if str(row.get("gnss_state", "")).upper() in {"REJECTED", "STALE"}
        )
        for name in ("gnss_good_to_degraded", "gnss_rejection_seen", "gnss_rejected_fix_not_accepted"):
            if not checks[name]:
                errors.append(name.upper())
    elif scenario.scenario_id == "S03":
        scores = [float(row["geometry_score"]) for row in samples if row.get("geometry_score") not in (None, "")]
        checks["lidar_good_seen"] = any(score >= 0.2 for score in scores)
        checks["lidar_degraded_score_seen"] = any(score < 0.2 for score in scores)
        checks["lidar_state_degraded_or_rejected"] = any(state in {"DEGRADED", "REJECTED"} for state in states_lidar)
        for name in ("lidar_good_seen", "lidar_degraded_score_seen", "lidar_state_degraded_or_rejected"):
            if not checks[name]:
                errors.append(name.upper())
    elif scenario.scenario_id == "S04":
        checks["gnss_rejection_seen"] = any(state in {"REJECTED", "STALE"} for state in states_gnss)
        checks["lidar_degradation_seen"] = any(state in {"DEGRADED", "REJECTED"} for state in states_lidar)
        checks["navigation_degradation_seen"] = any(state in {"DEGRADED", "LOCALIZATION_SUSPECT", "RECOVERING"} for state in states_nav)
        for name in ("gnss_rejection_seen", "lidar_degradation_seen", "navigation_degradation_seen"):
            if not checks[name]:
                errors.append(name.upper())
    elif scenario.scenario_id == "S05":
        scenario_checks = _s05_checks(samples, states_reloc_all, states_nav)
        checks.update(scenario_checks)
        errors.extend(name.upper() for name, ok in scenario_checks.items() if not ok)
    elif scenario.scenario_id == "S06":
        scenario_checks = _s06_checks(samples, states_reloc_all, states_nav)
        checks.update(scenario_checks)
        errors.extend(name.upper() for name, ok in scenario_checks.items() if not ok)
    elif scenario.scenario_id == "S07":
        scenario_checks = _s07_checks(samples)
        checks.update(scenario_checks)
        errors.extend(name.upper() for name, ok in scenario_checks.items() if not ok)
    elif scenario.scenario_id == "S08":
        scenario_checks = _s08_checks(samples, states_reloc_all, states_nav)
        checks.update(scenario_checks)
        errors.extend(name.upper() for name, ok in scenario_checks.items() if not ok)
    else:
        warnings.append("UNKNOWN_SCENARIO_NO_SCENARIO_SPECIFIC_ASSERTIONS")

    if not states_reloc:
        warnings.append("RELOCALIZATION_STATUS_NOT_OBSERVED")
    if not any(row.get("fusion_mode") not in (None, "") for row in samples):
        warnings.append("FUSION_STATUS_NOT_OBSERVED")
    if closed_loop and not any(row.get("match_message_count") not in (None, "") for row in samples):
        warnings.append("SCAN_MATCH_QUALITY_NOT_OBSERVED")

    normalized_run_class = str(run_class or "FINAL").strip().upper()
    is_precheck = normalized_run_class == "PRECHECK"
    evidence_level = (
        "SYNTHETIC_SOFTWARE_VALIDATION_PRECHECK" if is_precheck else "SYNTHETIC_SOFTWARE_VALIDATION"
    )
    if safety_cmd_vel_publishers_at_start != 0:
        errors.append(
            f"UNSAFE_REAL_CMD_VEL_PUBLISHERS_AT_START={safety_cmd_vel_publishers_at_start}"
        )

    return {
        **TRUTH_LABELS,
        "scenario_id": scenario.scenario_id,
        "scenario_name": scenario.name,
        "result": "PASS" if not errors else "FAIL",
        "overall_result": "PASS" if not errors else "FAIL",
        "evidence_level": evidence_level,
        "run_class": normalized_run_class,
        "not_final_evidence": is_precheck,
        "not_for_document_claim": is_precheck,
        "run_class_markers": {
            "RUN_CLASS": normalized_run_class,
            "NOT_FINAL_EVIDENCE": "TRUE" if is_precheck else "FALSE",
            "NOT_FOR_DOCUMENT_CLAIM": "TRUE" if is_precheck else "FALSE",
            "EVIDENCE_LEVEL": (
                "SYNTHETIC SOFTWARE VALIDATION PRECHECK"
                if is_precheck
                else "SYNTHETIC SOFTWARE VALIDATION"
            ),
        },
        "gazebo": False,
        "real_robot": False,
        "performance_claim": False,
        "seed": scenario.seed,
        "duration_sec": scenario.duration_sec,
        "sample_count": len(samples),
        "checks": checks,
        "non_discriminating_checks": non_discriminating_checks,
        "failures": errors,
        "errors": errors,
        "warnings": warnings,
        "important_transitions": _important_transitions(samples),
        "field_roles": FIELD_ROLES,
        "ground_truth_disclosure": _ground_truth_disclosure(scenario),
        "real_thresholds_used_unmodified": {
            "min_scan_match_score": REAL_MIN_SCAN_MATCH_SCORE,
            "min_scan_match_inlier_ratio": REAL_MIN_SCAN_MATCH_INLIER_RATIO,
            "max_scan_match_mean_distance": REAL_MAX_SCAN_MATCH_MEAN_DISTANCE,
            "verification_samples": REAL_VERIFICATION_SAMPLES,
            "max_covariance": REAL_MAX_COVARIANCE,
        },
        "match_quality_classification": "SYNTHETIC_MATCH_QUALITY_RESULT",
        "match_quality_note": (
            "match_mean_distance is a scan-to-map residual against synthetic grid "
            "geometry. It is NOT a positioning error in metres and must never be "
            "quoted as localization accuracy."
        ),
        "evaluation_notes": {
            "relocalization_startup_grace_sec": RELOCALIZATION_STARTUP_GRACE_SEC,
            "startup_samples_retained": True,
            "closed_loop_scenario": closed_loop,
        },
    }


def write_result(output_dir: Path, result: dict[str, Any]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "result.json").open("w", encoding="utf-8") as handle:
        json.dump(result, handle, indent=2, ensure_ascii=False, sort_keys=True)
    markers = result.get("run_class_markers", {})
    with (output_dir / "result.md").open("w", encoding="utf-8") as handle:
        handle.write(f"# {result['scenario_id']} — {result['result']}\n\n")
        if str(markers.get("RUN_CLASS", "")).upper() == "PRECHECK":
            handle.write(
                "> **PRECHECK RUN — NOT FINAL EVIDENCE.** "
                "`RUN_CLASS=PRECHECK`, `NOT_FINAL_EVIDENCE=TRUE`, "
                "`NOT_FOR_DOCUMENT_CLAIM=TRUE`. Do not cite this run in a "
                "proposal, report, or presentation.\n\n"
            )
        handle.write("- Authenticity marker: `THIS_IS_SYNTHETIC_SOFTWARE_VALIDATION`\n")
        handle.write("- Validation class: `SYNTHETIC_SOFTWARE_VALIDATION`\n")
        handle.write("- Data class: `NOT_REAL_ROBOT_DATA`\n")
        handle.write("- Simulator class: `NOT_GAZEBO_DATA`\n")
        handle.write("- Performance claim: `NOT_COMPETITION_PERFORMANCE_EVIDENCE` (boolean claim field is false)\n")
        for key in ("RUN_CLASS", "NOT_FINAL_EVIDENCE", "NOT_FOR_DOCUMENT_CLAIM", "EVIDENCE_LEVEL"):
            if key in markers:
                handle.write(f"- {key}: `{markers[key]}`\n")
        handle.write(f"\nSamples: {result['sample_count']}\n\n")
        disclosure = result.get("ground_truth_disclosure", {})
        if disclosure:
            handle.write("## Ground truth disclosure\n\n")
            for key, value in disclosure.items():
                handle.write(f"- `{key}`: {value}\n")
            handle.write(
                "\n`match_mean_distance` is a scan-to-map residual against synthetic "
                "grid geometry, NOT a positioning error in metres.\n\n"
            )
        handle.write("## Checks\n\n")
        for key, value in result["checks"].items():
            handle.write(f"- `{key}`: {value}\n")
        if result["errors"]:
            handle.write("\n## Errors\n\n" + "\n".join(f"- {item}" for item in result["errors"]) + "\n")
        if result["warnings"]:
            handle.write("\n## Warnings\n\n" + "\n".join(f"- {item}" for item in result["warnings"]) + "\n")
        visualization = result.get("visualization", {})
        if visualization:
            handle.write("\n## Visualization\n\n")
            for item in visualization.get("files", []):
                handle.write(f"- PNG: `{item}`\n")
            if visualization.get("warning"):
                handle.write(f"- `{visualization['warning']}`\n")
