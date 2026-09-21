from pathlib import Path

from dg_synthetic_validation.result_writer import (
    FIELD_ROLES,
    SAMPLE_COLUMNS,
    SampleRecorder,
    evaluate_scenario,
)
from dg_synthetic_validation.scenario_schema import load_scenario


def _row(scenario_id: str, elapsed: float, **updates):
    row = {
        "scenario_id": scenario_id,
        "phase": "P",
        "elapsed_time": elapsed,
        "ros_timestamp": elapsed,
        "gnss_state": "GOOD",
        "gnss_decision": "ACCEPT",
        "gnss_accepted": True,
        "lidar_state": "GOOD",
        "geometry_score": 0.8,
        "fusion_mode": "NOMINAL",
        "measurement_confidence": 0.8,
        "position_uncertainty": 0.1,
        "yaw_uncertainty": 0.1,
        "adaptive_position_uncertainty": 0.1,
        "adaptive_yaw_uncertainty": 0.1,
        "navigation_state": "NOMINAL",
        "relocalization_state": "NORMAL",
    }
    row.update(updates)
    return row


def test_timeline_deduplicates_state_changes(tmp_path: Path):
    """Only the first observation and genuine changes are recorded.

    The assertion is written as a property rather than a fixed row count so
    that adding a tracked component does not silently break it: three samples
    of which only ``gnss_state`` changes must yield exactly one STATE_CHANGE.
    """
    scenario = load_scenario(Path(__file__).parents[1] / "config/S01.yaml")
    recorder = SampleRecorder(scenario, tmp_path)
    recorder.add(_row("S01", 0.0))
    recorder.add(_row("S01", 0.1))
    recorder.add(_row("S01", 0.2, gnss_state="DEGRADED"))
    recorder.write_csvs()
    rows = recorder.timeline.rows
    initial = [row for row in rows if row["reason"] == "INITIAL_OBSERVATION"]
    changes = [row for row in rows if row["reason"] == "STATE_CHANGE"]
    assert len(changes) == 1
    assert changes[0]["component"] == "gnss"
    assert changes[0]["previous_state"] == "GOOD"
    assert changes[0]["new_state"] == "DEGRADED"
    # Every tracked component reports exactly one first observation.
    assert len(initial) == len({row["component"] for row in initial})
    assert len(rows) == len(initial) + len(changes)
    assert (tmp_path / "samples.csv").exists()
    assert (tmp_path / "timeline.csv").exists()
    assert (tmp_path / "field_roles.csv").exists()


def test_every_sample_column_has_exactly_one_role():
    assert len(SAMPLE_COLUMNS) == len(set(SAMPLE_COLUMNS))
    assert set(SAMPLE_COLUMNS) == set(FIELD_ROLES)
    assert set(FIELD_ROLES.values()) <= {
        "INPUT", "SUT_OUTPUT", "DERIVED_ASSERTION", "METADATA",
    }


def test_s02_requires_real_transition_evidence():
    scenario = load_scenario(Path(__file__).parents[1] / "config/S02.yaml")
    samples = [
        _row("S02", 1.0),
        _row("S02", 5.0, gnss_state="DEGRADED", gnss_decision="ACCEPT_DEGRADED"),
        _row("S02", 9.0, gnss_state="REJECTED", gnss_decision="REJECT", gnss_accepted=False),
    ]
    result = evaluate_scenario(scenario, samples, safety_cmd_vel_publishers=0, integration_alive=True)
    assert result["result"] == "PASS"
    assert result["validation_class"] == "SYNTHETIC_SOFTWARE_VALIDATION"
    assert result["data_class"] == "NOT_REAL_ROBOT_DATA"


def test_unknown_relocalization_state_fails_the_run():
    """A state outside the ten real names must fail, never be recorded silently."""
    scenario = load_scenario(Path(__file__).parents[1] / "config/S01.yaml")
    samples = [_row("S01", 3.0, relocalization_state="SEARCHING")]
    result = evaluate_scenario(scenario, samples, safety_cmd_vel_publishers=0, integration_alive=True)
    assert result["result"] == "FAIL"
    assert any("UNKNOWN_RELOCALIZATION_STATE" in error for error in result["errors"])


def test_precheck_run_class_is_marked_not_final():
    scenario = load_scenario(Path(__file__).parents[1] / "config/S01.yaml")
    result = evaluate_scenario(
        scenario,
        [_row("S01", 3.0)],
        safety_cmd_vel_publishers=0,
        integration_alive=True,
        run_class="PRECHECK",
    )
    assert result["run_class"] == "PRECHECK"
    assert result["not_final_evidence"] is True
    assert result["not_for_document_claim"] is True
    assert result["run_class_markers"]["NOT_FINAL_EVIDENCE"] == "TRUE"


def test_start_side_cmd_vel_publisher_is_a_failure():
    scenario = load_scenario(Path(__file__).parents[1] / "config/S01.yaml")
    result = evaluate_scenario(
        scenario,
        [_row("S01", 3.0)],
        safety_cmd_vel_publishers=0,
        integration_alive=True,
        safety_cmd_vel_publishers_at_start=1,
    )
    assert result["result"] == "FAIL"
    assert result["checks"]["real_cmd_vel_unpublished_at_start"] is False
