#!/usr/bin/env python3
"""Regression checks for the reproducible measured-UWB evidence analysis."""

from __future__ import annotations

import sys
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
ROOT_SCRIPTS = REPOSITORY_ROOT / "scripts"
if str(ROOT_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(ROOT_SCRIPTS))

from analyze_measured_uwb import analyze_scenario


DATA_DIRECTORY = (
    REPOSITORY_ROOT / "src" / "ylhb_base" / "data" / "uwb" / "20260905"
)


def test_measured_replay_reproduces_recorded_gate_counts() -> None:
    expected = {
        "static": {
            "file": "static_raw_20260905_165208.csv",
            "frames": 3000,
            "decisions": {
                "ACCEPT": 2949,
                "ACCEPT_DEGRADED": 0,
                "HOLD_FOR_RECOVERY": 30,
                "REJECT": 21,
            },
            "states": {
                "GOOD": 2949,
                "DEGRADED": 0,
                "RECOVERING": 30,
                "REJECTED": 21,
            },
        },
        "move": {
            "file": "move_raw_20260905_170425.csv",
            "frames": 3000,
            "decisions": {
                "ACCEPT": 2869,
                "ACCEPT_DEGRADED": 1,
                "HOLD_FOR_RECOVERY": 73,
                "REJECT": 57,
            },
            "states": {
                "GOOD": 2869,
                "DEGRADED": 1,
                "RECOVERING": 73,
                "REJECTED": 57,
            },
        },
        "anchor_fault": {
            "file": "anchor_fault.csv",
            "frames": 1000,
            "decisions": {
                "ACCEPT": 850,
                "ACCEPT_DEGRADED": 0,
                "HOLD_FOR_RECOVERY": 4,
                "REJECT": 146,
            },
            "states": {
                "GOOD": 850,
                "DEGRADED": 0,
                "RECOVERING": 4,
                "REJECTED": 146,
            },
        },
    }

    for scenario, contract in expected.items():
        rows, summary = analyze_scenario(
            DATA_DIRECTORY / contract["file"],
            scenario=scenario,
            replay_rate_hz=25.0,
        )
        assert summary["frame_count"] == contract["frames"]
        assert summary["decision_counts"] == contract["decisions"]
        assert summary["state_counts"] == contract["states"]
        assert len(rows) == contract["frames"]
        assert all(row["logical_slot_count"] == 6 for row in rows)
        assert all(row["unique_physical_count"] <= 3 for row in rows)


def test_anchor_fault_counts_real_sources_without_logical_duplication() -> None:
    rows, summary = analyze_scenario(
        DATA_DIRECTORY / "anchor_fault.csv",
        scenario="anchor_fault",
        replay_rate_hz=25.0,
    )

    assert summary["valid_physical_count_distribution"] == {"2": 143, "3": 857}
    assert summary["rejection_reason_classes"]["INSUFFICIENT_ANCHORS"] == 143
    assert sum(row["unique_physical_count"] == 2 for row in rows) == 143
    assert not any(row["unique_physical_count"] > 3 for row in rows)


def test_summary_does_not_turn_internal_consistency_into_accuracy() -> None:
    _, summary = analyze_scenario(
        DATA_DIRECTORY / "static_raw_20260905_165208.csv",
        scenario="static",
        replay_rate_hz=25.0,
    )

    assert summary["independent_ground_truth_available"] is False
    assert summary["host_timestamps_available"] is False
    assert summary["claims_supported"]["absolute_position_accuracy"] is False
    assert summary["claims_supported"]["end_to_end_latency"] is False
    assert summary["claims_supported"]["quality_gate_behavior"] is True
