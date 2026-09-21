import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import sys


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = REPOSITORY_ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from create_auxiliary_test_session import LOG_COLUMNS, SCENARIOS, create_session  # noqa: E402


def test_session_contains_every_frozen_trial_and_required_record(tmp_path: Path) -> None:
    session = create_session(
        tmp_path,
        operator="peng",
        radar_model="HU293",
        event_camera_model="EVS",
        carrier_model="RDK-X5",
        repository=REPOSITORY_ROOT,
        now=datetime(2026, 9, 11, 8, 0, tzinfo=timezone.utc),
    )
    plan = json.loads((session / "session_plan.json").read_text(encoding="utf-8"))
    assert plan["motion_commands_generated_by_this_tool"] is False
    assert plan["hardware"]["event_camera_carrier"] == "RDK-X5"
    expected_trials = sum(int(scenario["repeats"]) for scenario in SCENARIOS)
    with (session / "operator_log.csv").open(encoding="utf-8-sig", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == expected_trials
    assert tuple(rows[0]) == LOG_COLUMNS
    assert all(row["status"] == "NOT_RUN" for row in rows)
    assert (session / "scenarios" / "AUX00_EMPTY_R01" / "event_raw").is_dir()
    assert (session / "scenarios" / "AUX08_CONTROLLED_YAW_R03" / "rosbag").is_dir()
    assert len((session / "session_plan.sha256").read_text().split()[0]) == 64
