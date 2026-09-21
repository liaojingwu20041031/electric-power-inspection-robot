#!/usr/bin/env python3
"""Create the fixed DG-202611 event/mmWave one-pass test directory."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
from typing import Any, Sequence


SCENARIOS: tuple[dict[str, Any], ...] = (
    {"id": "AUX00_EMPTY", "duration_sec": 60, "repeats": 2, "action": "empty scene, all hardware stationary"},
    {"id": "AUX01_STATIC_RANGE", "duration_sec": 30, "repeats": 3, "action": "stationary target at measured 1 m, 2 m and 3 m"},
    {"id": "AUX02_LATERAL_SLOW", "duration_sec": 60, "repeats": 3, "action": "one target crosses laterally at slow walking speed"},
    {"id": "AUX03_LATERAL_FAST", "duration_sec": 60, "repeats": 3, "action": "one target crosses laterally at fast walking speed"},
    {"id": "AUX04_APPROACH_AWAY", "duration_sec": 60, "repeats": 3, "action": "one target approaches then moves away on measured line"},
    {"id": "AUX05_OCCLUSION", "duration_sec": 60, "repeats": 3, "action": "moving target passes behind one fixed occluder"},
    {"id": "AUX06_MULTI_TARGET", "duration_sec": 60, "repeats": 3, "action": "two targets cross in opposite directions"},
    {"id": "AUX07_LIGHT_CHANGE", "duration_sec": 60, "repeats": 3, "action": "controlled bright-dark transition; targets stationary then moving"},
    {"id": "AUX08_CONTROLLED_YAW", "duration_sec": 60, "repeats": 3, "action": "supervised robot yaw or hand-carried rigid sensor rig yaw"},
)


LOG_COLUMNS = (
    "scenario_id",
    "trial",
    "planned_duration_sec",
    "started_local",
    "ended_local",
    "target_distance_m",
    "target_speed_method",
    "lighting",
    "occlusion",
    "event_raw_path",
    "event_csv_path",
    "mmwave_raw_path",
    "mmwave_csv_path",
    "rosbag_path",
    "status",
    "failure_reason",
    "operator_notes",
)


def _git_revision(repository: Path) -> str | None:
    try:
        return subprocess.check_output(
            ("git", "-C", str(repository), "rev-parse", "HEAD"),
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def create_session(
    output_root: Path,
    *,
    operator: str,
    radar_model: str,
    event_camera_model: str,
    carrier_model: str,
    repository: Path,
    now: datetime | None = None,
) -> Path:
    timestamp = now or datetime.now(timezone.utc)
    session = output_root / timestamp.strftime("%Y%m%d_%H%M%S_AUXILIARY_ONE_PASS")
    session.mkdir(parents=True, exist_ok=False)
    for name in ("00_calibration", "00_protocols", "99_reports"):
        (session / name).mkdir()

    rows: list[dict[str, Any]] = []
    for scenario in SCENARIOS:
        for trial in range(1, int(scenario["repeats"]) + 1):
            trial_name = f"{scenario['id']}_R{trial:02d}"
            trial_dir = session / "scenarios" / trial_name
            for name in ("event_raw", "mmwave_raw", "normalized", "rosbag", "logs", "photos"):
                (trial_dir / name).mkdir(parents=True, exist_ok=True)
            rows.append(
                {
                    "scenario_id": scenario["id"],
                    "trial": trial,
                    "planned_duration_sec": scenario["duration_sec"],
                    "status": "NOT_RUN",
                }
            )

    with (session / "operator_log.csv").open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=LOG_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)

    plan = {
        "schema_version": 1,
        "created_utc": timestamp.isoformat(),
        "operator": operator,
        "hardware": {
            "mmwave_radar_model": radar_model,
            "event_camera_model": event_camera_model,
            "event_camera_carrier": carrier_model,
        },
        "git_commit": _git_revision(repository),
        "motion_commands_generated_by_this_tool": False,
        "scenarios": list(SCENARIOS),
        "required_calibration_files": [
            "00_calibration/sensor_mount_measurements.json",
            "00_calibration/time_sync.txt",
            "00_calibration/layout_photo_or_drawing.txt",
        ],
        "required_protocol_files": [
            "00_protocols/mmwave_vendor_protocol.pdf_or_url.txt",
            "00_protocols/event_camera_versions.txt",
        ],
    }
    encoded = json.dumps(plan, ensure_ascii=False, indent=2) + "\n"
    (session / "session_plan.json").write_text(encoded, encoding="utf-8")
    (session / "session_plan.sha256").write_text(
        hashlib.sha256(encoded.encode("utf-8")).hexdigest() + "  session_plan.json\n",
        encoding="ascii",
    )
    return session


def create_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--operator", default="peng")
    parser.add_argument("--radar-model", default="HU293-exact-suffix-pending")
    parser.add_argument("--event-camera-model", default="Shimeta-EVS-exact-model-pending")
    parser.add_argument("--carrier-model", default="RDK-X5")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    arguments = create_parser().parse_args(argv)
    repository = Path(__file__).resolve().parents[1]
    session = create_session(
        Path(arguments.output_root).expanduser().resolve(),
        operator=arguments.operator,
        radar_model=arguments.radar_model,
        event_camera_model=arguments.event_camera_model,
        carrier_model=arguments.carrier_model,
        repository=repository,
    )
    print(f"AUXILIARY_SESSION_READY={session}")
    print("No motion command was sent.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
