import csv
import hashlib
import json
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = REPO_ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from real_robot_capture import build_manifest, write_json  # noqa: E402
from teacher_requirements_evaluator import (  # noqa: E402
    BEIDOU_REQUIREMENT,
    FEATURE_REQUIREMENT,
    POSITION_REQUIREMENT,
    RELOCALIZATION_REQUIREMENT,
    evaluate_all,
)


def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def create_session(root: Path, name: str, protocol: dict, structured_files=None, extra_files=None):
    session = root / name
    structured = session / "structured"
    structured.mkdir(parents=True)
    metadata = {
        "run_class": "REAL_ROBOT_CAPTURE",
        "teacher_protocol": protocol,
        "evaluation_capture": {},
    }
    if protocol["requirement_id"] == POSITION_REQUIREMENT:
        metadata["evaluation_capture"] = {
            "ground_truth_enabled": True,
            "ground_truth_z": 0.50,
            "ground_truth_frame": "map",
        }
    write_json(session / "metadata.json", metadata)
    write_json(
        session / "result.json",
        {
            "run_class": "REAL_ROBOT_CAPTURE",
            "capture_status": "COMPLETE",
            "experiment_id": name,
            "rosbag_present": True,
            "rosbag_metadata_present": True,
        },
    )
    write_json(session / "teacher_protocol.json", protocol)
    bag_metadata = session / "rosbag" / "metadata.yaml"
    bag_metadata.parent.mkdir(parents=True)
    bag_metadata.write_text("rosbag2_bagfile_information: {}\n", encoding="utf-8")
    for relative, fieldnames, rows in structured_files or []:
        write_csv(session / relative, fieldnames, rows)
    for relative, content in extra_files or []:
        path = session / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
    entries = build_manifest(session)
    write_json(session / "manifest.json", {"files": entries})
    return session


def base_protocol(requirement_id: str) -> dict:
    return {
        "schema_version": 1,
        "requirement_id": requirement_id,
        "evidence_source": "REAL_HARDWARE",
        "operator_observed": True,
    }


def test_empty_capture_root_never_claims_a_requirement_pass(tmp_path):
    summary = evaluate_all(tmp_path)
    assert summary["overall_status"] == "NOT_COMPLETE"
    assert {item["status"] for item in summary["requirements"]} == {"NOT_PROVEN"}


def test_unlisted_critical_file_is_rejected_even_when_remaining_hashes_match(tmp_path):
    protocol = base_protocol(POSITION_REQUIREMENT)
    protocol.update(
        {
            "conditions": {
                "gnss_bds_interruption": True,
                "lidar_degradation": True,
                "concurrent": True,
            },
            "ground_truth": {
                "independent_xy": True,
                "independent_z": True,
                "method": "surveyed point",
                "coordinate_frame": "map",
            },
        }
    )
    diagnostic = {
        "received_timestamp": 100.0,
        "message_timestamp": 100.0,
        "topic": "/dg/navigation/status",
        "status_name": "DG Navigation Health",
        "level": 1,
        "message": "DEGRADED",
        "values_json": json.dumps(
            {"timestamp": 100.0, "gnss_state": "REJECTED", "lidar_state": "DEGRADED"}
        ),
    }
    position = {
        "timestamp": 100.0,
        "estimated_x": 1.0,
        "estimated_y": 2.0,
        "estimated_z": 0.5,
        "ground_truth_x": 1.0,
        "ground_truth_y": 2.0,
        "ground_truth_z": 0.5,
        "coordinate_frame": "map",
        "source_topic": "/dg/fusion/pose",
    }
    session = create_session(
        tmp_path,
        "unlisted-position",
        protocol,
        [
            ("structured/diagnostic_samples.csv", list(diagnostic), [diagnostic]),
            ("structured/position_samples.csv", list(position), [position]),
        ],
    )
    manifest_path = session / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["files"] = [
        item for item in manifest["files"]
        if item["path"] != "structured/position_samples.csv"
    ]
    write_json(manifest_path, manifest)

    summary = evaluate_all(tmp_path, minimum_position_samples=1)
    position_result = next(
        item for item in summary["requirements"]
        if item["requirement_id"] == POSITION_REQUIREMENT
    )
    assert position_result["status"] == "INCOMPLETE_EVIDENCE"
    assert any("未纳入 SHA256 清单" in reason for reason in position_result["reasons"])


def test_complete_traceable_sessions_can_pass_all_four_requirements(tmp_path):
    protocol = base_protocol(POSITION_REQUIREMENT)
    protocol.update(
        {
            "conditions": {
                "gnss_bds_interruption": True,
                "lidar_degradation": True,
                "concurrent": True,
            },
            "ground_truth": {
                "independent_xy": True,
                "independent_z": True,
                "method": "laser tracker",
                "coordinate_frame": "map",
            },
        }
    )
    diagnostic_rows = []
    position_rows = []
    for index in range(30):
        timestamp = 100.0 + index * 0.1
        diagnostic_rows.append(
            {
                "received_timestamp": timestamp,
                "message_timestamp": timestamp,
                "topic": "/dg/navigation/status",
                "status_name": "DG Navigation Health",
                "level": 1,
                "message": "DEGRADED",
                "values_json": json.dumps(
                    {
                        "timestamp": timestamp,
                        "gnss_state": "REJECTED",
                        "lidar_state": "DEGRADED",
                    }
                ),
            }
        )
        position_rows.append(
            {
                "timestamp": timestamp,
                "estimated_x": 1.05,
                "estimated_y": 2.00,
                "estimated_z": 0.54,
                "ground_truth_x": 1.00,
                "ground_truth_y": 2.00,
                "ground_truth_z": 0.50,
                "coordinate_frame": "map",
                "source_topic": "/dg/fusion/pose",
            }
        )
    create_session(
        tmp_path,
        "position",
        protocol,
        [
            (
                "structured/diagnostic_samples.csv",
                list(diagnostic_rows[0]),
                diagnostic_rows,
            ),
            (
                "structured/position_samples.csv",
                list(position_rows[0]),
                position_rows,
            ),
        ],
    )

    protocol = base_protocol(FEATURE_REQUIREMENT)
    protocol["conditions"] = {
        "robot_stationary": True,
        "dynamic_target_present": True,
        "dynamic_filter_enabled": True,
    }
    feature_rows = [
        {
            "frame_id": index,
            "timestamp": 200.0 + index * 0.1,
            "coordinate_frame": "laser_link",
            "feature_points": json.dumps([[1.0, 0.0], [2.0, 1.0]]),
            "valid_feature_count": 2,
        }
        for index in range(31)
    ]
    quality_rows = [
        {"dynamic_candidate_count": 5, "auxiliary_dynamic_point_count": 2}
    ]
    create_session(
        tmp_path,
        "features",
        protocol,
        [
            ("structured/lidar_features.csv", list(feature_rows[0]), feature_rows),
            ("structured/lidar_quality.csv", list(quality_rows[0]), quality_rows),
        ],
    )

    relocalization_specs = (
        ("relocal_violent", "VIOLENT_MOTION", 7),
        ("relocal_domain", "CROSS_DOMAIN", 7),
        ("relocal_perception", "PERCEPTION_DEGRADATION", 6),
    )
    attempt = 0
    for name, trigger, count in relocalization_specs:
        protocol = base_protocol(RELOCALIZATION_REQUIREMENT)
        protocol["conditions"] = {
            "controlled_real_trial": True,
            "operator_supervised": True,
            "trigger_class": trigger,
        }
        rows = []
        for _ in range(count):
            attempt += 1
            rows.append(
                {
                    "timestamp": 300.0 + attempt,
                    "attempt_id": attempt,
                    "outcome": "RECOVERED",
                    "time_to_recovery": 3.0,
                    "failure_reason": "",
                }
            )
        create_session(
            tmp_path,
            name,
            protocol,
            [("structured/relocalization_events.csv", list(rows[0]), rows)],
        )

    protocol = base_protocol(BEIDOU_REQUIREMENT)
    protocol.update(
        {
            "terminal_model": "REAL-BDS-SM-01",
            "vendor_protocol": "vendor-v1",
            "raw_robot_log": "raw/beidou_robot_terminal.log",
            "raw_remote_log": "raw/beidou_remote_terminal.log",
            "conditions": {
                "real_short_message_terminal": True,
                "satellite_transport": True,
                "service_active": True,
                "remote_endpoint_identified": True,
            },
        }
    )
    digest = hashlib.sha256(b"payload").hexdigest()
    events = [
        {
            "timestamp": 400.0 + index,
            "test_id": "cycle-1",
            "event": event,
            "message_id": f"message-{index}",
            "peer": "peer",
            "transport_reference": f"transport-{index}",
            "payload_sha256": digest,
        }
        for index, event in enumerate(
            (
                "ROBOT_SEND_ACCEPTED",
                "REMOTE_RECEIVED",
                "REMOTE_REPLY_SENT",
                "ROBOT_REPLY_RECEIVED",
            )
        )
    ]
    create_session(
        tmp_path,
        "beidou",
        protocol,
        [("structured/beidou_short_message_events.csv", list(events[0]), events)],
        [
            ("raw/beidou_robot_terminal.log", "real robot log\n"),
            ("raw/beidou_remote_terminal.log", "real remote log\n"),
        ],
    )

    summary = evaluate_all(tmp_path)
    assert summary["overall_status"] == "PASS"
    assert {item["status"] for item in summary["requirements"]} == {"PASS"}
