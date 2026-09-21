#!/usr/bin/env python3
"""Verify and summarize one or more DG-202611 real-robot capture sessions.

Derived reports are written outside the immutable capture directory.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
from typing import Any, Sequence

from feature_repeatability_evaluator import compute_metrics as compute_feature_metrics
from feature_repeatability_evaluator import read_csv as read_feature_csv
from position_error_evaluator import compute_metrics as compute_position_metrics
from position_error_evaluator import compute_vertical_metrics
from position_error_evaluator import read_csv as read_position_csv
from relocalization_evaluator import compute_metrics as compute_relocalization_metrics
from relocalization_evaluator import read_csv as read_relocalization_csv


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_json(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected a JSON object: {path}")
    return payload


def verify_manifest(session: Path) -> tuple[bool, list[str]]:
    manifest_path = session / "manifest.json"
    if not manifest_path.is_file():
        return False, ["manifest.json is missing"]
    try:
        entries = read_json(manifest_path).get("files", [])
    except (OSError, ValueError, json.JSONDecodeError) as error:
        return False, [f"manifest unreadable: {error}"]
    errors: list[str] = []
    root = session.resolve()
    for entry in entries:
        relative = str(entry.get("path", ""))
        candidate = (session / relative).resolve()
        if root not in candidate.parents:
            errors.append(f"unsafe manifest path: {relative}")
            continue
        if not candidate.is_file():
            errors.append(f"missing: {relative}")
            continue
        expected = str(entry.get("sha256", "")).lower()
        actual = sha256_file(candidate)
        if actual != expected:
            errors.append(f"hash mismatch: {relative}")
    return not errors, errors


def discover_sessions(root: Path) -> list[Path]:
    if (root / "result.json").is_file():
        return [root]
    return sorted({path.parent for path in root.rglob("result.json")})


def write_csv(path: Path, fieldnames: Sequence[str], rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def evaluate(root: Path, output_dir: Path, match_distance: float) -> dict[str, Any]:
    sessions = discover_sessions(root)
    if not sessions:
        raise ValueError(f"no capture sessions found below {root}")
    output_dir.mkdir(parents=True, exist_ok=False)

    session_rows: list[dict[str, Any]] = []
    feature_results: list[dict[str, Any]] = []
    all_position_samples = []
    all_relocalization_events = []
    position_rows: list[dict[str, Any]] = []
    relocalization_rows: list[dict[str, Any]] = []

    for session in sessions:
        result = read_json(session / "result.json")
        metadata = read_json(session / "metadata.json")
        manifest_valid, manifest_errors = verify_manifest(session)
        structured = session / "structured"
        experiment_id = str(result.get("experiment_id") or session.name)
        scenario = str(metadata.get("scenario") or "")

        feature_metrics = None
        feature_path = structured / "lidar_features.csv"
        if feature_path.is_file():
            frames = read_feature_csv(str(feature_path))
            feature_metrics = compute_feature_metrics(frames, match_distance)
            if feature_metrics is not None:
                feature_results.append(
                    {
                        "experiment_id": experiment_id,
                        "scenario": scenario,
                        "protocol_review_required": True,
                        **asdict(feature_metrics),
                    }
                )

        position_path = structured / "position_samples.csv"
        position_count = 0
        if position_path.is_file():
            samples = read_position_csv(str(position_path))
            position_count = len(samples)
            all_position_samples.extend(samples)
            for sample in samples:
                position_rows.append({"experiment_id": experiment_id, **asdict(sample)})

        relocalization_path = structured / "relocalization_events.csv"
        relocalization_count = 0
        if relocalization_path.is_file():
            events = read_relocalization_csv(str(relocalization_path))
            relocalization_count = len(events)
            all_relocalization_events.extend(events)
            for event in events:
                relocalization_rows.append({"experiment_id": experiment_id, **asdict(event)})

        session_rows.append(
            {
                "experiment_id": experiment_id,
                "scenario": scenario,
                "capture_status": result.get("capture_status"),
                "manifest_valid": manifest_valid,
                "manifest_errors": ";".join(manifest_errors),
                "position_sample_count": position_count,
                "relocalization_event_count": relocalization_count,
                "lidar_frame_count": 0 if feature_metrics is None else feature_metrics.total_frames,
                "lidar_r_pool": "" if feature_metrics is None else feature_metrics.overall_repeatability,
            }
        )

    position_metrics = compute_position_metrics(all_position_samples)
    vertical_metrics = compute_vertical_metrics(all_position_samples)
    relocalization_metrics = compute_relocalization_metrics(all_relocalization_events)
    summary = {
        "schema_version": 1,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "source_root": str(root.resolve()),
        "output_dir": str(output_dir.resolve()),
        "session_count": len(sessions),
        "complete_capture_count": sum(row["capture_status"] == "COMPLETE" for row in session_rows),
        "manifest_valid_count": sum(bool(row["manifest_valid"]) for row in session_rows),
        "position_metrics": None if position_metrics is None else asdict(position_metrics),
        "vertical_position_metrics": None if vertical_metrics is None else asdict(vertical_metrics),
        "relocalization_metrics": (
            None if relocalization_metrics is None else asdict(relocalization_metrics)
        ),
        "feature_metrics_by_session": feature_results,
        "claim_boundary": {
            "capture_complete_is_not_performance_pass": True,
            "feature_repeatability_requires_stationary_protocol_review": True,
            "position_error_requires_independent_coordinates_in_the_same_frame": True,
            "robot_runtime_results_must_not_be_filled_from_empty_csv_files": True,
        },
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    write_csv(
        output_dir / "sessions.csv",
        (
            "experiment_id",
            "scenario",
            "capture_status",
            "manifest_valid",
            "manifest_errors",
            "position_sample_count",
            "relocalization_event_count",
            "lidar_frame_count",
            "lidar_r_pool",
        ),
        session_rows,
    )
    write_csv(
        output_dir / "position_samples_aggregate.csv",
        (
            "experiment_id",
            "timestamp",
            "estimated_x",
            "estimated_y",
            "estimated_z",
            "ground_truth_x",
            "ground_truth_y",
            "ground_truth_z",
        ),
        position_rows,
    )
    write_csv(
        output_dir / "relocalization_events_aggregate.csv",
        (
            "experiment_id",
            "timestamp",
            "attempt_id",
            "outcome",
            "time_to_recovery",
            "failure_reason",
        ),
        relocalization_rows,
    )
    return summary


def create_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture_root")
    parser.add_argument("--output-dir")
    parser.add_argument("--match-distance", type=float, default=0.12)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    arguments = create_parser().parse_args(argv)
    root = Path(arguments.capture_root).expanduser().resolve()
    output_dir = (
        Path(arguments.output_dir).expanduser().resolve()
        if arguments.output_dir
        else root.parent / f"{root.name}_evaluation_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )
    if output_dir == root or root in output_dir.parents:
        print("Error: output directory must be outside the immutable capture root", file=sys.stderr)
        return 2
    try:
        summary = evaluate(root, output_dir, arguments.match_distance)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    print(output_dir)
    if summary["complete_capture_count"] != summary["session_count"]:
        print("Warning: one or more sessions are incomplete", file=sys.stderr)
    if summary["manifest_valid_count"] != summary["session_count"]:
        print("Warning: one or more session manifests failed verification", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
