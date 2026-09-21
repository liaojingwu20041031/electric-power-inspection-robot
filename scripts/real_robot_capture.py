#!/usr/bin/env python3
"""Capture traceable DG-202611 real-robot evidence without commanding motion."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import re
import shutil
import signal
import subprocess
import sys
import time
from typing import Iterable, Sequence


DEFAULT_TOPICS = (
    "/inspection_ai/system_status",
    "/patrol/status",
    "/patrol/event",
    "/scan",
    "/imu/data",
    "/odom",
    "/odometry/filtered",
    "/amcl_pose",
    "/map",
    "/tf",
    "/tf_static",
    "/cmd_vel",
    "/zlac8015d/status",
    "/dg/lidar/quality",
    "/dg/lidar/filtered_scan",
    "/dg/lidar/stable_features",
    "/dg/gnss/quality",
    "/gps/fix",
    "/gps/nmea_sentence",
    "/gps/rtk_status",
    "/dg/uwb/pose",
    "/dg/uwb/quality",
    "/dg/fusion/odom",
    "/dg/fusion/pose",
    "/dg/fusion/status",
    "/zed/zed_node/odom",
    "/dg/event_camera/events",
    "/dg/event_camera/quality",
    "/dg/event_camera/features",
    "/dg/event_camera/feature_quality",
    "/dg/mmwave/points",
    "/dg/mmwave/quality",
    "/dg/navigation/status",
    "/dg/relocalization/seed",
    "/dg/relocalization/match_quality",
    "/dg/relocalization/status",
    "/cmd_vel_recovery",
    "/cmd_vel_nav",
    "/dg/test_cmd_vel",
)


SNAPSHOT_COMMANDS = {
    "hostname.txt": ("hostname",),
    "uname.txt": ("uname", "-a"),
    "ip_addr.txt": ("ip", "-4", "-br", "addr"),
    "ros_nodes.txt": ("ros2", "node", "list"),
    "ros_topics.txt": ("ros2", "topic", "list", "-t"),
    "ros_services.txt": ("ros2", "service", "list", "-t"),
    "ros_actions.txt": ("ros2", "action", "list", "-t"),
    "tf_static_once.txt": ("ros2", "topic", "echo", "/tf_static", "--once"),
    "can1.txt": ("ip", "-details", "-statistics", "link", "show", "can1"),
    "mobile_bridge_service.txt": (
        "systemctl",
        "status",
        "ylhb-mobile-bridge.service",
        "--no-pager",
    ),
    "dg_models_service.txt": (
        "systemctl",
        "status",
        "dg202611-models.service",
        "--no-pager",
    ),
}

PARAMETER_NODES = (
    "/zlac8015d_canopen_controller",
    "/ekf_filter_node",
    "/mobile_bridge",
    "/dg_lidar_robust_node",
    "/dg_multisource_fusion_node",
    "/dg_active_relocalization_node",
    "/dg_navigation_health_node",
    "/dg_uwb_replay_node",
    "/dg_event_feature_node",
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sanitize_label(value: str) -> str:
    label = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value).strip())
    label = label.strip("._-")
    if not label:
        raise ValueError("scenario must contain at least one safe filename character")
    return label[:80]


def unique_topics(values: Iterable[str]) -> tuple[str, ...]:
    ordered: list[str] = []
    seen: set[str] = set()
    for value in values:
        topic = str(value).strip()
        if not topic or not topic.startswith("/"):
            raise ValueError(f"invalid absolute ROS topic: {value!r}")
        if topic not in seen:
            seen.add(topic)
            ordered.append(topic)
    return tuple(ordered)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json(path: Path, payload: object) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


TEACHER_REQUIREMENT_IDS = {
    "POSITION_CONCURRENT_DEGRADATION",
    "DYNAMIC_FEATURE_REPEATABILITY",
    "RELOCALIZATION_ROBUSTNESS",
    "BEIDOU_SHORT_MESSAGE_BIDIRECTIONAL",
}


def load_teacher_protocol(path: str | Path) -> dict[str, object]:
    """Load the protocol frozen before a teacher-requirement capture."""

    protocol_path = Path(path).expanduser().resolve()
    payload = json.loads(protocol_path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("teacher protocol must be a JSON object")
    if payload.get("schema_version") != 1:
        raise ValueError("teacher protocol schema_version must be 1")
    requirement_id = str(payload.get("requirement_id", "")).strip()
    if requirement_id not in TEACHER_REQUIREMENT_IDS:
        raise ValueError(f"unsupported teacher requirement_id: {requirement_id!r}")
    if payload.get("evidence_source") != "REAL_HARDWARE":
        raise ValueError("teacher protocol evidence_source must be REAL_HARDWARE")
    if payload.get("operator_observed") is not True:
        raise ValueError("teacher protocol operator_observed must be true")
    return payload


def git_snapshot(repository: Path) -> dict[str, object]:
    def git(*arguments: str) -> str | None:
        try:
            return subprocess.check_output(
                ("git", *arguments), cwd=repository, stderr=subprocess.DEVNULL, text=True
            ).strip()
        except (OSError, subprocess.CalledProcessError):
            return None

    status = git("status", "--porcelain=v1")
    return {
        "commit": git("rev-parse", "HEAD"),
        "branch": git("branch", "--show-current"),
        "status_porcelain": status,
        "clean": status == "" if status is not None else None,
    }


def run_snapshot(command: Sequence[str], destination: Path, timeout: float = 15.0) -> int:
    try:
        completed = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout,
            check=False,
        )
        destination.write_text(completed.stdout or "", encoding="utf-8")
        return int(completed.returncode)
    except (OSError, subprocess.TimeoutExpired) as error:
        destination.write_text(f"SNAPSHOT_FAILED: {type(error).__name__}: {error}\n", encoding="utf-8")
        return 124 if isinstance(error, subprocess.TimeoutExpired) else 127


def read_proc_sample(previous_cpu: tuple[int, int] | None) -> tuple[dict[str, object], tuple[int, int] | None]:
    sample: dict[str, object] = {
        "utc": utc_now(),
        "monotonic_sec": time.monotonic(),
        "load1": None,
        "load5": None,
        "load15": None,
        "cpu_percent": None,
        "memory_total_kib": None,
        "memory_available_kib": None,
        "thermal_max_c": None,
    }
    try:
        sample["load1"], sample["load5"], sample["load15"] = os.getloadavg()
    except (AttributeError, OSError):
        pass

    current_cpu: tuple[int, int] | None = None
    try:
        fields = Path("/proc/stat").read_text(encoding="ascii").splitlines()[0].split()[1:]
        counters = [int(value) for value in fields]
        total = sum(counters)
        idle = counters[3] + (counters[4] if len(counters) > 4 else 0)
        current_cpu = (total, idle)
        if previous_cpu is not None:
            total_delta = total - previous_cpu[0]
            idle_delta = idle - previous_cpu[1]
            if total_delta > 0:
                sample["cpu_percent"] = 100.0 * (1.0 - idle_delta / total_delta)
    except (OSError, ValueError, IndexError):
        pass

    try:
        meminfo: dict[str, int] = {}
        for line in Path("/proc/meminfo").read_text(encoding="ascii").splitlines():
            key, value = line.split(":", 1)
            meminfo[key] = int(value.strip().split()[0])
        sample["memory_total_kib"] = meminfo.get("MemTotal")
        sample["memory_available_kib"] = meminfo.get("MemAvailable")
    except (OSError, ValueError, IndexError):
        pass

    temperatures: list[float] = []
    for sensor in Path("/sys/class/thermal").glob("thermal_zone*/temp"):
        try:
            value = float(sensor.read_text(encoding="ascii").strip())
            temperatures.append(value / 1000.0 if value > 200.0 else value)
        except (OSError, ValueError):
            continue
    if temperatures:
        sample["thermal_max_c"] = max(temperatures)
    return sample, current_cpu


def terminate_process(process: subprocess.Popen[str] | None, timeout: float = 15.0) -> int | None:
    if process is None:
        return None
    if process.poll() is not None:
        return process.returncode
    try:
        os.killpg(process.pid, signal.SIGINT)
    except (AttributeError, OSError):
        process.send_signal(signal.SIGINT)
    try:
        return process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            return process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.kill()
            return process.wait(timeout=5.0)


def wait_for_ready_file(
    path: Path, process: subprocess.Popen[str], timeout: float = 10.0
) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if path.is_file():
            return True
        if process.poll() is not None:
            return False
        time.sleep(0.1)
    return path.is_file()


def build_manifest(session: Path) -> list[dict[str, object]]:
    entries: list[dict[str, object]] = []
    for path in sorted(session.rglob("*")):
        if not path.is_file() or path.name in {"manifest.json", "manifest.sha256"}:
            continue
        entries.append(
            {
                "path": path.relative_to(session).as_posix(),
                "bytes": path.stat().st_size,
                "sha256": sha256_file(path),
            }
        )
    return entries


def create_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Record DG-202611 real-robot evidence. This tool never publishes commands."
    )
    parser.add_argument("--scenario", required=True, help="Frozen scenario or trial identifier")
    parser.add_argument("--duration", type=float, default=120.0, help="Seconds; 0 records until Ctrl-C")
    parser.add_argument("--output-root", default="results/real_robot")
    parser.add_argument("--operator", default=os.environ.get("USER", "unknown"))
    parser.add_argument("--hardware-notes", default="")
    parser.add_argument("--sample-period", type=float, default=1.0)
    parser.add_argument("--wait-for-patrol-start", action="store_true")
    parser.add_argument("--stop-on-patrol-terminal", action="store_true")
    parser.add_argument("--post-roll-sec", type=float, default=5.0)
    parser.add_argument("--startup-timeout-sec", type=float, default=60.0)
    parser.add_argument("--topic", action="append", default=[], help="Additional absolute ROS topic")
    parser.add_argument("--estimated-pose-topic", default="/dg/fusion/pose")
    parser.add_argument("--ground-truth-x", type=float)
    parser.add_argument("--ground-truth-y", type=float)
    parser.add_argument("--ground-truth-z", type=float)
    parser.add_argument("--ground-truth-frame", default="map")
    parser.add_argument(
        "--teacher-protocol",
        help="JSON protocol frozen before capture for one teacher requirement",
    )
    parser.add_argument("--repo", default=str(Path(__file__).resolve().parents[1]))
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    arguments = create_parser().parse_args(argv)
    if arguments.duration < 0.0:
        raise SystemExit("--duration must be non-negative")
    if arguments.sample_period <= 0.0:
        raise SystemExit("--sample-period must be positive")
    if arguments.post_roll_sec < 0.0:
        raise SystemExit("--post-roll-sec must be non-negative")
    if arguments.startup_timeout_sec <= 0.0:
        raise SystemExit("--startup-timeout-sec must be positive")
    ground_truth_supplied = (
        arguments.ground_truth_x is not None,
        arguments.ground_truth_y is not None,
    )
    if ground_truth_supplied[0] != ground_truth_supplied[1]:
        raise SystemExit("--ground-truth-x and --ground-truth-y must be supplied together")
    if arguments.ground_truth_z is not None and not all(ground_truth_supplied):
        raise SystemExit("--ground-truth-z requires --ground-truth-x and --ground-truth-y")
    if not arguments.estimated_pose_topic.startswith("/"):
        raise SystemExit("--estimated-pose-topic must be an absolute ROS topic")
    if not arguments.ground_truth_frame.strip():
        raise SystemExit("--ground-truth-frame must not be empty")
    ground_truth_values = [arguments.ground_truth_x, arguments.ground_truth_y]
    if arguments.ground_truth_z is not None:
        ground_truth_values.append(arguments.ground_truth_z)
    if all(ground_truth_supplied) and not all(math.isfinite(value) for value in ground_truth_values):
        raise SystemExit("ground-truth coordinates must be finite")
    teacher_protocol = None
    teacher_protocol_source = None
    if arguments.teacher_protocol:
        try:
            protocol_path = Path(arguments.teacher_protocol).expanduser().resolve()
            teacher_protocol = load_teacher_protocol(protocol_path)
            teacher_protocol_source = {
                "source_filename": protocol_path.name,
                "source_sha256": sha256_file(protocol_path),
            }
        except (OSError, ValueError, json.JSONDecodeError) as error:
            print(f"ERROR: invalid teacher protocol: {error}", file=sys.stderr)
            return 2

    if shutil.which("ros2") is None:
        print("ERROR: ros2 is unavailable; source the ROS 2 and integration workspaces first.", file=sys.stderr)
        return 2

    scenario = sanitize_label(arguments.scenario)
    topics = unique_topics((*DEFAULT_TOPICS, *arguments.topic))
    repository = Path(arguments.repo).expanduser().resolve()
    output_root = Path(arguments.output_root).expanduser().resolve()
    session = output_root / f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_{scenario}"
    logs = session / "logs"
    snapshots = session / "snapshots"
    logs.mkdir(parents=True, exist_ok=False)
    snapshots.mkdir(parents=True)

    created_utc = utc_now()
    metadata = {
        "schema_version": 1,
        "run_class": "REAL_ROBOT_CAPTURE",
        "evaluation_status": "NOT_EVALUATED",
        "experiment_id": session.name,
        "scenario": scenario,
        "operator": arguments.operator,
        "hardware_notes": arguments.hardware_notes,
        "created_utc": created_utc,
        "started_utc": None,
        "requested_duration_sec": arguments.duration,
        "sample_period_sec": arguments.sample_period,
        "patrol_lifecycle": {
            "wait_for_start": arguments.wait_for_patrol_start,
            "stop_on_terminal": arguments.stop_on_patrol_terminal,
            "post_roll_sec": arguments.post_roll_sec,
            "startup_timeout_sec": arguments.startup_timeout_sec,
        },
        "hostname": platform.node(),
        "platform": platform.platform(),
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION"),
        "repository": str(repository),
        "git": git_snapshot(repository),
        "topics": list(topics),
        "evaluation_capture": {
            "estimated_pose_topic": arguments.estimated_pose_topic,
            "ground_truth_enabled": all(ground_truth_supplied),
            "ground_truth_x": arguments.ground_truth_x,
            "ground_truth_y": arguments.ground_truth_y,
            "ground_truth_z": arguments.ground_truth_z,
            "ground_truth_frame": arguments.ground_truth_frame,
        },
        "teacher_protocol": teacher_protocol,
        "teacher_protocol_source": teacher_protocol_source,
        "safety": {
            "publishes_motion_commands": False,
            "requires_human_supervision_for_motion": True,
            "motion_is_operator_controlled": True,
        },
    }
    write_json(session / "metadata.json", metadata)
    if teacher_protocol is not None:
        write_json(session / "teacher_protocol.json", teacher_protocol)

    snapshot_results: dict[str, int] = {}
    for filename, command in SNAPSHOT_COMMANDS.items():
        snapshot_results[filename] = run_snapshot(command, snapshots / filename)
    parameter_dir = snapshots / "parameters"
    parameter_dir.mkdir()
    for node in PARAMETER_NODES:
        filename = f"{node.strip('/').replace('/', '_')}.yaml"
        snapshot_results[f"parameters/{filename}"] = run_snapshot(
            ("ros2", "param", "dump", node), parameter_dir / filename, timeout=10.0
        )

    structured_dir = session / "structured"
    structured_dir.mkdir()
    evidence_ready_file = structured_dir / "READY.json"
    evidence_start_file = structured_dir / "START.json"
    patrol_start_file = structured_dir / "PATROL_STARTED.json"
    patrol_terminal_file = structured_dir / "PATROL_TERMINAL.json"
    evidence_script = repository / "src" / "ylhb_base" / "scripts" / "evidence_capture_node.py"
    if not evidence_script.is_file():
        raise RuntimeError(f"structured evidence node is missing: {evidence_script}")
    evidence_command = [
        sys.executable,
        str(evidence_script),
        "--output-dir",
        str(structured_dir),
        "--ready-file",
        str(evidence_ready_file),
        "--start-file",
        str(evidence_start_file),
        "--estimated-pose-topic",
        arguments.estimated_pose_topic,
        "--ground-truth-frame",
        arguments.ground_truth_frame,
        "--patrol-start-file",
        str(patrol_start_file),
        "--patrol-terminal-file",
        str(patrol_terminal_file),
    ]
    if all(ground_truth_supplied):
        evidence_command.extend(
            [
                "--ground-truth-x",
                str(arguments.ground_truth_x),
                "--ground-truth-y",
                str(arguments.ground_truth_y),
            ]
        )
        if arguments.ground_truth_z is not None:
            evidence_command.extend(["--ground-truth-z", str(arguments.ground_truth_z)])
    evidence_log_handle = (logs / "structured_evidence.log").open("w", encoding="utf-8")
    evidence_process = subprocess.Popen(
        evidence_command,
        stdout=evidence_log_handle,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )

    bag_path = session / "rosbag"
    bag_log_handle = (logs / "rosbag.log").open("w", encoding="utf-8")
    recorder = subprocess.Popen(
        ("ros2", "bag", "record", "-o", str(bag_path), *topics),
        stdout=bag_log_handle,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )

    tegrastats_process: subprocess.Popen[str] | None = None
    tegrastats_log_handle = None
    if shutil.which("tegrastats") is not None:
        tegrastats_log_handle = (logs / "tegrastats.log").open("w", encoding="utf-8")
        tegrastats_process = subprocess.Popen(
            ("tegrastats", "--interval", str(max(100, int(arguments.sample_period * 1000)))),
            stdout=tegrastats_log_handle,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )

    resource_path = session / "resource_samples.csv"
    interrupted = False
    capture_error: str | None = None
    capture_ready = False
    previous_cpu: tuple[int, int] | None = None
    started_utc: str | None = None
    started_monotonic: float | None = None
    try:
        if not wait_for_ready_file(evidence_ready_file, evidence_process):
            raise RuntimeError(
                f"structured evidence logger did not become ready; code={evidence_process.poll()}"
            )
        time.sleep(1.0)
        if recorder.poll() is not None:
            raise RuntimeError(f"ros2 bag recorder exited early with code {recorder.returncode}")
        if evidence_process.poll() is not None:
            raise RuntimeError(
                f"structured evidence logger exited early with code {evidence_process.returncode}"
            )
        if arguments.wait_for_patrol_start:
            startup_deadline = time.monotonic() + arguments.startup_timeout_sec
            while not patrol_start_file.is_file():
                if time.monotonic() >= startup_deadline:
                    raise RuntimeError("patrol did not enter running state before startup timeout")
                if recorder.poll() is not None:
                    raise RuntimeError(f"ros2 bag recorder stopped with code {recorder.returncode}")
                if evidence_process.poll() is not None:
                    raise RuntimeError(
                        f"structured evidence logger stopped with code {evidence_process.returncode}"
                    )
                time.sleep(0.1)
        started_utc = utc_now()
        started_monotonic = time.monotonic()
        metadata["started_utc"] = started_utc
        write_json(session / "metadata.json", metadata)
        write_json(
            evidence_start_file,
            {"started_utc": started_utc, "monotonic_sec": started_monotonic},
        )
        capture_ready = True
        print(f"CAPTURE_READY: {session}", flush=True)
        with resource_path.open("w", newline="", encoding="utf-8") as resource_handle:
            writer: csv.DictWriter[str] | None = None
            terminal_detected_at: float | None = None
            while arguments.duration == 0.0 or time.monotonic() - started_monotonic < arguments.duration:
                sample, previous_cpu = read_proc_sample(previous_cpu)
                sample["elapsed_sec"] = time.monotonic() - started_monotonic
                if writer is None:
                    writer = csv.DictWriter(resource_handle, fieldnames=list(sample))
                    writer.writeheader()
                writer.writerow(sample)
                resource_handle.flush()
                if recorder.poll() is not None:
                    raise RuntimeError(f"ros2 bag recorder stopped with code {recorder.returncode}")
                if evidence_process.poll() is not None:
                    raise RuntimeError(
                        f"structured evidence logger stopped with code {evidence_process.returncode}"
                    )
                if arguments.stop_on_patrol_terminal and patrol_terminal_file.is_file():
                    if terminal_detected_at is None:
                        terminal_detected_at = time.monotonic()
                    if time.monotonic() - terminal_detected_at >= arguments.post_roll_sec:
                        break
                time.sleep(arguments.sample_period)
    except KeyboardInterrupt:
        interrupted = True
    except Exception as error:  # Preserve the evidence directory on any runtime failure.
        capture_error = f"{type(error).__name__}: {error}"
    finally:
        recorder_code = terminate_process(recorder)
        evidence_code = terminate_process(evidence_process)
        tegrastats_code = terminate_process(tegrastats_process)
        bag_log_handle.close()
        evidence_log_handle.close()
        if tegrastats_log_handle is not None:
            tegrastats_log_handle.close()

    ended_utc = utc_now()
    actual_duration = time.monotonic() - started_monotonic if started_monotonic is not None else 0.0
    snapshot_results["ros_nodes_after.txt"] = run_snapshot(
        ("ros2", "node", "list"), snapshots / "ros_nodes_after.txt"
    )
    snapshot_results["ros_topics_after.txt"] = run_snapshot(
        ("ros2", "topic", "list", "-t"), snapshots / "ros_topics_after.txt"
    )

    bag_metadata = bag_path / "metadata.yaml"
    structured_summary = structured_dir / "capture_summary.json"
    result = {
        "schema_version": 1,
        "run_class": "REAL_ROBOT_CAPTURE",
        "evaluation_status": "NOT_EVALUATED",
        "capture_status": (
            "COMPLETE"
            if capture_ready
            and capture_error is None
            and bag_metadata.is_file()
            and structured_summary.is_file()
            else "FAILED"
        ),
        "capture_ready": capture_ready,
        "experiment_id": session.name,
        "started_utc": started_utc,
        "ended_utc": ended_utc,
        "actual_duration_sec": actual_duration,
        "interrupted_by_operator": interrupted,
        "rosbag_return_code": recorder_code,
        "structured_evidence_return_code": evidence_code,
        "structured_evidence_ready": evidence_ready_file.is_file(),
        "structured_evidence_started": evidence_start_file.is_file(),
        "structured_evidence_summary_present": structured_summary.is_file(),
        "patrol_start_observed": patrol_start_file.is_file(),
        "patrol_terminal_observed": patrol_terminal_file.is_file(),
        "tegrastats_return_code": tegrastats_code,
        "rosbag_path": str(bag_path),
        "rosbag_present": bag_path.exists(),
        "rosbag_metadata_present": bag_metadata.is_file(),
        "snapshot_return_codes": snapshot_results,
        "teacher_protocol_present": teacher_protocol is not None,
        "teacher_requirement_id": (
            None if teacher_protocol is None else teacher_protocol.get("requirement_id")
        ),
        "error": capture_error,
        "claim_boundary": "Capture completion is not a performance result; evaluation is required.",
    }
    write_json(session / "result.json", result)
    entries = build_manifest(session)
    write_json(session / "manifest.json", {"files": entries})
    (session / "manifest.sha256").write_text(
        "".join(f"{item['sha256']}  {item['path']}\n" for item in entries), encoding="ascii"
    )

    print(session)
    if result["capture_status"] != "COMPLETE":
        print(
            f"ERROR: capture failed: {capture_error or 'rosbag metadata or structured evidence summary is missing'}",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
