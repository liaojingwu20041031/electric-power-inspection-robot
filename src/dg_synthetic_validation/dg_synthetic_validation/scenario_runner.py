"""Run one or all synthetic scenarios against the existing ROS2 integration.

The runner owns process lifecycle, rosbag recording, safety checks, and final
artifact writing.  It never publishes a health/fusion/control result itself.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .evaluator_node import create_node as create_evaluator
from .plot_results import generate_plots
from .result_writer import evaluate_scenario, write_result
from .scenario_schema import Scenario, load_scenario
from .synthetic_injector_node import create_node as create_injector


BASELINE_COMMIT = "c41adca7f9bddb4240a1a4855437518b36d9fe13"
RUNTIME_INTEGRATION_COMMIT = "096bf66377e0dbaa3ed812782222ea95a2439c7a"
RECORDED_TOPICS = [
    "/gps/fix", "/gps/rtk_status", "/scan", "/odom", "/map", "/amcl_pose", "/initialpose",
    "/dg/gnss/quality", "/dg/gnss/accepted_fix", "/dg/lidar/filtered_scan", "/dg/lidar/stable_features", "/dg/lidar/quality",
    "/dg/relocalization/seed", "/dg/relocalization/status", "/dg/relocalization/match_quality",
    "/dg/fusion/odom", "/dg/fusion/pose", "/dg/fusion/status", "/dg/navigation/status",
    "/cmd_vel_nav", "/cmd_vel_recovery", "/dg/test_cmd_vel", "/tf", "/tf_static",
]


def _default_share_dir() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("dg_synthetic_validation"))
    except Exception:
        return Path(__file__).resolve().parents[1]


def _git(repo_root: Path, *args: str) -> str:
    try:
        return subprocess.check_output(["git", "-C", str(repo_root), *args], text=True, stderr=subprocess.STDOUT).strip()
    except (OSError, subprocess.CalledProcessError):
        return ""


def _stop_process(
    process: subprocess.Popen[str] | None,
    label: str,
    log_path: Path,
    graceful_signal: int = signal.SIGINT,
) -> int | None:
    if process is None or process.poll() is not None:
        return None if process is None else process.returncode
    try:
        os.killpg(os.getpgid(process.pid), graceful_signal)
        process.wait(timeout=8.0)
    except (ProcessLookupError, TimeoutError, subprocess.TimeoutExpired):
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=4.0)
        except (ProcessLookupError, TimeoutError, subprocess.TimeoutExpired):
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
    if process.returncode not in (0, 130, -signal.SIGINT, -signal.SIGTERM):
        with log_path.open("a", encoding="utf-8") as handle:
            handle.write(f"{label} exited with code {process.returncode}\n")
    return process.returncode


def _topic_publisher_count(topic: str) -> int:
    try:
        completed = subprocess.run(
            ["ros2", "topic", "info", topic],
            check=False,
            capture_output=True,
            text=True,
            timeout=8.0,
        )
    except (OSError, subprocess.TimeoutExpired):
        return 0
    for line in completed.stdout.splitlines():
        if "Publisher count:" in line:
            try:
                return int(line.split(":", 1)[1].strip())
            except ValueError:
                return 0
    return 0


def _write_metadata(run_dir: Path, scenario: Scenario, repo_root: Path, integration_command: list[str], run_class: str = "FINAL") -> None:
    metadata = {
        "scenario_id": scenario.scenario_id,
        "scenario_name": scenario.name,
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "baseline_commit": BASELINE_COMMIT,
        "runtime_integration_commit": RUNTIME_INTEGRATION_COMMIT,
        "build_commit": _git(repo_root, "rev-parse", "HEAD"),
        "branch": _git(repo_root, "branch", "--show-current"),
        "git_status": _git(repo_root, "status", "--short"),
        "integration_command": integration_command,
        "recorded_topics": RECORDED_TOPICS,
        "truth_labels": {
            "authenticity_marker": "THIS_IS_SYNTHETIC_SOFTWARE_VALIDATION",
            "validation_class": "SYNTHETIC_SOFTWARE_VALIDATION",
            "data_class": "NOT_REAL_ROBOT_DATA",
            "simulator_class": "NOT_GAZEBO_DATA",
            "performance_claim": "NOT_COMPETITION_PERFORMANCE_EVIDENCE",
            "performance_claim_label": "NOT_COMPETITION_PERFORMANCE_EVIDENCE",
        },
        "hardware": {"real_robot_connected": False, "gazebo_started": False},
        "RUN_CLASS": run_class,
        "NOT_FINAL_EVIDENCE": "TRUE" if run_class == "PRECHECK" else "FALSE",
        "NOT_FOR_DOCUMENT_CLAIM": "TRUE" if run_class == "PRECHECK" else "FALSE",
        "EVIDENCE_LEVEL": (
            "SYNTHETIC SOFTWARE VALIDATION PRECHECK"
            if run_class == "PRECHECK"
            else "SYNTHETIC SOFTWARE VALIDATION"
        ),
    }
    with (run_dir / "metadata.json").open("w", encoding="utf-8") as handle:
        json.dump(metadata, handle, indent=2, ensure_ascii=False)


def run_one(
    scenario: Scenario,
    scenario_file: Path,
    results_root: Path,
    repo_root: Path,
    settle_sec: float = 2.0,
    run_class: str = "FINAL",
) -> tuple[Path, dict[str, Any]]:
    """Run a single scenario and return its artifact directory and result."""
    run_stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    run_dir = results_root / f"{scenario.scenario_id}_{scenario.name.lower().replace(' ', '_')}_{run_stamp}"
    logs_dir = run_dir / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    scenario_copy = run_dir / "scenario.yaml"
    scenario_copy.write_text(scenario_file.read_text(encoding="utf-8"), encoding="utf-8")
    integration_command = [
        "ros2", "launch", "ylhb_base", "dg_navigation_integration.launch.py",
        "mission_gate_enabled:=false",
        "enable_nav2:=false", "enable_multisource_fusion:=true",
        "cmd_vel_output_topic:=/dg/test_cmd_vel",
    ]
    _write_metadata(run_dir, scenario, repo_root, integration_command, run_class)
    integration_log = (logs_dir / "integration.log").open("w", encoding="utf-8")
    bag_log = (logs_dir / "rosbag.log").open("w", encoding="utf-8")
    integration: subprocess.Popen[str] | None = None
    bag: subprocess.Popen[str] | None = None
    evaluator = None
    injector = None
    executor = None
    integration_alive = False
    startup_real_cmd_vel_publishers = 0
    try:
        integration = subprocess.Popen(
            integration_command,
            stdout=integration_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            text=True,
        )
        time.sleep(2.5)
        if integration.poll() is not None:
            integration_alive = False
        else:
            integration_alive = True
        # Safety gate: the integration must not expose the real /cmd_vel topic.
        startup_real_cmd_vel_publishers = _topic_publisher_count("/cmd_vel")
        bag_command = ["ros2", "bag", "record", "-o", str(run_dir / "rosbag"), *RECORDED_TOPICS]
        bag = subprocess.Popen(
            bag_command,
            stdout=bag_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            text=True,
        )

        import rclpy
        from rclpy.executors import SingleThreadedExecutor

        rclpy.init(args=None)
        injector = create_injector(scenario)
        # Publish one nominal/phase-zero frame immediately.  Without this
        # prewarm, the integration's health timers can observe a completely
        # empty graph for their first tick and enter recovery before the
        # synthetic timer's first 100 ms callback.  This is still sensor-only
        # input and does not alter any production node or threshold.
        injector._tick()
        executor = SingleThreadedExecutor()
        executor.add_node(injector)
        # Let the integration consume a few complete sensor cycles before the
        # measured scenario clock starts.  This removes DDS/startup ordering
        # races while keeping the scenario phase zero aligned to the first
        # recorded evaluator sample.
        warmup_deadline = time.monotonic() + 1.5
        while time.monotonic() < warmup_deadline:
            executor.spin_once(timeout_sec=0.1)
            if integration.poll() is not None:
                integration_alive = False
        injector._start = time.monotonic()
        injector._last_map = 0.0
        evaluator = create_evaluator(scenario, run_dir)
        executor.add_node(evaluator)
        deadline = time.monotonic() + scenario.duration_sec + settle_sec
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
            if integration.poll() is not None:
                integration_alive = False
        # Keep the last evaluator sample deterministic and flush CSV files.
        evaluator.finish()
        safety_publishers = _topic_publisher_count("/cmd_vel")
        result = evaluate_scenario(
            scenario,
            evaluator.recorder.samples,
            safety_publishers,
            integration_alive,
            run_class=run_class,
            safety_cmd_vel_publishers_at_start=startup_real_cmd_vel_publishers,
        )
        result.update({
            "start_time": datetime.now(timezone.utc).isoformat(),
            "duration": scenario.duration_sec,
            "seed": scenario.seed,
            "build_commit": _git(repo_root, "rev-parse", "HEAD"),
            "baseline_commit": BASELINE_COMMIT,
            "artifacts": {
                "bag": str(run_dir / "rosbag"),
                "csv": str(run_dir / "samples.csv"),
                "timeline": str(run_dir / "timeline.csv"),
                "logs": str(logs_dir),
            },
        })
        plots = generate_plots(run_dir)
        result["visualization"] = plots
        result["artifacts"]["plots"] = str(run_dir / "plots")
        if plots.get("warning"):
            result.setdefault("warnings", []).append(plots["warning"])
        result["real_cmd_vel_publishers"] = safety_publishers
        result["real_cmd_vel_publishers_at_start"] = startup_real_cmd_vel_publishers
        result["recorded_topics"] = RECORDED_TOPICS
        result["artifact_directory"] = str(run_dir)
        write_result(run_dir, result)
        return run_dir, result
    except Exception as exc:  # report the exception without hiding cleanup
        result = {
            "validation_class": "SYNTHETIC_SOFTWARE_VALIDATION",
            "data_class": "NOT_REAL_ROBOT_DATA",
            "simulator_class": "NOT_GAZEBO_DATA",
            "performance_claim": "NOT_COMPETITION_PERFORMANCE_EVIDENCE",
            "authenticity_marker": "THIS_IS_SYNTHETIC_SOFTWARE_VALIDATION",
            "scenario_id": scenario.scenario_id,
            "scenario_name": scenario.name,
            "run_class": run_class,
            "run_class_markers": {
                "RUN_CLASS": run_class,
                "NOT_FINAL_EVIDENCE": "TRUE" if run_class == "PRECHECK" else "FALSE",
                "NOT_FOR_DOCUMENT_CLAIM": "TRUE" if run_class == "PRECHECK" else "FALSE",
                "EVIDENCE_LEVEL": (
                    "SYNTHETIC SOFTWARE VALIDATION PRECHECK"
                    if run_class == "PRECHECK"
                    else "SYNTHETIC SOFTWARE VALIDATION"
                ),
            },
            "result": "FAIL",
            "overall_result": "FAIL",
            "evidence_level": "SYNTHETIC_SOFTWARE_VALIDATION",
            "gazebo": False,
            "real_robot": False,
            "performance_claim": False,
            "sample_count": 0,
            "checks": {"runner_exception": False},
            "errors": [f"RUNNER_EXCEPTION: {type(exc).__name__}: {exc}"],
            "warnings": [],
        }
        write_result(run_dir, result)
        return run_dir, result
    finally:
        # Stop the external ROS launch and rosbag before tearing down this
        # process' rclpy context.  This avoids shutdown-time callbacks in the
        # child nodes observing an already-invalid context.
        _stop_process(bag, "rosbag", logs_dir / "rosbag.log")
        # SIGTERM avoids duplicate Python rclpy shutdown handlers in the
        # independently launched child nodes; rosbag keeps SIGINT so it can
        # flush its metadata cleanly.
        _stop_process(
            integration,
            "integration",
            logs_dir / "integration.log",
            graceful_signal=signal.SIGTERM,
        )
        if executor is not None:
            try:
                executor.shutdown()
            except Exception:
                pass
        for node in (evaluator, injector):
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:
                    pass
        try:
            import rclpy

            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        integration_log.close()
        bag_log.close()


def _write_summary(results_root: Path, entries: list[tuple[Path, dict[str, Any]]]) -> None:
    results_root.mkdir(parents=True, exist_ok=True)
    summary_csv = results_root / "summary.csv"
    with summary_csv.open("w", newline="", encoding="utf-8") as handle:
        fields = [
            "scenario_id", "scenario", "run_class", "result", "duration", "sample_count",
            "failure_count", "error_count", "warning_count", "important_transition",
            "artifact_directory", "artifact_path",
        ]
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for path, result in entries:
            writer.writerow({
                "scenario_id": result.get("scenario_id", path.name),
                "scenario": result.get("scenario_name", ""),
                "run_class": result.get("run_class", "FINAL"),
                "result": result.get("result", "FAIL"),
                "duration": result.get("duration", result.get("duration_sec", "")),
                "sample_count": result.get("sample_count", 0),
                "failure_count": len(result.get("failures", result.get("errors", []))),
                "error_count": len(result.get("errors", [])),
                "warning_count": len(result.get("warnings", [])),
                "important_transition": "; ".join(result.get("important_transitions", [])),
                "artifact_directory": str(path),
                "artifact_path": str(path),
            })
    with (results_root / "summary.md").open("w", encoding="utf-8") as handle:
        handle.write("# DG-202611 Synthetic Validation Summary\n\n")
        handle.write("This is `SYNTHETIC_SOFTWARE_VALIDATION` only: `NOT_REAL_ROBOT_DATA`, `NOT_GAZEBO_DATA`, and `NOT_COMPETITION_PERFORMANCE_EVIDENCE`.\n\n")
        for path, result in entries:
            transitions = "; ".join(result.get("important_transitions", [])) or "none recorded"
            handle.write(
                f"- **{result.get('scenario_id', path.name)}** ({result.get('scenario_name', '')}): "
                f"**{result.get('result', 'FAIL')}**, duration={result.get('duration', result.get('duration_sec', ''))}s, "
                f"samples={result.get('sample_count', 0)}, failures={len(result.get('failures', result.get('errors', [])))}, "
                f"important_transition={transitions}, artifacts=`{path}`\n"
            )


def _paths(args: argparse.Namespace) -> tuple[Path, Path, Path]:
    share = _default_share_dir()
    scenario_dir = Path(args.scenario_dir) if args.scenario_dir else share / "config"
    results_root = Path(args.results_root) if args.results_root else Path.home() / "dg202611_ws" / "results" / "synthetic"
    repo_root = Path(args.repo_root) if args.repo_root else Path.cwd()
    return scenario_dir, results_root, repo_root


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run DG-202611 S01-S04 synthetic ROS2 validation")
    parser.add_argument("scenario", nargs="?", help="Scenario ID, e.g. S01")
    parser.add_argument("--scenario-dir", type=Path)
    parser.add_argument("--results-root", type=Path)
    parser.add_argument("--repo-root", type=Path)
    parser.add_argument("--settle-sec", type=float, default=2.0)
    parser.add_argument(
        "--run-class",
        choices=["PRECHECK", "FINAL"],
        default="FINAL",
        help="PRECHECK marks the artifacts as not-final evidence",
    )
    return parser


def main(args: list[str] | None = None) -> int:
    parsed = _parser().parse_args(args)
    if not parsed.scenario:
        _parser().error("scenario is required (use run_all_s01_s04 for the complete batch)")
    scenario_dir, results_root, repo_root = _paths(parsed)
    scenario_file = scenario_dir / f"{parsed.scenario.upper()}.yaml"
    scenario = load_scenario(scenario_file)
    path, result = run_one(scenario, scenario_file, results_root, repo_root, parsed.settle_sec, parsed.run_class)
    print(json.dumps({"scenario": scenario.scenario_id, "result": result["result"], "artifacts": str(path)}, ensure_ascii=False))
    return 0 if result["result"] == "PASS" else 1


def run_all_main(args: list[str] | None = None) -> int:
    parsed = _parser().parse_args(args)
    scenario_dir, results_root, repo_root = _paths(parsed)
    entries: list[tuple[Path, dict[str, Any]]] = []
    for scenario_id in ("S01", "S02", "S03", "S04"):
        scenario_file = scenario_dir / f"{scenario_id}.yaml"
        scenario = load_scenario(scenario_file)
        path, result = run_one(scenario, scenario_file, results_root, repo_root, parsed.settle_sec, parsed.run_class)
        entries.append((path, result))
        print(json.dumps({"scenario": scenario_id, "result": result["result"], "artifacts": str(path)}, ensure_ascii=False), flush=True)
    _write_summary(results_root, entries)
    return 0 if all(result.get("result") == "PASS" for _, result in entries) else 1


if __name__ == "__main__":
    sys.exit(main())
