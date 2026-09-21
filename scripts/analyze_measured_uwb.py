#!/usr/bin/env python3
"""Build reproducible evidence tables and figures from measured UWB captures.

The analysis deliberately keeps two evidence boundaries visible:

* CSV rows are real measurements from three physical anchors.
* The source captures contain neither independent position ground truth nor
  host-side timestamps, so the output cannot establish absolute accuracy or
  end-to-end latency.

No ROS graph is required and source files are opened read-only.  The same gate
and trilateration code used by the robot replay node is imported from
``src/ylhb_base/scripts``.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import statistics
import sys
from collections import Counter
from pathlib import Path
from typing import Any, Iterable, Sequence


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
YLHB_BASE_SCRIPTS = REPOSITORY_ROOT / "src" / "ylhb_base" / "scripts"
if str(YLHB_BASE_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(YLHB_BASE_SCRIPTS))

from uwb_input import (  # noqa: E402
    MEASURED_ANCHOR_COORDINATES,
    build_logical_observations,
    load_replay_frames,
)
from uwb_quality_gate import UwbQualityConfig, UwbQualityGate  # noqa: E402


DEFAULT_DATA_DIRECTORY = (
    REPOSITORY_ROOT / "src" / "ylhb_base" / "data" / "uwb" / "20260905"
)
DEFAULT_OUTPUT_DIRECTORY = REPOSITORY_ROOT / "evidence" / "uwb" / "20260905"
SCENARIOS = (
    ("static", "static_raw_20260905_165208.csv"),
    ("move", "move_raw_20260905_170425.csv"),
    ("anchor_fault", "anchor_fault.csv"),
)
DECISION_ORDER = ("ACCEPT", "ACCEPT_DEGRADED", "HOLD_FOR_RECOVERY", "REJECT")
STATE_ORDER = ("GOOD", "DEGRADED", "RECOVERING", "REJECTED")
FRAME_FIELDS = (
    "scenario",
    "frame_index",
    "time_s_assumed",
    "a2_m",
    "a3_m",
    "a4_m",
    "logical_slot_count",
    "unique_physical_count",
    "decision",
    "state",
    "rejection_reason",
    "rejection_reason_class",
    "accepted",
    "estimate_available",
    "estimated_x_m",
    "estimated_y_m",
    "residual_rms_m",
    "geometry_metric",
    "confidence",
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def percentile(values: Sequence[float], percent: float) -> float | None:
    """Return a linearly interpolated percentile without a NumPy dependency."""
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return float(ordered[0])
    rank = (len(ordered) - 1) * percent / 100.0
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return float(ordered[lower])
    weight = rank - lower
    return float(ordered[lower] * (1.0 - weight) + ordered[upper] * weight)


def numeric_summary(values: Iterable[float | None]) -> dict[str, float | int | None]:
    usable = [float(value) for value in values if value is not None and math.isfinite(float(value))]
    if not usable:
        return {
            "count": 0,
            "min": None,
            "max": None,
            "mean": None,
            "median": None,
            "p95": None,
            "sample_stddev": None,
        }
    return {
        "count": len(usable),
        "min": min(usable),
        "max": max(usable),
        "mean": statistics.fmean(usable),
        "median": statistics.median(usable),
        "p95": percentile(usable, 95.0),
        "sample_stddev": statistics.stdev(usable) if len(usable) > 1 else 0.0,
    }


def rejection_reason_class(reason: str) -> str:
    if not reason:
        return "NONE"
    for prefix in (
        "NO_OBSERVATIONS",
        "NO_VALID_OBSERVATIONS",
        "INSUFFICIENT_ANCHORS",
        "RANGE_OUT_OF_BOUNDS",
        "RANGE_JUMP",
        "RANGE_RATE",
        "POSITION_ESTIMATION_FAILED",
        "GEOMETRY_POOR",
        "RESIDUAL_HIGH",
        "POSITION_JUMP",
        "IMPLIED_SPEED",
        "INNOVATION_HIGH",
        "STALE",
    ):
        if reason.startswith(prefix):
            return prefix
    return "OTHER"


def _ordered_counts(counter: Counter[str], keys: Sequence[str]) -> dict[str, int]:
    return {key: int(counter.get(key, 0)) for key in keys}


def analyze_scenario(
    source_path: str | Path,
    *,
    scenario: str,
    replay_rate_hz: float = 25.0,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """Replay one measured CSV through the production UWB quality gate."""
    if replay_rate_hz <= 0.0 or not math.isfinite(replay_rate_hz):
        raise ValueError("replay_rate_hz must be a finite positive value")

    source = Path(source_path).resolve()
    frames = load_replay_frames(source)
    gate = UwbQualityGate(UwbQualityConfig(), list(MEASURED_ANCHOR_COORDINATES))
    rows: list[dict[str, Any]] = []
    decisions: Counter[str] = Counter()
    states: Counter[str] = Counter()
    reason_classes: Counter[str] = Counter()
    valid_source_counts: Counter[str] = Counter()

    for frame_index, ranges in enumerate(frames):
        timestamp = frame_index / replay_rate_hz
        observations = build_logical_observations(
            timestamp,
            ranges,
            six_logical_slots=True,
        )
        unique_physical_count = len(
            {item.physical_source_id for item in observations if item.valid}
        )
        estimate, decision = gate.update(observations, timestamp)
        state = gate.state.state
        reason = gate.state.rejection_reason
        reason_group = rejection_reason_class(reason)
        accepted = decision in {"ACCEPT", "ACCEPT_DEGRADED"}

        decisions[decision] += 1
        states[state] += 1
        valid_source_counts[str(unique_physical_count)] += 1
        if reason_group != "NONE":
            reason_classes[reason_group] += 1

        rows.append(
            {
                "scenario": scenario,
                "frame_index": frame_index,
                "time_s_assumed": timestamp,
                "a2_m": ranges[0],
                "a3_m": ranges[1],
                "a4_m": ranges[2],
                "logical_slot_count": len(observations),
                "unique_physical_count": unique_physical_count,
                "decision": decision,
                "state": state,
                "rejection_reason": reason,
                "rejection_reason_class": reason_group,
                "accepted": accepted,
                "estimate_available": estimate is not None,
                "estimated_x_m": estimate.x if estimate is not None else None,
                "estimated_y_m": estimate.y if estimate is not None else None,
                "residual_rms_m": estimate.residual_rms if estimate is not None else None,
                "geometry_metric": estimate.geometry_metric if estimate is not None else None,
                "confidence": estimate.confidence if estimate is not None else None,
            }
        )

    accepted_frames = decisions["ACCEPT"] + decisions["ACCEPT_DEGRADED"]
    summary: dict[str, Any] = {
        "scenario": scenario,
        "source_file": source.name,
        "source_sha256": sha256_file(source),
        "frame_count": len(rows),
        "replay_rate_hz_assumed": replay_rate_hz,
        "duration_s_assumed": len(rows) / replay_rate_hz,
        "logical_slot_count": 6,
        "physical_anchor_count": 3,
        "decision_counts": _ordered_counts(decisions, DECISION_ORDER),
        "state_counts": _ordered_counts(states, STATE_ORDER),
        "accepted_frames": accepted_frames,
        "accepted_rate_percent": 100.0 * accepted_frames / len(rows),
        "estimate_available_frames": sum(bool(row["estimate_available"]) for row in rows),
        "valid_physical_count_distribution": dict(
            sorted(valid_source_counts.items(), key=lambda item: int(item[0]))
        ),
        "rejection_reason_classes": dict(sorted(reason_classes.items())),
        "range_statistics_m": {
            anchor: numeric_summary(row[anchor] for row in rows)
            for anchor in ("a2_m", "a3_m", "a4_m")
        },
        "estimator_internal_consistency": {
            "residual_rms_m": numeric_summary(row["residual_rms_m"] for row in rows),
            "confidence": numeric_summary(row["confidence"] for row in rows),
        },
        "estimated_trace_bounds_m": {
            "x": numeric_summary(row["estimated_x_m"] for row in rows),
            "y": numeric_summary(row["estimated_y_m"] for row in rows),
        },
        "independent_ground_truth_available": False,
        "host_timestamps_available": False,
        "claims_supported": {
            "measured_input_parsing": True,
            "three_physical_to_six_logical_deduplication": True,
            "quality_gate_behavior": True,
            "two_dimensional_estimator_execution": True,
            "absolute_position_accuracy": False,
            "end_to_end_latency": False,
        },
    }
    return rows, summary


def write_frame_csv(path: Path, rows: Sequence[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=FRAME_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def write_summary_csv(path: Path, summaries: Sequence[dict[str, Any]]) -> None:
    fields = (
        "scenario",
        "source_file",
        "source_sha256",
        "frame_count",
        "duration_s_assumed",
        "accepted_frames",
        "accepted_rate_percent",
        "accept",
        "accept_degraded",
        "hold_for_recovery",
        "reject",
        "good",
        "degraded",
        "recovering",
        "rejected",
        "two_physical_source_frames",
        "three_physical_source_frames",
        "estimate_available_frames",
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for summary in summaries:
            writer.writerow(
                {
                    "scenario": summary["scenario"],
                    "source_file": summary["source_file"],
                    "source_sha256": summary["source_sha256"],
                    "frame_count": summary["frame_count"],
                    "duration_s_assumed": summary["duration_s_assumed"],
                    "accepted_frames": summary["accepted_frames"],
                    "accepted_rate_percent": summary["accepted_rate_percent"],
                    "accept": summary["decision_counts"]["ACCEPT"],
                    "accept_degraded": summary["decision_counts"]["ACCEPT_DEGRADED"],
                    "hold_for_recovery": summary["decision_counts"]["HOLD_FOR_RECOVERY"],
                    "reject": summary["decision_counts"]["REJECT"],
                    "good": summary["state_counts"]["GOOD"],
                    "degraded": summary["state_counts"]["DEGRADED"],
                    "recovering": summary["state_counts"]["RECOVERING"],
                    "rejected": summary["state_counts"]["REJECTED"],
                    "two_physical_source_frames": summary[
                        "valid_physical_count_distribution"
                    ].get("2", 0),
                    "three_physical_source_frames": summary[
                        "valid_physical_count_distribution"
                    ].get("3", 0),
                    "estimate_available_frames": summary["estimate_available_frames"],
                }
            )


def _import_plotting() -> Any:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    return plt


def plot_state_distribution(
    path: Path,
    summaries: Sequence[dict[str, Any]],
) -> None:
    plt = _import_plotting()
    fig, ax = plt.subplots(figsize=(8.0, 4.6), constrained_layout=True)
    x = list(range(len(summaries)))
    width = 0.19
    colors = {
        "GOOD": "#2E7D32",
        "DEGRADED": "#F9A825",
        "RECOVERING": "#1565C0",
        "REJECTED": "#C62828",
    }
    for offset, state in enumerate(STATE_ORDER):
        values = [
            100.0 * item["state_counts"][state] / item["frame_count"]
            for item in summaries
        ]
        positions = [value + (offset - 1.5) * width for value in x]
        bars = ax.bar(positions, values, width=width, color=colors[state], label=state)
        for bar, value in zip(bars, values):
            if value > 0.0:
                ax.text(
                    bar.get_x() + bar.get_width() / 2.0,
                    value + 0.45,
                    f"{value:.1f}",
                    ha="center",
                    va="bottom",
                    fontsize=7,
                )
    ax.set_xticks(x, [item["scenario"] for item in summaries])
    ax.set_ylabel("Frames (%)")
    ax.set_ylim(0.0, 105.0)
    ax.set_title("Measured UWB replay: quality-gate states")
    ax.grid(axis="y", alpha=0.25)
    ax.legend(ncols=2)
    fig.savefig(path, dpi=300, metadata={"Description": "Measured data; no ground-truth accuracy claim."})
    plt.close(fig)


def plot_range_series(path: Path, all_rows: dict[str, list[dict[str, Any]]]) -> None:
    plt = _import_plotting()
    fig, axes = plt.subplots(3, 1, figsize=(9.2, 8.2), sharex=False, constrained_layout=True)
    colors = {"a2_m": "#1565C0", "a3_m": "#EF6C00", "a4_m": "#2E7D32"}
    labels = {"a2_m": "A2", "a3_m": "A3", "a4_m": "A4"}
    for ax, (scenario, rows) in zip(axes, all_rows.items()):
        times = [row["time_s_assumed"] for row in rows]
        for field in ("a2_m", "a3_m", "a4_m"):
            ax.plot(times, [row[field] for row in rows], linewidth=0.7, color=colors[field], label=labels[field])
        ax.set_title(scenario)
        ax.set_ylabel("Range (m)")
        ax.grid(alpha=0.2)
        ax.legend(ncols=3, loc="upper right")
    axes[-1].set_xlabel("Replay time (s, 25 Hz assumed)")
    fig.suptitle("Measured ranges from three physical UWB anchors")
    fig.savefig(path, dpi=300, metadata={"Description": "Real measured ranges; replay time is derived at 25 Hz."})
    plt.close(fig)


def plot_estimated_traces(path: Path, all_rows: dict[str, list[dict[str, Any]]]) -> None:
    plt = _import_plotting()
    fig, axes = plt.subplots(1, 2, figsize=(10.0, 4.8), constrained_layout=True)
    anchors_x = [anchor.x for anchor in MEASURED_ANCHOR_COORDINATES]
    anchors_y = [anchor.y for anchor in MEASURED_ANCHOR_COORDINATES]
    for ax, scenario in zip(axes, ("static", "move")):
        rows = [row for row in all_rows[scenario] if row["estimate_available"]]
        x_values = [row["estimated_x_m"] for row in rows]
        y_values = [row["estimated_y_m"] for row in rows]
        accepted = [row["accepted"] for row in rows]
        ax.plot(x_values, y_values, color="#90A4AE", linewidth=0.45, alpha=0.55)
        ax.scatter(
            [x for x, keep in zip(x_values, accepted) if keep],
            [y for y, keep in zip(y_values, accepted) if keep],
            s=4,
            color="#1565C0",
            alpha=0.55,
            label="accepted estimate",
        )
        ax.scatter(anchors_x, anchors_y, marker="^", s=65, color="#C62828", label="surveyed anchor")
        for anchor, x_value, y_value in zip(("A2", "A3", "A4"), anchors_x, anchors_y):
            ax.annotate(anchor, (x_value, y_value), xytext=(4, 4), textcoords="offset points")
        ax.set_title(scenario)
        ax.set_xlabel("Estimated UWB-local x (m)")
        ax.set_ylabel("Estimated UWB-local y (m)")
        ax.axis("equal")
        ax.grid(alpha=0.2)
        ax.legend(loc="best")
    fig.suptitle("2D estimator output (no independent ground truth; not an accuracy plot)")
    fig.savefig(path, dpi=300, metadata={"Description": "Estimator output only; independent ground truth is unavailable."})
    plt.close(fig)


def plot_anchor_fault_timeline(path: Path, rows: Sequence[dict[str, Any]]) -> None:
    plt = _import_plotting()
    fig, axes = plt.subplots(2, 1, figsize=(9.2, 5.6), sharex=True, constrained_layout=True)
    times = [row["time_s_assumed"] for row in rows]
    axes[0].step(times, [row["unique_physical_count"] for row in rows], where="post", color="#512DA8")
    axes[0].axhline(3, color="#2E7D32", linestyle="--", linewidth=1.0, label="required physical sources")
    axes[0].set_ylim(1.7, 3.3)
    axes[0].set_yticks((2, 3))
    axes[0].set_ylabel("Unique physical anchors")
    axes[0].legend(loc="lower left")
    axes[0].grid(alpha=0.2)

    state_value = {"REJECTED": 0, "RECOVERING": 1, "DEGRADED": 2, "GOOD": 3}
    axes[1].step(times, [state_value[row["state"]] for row in rows], where="post", color="#00695C")
    axes[1].set_yticks((0, 1, 2, 3), ("REJECTED", "RECOVERING", "DEGRADED", "GOOD"))
    axes[1].set_xlabel("Replay time (s, 25 Hz assumed)")
    axes[1].set_ylabel("Gate state")
    axes[1].grid(alpha=0.2)
    fig.suptitle("Measured anchor-loss sequence and quality-gate response")
    fig.savefig(path, dpi=300, metadata={"Description": "Three physical sources exposed as six deduplicated logical slots."})
    plt.close(fig)


def write_report(path: Path, summaries: Sequence[dict[str, Any]]) -> None:
    by_name = {item["scenario"]: item for item in summaries}
    lines = [
        "# 2026-09-05 UWB real-measurement evidence",
        "",
        "The source files contain measured ranges from three physical anchors. The robot interface exposes six logical slots while preserving physical source identity, so duplicated slots do not increase geometry or confidence counts.",
        "",
        "| Scenario | Frames | Accepted | Accepted rate | Rejected | Recovering | 2-source frames |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for name in ("static", "move", "anchor_fault"):
        item = by_name[name]
        lines.append(
            f"| {name} | {item['frame_count']} | {item['accepted_frames']} | "
            f"{item['accepted_rate_percent']:.2f}% | {item['state_counts']['REJECTED']} | "
            f"{item['state_counts']['RECOVERING']} | "
            f"{item['valid_physical_count_distribution'].get('2', 0)} |"
        )
    lines.extend(
        [
            "",
            "Reproduction chain: measured CSV -> six logical observations -> physical-source deduplication -> UwbQualityGate -> 2D estimator.",
            "",
            "## Evidence boundary",
            "",
            "The captures contain no independent position ground truth and no host-side per-frame timestamps. Residual RMS is an estimator internal-consistency value, not position error. The derived 25 Hz replay axis is not an end-to-end latency measurement. These files support parsing, gate-state, rejection/recovery, deduplication and estimator-execution claims only.",
            "",
            "## Figures",
            "",
            "- `figures/state_distribution.png`: gate-state distribution.",
            "- `figures/measured_ranges.png`: three measured physical ranges.",
            "- `figures/estimated_traces.png`: estimator output with the evidence boundary in the title.",
            "- `figures/anchor_fault_timeline.png`: physical-source loss and gate response.",
            "- `LONG_RUN_TERMINAL_RECORD.md`: secondary terminal record for the five-minute capture; the raw file is not bundled.",
            "",
            "Exact parameters, source hashes and numeric statistics are in `summary.json`; per-frame decisions are in `frames/*.csv`.",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def write_manifest(output_directory: Path, sources: Sequence[Path]) -> None:
    output_files = sorted(
        path
        for path in output_directory.rglob("*")
        if path.is_file() and path.name != "manifest.sha256"
    )
    lines = [
        f"{sha256_file(path)}  {path.relative_to(output_directory).as_posix()}"
        for path in output_files
    ]
    (output_directory / "manifest.sha256").write_text("\n".join(lines) + "\n", encoding="ascii")
    source_manifest = {
        "sources_opened_read_only": [
            {
                "path": source.relative_to(REPOSITORY_ROOT).as_posix()
                if source.is_relative_to(REPOSITORY_ROOT)
                else str(source),
                "sha256": sha256_file(source),
            }
            for source in sources
        ],
        "output_manifest": "manifest.sha256",
    }
    (output_directory / "source_manifest.json").write_text(
        json.dumps(source_manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    # Include source_manifest itself after it has been created.
    output_files = sorted(
        path
        for path in output_directory.rglob("*")
        if path.is_file() and path.name != "manifest.sha256"
    )
    lines = [
        f"{sha256_file(path)}  {path.relative_to(output_directory).as_posix()}"
        for path in output_files
    ]
    (output_directory / "manifest.sha256").write_text("\n".join(lines) + "\n", encoding="ascii")


def build_evidence(
    data_directory: Path,
    output_directory: Path,
    *,
    replay_rate_hz: float,
    make_plots: bool,
) -> list[dict[str, Any]]:
    output_directory.mkdir(parents=True, exist_ok=True)
    all_rows: dict[str, list[dict[str, Any]]] = {}
    summaries: list[dict[str, Any]] = []
    sources: list[Path] = []
    for scenario, filename in SCENARIOS:
        source = (data_directory / filename).resolve()
        rows, summary = analyze_scenario(
            source,
            scenario=scenario,
            replay_rate_hz=replay_rate_hz,
        )
        all_rows[scenario] = rows
        summaries.append(summary)
        sources.append(source)
        write_frame_csv(output_directory / "frames" / f"{scenario}.csv", rows)

    (output_directory / "summary.json").write_text(
        json.dumps(
            {
                "analysis": "measured UWB replay through production gate and estimator",
                "replay_rate_hz_assumed": replay_rate_hz,
                "anchor_coordinates_m": [
                    {
                        "physical_id": anchor.physical_id,
                        "x": anchor.x,
                        "y": anchor.y,
                        "z": anchor.z,
                    }
                    for anchor in MEASURED_ANCHOR_COORDINATES
                ],
                "logical_to_physical_mapping": {
                    "A0": 0,
                    "A1": 1,
                    "A2": 2,
                    "A3": 0,
                    "A4": 1,
                    "A5": 2,
                },
                "quality_gate_config": vars(UwbQualityConfig()),
                "scenarios": summaries,
                "evidence_boundary": {
                    "independent_ground_truth_available": False,
                    "host_timestamps_available": False,
                    "absolute_position_accuracy_supported": False,
                    "end_to_end_latency_supported": False,
                },
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    write_summary_csv(output_directory / "summary.csv", summaries)
    write_report(output_directory / "README.md", summaries)

    if make_plots:
        figure_directory = output_directory / "figures"
        figure_directory.mkdir(parents=True, exist_ok=True)
        plot_state_distribution(figure_directory / "state_distribution.png", summaries)
        plot_range_series(figure_directory / "measured_ranges.png", all_rows)
        plot_estimated_traces(figure_directory / "estimated_traces.png", all_rows)
        plot_anchor_fault_timeline(
            figure_directory / "anchor_fault_timeline.png",
            all_rows["anchor_fault"],
        )

    write_manifest(output_directory, sources)
    return summaries


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA_DIRECTORY)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIRECTORY)
    parser.add_argument("--replay-rate-hz", type=float, default=25.0)
    parser.add_argument("--skip-plots", action="store_true")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    summaries = build_evidence(
        args.data_dir.resolve(),
        args.output_dir.resolve(),
        replay_rate_hz=args.replay_rate_hz,
        make_plots=not args.skip_plots,
    )
    for summary in summaries:
        print(
            f"{summary['scenario']}: frames={summary['frame_count']} "
            f"accepted={summary['accepted_frames']} "
            f"rate={summary['accepted_rate_percent']:.2f}% "
            f"rejected={summary['state_counts']['REJECTED']}"
        )
    print(f"Evidence written to: {args.output_dir.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
