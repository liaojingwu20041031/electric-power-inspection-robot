#!/usr/bin/env python3
"""Evaluate LiDAR feature repeatability from consecutive real scan frames."""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import sys
from typing import List, Optional, Sequence


@dataclass
class FrameFeatures:
    """Features detected in one frame.

    ``feature_points`` is the real-robot format. ``feature_ids`` remains
    supported for older synthetic data.
    """

    frame_id: int
    timestamp: float
    feature_ids: List[int]
    valid_feature_count: int
    feature_points: List[tuple[float, float]] | None = None
    coordinate_frame: str = ""


@dataclass
class RepeatabilityMetrics:
    total_frames: int
    total_pairs: int
    zero_feature_pair_count: int
    total_features: int
    repeated_features: int
    overall_repeatability: float
    min_repeatability: float
    per_frame_repeatability: List[float]
    mean_per_frame: float
    matching_method: str
    match_distance_m: float


def _spatial_match_count(
    first: Sequence[tuple[float, float]],
    second: Sequence[tuple[float, float]],
    match_distance: float,
) -> int:
    limit_squared = match_distance * match_distance
    adjacency: list[list[int]] = []
    for first_index, (first_x, first_y) in enumerate(first):
        candidates: list[tuple[float, int]] = []
        for second_index, (second_x, second_y) in enumerate(second):
            distance_squared = (first_x - second_x) ** 2 + (first_y - second_y) ** 2
            if distance_squared <= limit_squared:
                candidates.append((distance_squared, second_index))
        adjacency.append([index for _, index in sorted(candidates)])

    matched_second: dict[int, int] = {}

    def augment(first_index: int, seen: set[int]) -> bool:
        for second_index in adjacency[first_index]:
            if second_index in seen:
                continue
            seen.add(second_index)
            previous = matched_second.get(second_index)
            if previous is None or augment(previous, seen):
                matched_second[second_index] = first_index
                return True
        return False

    return sum(augment(first_index, set()) for first_index in range(len(first)))


def _pair_match_count(
    first: FrameFeatures, second: FrameFeatures, match_distance: float
) -> tuple[int, str]:
    if first.feature_points is not None and second.feature_points is not None:
        if (
            first.coordinate_frame
            and second.coordinate_frame
            and first.coordinate_frame != second.coordinate_frame
        ):
            raise ValueError(
                f"coordinate frame changed from {first.coordinate_frame!r} to {second.coordinate_frame!r}"
            )
        return _spatial_match_count(first.feature_points, second.feature_points, match_distance), "SPATIAL_2D"
    return len(set(first.feature_ids) & set(second.feature_ids)), "LEGACY_ID"


def compute_frame_pair_repeatability(
    frame1: FrameFeatures, frame2: FrameFeatures, match_distance: float = 0.12
) -> Optional[float]:
    """Return symmetric repeatability: ``2M / (N1 + N2)``.

    A pair with an empty frame is undefined and is counted separately by
    :func:`compute_metrics`; it is never reported as perfect repeatability.
    """

    if frame1.valid_feature_count == 0 or frame2.valid_feature_count == 0:
        return None
    matched, _ = _pair_match_count(frame1, frame2, match_distance)
    return 2.0 * matched / (frame1.valid_feature_count + frame2.valid_feature_count)


def compute_metrics(
    frames: List[FrameFeatures], match_distance: float = 0.12
) -> Optional[RepeatabilityMetrics]:
    if len(frames) < 2:
        return None
    if not math.isfinite(match_distance) or match_distance <= 0.0:
        raise ValueError("match_distance must be finite and positive")

    pair_values: list[float] = []
    total_features = 0
    repeated_features = 0
    zero_feature_pairs = 0
    methods: set[str] = set()
    for frame1, frame2 in zip(frames, frames[1:]):
        if frame1.valid_feature_count == 0 or frame2.valid_feature_count == 0:
            zero_feature_pairs += 1
            continue
        matched, method = _pair_match_count(frame1, frame2, match_distance)
        methods.add(method)
        denominator = frame1.valid_feature_count + frame2.valid_feature_count
        pair_values.append(2.0 * matched / denominator)
        repeated_features += matched
        total_features += denominator

    if not pair_values:
        return None
    return RepeatabilityMetrics(
        total_frames=len(frames),
        total_pairs=len(pair_values),
        zero_feature_pair_count=zero_feature_pairs,
        total_features=total_features,
        repeated_features=repeated_features,
        overall_repeatability=2.0 * repeated_features / total_features,
        min_repeatability=min(pair_values),
        per_frame_repeatability=pair_values,
        mean_per_frame=sum(pair_values) / len(pair_values),
        matching_method="+".join(sorted(methods)),
        match_distance_m=match_distance,
    )


def _parse_feature_points(raw: str) -> list[tuple[float, float]]:
    payload = json.loads(raw)
    if not isinstance(payload, list):
        raise ValueError("feature_points must be a JSON list")
    points: list[tuple[float, float]] = []
    for item in payload:
        if not isinstance(item, list) or len(item) != 2:
            raise ValueError("each feature point must be [x, y]")
        x, y = float(item[0]), float(item[1])
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError("feature coordinates must be finite")
        points.append((x, y))
    return points


def read_csv(filepath: str) -> List[FrameFeatures]:
    frames: list[FrameFeatures] = []
    with open(filepath, "r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for line_number, row in enumerate(reader, 2):
            try:
                raw_points = (row.get("feature_points") or "").strip()
                raw_ids = (row.get("feature_ids") or "").strip()
                if raw_points:
                    points = _parse_feature_points(raw_points)
                    ids: list[int] = []
                    actual_count = len(points)
                else:
                    points = None
                    ids = [int(value) for value in raw_ids.split(",") if value.strip()]
                    actual_count = len(ids)
                declared_count = int(row["valid_feature_count"])
                if declared_count != actual_count:
                    raise ValueError(
                        f"valid_feature_count={declared_count} does not match captured features={actual_count}"
                    )
                frames.append(
                    FrameFeatures(
                        frame_id=int(row["frame_id"]),
                        timestamp=float(row["timestamp"]),
                        feature_ids=ids,
                        valid_feature_count=declared_count,
                        feature_points=points,
                        coordinate_frame=(row.get("coordinate_frame") or "").strip(),
                    )
                )
            except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
                print(f"Warning: skipping invalid row {line_number}: {error}", file=sys.stderr)
    frames.sort(key=lambda frame: frame.frame_id)
    return frames


def print_metrics(metrics: RepeatabilityMetrics) -> None:
    print("Feature Repeatability Metrics:")
    print(f"  Total Frames:           {metrics.total_frames}")
    print(f"  Valid Pairs:            {metrics.total_pairs}")
    print(f"  Zero-Feature Pairs:     {metrics.zero_feature_pair_count}")
    print(f"  Matched Feature Pairs:  {metrics.repeated_features}")
    print(f"  Overall R_pool:         {metrics.overall_repeatability:.4f}")
    print(f"  Mean R_avg:             {metrics.mean_per_frame:.4f}")
    print(f"  Minimum R_min:          {metrics.min_repeatability:.4f}")
    print(f"  Matching Method:        {metrics.matching_method}")
    if metrics.matching_method == "SPATIAL_2D":
        print(f"  Match Distance:         {metrics.match_distance_m:.3f} m")


def create_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_file")
    parser.add_argument("--match-distance", type=float, default=0.12)
    parser.add_argument("--output-json")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    arguments = create_parser().parse_args(argv)
    try:
        frames = read_csv(arguments.csv_file)
        metrics = compute_metrics(frames, arguments.match_distance)
    except (FileNotFoundError, OSError, ValueError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    if metrics is None:
        print("Error: need at least one pair with features in both frames", file=sys.stderr)
        return 1
    print_metrics(metrics)
    if arguments.output_json:
        Path(arguments.output_json).write_text(
            json.dumps(asdict(metrics), ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
