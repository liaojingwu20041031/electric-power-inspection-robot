#!/usr/bin/env python3
"""
Position error evaluator for DG202611 experiments.

Computes localization accuracy metrics from estimated vs ground truth positions.
"""

import sys
import csv
import math
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class PositionSample:
    """Single position measurement."""
    timestamp: float
    estimated_x: float
    estimated_y: float
    ground_truth_x: float
    ground_truth_y: float
    estimated_z: Optional[float] = None
    ground_truth_z: Optional[float] = None


@dataclass
class PositionErrorMetrics:
    """Position error statistics."""
    count: int
    mean: float
    median: float
    rmse: float
    p95: float
    max_error: float
    min_error: float


def compute_error(sample: PositionSample) -> float:
    """Compute Euclidean distance error for one sample."""
    dx = sample.estimated_x - sample.ground_truth_x
    dy = sample.estimated_y - sample.ground_truth_y
    return math.sqrt(dx * dx + dy * dy)


def compute_vertical_error(sample: PositionSample) -> Optional[float]:
    """Compute absolute elevation error when independent Z truth is present."""
    if sample.estimated_z is None or sample.ground_truth_z is None:
        return None
    if not (math.isfinite(sample.estimated_z) and math.isfinite(sample.ground_truth_z)):
        return None
    return abs(sample.estimated_z - sample.ground_truth_z)


def _metrics_from_errors(errors: List[float]) -> Optional[PositionErrorMetrics]:
    if not errors:
        return None
    errors_sorted = sorted(errors)
    count = len(errors)
    p95_index = min(int(count * 0.95), count - 1)
    return PositionErrorMetrics(
        count=count,
        mean=sum(errors) / count,
        median=errors_sorted[count // 2],
        rmse=math.sqrt(sum(error * error for error in errors) / count),
        p95=errors_sorted[p95_index],
        max_error=max(errors),
        min_error=min(errors),
    )


def compute_metrics(samples: List[PositionSample]) -> Optional[PositionErrorMetrics]:
    """Compute position error metrics from samples."""
    if not samples:
        return None

    return _metrics_from_errors([compute_error(sample) for sample in samples])


def compute_vertical_metrics(samples: List[PositionSample]) -> Optional[PositionErrorMetrics]:
    """Compute elevation metrics without treating missing Z truth as zero."""
    errors = [error for sample in samples if (error := compute_vertical_error(sample)) is not None]
    return _metrics_from_errors(errors)


def read_csv(filepath: str) -> List[PositionSample]:
    """Read position samples from CSV file.

    Expected columns: timestamp, estimated_x, estimated_y, ground_truth_x, ground_truth_y
    """
    samples = []

    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                estimated_z = row.get('estimated_z', '').strip()
                ground_truth_z = row.get('ground_truth_z', '').strip()
                sample = PositionSample(
                    timestamp=float(row['timestamp']),
                    estimated_x=float(row['estimated_x']),
                    estimated_y=float(row['estimated_y']),
                    ground_truth_x=float(row['ground_truth_x']),
                    ground_truth_y=float(row['ground_truth_y']),
                    estimated_z=float(estimated_z) if estimated_z else None,
                    ground_truth_z=float(ground_truth_z) if ground_truth_z else None,
                )
                samples.append(sample)
            except (KeyError, ValueError) as e:
                print(f"Warning: skipping invalid row: {e}", file=sys.stderr)
                continue

    return samples


def print_metrics(metrics: PositionErrorMetrics, label: str = "Horizontal Position") -> None:
    """Print metrics in human-readable format."""
    print(f"{label} Error Metrics:")
    print(f"  Sample Count: {metrics.count}")
    print(f"  Mean Error:   {metrics.mean:.4f} m")
    print(f"  Median Error: {metrics.median:.4f} m")
    print(f"  RMSE:         {metrics.rmse:.4f} m")
    print(f"  P95 Error:    {metrics.p95:.4f} m")
    print(f"  Max Error:    {metrics.max_error:.4f} m")
    print(f"  Min Error:    {metrics.min_error:.4f} m")


def main():
    """Main entry point."""
    if len(sys.argv) < 2:
        print("Usage: position_error_evaluator.py <csv_file>", file=sys.stderr)
        print("", file=sys.stderr)
        print("CSV format: timestamp,estimated_x,estimated_y,ground_truth_x,ground_truth_y", file=sys.stderr)
        return 1

    filepath = sys.argv[1]

    try:
        samples = read_csv(filepath)
    except FileNotFoundError:
        print(f"Error: file not found: {filepath}", file=sys.stderr)
        return 1
    except Exception as e:
        print(f"Error reading file: {e}", file=sys.stderr)
        return 1

    if not samples:
        print("Error: no valid samples found", file=sys.stderr)
        return 1

    metrics = compute_metrics(samples)
    if not metrics:
        print("Error: failed to compute metrics", file=sys.stderr)
        return 1

    print_metrics(metrics)
    vertical_metrics = compute_vertical_metrics(samples)
    if vertical_metrics is not None:
        print()
        print_metrics(vertical_metrics, "Vertical Position")
    return 0


if __name__ == "__main__":
    sys.exit(main())
