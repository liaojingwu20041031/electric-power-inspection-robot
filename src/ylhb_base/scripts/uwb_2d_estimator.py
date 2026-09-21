#!/usr/bin/env python3
"""2D UWB position estimator via trilateration.

Requirements:
- Minimum 3 unique physical anchors
- 2D only (no Z estimation)
- Deduplication by physical_source_id
- Geometry quality check (not collinear)
"""

from __future__ import annotations
import math
from typing import List, Optional, Tuple
import sys
from pathlib import Path

# Add local scripts directory to path
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from uwb_data_model import (
    UwbRangeObservation,
    UwbAnchorConfig,
    Uwb2DPositionEstimate,
)


def estimate_2d_position(
    observations: List[UwbRangeObservation],
    anchors: List[UwbAnchorConfig],
    min_anchors: int = 3,
) -> Optional[Uwb2DPositionEstimate]:
    """Estimate 2D position from UWB range measurements.

    Args:
        observations: Range measurements (may include duplicates from logical slots)
        anchors: Physical anchor positions
        min_anchors: Minimum unique physical anchors required

    Returns:
        Position estimate or None if insufficient/invalid data
    """
    # Deduplicate by physical_source_id
    unique_obs = {}
    for obs in observations:
        if obs.valid and obs.physical_source_id not in unique_obs:
            unique_obs[obs.physical_source_id] = obs

    if len(unique_obs) < min_anchors:
        return None

    # Get anchor positions
    anchor_map = {a.physical_id: a for a in anchors if a.active}

    # Build trilateration inputs
    points = []
    ranges = []
    for phys_id, obs in unique_obs.items():
        if phys_id not in anchor_map:
            continue
        anchor = anchor_map[phys_id]
        points.append((anchor.x, anchor.y))
        ranges.append(obs.range_m)

    if len(points) < min_anchors:
        return None

    # Check geometry (not collinear)
    geometry_metric = compute_geometry_metric(points)
    if geometry_metric < 0.1:  # Too close to collinear
        return None

    # Trilateration (least squares)
    x_est, y_est, residual = trilaterate_2d_ls(points, ranges)

    if x_est is None:
        return None

    # Estimate covariance (simplified)
    cov_xx = (residual + 0.1) ** 2
    cov_yy = (residual + 0.1) ** 2
    cov_xy = 0.0

    # Confidence based on residual and geometry
    confidence = compute_confidence(residual, geometry_metric, len(points))

    # Quality state
    if residual > 1.0:
        quality = "REJECTED"
    elif residual > 0.5:
        quality = "DEGRADED"
    else:
        quality = "GOOD"

    timestamp = max(obs.timestamp for obs in unique_obs.values())

    return Uwb2DPositionEstimate(
        timestamp=timestamp,
        x=x_est,
        y=y_est,
        covariance_xx=cov_xx,
        covariance_yy=cov_yy,
        covariance_xy=cov_xy,
        residual_rms=residual,
        unique_anchor_count=len(points),
        geometry_metric=geometry_metric,
        confidence=confidence,
        quality_state=quality,
    )


def compute_geometry_metric(points: List[Tuple[float, float]]) -> float:
    """Compute geometry quality metric.

    Returns value in [0, 1]:
    - 0 = collinear (bad)
    - 1 = good spread
    """
    if len(points) < 3:
        return 0.0

    # Use area of triangle formed by first 3 points
    p0, p1, p2 = points[0], points[1], points[2]

    # Triangle area via cross product
    area = abs(
        (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p2[0] - p0[0]) * (p1[1] - p0[1])
    ) / 2.0

    # Normalize by perimeter
    d01 = math.sqrt((p1[0] - p0[0])**2 + (p1[1] - p0[1])**2)
    d12 = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
    d20 = math.sqrt((p0[0] - p2[0])**2 + (p0[1] - p2[1])**2)
    perimeter = d01 + d12 + d20

    if perimeter < 1e-6:
        return 0.0

    # Normalized metric
    metric = 4.0 * math.sqrt(3.0) * area / (perimeter ** 2)
    return min(1.0, metric)


def trilaterate_2d_ls(
    anchors: List[Tuple[float, float]],
    ranges: List[float],
) -> Tuple[Optional[float], Optional[float], float]:
    """2D trilateration using least squares.

    Returns:
        (x, y, residual_rms) or (None, None, inf) if failed
    """
    if len(anchors) < 3:
        return None, None, float('inf')

    # Least squares setup: minimize sum of squared errors
    # (x - ax_i)^2 + (y - ay_i)^2 = r_i^2

    # Linearize around first anchor
    ax0, ay0 = anchors[0]
    r0 = ranges[0]

    A = []
    b = []

    for i in range(1, len(anchors)):
        ax_i, ay_i = anchors[i]
        r_i = ranges[i]

        # Linear equation
        A.append([2*(ax_i - ax0), 2*(ay_i - ay0)])
        b.append(
            r0**2 - r_i**2 - ax0**2 + ax_i**2 - ay0**2 + ay_i**2
        )

    # Solve using numpy-free approach (2x2 or 3x2 system)
    if len(A) == 2:
        # Exact solution for 3 anchors
        a11, a12 = A[0]
        a21, a22 = A[1]
        b1, b2 = b[0], b[1]

        det = a11 * a22 - a12 * a21
        if abs(det) < 1e-10:
            return None, None, float('inf')

        x = (b1 * a22 - b2 * a12) / det
        y = (a11 * b2 - a21 * b1) / det

    else:
        # Overdetermined (more than 3 anchors) - use normal equations
        # A^T A x = A^T b
        At_A = [[0.0, 0.0], [0.0, 0.0]]
        At_b = [0.0, 0.0]

        for i in range(len(A)):
            At_A[0][0] += A[i][0] * A[i][0]
            At_A[0][1] += A[i][0] * A[i][1]
            At_A[1][0] += A[i][1] * A[i][0]
            At_A[1][1] += A[i][1] * A[i][1]
            At_b[0] += A[i][0] * b[i]
            At_b[1] += A[i][1] * b[i]

        det = At_A[0][0] * At_A[1][1] - At_A[0][1] * At_A[1][0]
        if abs(det) < 1e-10:
            return None, None, float('inf')

        x = (At_b[0] * At_A[1][1] - At_b[1] * At_A[0][1]) / det
        y = (At_A[0][0] * At_b[1] - At_A[1][0] * At_b[0]) / det

    # Compute residual
    residual_sum = 0.0
    for i, (ax, ay) in enumerate(anchors):
        predicted_range = math.sqrt((x - ax)**2 + (y - ay)**2)
        error = predicted_range - ranges[i]
        residual_sum += error ** 2

    residual_rms = math.sqrt(residual_sum / len(anchors))

    return x, y, residual_rms


def compute_confidence(residual: float, geometry: float, anchor_count: int) -> float:
    """Compute position confidence [0, 1]."""
    # Base confidence from residual
    conf_residual = max(0.0, 1.0 - residual / 2.0)

    # Geometry factor
    conf_geometry = geometry

    # Anchor count factor (more is better)
    conf_count = min(1.0, anchor_count / 4.0)

    # Combined
    confidence = conf_residual * conf_geometry * conf_count
    return max(0.0, min(1.0, confidence))


if __name__ == "__main__":
    # Test with synthetic data
    from uwb_data_model import SYNTHETIC_ANCHOR_COORDINATES, DEFAULT_LOGICAL_PHYSICAL_MAPPING

    # Create test observations (6 logical, 3 physical)
    test_obs = [
        UwbRangeObservation(
            timestamp=1.0,
            logical_anchor_id="A0",
            physical_source_id=0,
            correlation_group=0,
            range_m=5.0,
            valid=True,
        ),
        UwbRangeObservation(
            timestamp=1.0,
            logical_anchor_id="A1",
            physical_source_id=1,
            correlation_group=1,
            range_m=7.0,
            valid=True,
        ),
        UwbRangeObservation(
            timestamp=1.0,
            logical_anchor_id="A2",
            physical_source_id=2,
            correlation_group=2,
            range_m=6.0,
            valid=True,
        ),
        UwbRangeObservation(
            timestamp=1.0,
            logical_anchor_id="A3",  # Duplicate of A0
            physical_source_id=0,
            correlation_group=0,
            range_m=5.0,
            valid=True,
        ),
        UwbRangeObservation(
            timestamp=1.0,
            logical_anchor_id="A4",  # Duplicate of A1
            physical_source_id=1,
            correlation_group=1,
            range_m=7.0,
            valid=True,
        ),
        UwbRangeObservation(
            timestamp=1.0,
            logical_anchor_id="A5",  # Duplicate of A2
            physical_source_id=2,
            correlation_group=2,
            range_m=6.0,
            valid=True,
        ),
    ]

    print("Test: UWB 2D Position Estimation")
    print(f"Input: {len(test_obs)} logical observations")

    result = estimate_2d_position(test_obs, SYNTHETIC_ANCHOR_COORDINATES)

    if result:
        print(f"\n✅ Estimation successful")
        print(f"   Position: ({result.x:.2f}, {result.y:.2f})")
        print(f"   Unique anchors: {result.unique_anchor_count} (should be 3)")
        print(f"   Geometry metric: {result.geometry_metric:.3f}")
        print(f"   Residual RMS: {result.residual_rms:.3f}m")
        print(f"   Confidence: {result.confidence:.3f}")
        print(f"   Quality: {result.quality_state}")

        if result.unique_anchor_count == 3:
            print("\n✅ Deduplication working (6 logical → 3 unique physical)")
        else:
            print(f"\n❌ Deduplication failed: expected 3, got {result.unique_anchor_count}")
    else:
        print("\n❌ Estimation failed")
