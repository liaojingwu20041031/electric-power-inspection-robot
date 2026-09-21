#!/usr/bin/env python3
"""Freshness helpers for pose publishers that update only after motion."""

from __future__ import annotations

import math
from typing import Sequence


Pose2D = tuple[float, float, float]


def _finite_pose(pose: Sequence[float] | None) -> bool:
    if pose is None or len(pose) != 3:
        return False
    return all(math.isfinite(float(value)) for value in pose)


def _wrapped_angle_delta(first: float, second: float) -> float:
    return (float(second) - float(first) + math.pi) % (2.0 * math.pi) - math.pi


def motion_aware_pose_fresh(
    message_recent: bool | None,
    reference_odom_pose: Sequence[float] | None,
    current_odom_pose: Sequence[float] | None,
    max_translation: float,
    max_rotation: float,
) -> bool | None:
    """Keep a stale pose usable while odometry has not moved since it arrived.

    ``message_recent`` retains the normal wall-clock freshness result. When it
    is stale, a known odometry reference and current pose allow stationary
    AMCL-like publishers to remain valid. Missing or invalid odometry remains
    conservative and returns ``False``.
    """
    if message_recent is None or message_recent is True:
        return message_recent
    if max_translation < 0.0 or max_rotation < 0.0:
        raise ValueError("motion tolerances must be non-negative")
    if not _finite_pose(reference_odom_pose) or not _finite_pose(current_odom_pose):
        return False
    reference = tuple(float(value) for value in reference_odom_pose)
    current = tuple(float(value) for value in current_odom_pose)
    translation = math.hypot(current[0] - reference[0], current[1] - reference[1])
    rotation = abs(_wrapped_angle_delta(reference[2], current[2]))
    return translation <= max_translation and rotation <= max_rotation
