#!/usr/bin/env python3
"""UWB observation data model for HR-RTLS1 integration into MODEL-1.

Designed to support:
- 3 physical anchors (physical_0, physical_1, physical_2)
- 6 logical anchor slots (A0-A5)
- Logical-to-physical mapping with correlation groups
- Proper deduplication for geometry/confidence calculations
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class UwbRangeObservation:
    """Single UWB range measurement from one logical anchor slot.

    Key distinction:
    - logical_anchor_id: A0-A5 (6 slots)
    - physical_source_id: 0-2 (3 actual anchors)
    - correlation_group: groups measurements from same physical source

    For confidence/geometry calculations, MUST deduplicate by physical_source_id
    or correlation_group, NOT by logical_anchor_id.
    """
    timestamp: float
    logical_anchor_id: str  # "A0" through "A5"
    physical_source_id: int  # 0, 1, 2 (actual hardware anchors)
    correlation_group: int  # 0, 1, 2 (for deduplication)
    range_m: float
    valid: bool
    receive_power_dbm: Optional[float] = None
    diagnostics: Optional[str] = None


@dataclass(frozen=True)
class UwbAnchorConfig:
    """Physical anchor position (ground truth or surveyed)."""
    physical_id: int
    x: float
    y: float
    z: float  # Reserved for future, not used in 2D estimation
    active: bool = True


@dataclass
class UwbQualityState:
    """UWB source health state for MODEL-1 fusion."""

    # State machine
    INITIAL = "INITIAL"
    GOOD = "GOOD"
    DEGRADED = "DEGRADED"
    REJECTED = "REJECTED"
    RECOVERING = "RECOVERING"

    state: str = INITIAL

    # Metrics
    last_update_time: Optional[float] = None
    unique_physical_count: int = 0
    geometry_metric: Optional[float] = None
    residual_rms: Optional[float] = None
    position_jump: Optional[float] = None
    innovation: Optional[float] = None

    # Recovery tracking
    consecutive_good_frames: int = 0

    # Reason tracking
    rejection_reason: str = ""


@dataclass(frozen=True)
class Uwb2DPositionEstimate:
    """2D UWB position output from trilateration.

    NOTE: 2D only. No reliable 3D elevation from 3 coplanar anchors.
    NOTE: No yaw estimation from UWB ranges.
    """
    timestamp: float
    x: float
    y: float
    covariance_xx: float
    covariance_yy: float
    covariance_xy: float

    # Quality metrics
    residual_rms: float
    unique_anchor_count: int
    geometry_metric: float  # Higher = better geometry (not collinear)

    # Confidence
    confidence: float  # 0.0 to 1.0
    quality_state: str  # GOOD, DEGRADED, REJECTED


# Default logical-to-physical mapping
DEFAULT_LOGICAL_PHYSICAL_MAPPING = {
    "A0": (0, 0),  # (physical_source_id, correlation_group)
    "A1": (1, 1),
    "A2": (2, 2),
    "A3": (0, 0),  # Mirror of A0
    "A4": (1, 1),  # Mirror of A1
    "A5": (2, 2),  # Mirror of A2
}


# REAL_HARDWARE_PENDING: Actual anchor coordinates not yet surveyed
# These are synthetic placeholders for development/testing
SYNTHETIC_ANCHOR_COORDINATES = [
    UwbAnchorConfig(physical_id=0, x=0.0, y=0.0, z=0.0),
    UwbAnchorConfig(physical_id=1, x=10.0, y=0.0, z=0.0),
    UwbAnchorConfig(physical_id=2, x=5.0, y=8.66, z=0.0),  # ~equilateral triangle
]
