from pathlib import Path
import sys

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = REPO_ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from feature_repeatability_evaluator import (  # noqa: E402
    FrameFeatures,
    compute_frame_pair_repeatability,
    compute_metrics,
)


def frame(
    frame_id: int,
    points: list[tuple[float, float]],
    coordinate_frame: str = "laser",
) -> FrameFeatures:
    return FrameFeatures(
        frame_id=frame_id,
        timestamp=float(frame_id),
        feature_ids=[],
        valid_feature_count=len(points),
        feature_points=points,
        coordinate_frame=coordinate_frame,
    )


def test_spatial_matching_is_one_to_one_and_symmetric():
    first = frame(1, [(0.0, 0.0), (0.05, 0.0)])
    second = frame(2, [(0.02, 0.0)])

    assert compute_frame_pair_repeatability(first, second, 0.12) == pytest.approx(2.0 / 3.0)
    assert compute_frame_pair_repeatability(second, first, 0.12) == pytest.approx(2.0 / 3.0)


def test_zero_feature_pair_is_reported_and_never_counted_as_perfect():
    metrics = compute_metrics(
        [
            frame(1, []),
            frame(2, [(0.0, 0.0)]),
            frame(3, [(0.01, 0.0)]),
        ],
        0.12,
    )

    assert metrics is not None
    assert metrics.zero_feature_pair_count == 1
    assert metrics.total_pairs == 1
    assert metrics.overall_repeatability == pytest.approx(1.0)


def test_frame_change_is_rejected_instead_of_mixing_coordinates():
    with pytest.raises(ValueError, match="coordinate frame changed"):
        compute_metrics(
            [frame(1, [(0.0, 0.0)], "laser"), frame(2, [(0.0, 0.0)], "base_link")]
        )


def test_pool_mean_and_min_are_distinct_and_reproducible():
    metrics = compute_metrics(
        [
            frame(1, [(0.0, 0.0), (1.0, 0.0)]),
            frame(2, [(0.01, 0.0), (1.01, 0.0)]),
            frame(3, [(0.02, 0.0), (3.0, 0.0)]),
        ],
        0.12,
    )

    assert metrics is not None
    assert metrics.per_frame_repeatability == pytest.approx([1.0, 0.5])
    assert metrics.mean_per_frame == pytest.approx(0.75)
    assert metrics.min_repeatability == pytest.approx(0.5)
    assert metrics.overall_repeatability == pytest.approx(0.75)
