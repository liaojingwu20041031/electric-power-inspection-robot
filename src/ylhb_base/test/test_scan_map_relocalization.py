import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "scan_map_relocalization_node.py"
)


def load_scan_map_module():
    spec = importlib.util.spec_from_file_location("scan_map_relocalization_node", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_initialpose_must_be_in_map_frame():
    scan_map = load_scan_map_module()

    assert scan_map.is_supported_initialpose_frame("map") is True
    assert scan_map.is_supported_initialpose_frame("/map") is True
    assert scan_map.is_supported_initialpose_frame("odom") is False
    assert scan_map.is_supported_initialpose_frame("") is False


def test_initial_pose_publisher_uses_transient_local_reliable_qos():
    scan_map = load_scan_map_module()
    qos = scan_map.initial_pose_qos_profile()

    assert qos.depth == 10
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_distance_field_uses_eight_connected_grid_distances():
    scan_map = load_scan_map_module()
    occupied = np.zeros((5, 5), dtype=bool)
    occupied[2, 2] = True

    distances = scan_map.approximate_distance_field(occupied, resolution=0.5)

    assert distances[2, 2] == 0.0
    assert distances[2, 3] == pytest.approx(0.5)
    assert distances[3, 3] == pytest.approx(math.sqrt(0.5))
    assert distances[0, 0] == pytest.approx(math.sqrt(2.0))


def test_scan_match_score_rewards_close_inlier_points():
    scan_map = load_scan_map_module()
    distances = np.full((9, 9), 2.0, dtype=float)
    distances[4, 4] = 0.0
    distances[4, 5] = 0.05
    distances[5, 4] = 0.05
    origin = (-0.4, -0.4)

    good = scan_map.score_scan_points(
        [(0.0, 0.0), (0.05, 0.0), (0.0, 0.05)],
        pose=(0.0, 0.0, 0.0),
        distance_field=distances,
        origin=origin,
        resolution=0.1,
        inlier_distance=0.15,
        max_mean_distance=0.4,
    )
    bad = scan_map.score_scan_points(
        [(0.0, 0.0), (0.05, 0.0), (0.0, 0.05)],
        pose=(0.3, 0.3, 0.0),
        distance_field=distances,
        origin=origin,
        resolution=0.1,
        inlier_distance=0.15,
        max_mean_distance=0.4,
    )

    assert good.score > 0.80
    assert good.inlier_ratio == 1.0
    assert bad.score < good.score
    assert bad.inlier_ratio < good.inlier_ratio


def test_refinement_returns_none_when_score_is_below_threshold():
    scan_map = load_scan_map_module()
    distances = np.full((5, 5), 2.0, dtype=float)

    result = scan_map.refine_pose_near_seed(
        scan_points=[(0.0, 0.0), (0.1, 0.0)],
        seed_pose=(0.0, 0.0, 0.0),
        distance_field=distances,
        origin=(-0.25, -0.25),
        resolution=0.1,
        min_score=0.5,
    )

    assert result is None
