import sys
from pathlib import Path

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from event_feature_core import (  # noqa: E402
    EventFeature,
    EventFeatureConfig,
    EventObservation,
    extract_event_features,
    feature_repeatability,
)
from hybrid_vision_assist_core import (  # noqa: E402
    HybridVisionAssistConfig,
    HybridVisionObservation,
    evaluate_hybrid_vision_assist,
)


def observation(x: int, y: int, polarity: int = 1, offset: int = 0) -> EventObservation:
    return EventObservation(x, y, polarity, offset)


def test_dense_cells_become_deterministic_features() -> None:
    config = EventFeatureConfig(
        width=64,
        height=48,
        cell_size_px=8,
        min_events_per_cell=3,
        min_unique_pixels=2,
        max_features=8,
    )
    samples = [
        observation(1, 1),
        observation(2, 1, 0),
        observation(2, 2),
        observation(17, 9),
        observation(18, 9),
        observation(18, 10, 0),
        observation(40, 40),
    ]
    features = extract_event_features(samples, config)
    assert len(features) == 2
    assert features[0].x == pytest.approx(5.0 / 3.0)
    assert features[0].positive_fraction == pytest.approx(2.0 / 3.0)
    assert features[1].unique_pixel_count == 3


def test_sparse_or_hot_single_pixel_activity_is_rejected() -> None:
    config = EventFeatureConfig(
        width=32,
        height=32,
        cell_size_px=8,
        min_events_per_cell=3,
        min_unique_pixels=2,
    )
    samples = [observation(3, 3, offset=index) for index in range(10)]
    assert extract_event_features(samples, config) == []


def test_repeatability_is_one_to_one_and_zero_feature_is_unknown() -> None:
    previous = [
        EventFeature(10.0, 10.0, 5, 3, 0.6),
        EventFeature(11.0, 10.0, 5, 3, 0.6),
    ]
    current = [EventFeature(10.5, 10.0, 6, 4, 0.5)]
    assert feature_repeatability(previous, current, 2.0) == pytest.approx(2.0 / 3.0)
    assert feature_repeatability([], current, 2.0) is None


def test_invalid_event_is_rejected() -> None:
    config = EventFeatureConfig(width=32, height=32)
    with pytest.raises(ValueError, match="outside"):
        extract_event_features([observation(40, 1)], config)


def assist_observation(**overrides: object) -> HybridVisionObservation:
    values: dict[str, object] = {
        "now_sec": 10.0,
        "camera_quality_received_sec": 9.9,
        "features_received_sec": 9.9,
        "mask_received_sec": 9.9,
        "camera_state": "GOOD",
        "event_feature_count": 12,
        "motion_mask_active_fraction": 0.1,
        "camera_intrinsics_valid": True,
        "lidar_extrinsics_valid": True,
        "motion_compensation_ready": True,
        "robot_linear_speed": 0.0,
        "robot_yaw_rate": 0.0,
    }
    values.update(overrides)
    return HybridVisionObservation(**values)  # type: ignore[arg-type]


def test_hybrid_assist_honors_calibration_static_idle_and_fault_boundaries() -> None:
    config = HybridVisionAssistConfig(input_timeout_sec=0.5)

    uncalibrated = evaluate_hybrid_vision_assist(
        assist_observation(lidar_extrinsics_valid=False), config
    )
    assert uncalibrated.state == "DIAGNOSTIC_ONLY"
    assert uncalibrated.reason == "CALIBRATION_REQUIRED"
    assert not uncalibrated.may_modify_lidar

    static_idle = evaluate_hybrid_vision_assist(
        assist_observation(camera_state="STATIC_IDLE", motion_mask_active_fraction=0.0),
        config,
    )
    assert static_idle.state == "READY_IDLE"
    assert static_idle.model2_consumes_event_assist
    assert not static_idle.may_modify_lidar

    fault = evaluate_hybrid_vision_assist(assist_observation(camera_state="FAULT"), config)
    assert fault.state == "UNAVAILABLE"
    assert not fault.model2_consumes_event_assist
    assert not fault.may_modify_lidar

    moving_uncompensated = evaluate_hybrid_vision_assist(
        assist_observation(robot_yaw_rate=0.5, motion_compensation_ready=False), config
    )
    assert moving_uncompensated.reason == "SELF_MOTION_COMPENSATION_REQUIRED"
    assert not moving_uncompensated.may_modify_lidar


def test_hybrid_assist_requires_declared_and_observed_camera_intrinsics() -> None:
    node = (SCRIPTS / "hybrid_vision_assist_node.py").read_text(encoding="utf-8")
    launch = (SCRIPTS.parent / "launch" / "dg_navigation_integration.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'self.declare_parameter("intrinsics_valid", False)' in node
    assert "self._intrinsics_declared_valid and self._intrinsics_valid" in node
    assert '"intrinsics_valid": ParameterValue(event_camera_intrinsics_valid' in launch
    assert (
        'DeclareLaunchArgument(\n'
        '                "event_camera_intrinsics_valid",\n'
        '                default_value="false",'
    ) in launch
