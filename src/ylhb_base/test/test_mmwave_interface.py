import sys
from pathlib import Path

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from mmwave_interface import (  # noqa: E402
    MmwaveInterfaceError,
    interface_contract,
    iter_normalized_mmwave_detections,
    normalize_mmwave_detection,
)


def record(frame: int = 3, timestamp: int = 1000) -> dict[str, object]:
    return {
        "timestamp_us": timestamp,
        "frame_id": frame,
        "track_id": 7,
        "x_m": 1.2,
        "y_m": -0.3,
        "z_m": 0.1,
        "radial_velocity_mps": 0.4,
        "snr_db": 18.0,
        "confidence": 0.9,
    }


def test_normalizes_vendor_record_with_explicit_field_map() -> None:
    vendor = {
        "ts": "1000",
        "frame": "3",
        "track": "7",
        "px_m": 1.2,
        "py_m": -0.3,
        "pz_m": 0.1,
        "vr_mps": 0.4,
        "snr": 18.0,
        "probability": 0.9,
    }
    mapped = {
        "timestamp_us": "ts",
        "frame_id": "frame",
        "track_id": "track",
        "x_m": "px_m",
        "y_m": "py_m",
        "z_m": "pz_m",
        "radial_velocity_mps": "vr_mps",
        "snr_db": "snr",
        "confidence": "probability",
    }
    detection = normalize_mmwave_detection(vendor, field_map=mapped)
    assert detection.frame_id == 3
    assert detection.x_m == pytest.approx(1.2)


def test_rejects_implicit_units_and_invalid_quality() -> None:
    with pytest.raises(MmwaveInterfaceError, match="missing normalized field"):
        normalize_mmwave_detection({"x": 1.0})
    invalid = record()
    invalid["confidence"] = 1.2
    with pytest.raises(MmwaveInterfaceError, match="confidence"):
        normalize_mmwave_detection(invalid)


def test_stream_requires_monotonic_contiguous_frames() -> None:
    assert len(list(iter_normalized_mmwave_detections([record(), record(timestamp=1001)]))) == 2
    with pytest.raises(MmwaveInterfaceError, match="not monotonic"):
        list(iter_normalized_mmwave_detections([record(timestamp=1001), record(timestamp=1000)]))
    with pytest.raises(MmwaveInterfaceError, match="not contiguous"):
        list(iter_normalized_mmwave_detections([record(frame=1), record(frame=2), record(frame=1)]))


def test_contract_is_observation_only_and_stable() -> None:
    contract = interface_contract()
    assert contract["data_topic"] == "/dg/mmwave/points"
    assert contract["frame_id"] == "mmwave_link"
    assert contract["observation_only"] is True
    assert contract["publishes_cmd_vel"] is False
    assert contract["dynamic_mask_default"] is False
