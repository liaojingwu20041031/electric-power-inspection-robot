import sys
from pathlib import Path

import pytest


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = REPOSITORY_ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from validate_auxiliary_sensor_data import validate_event, validate_mmwave  # noqa: E402


def test_event_validation_reports_gaps_and_rate(tmp_path: Path) -> None:
    source = tmp_path / "events.csv"
    source.write_text(
        "x,y,polarity,timestamp\n"
        "1,1,1,1000\n"
        "2,1,0,2000\n"
        "3,1,1,61000\n",
        encoding="utf-8",
    )
    report = validate_event(source, 20_000)
    assert report["packet_count"] == 2
    assert report["missing_packet_window_count"] == 2
    assert report["max_packet_gap_sec"] == pytest.approx(0.06)
    assert report["mean_event_rate_hz"] == pytest.approx(50.0)


def test_mmwave_validation_reports_frame_loss_rate_and_density(tmp_path: Path) -> None:
    source = tmp_path / "mmwave.csv"
    source.write_text(
        "timestamp_us,frame_id,track_id,x_m,y_m,z_m,radial_velocity_mps,snr_db,confidence\n"
        "1000,10,1,1,0,0,0.4,10,0.9\n"
        "1000,10,2,2,0,0,0.0,8,0.7\n"
        "101000,12,1,1,0,0,0.3,9,0.8\n",
        encoding="utf-8",
    )
    report = validate_mmwave(source, 0.15)
    assert report["frame_count"] == 2
    assert report["missing_frame_id_count"] == 1
    assert report["effective_frame_rate_hz"] == pytest.approx(10.0)
    assert report["mean_detections_per_frame"] == pytest.approx(1.5)
    assert report["max_interframe_gap_sec"] == pytest.approx(0.1)
