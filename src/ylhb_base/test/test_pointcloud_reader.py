from types import SimpleNamespace
import struct
import sys
from pathlib import Path

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from pointcloud_reader import read_named_points  # noqa: E402


def field(name: str, offset: int, datatype: int, count: int = 1) -> SimpleNamespace:
    return SimpleNamespace(name=name, offset=offset, datatype=datatype, count=count)


def cloud(data: bytes, *, width: int = 2, height: int = 1, bigendian: bool = False) -> SimpleNamespace:
    return SimpleNamespace(
        fields=[field("x", 0, 7), field("track_id", 4, 6)],
        point_step=8,
        row_step=16,
        width=width,
        height=height,
        is_bigendian=bigendian,
        data=data,
    )


def test_reads_selected_little_endian_scalar_fields() -> None:
    message = cloud(struct.pack("<fIfI", 1.25, 7, -2.5, 8))
    assert list(read_named_points(message, ("x", "track_id"))) == [
        pytest.approx((1.25, 7)),
        pytest.approx((-2.5, 8)),
    ]


def test_respects_big_endian_and_skips_nonfinite_points() -> None:
    message = cloud(struct.pack(">fIfI", float("nan"), 7, 3.0, 9), bigendian=True)
    assert list(read_named_points(message, ("x", "track_id"))) == [pytest.approx((3.0, 9))]


def test_rejects_missing_or_truncated_contract() -> None:
    message = cloud(struct.pack("<fI", 1.0, 1))
    with pytest.raises(ValueError, match="shorter"):
        list(read_named_points(message, ("x",)))
    message.data = struct.pack("<fIfI", 1.0, 1, 2.0, 2)
    with pytest.raises(ValueError, match="missing"):
        list(read_named_points(message, ("confidence",)))
