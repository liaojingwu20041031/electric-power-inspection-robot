#!/usr/bin/env python3
"""Small dependency-free reader for named scalar PointCloud2 fields."""

from __future__ import annotations

import math
import struct
from typing import Any, Iterator, Sequence


_DATATYPES = {
    1: ("b", 1),   # INT8
    2: ("B", 1),   # UINT8
    3: ("h", 2),   # INT16
    4: ("H", 2),   # UINT16
    5: ("i", 4),   # INT32
    6: ("I", 4),   # UINT32
    7: ("f", 4),   # FLOAT32
    8: ("d", 8),   # FLOAT64
}


def read_named_points(
    message: Any,
    field_names: Sequence[str],
    *,
    skip_nonfinite: bool = True,
) -> Iterator[tuple[int | float, ...]]:
    """Yield requested scalar fields while respecting rows, offsets and endian.

    The normalized DG event/mmWave contracts use scalar fields only. Rejecting
    array fields keeps parsing explicit and prevents silently shifting offsets.
    """

    requested = tuple(str(name) for name in field_names)
    if not requested or len(set(requested)) != len(requested):
        raise ValueError("field_names must be unique and non-empty")
    available = {str(field.name): field for field in message.fields}
    missing = [name for name in requested if name not in available]
    if missing:
        raise ValueError("missing PointCloud2 fields: " + ",".join(missing))

    point_step = int(message.point_step)
    row_step = int(message.row_step)
    width = int(message.width)
    height = int(message.height)
    if point_step <= 0 or row_step < point_step * width or width < 0 or height < 0:
        raise ValueError("invalid PointCloud2 dimensions or strides")
    raw = bytes(message.data)
    if len(raw) < row_step * height:
        raise ValueError("PointCloud2 data is shorter than declared dimensions")

    endian = ">" if bool(message.is_bigendian) else "<"
    readers: list[tuple[int, struct.Struct]] = []
    for name in requested:
        field = available[name]
        if int(field.count) != 1:
            raise ValueError(f"PointCloud2 field {name} must be scalar")
        datatype = int(field.datatype)
        if datatype not in _DATATYPES:
            raise ValueError(f"PointCloud2 field {name} has unsupported datatype {datatype}")
        format_code, size = _DATATYPES[datatype]
        offset = int(field.offset)
        if offset < 0 or offset + size > point_step:
            raise ValueError(f"PointCloud2 field {name} exceeds point_step")
        readers.append((offset, struct.Struct(endian + format_code)))

    for row in range(height):
        row_offset = row * row_step
        for column in range(width):
            point_offset = row_offset + column * point_step
            values = tuple(reader.unpack_from(raw, point_offset + offset)[0] for offset, reader in readers)
            if skip_nonfinite and any(
                isinstance(value, float) and not math.isfinite(value) for value in values
            ):
                continue
            yield values
