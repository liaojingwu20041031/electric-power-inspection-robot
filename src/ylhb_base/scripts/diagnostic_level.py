"""Normalize diagnostic_msgs/msg/DiagnosticStatus.level across Python bindings."""

from __future__ import annotations

from typing import Any


def normalize_diagnostic_level(level: Any, default: int | None = None) -> int:
    """Return a DiagnosticStatus uint8 level as a plain integer.

    ROS2 Humble's generated Python message exposes the uint8 field as a
    one-byte ``bytes`` object, while test doubles and older bindings commonly
    use ``int``. ``default`` is used by ROS callbacks to fail closed on
    malformed input; omitting it raises ``ValueError`` for strict callers.
    """
    try:
        if isinstance(level, bool):
            raise TypeError("bool is not a diagnostic level")
        if isinstance(level, int):
            value = level
        elif isinstance(level, (bytes, bytearray, memoryview)):
            if len(level) != 1:
                raise ValueError("diagnostic level byte sequence must have length 1")
            value = int(level[0])
        elif isinstance(level, str):
            value = int(level.strip(), 10)
        else:
            raise TypeError(f"unsupported diagnostic level type: {type(level).__name__}")
        if value < 0 or value > 255:
            raise ValueError("diagnostic level must be in uint8 range")
        return value
    except (TypeError, ValueError, IndexError) as exc:
        if default is not None:
            return int(default)
        raise ValueError(f"invalid diagnostic level: {level!r}") from exc
