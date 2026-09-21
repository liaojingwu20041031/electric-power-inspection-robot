import sys
from pathlib import Path

import pytest


sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from diagnostic_level import normalize_diagnostic_level  # noqa: E402


@pytest.mark.parametrize(
    "raw, expected",
    [
        (0, 0),
        (1, 1),
        (2, 2),
        (3, 3),
        (b"\x00", 0),
        (b"\x01", 1),
        (b"\x02", 2),
        (b"\x03", 3),
        (bytearray(b"\x02"), 2),
        (memoryview(b"\x03"), 3),
        ("2", 2),
    ],
)
def test_level_representations_are_equivalent(raw, expected):
    assert normalize_diagnostic_level(raw) == expected


@pytest.mark.parametrize("raw", [True, b"", b"\x00\x01", -1, 256, object()])
def test_invalid_level_rejected(raw):
    with pytest.raises(ValueError):
        normalize_diagnostic_level(raw)


def test_callback_fallback_is_fail_closed():
    assert normalize_diagnostic_level(object(), default=3) == 3
