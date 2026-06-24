import importlib.util

import pytest

HAS_PYDANTIC = importlib.util.find_spec("pydantic") is not None

if HAS_PYDANTIC:
    from pydantic import ValidationError
    from ylhb_mobile_bridge.schemas import ApiResponse, MappingSaveRequest

pytestmark = pytest.mark.skipif(
    not HAS_PYDANTIC,
    reason="pydantic is not installed",
)


@pytest.mark.parametrize("map_name", ["my_map", "map-001", "map_001"])
def test_mapping_save_accepts_safe_map_names(map_name):
    assert MappingSaveRequest(map_name=map_name).map_name == map_name


@pytest.mark.parametrize("map_name", ["../map", "a/b", "map.name", ""])
def test_mapping_save_rejects_unsafe_map_names(map_name):
    with pytest.raises(ValidationError):
        MappingSaveRequest(map_name=map_name)


def test_api_response_includes_timestamp_by_default():
    response = ApiResponse(ok=True, message="ok")
    assert isinstance(response.timestamp, float)
