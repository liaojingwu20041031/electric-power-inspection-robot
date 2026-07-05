from pathlib import Path

from ylhb_llm.route_toolpack import RouteCatalog, RouteToolPack


ROUTE_PATH = Path(__file__).resolve().parents[3] / "maps" / "route_patrol_001.json"


def test_route_toolpack_builds_catalog_from_route_file():
    catalog = RouteCatalog.from_file(str(ROUTE_PATH))
    toolpack = RouteToolPack(catalog)

    assert catalog.route_ids == ["route_patrol_001"]
    assert catalog.target_ids == ["target_001", "target_002", "target_003", "target_004"]
    assert toolpack.tool_schemas()["start_route"]["properties"]["route_id"]["enum"] == ["route_patrol_001"]
    assert toolpack.tool_schemas()["go_to_checkpoint"]["properties"]["target_id"]["enum"][-1] == "target_004"


def test_target_name_resolves_to_target_id():
    catalog = RouteCatalog.from_file(str(ROUTE_PATH))

    assert catalog.resolve_target_id("巡检点3") == "target_003"
