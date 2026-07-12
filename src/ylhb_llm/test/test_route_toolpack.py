from pathlib import Path

from ylhb_llm.route_toolpack import RouteCatalog, RouteToolPack
from ylhb_llm.skill_toolpack import SkillToolPack


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


def test_auto_route_catalog_records_resolved_file_and_exports_closed_schemas():
    catalog = RouteCatalog.from_file('auto', route_directory=ROUTE_PATH.parent)
    route_schemas = RouteToolPack(catalog).tool_schemas()
    schemas = SkillToolPack.from_file(
        str(ROUTE_PATH.parents[1] / 'src' / 'ylhb_llm' / 'config' / 'robot_capabilities.yaml'),
        route_schemas,
    ).tool_schemas()

    assert catalog.route_file_path == str(ROUTE_PATH)
    assert schemas['start_route']['properties']['route_id']['enum'] == ['route_patrol_001']
    assert all(schema['additionalProperties'] is False for schema in schemas.values())
