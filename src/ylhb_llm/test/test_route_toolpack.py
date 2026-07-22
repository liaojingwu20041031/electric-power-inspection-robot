from pathlib import Path

from ylhb_llm.route_toolpack import RouteCatalog, RouteToolPack
from ylhb_llm.skill_toolpack import SkillToolPack


ROUTE_PATH = Path(__file__).resolve().parents[3] / "maps" / "route_patrol_001.json"
UNCONFIGURED_ROUTE_PATH = ROUTE_PATH.with_name('route_patrol_002.json')


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


def test_skill_toolpack_forwards_refresh_summary_flag(tmp_path):
    path = tmp_path / 'capabilities.yaml'
    path.write_text(
        'capabilities:\n  recover_component:\n    refresh_summary_after: true\n    requires_fresh_diagnostic: true\n',
        encoding='utf-8',
    )

    schema = SkillToolPack.from_file(str(path), {}).tool_schemas()['recover_component']

    assert schema['refresh_summary_after'] is True
    assert schema['requires_fresh_diagnostic'] is True


def test_start_route_schema_preserves_fixed_arguments():
    schemas = SkillToolPack.from_file(
        str(ROUTE_PATH.parents[1] / 'src' / 'ylhb_llm' / 'config' / 'robot_capabilities.yaml'),
        {},
    ).tool_schemas()

    assert schemas['start_route']['fixed_arguments'] == {'profile': 'inspection'}
    assert schemas['generate_local_status_reply']['model_visible'] is False
    assert '怎么进行三维采集与重建' in schemas['search_robot_help']['description']
    assert '是否具备巡逻条件' in schemas['run_self_check']['description']
    assert '机器人端连接条件' in schemas['get_connection_info']['description']


def test_describe_route_reports_whether_inspection_items_are_configured():
    configured = RouteToolPack(RouteCatalog({
        'active_route_id': 'route_1',
        'routes': [{'id': 'route_1', 'target_ids': ['target_1']}],
        'targets': [{'id': 'target_1', 'inspection_items': ['设备外观']}],
    }))
    unconfigured = RouteToolPack(RouteCatalog.from_file(str(UNCONFIGURED_ROUTE_PATH)))

    assert configured.describe_route('route_1')['inspection_configured'] is True
    assert unconfigured.describe_route(
        unconfigured.catalog.data['active_route_id'])['inspection_configured'] is False
