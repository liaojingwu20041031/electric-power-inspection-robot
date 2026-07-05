import copy
import os
from pathlib import Path

import pytest

from ylhb_mobile_bridge import patrol_route_store


get_route = patrol_route_store.get_route
load_route_file = patrol_route_store.load_route_file
validate_route_file = patrol_route_store.validate_route_file
TEST_ROUTE_PATH = (
    Path(__file__).resolve().parents[1]
    / "test"
    / "fixtures"
    / "patrol_routes.json"
)


def valid_route_data():
    return copy.deepcopy(load_route_file(str(TEST_ROUTE_PATH)))


def expect_validation_error(data, message_part):
    with pytest.raises(ValueError, match=message_part):
        validate_route_file(data)


def test_valid_v2_example_passes_and_expands_route():
    data = validate_route_file(valid_route_data())
    route_id = data["active_route_id"]

    route = get_route(data, route_id)
    targets = patrol_route_store.expand_route_targets(
        data,
        route_id,
    )

    assert route["id"] == route_id
    assert [target["id"] for target in targets] == [
        "target_001",
        "target_002",
    ]
    assert targets[0]["pose"] == data["targets"][0]["pose"]
    assert data["start_pose"]["publish_initial_pose"] is True


def test_valid_v3_optional_business_fields_are_preserved():
    data = valid_route_data()
    data["version"] = 3
    data["site"] = {"id": "site_1", "name": "实训站"}
    data["areas"] = [{"id": "area_1", "name": "主变区"}]
    data["targets"][0]["aliases"] = ["巡检点一"]
    data["targets"][0]["area_id"] = "area_1"
    data["targets"][0]["inspection_items"] = ["设备外观"]
    data["routes"][0]["aliases"] = ["默认路线"]
    data["routes"][0]["description"] = "本地巡逻"

    normalized = validate_route_file(data)

    assert normalized["version"] == 3
    assert normalized["site"]["name"] == "实训站"
    assert normalized["areas"][0]["id"] == "area_1"
    assert normalized["targets"][0]["aliases"] == ["巡检点一"]
    assert normalized["targets"][0]["area_id"] == "area_1"
    assert normalized["targets"][0]["inspection_items"] == ["设备外观"]
    assert normalized["routes"][0]["aliases"] == ["默认路线"]
    assert normalized["routes"][0]["description"] == "本地巡逻"


def test_load_route_file_reads_and_validates_json():
    loaded = load_route_file(str(TEST_ROUTE_PATH))

    assert loaded["active_route_id"] == loaded["routes"][0]["id"]


def test_auto_route_path_selects_only_matching_route(tmp_path):
    route_path = tmp_path / "route_patrol_001.json"
    route_path.write_text("{}", encoding="utf-8")
    (tmp_path / "my_map.json").write_text("{}", encoding="utf-8")

    assert patrol_route_store.resolve_route_file_path(
        "auto",
        tmp_path,
    ) == route_path


def test_auto_route_path_selects_highest_route_number(tmp_path):
    first = tmp_path / "route_patrol_001.json"
    latest = tmp_path / "route_patrol_010.json"
    first.write_text("{}", encoding="utf-8")
    latest.write_text("{}", encoding="utf-8")
    os.utime(first, (200.0, 200.0))
    os.utime(latest, (100.0, 100.0))

    assert patrol_route_store.resolve_route_file_path(
        "auto",
        tmp_path,
    ) == latest


def test_auto_route_path_uses_mtime_when_numbers_cannot_be_parsed(tmp_path):
    older = tmp_path / "route_patrol_alpha.json"
    newer = tmp_path / "route_patrol_latest.json"
    older.write_text("{}", encoding="utf-8")
    newer.write_text("{}", encoding="utf-8")
    os.utime(older, (100.0, 100.0))
    os.utime(newer, (200.0, 200.0))

    assert patrol_route_store.resolve_route_file_path(
        "auto",
        tmp_path,
    ) == newer


def test_auto_route_path_fails_clearly_when_no_routes_exist(tmp_path):
    with pytest.raises(ValueError, match="no patrol route files found"):
        patrol_route_store.resolve_route_file_path("auto", tmp_path)


def test_explicit_absolute_route_path_is_preserved(tmp_path):
    route_path = tmp_path / "custom.json"
    route_path.write_text("{}", encoding="utf-8")

    assert patrol_route_store.resolve_route_file_path(
        str(route_path),
        tmp_path,
    ) == route_path


def test_missing_start_pose_fails():
    data = valid_route_data()
    del data["start_pose"]

    expect_validation_error(data, "start_pose")


@pytest.mark.parametrize(
    "pose",
    [
        {"x": 1.0, "y": 2.0},
        {"x": "1", "y": 2.0, "yaw": 0.0},
        {"x": True, "y": 2.0, "yaw": 0.0},
    ],
)
def test_invalid_start_pose_fails(pose):
    data = valid_route_data()
    data["start_pose"]["pose"] = pose

    expect_validation_error(data, "start_pose.pose")


def test_duplicate_target_id_fails():
    data = valid_route_data()
    data["targets"].append(copy.deepcopy(data["targets"][0]))

    expect_validation_error(data, "duplicate target id")


def test_route_referencing_missing_target_fails():
    data = valid_route_data()
    data["routes"][0]["target_ids"].append("missing_target")

    expect_validation_error(data, "unknown target")


def test_non_map_frame_fails():
    data = valid_route_data()
    data["frame_id"] = "odom"

    expect_validation_error(data, "frame_id")


def test_boolean_version_fails():
    data = valid_route_data()
    data["version"] = True

    expect_validation_error(data, "version")


@pytest.mark.parametrize(
    "pose",
    [
        {"x": 1.0, "y": 2.0},
        {"x": "1", "y": 2.0, "yaw": 0.0},
        {"x": True, "y": 2.0, "yaw": 0.0},
    ],
)
def test_invalid_target_pose_fails(pose):
    data = valid_route_data()
    data["targets"][0]["pose"] = pose

    expect_validation_error(data, "targets")


def test_negative_task_duration_fails():
    data = valid_route_data()
    data["targets"][0]["task_duration_sec"] = -0.1

    expect_validation_error(data, "task_duration_sec")


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("enabled", "yes"),
        ("wait_sec", -0.1),
        ("max_cycles", -1),
        ("max_cycles", True),
    ],
)
def test_invalid_loop_configuration_fails(field, value):
    data = valid_route_data()
    data["routes"][0]["loop"][field] = value

    expect_validation_error(data, f"loop.{field}")


def test_return_to_start_requires_start_pose():
    data = valid_route_data()
    del data["start_pose"]

    expect_validation_error(data, "start_pose")


def test_invalid_failure_policy_fails():
    data = valid_route_data()
    data["routes"][0]["failure_policy"] = "continue"

    expect_validation_error(data, "failure_policy")


def test_schedule_referencing_missing_route_fails():
    data = valid_route_data()
    data["schedules"][0]["route_id"] = "missing_route"

    expect_validation_error(data, "unknown route")


@pytest.mark.parametrize(
    "start_pose",
    [
        {
            "name": "起点",
            "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "publish_initial_pose": "yes",
            "covariance": {"x": 0.25, "y": 0.25, "yaw": 0.0685},
        },
        {
            "name": "起点",
            "pose": {"x": 0.0, "y": 0.0, "yaw": False},
            "publish_initial_pose": True,
            "covariance": {"x": 0.25, "y": 0.25, "yaw": 0.0685},
        },
        {
            "name": "起点",
            "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "publish_initial_pose": True,
            "covariance": {"x": -0.1, "y": 0.25, "yaw": 0.0685},
        },
    ],
)
def test_invalid_start_pose_configuration_fails(start_pose):
    data = valid_route_data()
    data["start_pose"] = start_pose

    expect_validation_error(data, "start_pose")
