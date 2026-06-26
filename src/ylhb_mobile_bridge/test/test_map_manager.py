import base64
import io
from pathlib import Path
from unittest.mock import patch

import pytest
from PIL import Image

from ylhb_mobile_bridge.map_manager import MapManager, MapManagerError


def write_map_pair(
    directory: Path,
    name: str,
    image: str | None = None,
) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    (directory / f"{name}.yaml").write_text(
        "\n".join(
            [
                f"image: {image or f'{name}.pgm'}",
                "resolution: 0.05",
                "origin: [1.0, 2.0, 0.0]",
                "",
            ]
        ),
        encoding="utf-8",
    )
    (directory / f"{name}.pgm").write_bytes(b"P5\n1 1\n255\n\x00")


def test_list_maps_returns_valid_pairs_and_ignores_other_files(tmp_path):
    write_map_pair(tmp_path, "my_map")
    write_map_pair(tmp_path, "factory_floor_1")
    (tmp_path / "route_patrol_001.json").write_text("{}", encoding="utf-8")

    maps = MapManager(tmp_path / "my_map").list_maps()

    assert maps["count"] == 2
    by_name = {item["name"]: item for item in maps["maps"]}
    assert set(by_name) == {"factory_floor_1", "my_map"}
    assert by_name["my_map"]["is_default"] is True
    assert by_name["factory_floor_1"]["valid"] is True
    assert by_name["factory_floor_1"]["resolution"] == 0.05
    assert by_name["factory_floor_1"]["origin"] == [1.0, 2.0, 0.0]
    assert by_name["factory_floor_1"]["size_bytes"] > 0


def test_list_maps_marks_incomplete_broken_and_unsafe_maps_invalid(tmp_path):
    write_map_pair(tmp_path, "missing_pgm")
    (tmp_path / "missing_pgm.pgm").unlink()
    write_map_pair(tmp_path, "bad_yaml")
    (tmp_path / "bad_yaml.yaml").write_text("image: [", encoding="utf-8")
    write_map_pair(tmp_path, "wrong_image", image="../outside.pgm")
    write_map_pair(tmp_path, "linked")
    (tmp_path / "linked.pgm").unlink()
    (tmp_path / "linked.pgm").symlink_to(tmp_path / "my_real_map.pgm")

    maps = MapManager(tmp_path / "my_map").list_maps()["maps"]
    by_name = {item["name"]: item for item in maps}

    assert "pgm_missing" in by_name["missing_pgm"]["issues"]
    assert "yaml_invalid" in by_name["bad_yaml"]["issues"]
    assert "image_reference_invalid" in by_name["wrong_image"]["issues"]
    assert "symlink_not_allowed" in by_name["linked"]["issues"]
    assert all(item["valid"] is False for item in by_name.values())


def test_rename_map_moves_pair_and_updates_yaml_image(tmp_path):
    write_map_pair(tmp_path, "factory")
    manager = MapManager(tmp_path / "my_map")

    result = manager.rename_map("factory", "factory_floor_1")

    assert result["name"] == "factory_floor_1"
    assert sorted(result["renamed"]) == [
        "factory.pgm",
        "factory.yaml",
    ]
    assert not (tmp_path / "factory.yaml").exists()
    assert not (tmp_path / "factory.pgm").exists()
    assert (tmp_path / "factory_floor_1.pgm").exists()
    assert "image: factory_floor_1.pgm" in (
        tmp_path / "factory_floor_1.yaml"
    ).read_text(encoding="utf-8")


def test_rename_rejects_existing_target_invalid_source_and_default(tmp_path):
    write_map_pair(tmp_path, "my_map")
    write_map_pair(tmp_path, "factory")
    write_map_pair(tmp_path, "taken")
    (tmp_path / "factory.pgm").unlink()
    manager = MapManager(tmp_path / "my_map")

    with pytest.raises(MapManagerError) as missing:
        manager.rename_map("missing", "new_map")
    assert missing.value.error == "map_not_found"

    with pytest.raises(MapManagerError) as existing:
        manager.rename_map("taken", "my_map")
    assert existing.value.error == "map_exists"

    with pytest.raises(MapManagerError) as invalid_pair:
        manager.rename_map("factory", "new_map")
    assert invalid_pair.value.error == "invalid_map_pair"

    with pytest.raises(MapManagerError) as protected:
        manager.rename_map("my_map", "new_map")
    assert protected.value.error == "default_map_protected"


def test_rename_rolls_back_if_yaml_update_fails(tmp_path):
    write_map_pair(tmp_path, "factory")
    manager = MapManager(tmp_path / "my_map")
    real_write_text = Path.write_text

    def fail_new_yaml(path, *args, **kwargs):
        if path.name == "new_factory.yaml":
            raise OSError("disk full")
        return real_write_text(path, *args, **kwargs)

    with patch.object(Path, "write_text", fail_new_yaml):
        with pytest.raises(MapManagerError) as exc:
            manager.rename_map("factory", "new_factory")

    assert exc.value.error == "map_operation_failed"
    assert (tmp_path / "factory.yaml").exists()
    assert (tmp_path / "factory.pgm").exists()
    assert not (tmp_path / "new_factory.yaml").exists()
    assert not (tmp_path / "new_factory.pgm").exists()
    assert "image: factory.pgm" in (tmp_path / "factory.yaml").read_text(
        encoding="utf-8"
    )


def test_delete_removes_complete_and_incomplete_maps(tmp_path):
    write_map_pair(tmp_path, "my_map")
    write_map_pair(tmp_path, "factory")
    (tmp_path / "partial.yaml").write_text(
        "image: partial.pgm\n",
        encoding="utf-8",
    )
    manager = MapManager(tmp_path / "my_map")

    complete = manager.delete_map("factory")
    partial = manager.delete_map("partial")

    assert sorted(complete["deleted"]) == [
        "factory.pgm",
        "factory.yaml",
    ]
    assert partial["deleted"] == ["partial.yaml"]
    assert not (tmp_path / "factory.yaml").exists()
    assert not (tmp_path / "partial.yaml").exists()
    with pytest.raises(MapManagerError) as protected:
        manager.delete_map("my_map")
    assert protected.value.error == "default_map_protected"


def test_preview_map_returns_png_base64_and_metadata(tmp_path):
    write_map_pair(tmp_path, "factory")
    manager = MapManager(tmp_path / "my_map")

    preview = manager.preview_map("factory", max_size_px=32)

    assert preview["map_meta"]["resolution"] == 0.05
    assert preview["map_meta"]["origin"] == [1.0, 2.0, 0.0]
    assert preview["map_meta"]["yaml_file"] == "factory.yaml"
    assert preview["map_meta"]["pgm_file"] == "factory.pgm"
    assert preview["png_base64"]
    Image.open(io.BytesIO(base64.b64decode(preview["png_base64"]))).verify()


def test_confirm_default_archives_old_default_and_routes(tmp_path):
    write_map_pair(tmp_path, "my_map")
    write_map_pair(tmp_path, "factory")
    (tmp_path / "route_patrol_001.json").write_text("{}", encoding="utf-8")
    manager = MapManager(tmp_path / "my_map")

    result = manager.confirm_default("factory")

    assert result["changed"] is True
    assert result["default"]["name"] == "my_map"
    assert (tmp_path / "my_map.yaml").exists()
    assert (tmp_path / "my_map.pgm").exists()
    assert "image: my_map.pgm" in (tmp_path / "my_map.yaml").read_text(
        encoding="utf-8"
    )
    assert not (tmp_path / "factory.yaml").exists()
    assert not (tmp_path / "factory.pgm").exists()
    assert not list(tmp_path.glob("route_patrol_*.json"))
    assert result["archived_previous_map"]["yaml_file"].startswith(
        "my_map_deprecated_"
    )
    archived_yaml = (
        tmp_path / result["archived_previous_map"]["yaml_file"]
    )
    archived_pgm = result["archived_previous_map"]["pgm_file"]
    assert f"image: {archived_pgm}" in archived_yaml.read_text(
        encoding="utf-8"
    )
    assert result["archived_routes"][0]["to"].startswith(
        "deprecated_route_patrol_001_"
    )


def test_confirm_default_is_noop_for_current_default(tmp_path):
    write_map_pair(tmp_path, "my_map")
    manager = MapManager(tmp_path / "my_map")

    result = manager.confirm_default("my_map")

    assert result["changed"] is False
    assert result["default"]["name"] == "my_map"


def test_confirm_default_rolls_back_when_route_archive_fails(tmp_path):
    write_map_pair(tmp_path, "my_map")
    write_map_pair(tmp_path, "factory")
    route_path = tmp_path / "route_patrol_001.json"
    route_path.write_text("{}", encoding="utf-8")
    manager = MapManager(tmp_path / "my_map")
    real_rename = Path.rename

    def fail_route_archive(path, target):
        if path.name == "route_patrol_001.json":
            raise OSError("route archive failed")
        return real_rename(path, target)

    with patch.object(Path, "rename", fail_route_archive):
        with pytest.raises(MapManagerError) as exc:
            manager.confirm_default("factory")

    assert exc.value.error == "map_operation_failed"
    assert route_path.exists()
    assert (tmp_path / "my_map.yaml").exists()
    assert (tmp_path / "my_map.pgm").exists()
    assert (tmp_path / "factory.yaml").exists()
    assert (tmp_path / "factory.pgm").exists()
    assert not list(tmp_path.glob("my_map_deprecated_*"))
    assert "image: factory.pgm" in (tmp_path / "factory.yaml").read_text(
        encoding="utf-8"
    )


@pytest.mark.parametrize("name", ["../bad", "bad.name", "", "bad/name"])
def test_names_must_be_simple_stems(tmp_path, name):
    manager = MapManager(tmp_path / "my_map")

    with pytest.raises(MapManagerError) as exc:
        manager.delete_map(name)

    assert exc.value.error == "invalid_map_name"
