import json
import importlib.util
import shutil
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "scripts"
NAV2_PARAMS = ROOT / "src" / "ylhb_base" / "config" / "nav2_params_keepout.yaml"


def _checker_module():
    spec = importlib.util.spec_from_file_location(
        "check_keepout_setup", SCRIPTS / "check_keepout_setup.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _mask_pixels(path):
    data = Path(path).read_bytes()
    marker = b"\n255\n"
    start = data.index(marker) + len(marker)
    return data[start:]


def _copy_assets(tmp_path):
    for name in ("my_map.yaml", "my_map.pgm"):
        shutil.copy(ROOT / "maps" / name, tmp_path / name)
    return tmp_path / "my_map.yaml"


def _base_route():
    return {
        "version": 3,
        "frame_id": "map",
        "map": {
            "yaml": "my_map.yaml",
            "image": "my_map.pgm",
            "resolution": 0.025,
            "origin": [-7.07, -13.3, 0],
            "width": 395,
            "height": 675,
            "image_sha256": "81ade4d8b354272d012789e1d7b79ea11dd6629447ae20a3e579409086bac270",
        },
        "active_route_id": "route_patrol_001",
        "start_pose": {
            "name": "起点",
            "frame_id": "map",
            "pose": {"x": -0.441, "y": 0.001, "yaw": -0.113},
            "location": {
                "type": "map_pose",
                "frame_id": "map",
                "x": -0.441,
                "y": 0.001,
                "yaw": -0.113,
            },
            "publish_initial_pose": True,
            "covariance": {"x": 0.25, "y": 0.25, "yaw": 0.0685},
        },
        "targets": [
            {
                "id": "target_001",
                "name": "巡检点1",
                "pose": {"x": -1.919, "y": -0.726, "yaw": -1.795},
                "location": {
                    "type": "map_pose",
                    "frame_id": "map",
                    "x": -1.919,
                    "y": -0.726,
                    "yaw": -1.795,
                },
                "safety": {
                    "validation_status": "ok",
                    "min_keepout_distance_m": None,
                    "warnings": [],
                },
                "task_duration_sec": 5,
            }
        ],
        "routes": [
            {
                "id": "route_patrol_001",
                "name": "本地巡逻路线",
                "target_ids": ["target_001"],
                "return_to_start": True,
                "loop": {"enabled": True, "wait_sec": 60, "max_cycles": 0},
                "goal_timeout_sec": 600,
                "max_retries_per_checkpoint": 1,
                "failure_policy": "abort_and_return_home",
            }
        ],
        "keepout_zones": [],
        "schedules": [],
        "safety": {
            "validation_status": "ok",
            "min_keepout_distance_m": None,
            "warnings": [],
        },
    }


def _write_route(tmp_path, route):
    path = tmp_path / "route_patrol_001.json"
    path.write_text(json.dumps(route, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return path


def _generate(map_yaml, route, output):
    return subprocess.run(
        [
            sys.executable,
            SCRIPTS / "generate_keepout_mask.py",
            "--map", map_yaml,
            "--route", route,
            "--nav2-params", NAV2_PARAMS,
            "--output-dir", output,
        ],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )


def _check(map_yaml, route, output):
    return subprocess.run(
        [
            sys.executable,
            SCRIPTS / "check_keepout_setup.py",
            "--map", map_yaml,
            "--route", route,
            "--nav2-params", NAV2_PARAMS,
            "--output-dir", output,
        ],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )


def _zone(route, polygon, zone_id="keepout_001"):
    route = json.loads(json.dumps(route))
    route["keepout_zones"] = [
        {
            "id": zone_id,
            "name": zone_id,
            "type": "hard_keepout",
            "enabled": True,
            "mask_padding_m": 0.025,
            "polygon": polygon,
        }
    ]
    return route


def test_checker_preserves_raw_whitespace_weight(tmp_path):
    pgm = tmp_path / "raw.pgm"
    pgm.write_bytes(b"P5\n1 1\n255\n" + bytes([10]))

    assert _checker_module().read_pgm(pgm)[2] == bytes([10])


def test_empty_zones_generate_all_free(tmp_path):
    map_yaml = _copy_assets(tmp_path)
    route = _write_route(tmp_path, _base_route())
    output = tmp_path / "keepout"

    generated = _generate(map_yaml, route, output)

    assert generated.returncode == 0, generated.stdout
    assert "keepout mode: all_free" in generated.stdout
    metadata = json.loads((output / "keepout_masks.metadata.json").read_text(encoding="utf-8"))
    assert metadata["keepout_mode"] == "all_free"
    assert metadata["enabled_hard_keepout_count"] == 0
    assert metadata["zones"] == {}
    for name in ("keepout_global_mask.pgm", "keepout_local_mask.pgm"):
        pixels = _mask_pixels(output / name)
        assert set(pixels) == {0}


def test_checker_accepts_empty_mask(tmp_path):
    map_yaml = _copy_assets(tmp_path)
    route = _write_route(tmp_path, _base_route())
    output = tmp_path / "keepout"

    generated = _generate(map_yaml, route, output)
    assert generated.returncode == 0, generated.stdout

    checked = _check(map_yaml, route, output)
    assert checked.returncode == 0, checked.stdout
    assert "keepout setup OK" in checked.stdout


def test_no_stale_keepout_pixels_after_clearing(tmp_path):
    map_yaml = _copy_assets(tmp_path)
    output = tmp_path / "keepout"

    # First generate with a real keepout zone far from any target.
    with_zone = _zone(
        _base_route(),
        [
            {"x": 1.5, "y": 2.0},
            {"x": 2.5, "y": 2.0},
            {"x": 2.5, "y": 3.0},
            {"x": 1.5, "y": 3.0},
        ],
    )
    route_with_zone = _write_route(tmp_path, with_zone)
    first = _generate(map_yaml, route_with_zone, output)
    assert first.returncode == 0, first.stdout
    global_pixels = _mask_pixels(output / "keepout_global_mask.pgm")
    assert 0 in global_pixels

    # Now clear the zones and regenerate; old black pixels must be overwritten.
    route_empty = _write_route(tmp_path, _base_route())
    second = _generate(map_yaml, route_empty, output)
    assert second.returncode == 0, second.stdout
    for name in ("keepout_global_mask.pgm", "keepout_local_mask.pgm"):
        pixels = _mask_pixels(output / name)
        assert set(pixels) == {0}

    checked = _check(map_yaml, route_empty, output)
    assert checked.returncode == 0, checked.stdout


def test_nonempty_zones_generate_binary_virtual_walls(tmp_path):
    map_yaml = _copy_assets(tmp_path)
    output = tmp_path / "keepout"

    with_zone = _zone(
        _base_route(),
        [
            {"x": 1.5, "y": 2.0},
            {"x": 2.5, "y": 2.0},
            {"x": 2.5, "y": 3.0},
            {"x": 1.5, "y": 3.0},
        ],
    )
    route = _write_route(tmp_path, with_zone)

    generated = _generate(map_yaml, route, output)
    assert generated.returncode == 0, generated.stdout
    assert "keepout mode: active" in generated.stdout
    metadata = json.loads((output / "keepout_masks.metadata.json").read_text(encoding="utf-8"))
    assert metadata["keepout_mode"] == "active"
    assert metadata["enabled_hard_keepout_count"] == 1
    assert set(metadata["zones"].keys()) == {"keepout_001"}
    zone = metadata["zones"]["keepout_001"]
    assert 0.0 <= zone["hard_padding_m"] <= 0.025
    global_pixels = _mask_pixels(output / "keepout_global_mask.pgm")
    local_pixels = _mask_pixels(output / "keepout_local_mask.pgm")
    assert {0, 100} <= set(global_pixels)
    assert set(global_pixels) <= {0, 100}
    assert {0, 100} <= set(local_pixels)
    assert global_pixels == local_pixels
    assert metadata["global_mask"]["weighted_cells"] == 0
    assert metadata["local_mask"]["weighted_cells"] == 0

    checked = _check(map_yaml, route, output)
    assert checked.returncode == 0, checked.stdout
