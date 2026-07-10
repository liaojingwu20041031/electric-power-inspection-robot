import json
import shutil
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "scripts"


def copied_assets(tmp_path):
    for name in ("my_map.yaml", "my_map.pgm", "route_patrol_001.json"):
        shutil.copy(ROOT / "maps" / name, tmp_path / name)
    return tmp_path / "my_map.yaml", tmp_path / "route_patrol_001.json"


def test_safety_report_is_read_only(tmp_path):
    map_yaml, route = copied_assets(tmp_path)
    before = route.read_bytes()

    result = subprocess.run(
        [sys.executable, SCRIPTS / "validate_route_safety.py", "--map", map_yaml,
         "--route", route, "--nav2-params", ROOT / "src/ylhb_base/config/nav2_params.yaml", "--report"],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )

    assert result.returncode in (0, 1)
    assert json.loads(result.stdout)["status"] in {"ok", "warning", "unsafe"}
    assert route.read_bytes() == before


def test_keepout_metadata_rejects_changed_route(tmp_path):
    map_yaml, route = copied_assets(tmp_path)
    output = tmp_path / "keepout"
    generated = subprocess.run(
        [sys.executable, SCRIPTS / "generate_keepout_mask.py", "--map", map_yaml,
         "--route", route, "--output-dir", output, "--name", "mask"],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    )
    assert generated.returncode == 0, generated.stdout
    route.write_text(route.read_text(encoding="utf-8") + "\n", encoding="utf-8")

    checked = subprocess.run(
        [sys.executable, SCRIPTS / "check_keepout_setup.py", "--map", map_yaml,
         "--route", route, "--mask", output / "mask.yaml",
         "--nav2-params", ROOT / "src/ylhb_base/config/nav2_params.yaml"],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    )
    assert checked.returncode != 0
    assert "route hash differs" in checked.stdout
