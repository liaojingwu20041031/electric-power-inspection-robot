import json
from pathlib import Path

import pytest

from ylhb_3d_mapping import zed_3d_asset_manager as assets


def write_asset(root: Path, name: str, metadata: dict) -> Path:
    out = root / name
    out.mkdir(parents=True)
    (out / 'metadata.json').write_text(json.dumps({'session_id': name, 'output_dir': str(out), **metadata}), encoding='utf-8')
    return out


def test_asset_manager_lists_renames_trashes_and_sets_latest(tmp_path):
    svo = tmp_path / 'runs' / '3d_capture'
    write_asset(svo, 'capture_1', {'svo_file': str(svo / 'capture_1' / 'capture.svo2'), 'svo_frame_count': 7})

    listed = assets.list_assets(str(svo), 'capture')
    assert listed[0]['session_id'] == 'capture_1'
    assert listed[0]['type'] == 'capture'

    renamed = assets.rename_asset(str(svo), 'capture_1', 'Panel A')
    assert renamed['display_name'] == 'Panel A'
    assert assets.get_asset_info(str(svo), 'capture_1')['display_name'] == 'Panel A'

    latest = assets.set_latest_asset(str(svo), 'capture_1')
    assert latest['session_id'] == 'capture_1'
    assert json.loads((svo / 'latest.json').read_text())['display_name'] == 'Panel A'

    trashed = assets.delete_asset(str(svo), 'capture_1')
    assert Path(trashed['trash_dir']).exists()
    assert not (svo / 'capture_1').exists()


def test_asset_manager_rejects_path_traversal(tmp_path):
    root = tmp_path / 'runs' / '3d_capture'
    root.mkdir(parents=True)

    with pytest.raises(ValueError):
        assets.get_asset_info(str(root), '../outside')
