import json
import shutil
import time
from pathlib import Path
from typing import Any, Dict, List


def _root(root: str) -> Path:
    return Path(root).expanduser().resolve()


def _asset_dir(root: str, session_id: str) -> Path:
    base = _root(root)
    path = (base / session_id).resolve()
    if path == base or base not in path.parents:
        raise ValueError(f'asset path escapes root: {session_id}')
    return path


def _read_json(path: Path) -> Any:
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except Exception:
        return {}


def _write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def _asset_type(session_id: str, requested: str) -> str:
    if requested:
        return requested
    if session_id.startswith('reconstruct_'):
        return 'reconstruct'
    return 'capture'


def _normalize(root: str, session_id: str, payload: Dict[str, Any], asset_type: str) -> Dict[str, Any]:
    out_dir = _asset_dir(root, session_id)
    file_path = payload.get('svo_file') or payload.get('output_file') or ''
    size = int(payload.get('file_size_bytes') or 0)
    if not size and file_path:
        try:
            size = Path(str(file_path)).stat().st_size
        except OSError:
            size = 0
    return {
        'session_id': session_id,
        'display_name': payload.get('display_name') or session_id,
        'type': _asset_type(session_id, asset_type),
        'state': payload.get('state') or 'ready',
        'output_dir': str(payload.get('output_dir') or out_dir),
        'svo_file': str(payload.get('svo_file') or ''),
        'output_file': str(payload.get('output_file') or ''),
        'created_at': payload.get('created_at') or payload.get('started_at') or 0,
        'file_size_bytes': size,
        'capture_duration_sec': payload.get('capture_duration_sec'),
        'export_point_count': payload.get('export_point_count'),
        'note': payload.get('note') or '',
        'tags': payload.get('tags') if isinstance(payload.get('tags'), list) else [],
    }


def _load_asset(root: str, session_id: str, asset_type: str = '') -> Dict[str, Any]:
    out_dir = _asset_dir(root, session_id)
    data = _read_json(out_dir / 'metadata.json') or _read_json(out_dir / 'status.json')
    if not isinstance(data, dict):
        data = {}
    return _normalize(root, session_id, data, asset_type)


def _write_asset(root: str, session_id: str, payload: Dict[str, Any]) -> Dict[str, Any]:
    out_dir = _asset_dir(root, session_id)
    metadata_path = out_dir / 'metadata.json'
    data = _read_json(metadata_path)
    if not isinstance(data, dict):
        data = {}
    data.update(payload)
    data.setdefault('session_id', session_id)
    data.setdefault('output_dir', str(out_dir))
    _write_json(metadata_path, data)
    return _normalize(root, session_id, data, str(payload.get('type') or ''))


def _write_index(root: str, entries: List[Dict[str, Any]]) -> None:
    _write_json(_root(root) / 'index.json', entries)


def list_assets(root: str, asset_type: str = '') -> List[Dict[str, Any]]:
    base = _root(root)
    entries = []
    for path in sorted(base.glob('*_*'), reverse=True):
        if path.is_dir() and path.name != '.trash':
            try:
                entries.append(_load_asset(str(base), path.name, asset_type))
            except ValueError:
                pass
    _write_index(str(base), entries)
    return entries


def get_asset_info(root: str, session_id: str) -> Dict[str, Any]:
    return _load_asset(root, session_id)


def rename_asset(root: str, session_id: str, display_name: str) -> Dict[str, Any]:
    asset = _write_asset(root, session_id, {'display_name': str(display_name or session_id)})
    list_assets(root, asset.get('type') or '')
    return asset


def delete_asset(root: str, session_id: str, mode: str = 'trash') -> Dict[str, Any]:
    if mode != 'trash':
        raise ValueError('only trash delete is supported')
    src = _asset_dir(root, session_id)
    if not src.exists():
        raise FileNotFoundError(str(src))
    dst = _root(root) / '.trash' / f'{session_id}_{int(time.time())}'
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.move(str(src), str(dst))
    list_assets(root)
    return {'session_id': session_id, 'trash_dir': str(dst)}


def set_latest_asset(root: str, session_id: str) -> Dict[str, Any]:
    asset = _load_asset(root, session_id)
    _write_json(_root(root) / 'latest.json', asset)
    return asset


def storage_summary(*roots: str) -> Dict[str, Any]:
    total = 0
    count = 0
    for root in roots:
        base = _root(root)
        if not base.exists():
            continue
        for path in base.rglob('*'):
            if path.is_file():
                count += 1
                total += path.stat().st_size
    return {'file_count': count, 'total_bytes': total}
