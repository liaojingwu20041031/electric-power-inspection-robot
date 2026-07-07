import json
import os
import signal
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

from .zed_spatial_mapping_node import (
    RANGE_METERS,
    RESOLUTION_METERS,
    count_spatial_map_points,
)


CAPTURE_DEFAULTS = {
    'output_root': '/home/nvidia/ros2_DL/runs/3d_capture',
    'duration_sec': 30.0,
    'camera_resolution': 'HD720',
    'camera_fps': 30,
    'depth_mode': 'NONE',
    'compression_mode': 'H264',
}

PROFILES = {
    'fast_check': {
        'depth_mode': 'NEURAL',
        'resolution_preset': 'medium',
        'range_preset': 'near',
        'spatial_mapping_max_memory_mb': 768,
    },
    'quality_safe': {
        'depth_mode': 'NEURAL',
        'resolution_preset': 'high',
        'range_preset': 'near',
        'spatial_mapping_max_memory_mb': 1024,
    },
    'quality_plus': {
        'depth_mode': 'NEURAL_PLUS',
        'resolution_preset': 'high',
        'range_preset': 'near',
        'spatial_mapping_max_memory_mb': 1024,
    },
}


_STOP_REQUESTED = False


def parse_args(args: List[str]) -> Dict[str, str]:
    parsed = {}
    for arg in args:
        if ':=' in arg:
            key, value = arg.split(':=', 1)
            parsed[key] = value
    return parsed


def load_zed():
    try:
        import pyzed.sl as sl
    except Exception as exc:
        raise RuntimeError(f'pyzed.sl import failed; install ZED SDK Python API: {exc}') from exc
    return sl


def write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def write_latest_json(root: str, metadata: Dict[str, Any]) -> Dict[str, Any]:
    root_path = Path(os.path.expanduser(root))
    latest = {
        'session_id': str(metadata.get('session_id') or Path(str(metadata.get('output_dir', ''))).name),
        'output_dir': str(metadata.get('output_dir') or ''),
        'svo_file': str(metadata.get('svo_file') or ''),
        'metadata_file': str(metadata.get('metadata_file') or ''),
        'status_file': str(metadata.get('status_file') or ''),
        'created_at': metadata.get('created_at') or metadata.get('started_at'),
        'svo_frame_count': int(metadata.get('svo_frame_count') or 0),
    }
    for key in ('output_file', 'export_point_count', 'reconstruct_profile', 'source_capture_session_id', 'source_svo_file'):
        if key in metadata:
            latest[key] = metadata[key]
    write_json(root_path / 'latest.json', latest)
    return latest


def update_index_json(root: str, metadata: Dict[str, Any]) -> List[Dict[str, Any]]:
    root_path = Path(os.path.expanduser(root))
    index_path = root_path / 'index.json'
    try:
        current = json.loads(index_path.read_text(encoding='utf-8'))
    except Exception:
        current = []
    if not isinstance(current, list):
        current = []
    session_id = str(metadata.get('session_id') or Path(str(metadata.get('output_dir', ''))).name)
    latest = write_latest_json(root, metadata)
    updated = [latest] + [item for item in current if item.get('session_id') != session_id]
    write_json(index_path, updated)
    return updated


def resolve_latest_capture(root: str) -> Path:
    latest_path = Path(os.path.expanduser(root)) / 'latest.json'
    try:
        latest = json.loads(latest_path.read_text(encoding='utf-8'))
    except FileNotFoundError as exc:
        raise FileNotFoundError(f'请先完成一次现场采集: {latest_path}') from exc
    svo_file = Path(str(latest.get('svo_file') or '')).expanduser()
    if not svo_file:
        raise FileNotFoundError(f'latest.json missing svo_file: {latest_path}')
    return svo_file


def _read_json(path: Path) -> Dict[str, Any]:
    try:
        data = json.loads(path.read_text(encoding='utf-8'))
    except Exception:
        return {}
    return data if isinstance(data, dict) else {}


def _resolve_capture(opts: Dict[str, str]) -> Tuple[Path, Dict[str, Any]]:
    capture_root = str(opts.get('capture_root') or CAPTURE_DEFAULTS['output_root'])
    if opts.get('session'):
        out_dir = Path(os.path.expanduser(capture_root)) / str(opts['session'])
        return out_dir / 'capture.svo2', _read_json(out_dir / 'metadata.json')
    input_value = str(opts.get('input') or 'latest')
    if input_value == 'latest':
        svo_file = resolve_latest_capture(capture_root)
        return svo_file, _read_json(svo_file.parent / 'metadata.json')
    return Path(input_value).expanduser(), _read_json(Path(input_value).expanduser().parent / 'metadata.json')


def set_enum(enum_cls, obj: Any, attr: str, value: str) -> None:
    enum_value = getattr(enum_cls, str(value).upper(), None)
    if enum_value is None:
        return
    try:
        setattr(obj, attr, enum_value)
    except AttributeError:
        pass


def check_success(sl, result, action: str) -> None:
    if result is True:
        return
    success = getattr(getattr(sl, 'ERROR_CODE', object), 'SUCCESS', None)
    if success is not None and result != success:
        raise RuntimeError(f'{action} failed: {result}')


def output_dir(root: str, prefix: str, timestamp: float) -> Path:
    stamp = datetime.fromtimestamp(timestamp).strftime('%Y%m%d_%H%M%S')
    return Path(os.path.expanduser(root)) / f'{prefix}_{stamp}'


def capture_svo(
    args: List[str],
    *,
    sl=None,
    now: Callable[[], float] = time.time,
    monotonic: Callable[[], float] = time.monotonic,
    stop_requested: Optional[Callable[[], bool]] = None,
) -> Dict[str, Any]:
    global _STOP_REQUESTED
    sl = sl or load_zed()
    opts = {**CAPTURE_DEFAULTS, **parse_args(args)}
    started_at = now()
    output_root = str(opts['output_root'])
    out_dir = output_dir(output_root, 'capture', started_at)
    session_id = out_dir.name
    svo_file = out_dir / 'capture.svo2'
    metadata_file = out_dir / 'metadata.json'
    status_file = out_dir / 'status.json'
    out_dir.mkdir(parents=True, exist_ok=True)
    write_json(status_file, {'state': 'recording', 'svo_file': str(svo_file), 'parameters': opts})

    camera = sl.Camera()
    previous_handlers = {}

    def request_stop(_signum, _frame) -> None:
        global _STOP_REQUESTED
        _STOP_REQUESTED = True

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            previous_handlers[sig] = signal.getsignal(sig)
            signal.signal(sig, request_stop)
        except ValueError:
            pass
    frames = 0
    state = 'succeeded'
    try:
        init_params = sl.InitParameters()
        set_enum(sl.RESOLUTION, init_params, 'camera_resolution', str(opts['camera_resolution']))
        set_enum(sl.DEPTH_MODE, init_params, 'depth_mode', str(opts['depth_mode']))
        try:
            init_params.camera_fps = int(opts['camera_fps'])
        except AttributeError:
            pass
        check_success(sl, camera.open(init_params), 'Camera.open')

        rec_params = sl.RecordingParameters()
        rec_params.video_filename = str(svo_file)
        set_enum(sl.SVO_COMPRESSION_MODE, rec_params, 'compression_mode', str(opts['compression_mode']))
        check_success(sl, camera.enable_recording(rec_params), 'Camera.enable_recording')

        start = monotonic()
        duration = float(opts['duration_sec'])
        while duration <= 0 or monotonic() - start < duration:
            if _STOP_REQUESTED or (stop_requested is not None and stop_requested()):
                state = 'stopped'
                break
            if camera.grab() == sl.ERROR_CODE.SUCCESS:
                frames += 1
    finally:
        if hasattr(camera, 'disable_recording'):
            camera.disable_recording()
        camera.close()
        for sig, handler in previous_handlers.items():
            try:
                signal.signal(sig, handler)
            except ValueError:
                pass
        _STOP_REQUESTED = False

    finished_at = now()
    metadata = {
        'schema_version': '1.0',
        'state': state,
        'session_id': session_id,
        'svo_file': str(svo_file),
        'output_dir': str(out_dir),
        'metadata_file': str(metadata_file),
        'status_file': str(status_file),
        'created_at': started_at,
        'started_at': started_at,
        'finished_at': finished_at,
        'capture_duration_sec': max(0.0, finished_at - started_at),
        'svo_frame_count': frames,
        'parameters': opts,
    }
    write_json(metadata_file, metadata)
    write_json(status_file, metadata)
    update_index_json(output_root, metadata)
    return metadata


def reconstruct_svo(
    args: List[str],
    *,
    sl=None,
    now: Callable[[], float] = time.time,
) -> Dict[str, Any]:
    sl = sl or load_zed()
    opts = parse_args(args)
    profile_name = opts.get('profile', 'quality_safe')
    params = {**PROFILES.get(profile_name, PROFILES['quality_safe']), **opts}
    params.setdefault('input', 'latest')
    params.setdefault('output_type', 'pointcloud')
    svo_file, capture_metadata = _resolve_capture(params)
    params['input'] = str(svo_file)
    started_at = now()
    output_root = str(params.get('output_root', '/home/nvidia/ros2_DL/runs/3d_reconstruct'))
    out_dir = output_dir(output_root, 'reconstruct', started_at)
    session_id = out_dir.name
    out_dir.mkdir(parents=True, exist_ok=True)
    output_file = out_dir / 'pointcloud.ply'
    write_json(out_dir / 'status.json', {'state': 'reconstructing', 'svo_file': str(svo_file), 'parameters': params})

    camera = sl.Camera()
    try:
        init_params = sl.InitParameters()
        init_params.set_from_svo_file(str(svo_file))
        init_params.svo_real_time_mode = False
        set_enum(sl.DEPTH_MODE, init_params, 'depth_mode', str(params['depth_mode']))
        set_enum(sl.UNIT, init_params, 'coordinate_units', 'METER')
        set_enum(sl.COORDINATE_SYSTEM, init_params, 'coordinate_system', 'RIGHT_HANDED_Z_UP')
        check_success(sl, camera.open(init_params), 'Camera.open')
        check_success(sl, camera.enable_positional_tracking(sl.PositionalTrackingParameters()), 'enable_positional_tracking')

        spatial_params = sl.SpatialMappingParameters()
        spatial_params.map_type = sl.SPATIAL_MAP_TYPE.FUSED_POINT_CLOUD
        if hasattr(spatial_params, 'resolution_meter'):
            spatial_params.resolution_meter = RESOLUTION_METERS[str(params['resolution_preset'])]
        if hasattr(spatial_params, 'range_meter'):
            spatial_params.range_meter = RANGE_METERS[str(params['range_preset'])]
        if hasattr(spatial_params, 'max_memory_usage'):
            spatial_params.max_memory_usage = int(params['spatial_mapping_max_memory_mb'])
        check_success(sl, camera.enable_spatial_mapping(spatial_params), 'enable_spatial_mapping')

        runtime_params = sl.RuntimeParameters()
        end_code = getattr(sl.ERROR_CODE, 'END_OF_SVOFILE_REACHED', object())
        while True:
            result = camera.grab(runtime_params)
            if result == end_code:
                break
            check_success(sl, result, 'Camera.grab')

        spatial_map = sl.FusedPointCloud()
        camera.extract_whole_spatial_map(spatial_map)
        point_count = count_spatial_map_points(spatial_map)
        check_success(sl, spatial_map.save(str(output_file), sl.MESH_FILE_FORMAT.PLY), 'spatial_map.save')
        frame_count = camera.get_svo_number_of_frames() if hasattr(camera, 'get_svo_number_of_frames') else 0
    finally:
        for method_name in ('disable_spatial_mapping', 'disable_positional_tracking', 'close'):
            method = getattr(camera, method_name, None)
            if method is not None:
                method()

    finished_at = now()
    metadata = {
        'schema_version': '1.0',
        'state': 'succeeded',
        'session_id': session_id,
        'svo_file': str(svo_file),
        'svo_frame_count': frame_count,
        'source_capture_session_id': str(capture_metadata.get('session_id') or svo_file.parent.name),
        'source_svo_file': str(svo_file),
        'reconstruct_profile': profile_name,
        'output_dir': str(out_dir),
        'output_file': str(output_file),
        'metadata_file': str(out_dir / 'metadata.json'),
        'status_file': str(out_dir / 'status.json'),
        'created_at': started_at,
        'started_at': started_at,
        'finished_at': finished_at,
        'duration_sec': max(0.0, finished_at - started_at),
        'export_point_count': point_count,
        'parameters': params,
    }
    write_json(out_dir / 'metadata.json', metadata)
    write_json(out_dir / 'status.json', metadata)
    update_index_json(output_root, metadata)
    return metadata


def main_capture(args: Optional[List[str]] = None) -> None:
    print(json.dumps(capture_svo(sys.argv[1:] if args is None else args), ensure_ascii=False))


def main_reconstruct(args: Optional[List[str]] = None) -> None:
    print(json.dumps(reconstruct_svo(sys.argv[1:] if args is None else args), ensure_ascii=False))
