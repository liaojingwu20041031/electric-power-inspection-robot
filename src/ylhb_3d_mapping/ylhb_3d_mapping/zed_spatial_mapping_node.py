import json
import os
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


VALID_COMMANDS = {'start', 'stop', 'stop_and_export', 'export', 'reset'}
VALID_MAP_TYPES = {'fused_point_cloud', 'mesh'}
FINAL_STATES = {'idle', 'succeeded', 'failed'}


def parse_command_json(data: str) -> Dict[str, Any]:
    try:
        payload = json.loads(data)
    except json.JSONDecodeError as exc:
        raise ValueError(f'Invalid 3D mapping command JSON: {exc}') from exc
    if not isinstance(payload, dict):
        raise ValueError('3D mapping command must be a JSON object.')
    command = str(payload.get('command') or '').strip()
    if command not in VALID_COMMANDS:
        raise ValueError(f'Unknown 3D mapping command: {command or "<empty>"}')
    payload['command'] = command
    return payload


def build_metadata(
    *,
    map_type: str,
    output_file: str,
    output_dir: str,
    started_at: float,
    finished_at: float,
    params: Dict[str, Any],
) -> Dict[str, Any]:
    return {
        'schema_version': '1.0',
        'map_type': map_type,
        'output_file': output_file,
        'output_dir': output_dir,
        'started_at': started_at,
        'finished_at': finished_at,
        'duration_sec': max(0.0, finished_at - started_at),
        'parameters': params,
        'note': 'ZED SDK 3D spatial mapping export for inspection only; not a Nav2 occupancy map.',
    }


class ZedSpatialMappingNode(Node):
    def __init__(self) -> None:
        super().__init__('zed_spatial_mapping_node')
        self.declare_parameter('command_topic', '/inspection_ai/mapping3d_command')
        self.declare_parameter('status_topic', '/inspection_ai/mapping3d_status')
        self.declare_parameter('result_topic', '/inspection_ai/mapping3d_result')
        self.declare_parameter('output_root', '/home/nvidia/ros2_DL/runs/3d_mapping')
        self.declare_parameter('map_type', 'fused_point_cloud')
        self.declare_parameter('resolution_preset', 'medium')
        self.declare_parameter('range_preset', 'medium')
        self.declare_parameter('save_texture', False)
        self.declare_parameter('publish_rate_hz', 5.0)

        self.output_root = os.path.expanduser(str(self.get_parameter('output_root').value))
        self.map_type = str(self.get_parameter('map_type').value)
        self.resolution_preset = str(self.get_parameter('resolution_preset').value)
        self.range_preset = str(self.get_parameter('range_preset').value)
        self.save_texture = bool(self.get_parameter('save_texture').value)
        self.publish_period = 1.0 / max(0.1, float(self.get_parameter('publish_rate_hz').value))

        self.status_pub = self.create_publisher(String, self.get_parameter('status_topic').value, 10)
        self.result_pub = self.create_publisher(String, self.get_parameter('result_topic').value, 10)
        self.create_subscription(String, self.get_parameter('command_topic').value, self.command_callback, 10)

        self.lock = threading.Lock()
        self.worker: Optional[threading.Thread] = None
        self.stop_requested = False
        self.export_requested = False
        self.state = 'idle'
        self.message = 'ready'
        self.camera = None
        self.sl = None
        self.started_at = 0.0
        self.current_output_dir = ''
        self.publish_status('idle', 'ready')

    def command_callback(self, msg: String) -> None:
        try:
            payload = parse_command_json(msg.data)
        except ValueError as exc:
            self.publish_status('failed', str(exc))
            return
        self.handle_command(payload)

    def handle_command(self, payload: Dict[str, Any]) -> None:
        command = payload['command']
        if command == 'start':
            self.start_mapping(export_on_stop=False)
        elif command == 'stop':
            self.request_stop(export_on_stop=False)
        elif command == 'stop_and_export':
            self.request_stop(export_on_stop=True)
        elif command == 'export':
            self.export_current_map()
        elif command == 'reset':
            self.reset()

    def start_mapping(self, export_on_stop: bool = False) -> None:
        with self.lock:
            if self.worker and self.worker.is_alive():
                self.publish_status('running', '3D mapping already running')
                return
            self.stop_requested = False
            self.export_requested = export_on_stop
            self.started_at = time.time()
            self.current_output_dir = self.make_output_dir(self.started_at)
            self.worker = threading.Thread(target=self.mapping_loop, daemon=True)
            self.worker.start()

    def request_stop(self, export_on_stop: bool) -> None:
        with self.lock:
            self.stop_requested = True
            self.export_requested = self.export_requested or export_on_stop
            running = bool(self.worker and self.worker.is_alive())
        if not running:
            if export_on_stop:
                self.export_current_map()
            else:
                self.publish_status('idle', '3D mapping is not running')

    def reset(self) -> None:
        self.request_stop(export_on_stop=False)
        self.camera = None
        self.sl = None
        self.current_output_dir = ''
        self.publish_status('idle', 'reset')

    def mapping_loop(self) -> None:
        try:
            sl = self.load_zed()
            self.sl = sl
            self.publish_status('opening_camera', 'opening ZED camera')
            camera = sl.Camera()
            self.camera = camera
            init_params = sl.InitParameters()
            self.check_success(camera.open(init_params), 'Camera.open')

            self.publish_status('tracking_enabled', 'enabling positional tracking')
            self.check_success(
                camera.enable_positional_tracking(sl.PositionalTrackingParameters()),
                'enable_positional_tracking',
            )

            self.publish_status('mapping_enabled', 'enabling spatial mapping')
            spatial_params = sl.SpatialMappingParameters()
            self.apply_spatial_mapping_params(sl, spatial_params)
            self.check_success(camera.enable_spatial_mapping(spatial_params), 'enable_spatial_mapping')

            self.publish_status('running', '3D mapping running')
            runtime_params = sl.RuntimeParameters()
            last_publish = 0.0
            while not self.stop_requested:
                camera.grab(runtime_params)
                now = time.time()
                if now - last_publish >= self.publish_period:
                    self.publish_status('running', '3D mapping running')
                    last_publish = now
            if self.export_requested:
                self.export_current_map()
            else:
                self.publish_status('idle', '3D mapping stopped')
        except Exception as exc:
            self.publish_status('failed', str(exc))
        finally:
            self.close_camera()

    def load_zed(self):
        try:
            import pyzed.sl as sl
        except Exception as exc:
            raise RuntimeError(f'pyzed.sl import failed; install ZED SDK Python API: {exc}') from exc
        return sl

    def apply_spatial_mapping_params(self, sl, params) -> None:
        map_type = self.map_type if self.map_type in VALID_MAP_TYPES else 'fused_point_cloud'
        if map_type == 'mesh':
            params.map_type = sl.SPATIAL_MAP_TYPE.MESH
        else:
            params.map_type = sl.SPATIAL_MAP_TYPE.FUSED_POINT_CLOUD
        self.set_enum_param(sl.MAPPING_RESOLUTION, params, 'resolution', self.resolution_preset)
        self.set_enum_param(sl.MAPPING_RANGE, params, 'range_meter', self.range_preset)
        if hasattr(params, 'save_texture'):
            params.save_texture = self.save_texture

    def set_enum_param(self, enum_cls, params, attr: str, preset: str) -> None:
        value = getattr(enum_cls, preset.upper(), None)
        if value is not None and hasattr(params, attr):
            setattr(params, attr, value)

    def export_current_map(self) -> None:
        try:
            if self.camera is None or self.sl is None:
                raise RuntimeError('ZED camera is not open; start mapping before export.')
            output_dir = self.current_output_dir or self.make_output_dir(time.time())
            Path(output_dir).mkdir(parents=True, exist_ok=True)
            self.publish_status('extracting', 'extracting whole spatial map')
            spatial_map = self.make_spatial_map()
            self.camera.extract_whole_spatial_map(spatial_map)
            self.publish_status('saving', f'saving {self.map_type}')
            output_file = os.path.join(output_dir, 'mesh.obj' if self.map_type == 'mesh' else 'pointcloud.ply')
            file_format = self.sl.MESH_FILE_FORMAT.OBJ if self.map_type == 'mesh' else self.sl.MESH_FILE_FORMAT.PLY
            self.check_success(spatial_map.save(output_file, file_format), 'spatial_map.save')
            finished_at = time.time()
            metadata = build_metadata(
                map_type=self.map_type,
                output_file=output_file,
                output_dir=output_dir,
                started_at=self.started_at or finished_at,
                finished_at=finished_at,
                params=self.mapping_params(),
            )
            self.write_json(os.path.join(output_dir, 'metadata.json'), metadata)
            self.write_json(os.path.join(output_dir, 'status.json'), {'state': 'succeeded', **metadata})
            self.publish_result(metadata)
            self.publish_status('succeeded', f'3D map exported: {output_file}')
        except Exception as exc:
            self.publish_status('failed', str(exc))

    def make_spatial_map(self):
        return self.sl.Mesh() if self.map_type == 'mesh' else self.sl.FusedPointCloud()

    def close_camera(self) -> None:
        camera = getattr(self, 'camera', None)
        self.camera = None
        if camera is None:
            return
        for method_name in ('disable_spatial_mapping', 'disable_positional_tracking', 'close'):
            method = getattr(camera, method_name, None)
            if method is not None:
                try:
                    method()
                except Exception:
                    pass

    def mapping_params(self) -> Dict[str, Any]:
        return {
            'map_type': self.map_type,
            'resolution_preset': self.resolution_preset,
            'range_preset': self.range_preset,
            'save_texture': self.save_texture,
        }

    def make_output_dir(self, timestamp: float) -> str:
        stamp = datetime.fromtimestamp(timestamp).strftime('%Y%m%d_%H%M%S')
        return os.path.join(self.output_root, f'map_{stamp}')

    def write_json(self, path: str, payload: Dict[str, Any]) -> None:
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)
            f.write('\n')

    def publish_status(self, state: str, message: str) -> None:
        self.state = state
        self.message = message
        payload = {
            'schema_version': '1.0',
            'timestamp': time.time(),
            'state': state,
            'message': message,
            'output_dir': self.current_output_dir,
            'map_type': self.map_type,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)

    def publish_result(self, payload: Dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.result_pub.publish(msg)

    def check_success(self, result, action: str) -> None:
        success = getattr(getattr(self.sl, 'ERROR_CODE', object), 'SUCCESS', None)
        if success is not None and result != success:
            raise RuntimeError(f'{action} failed: {result}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZedSpatialMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
