import asyncio
import logging
import os
import threading
import uuid
from pathlib import Path
from typing import Optional

import rclpy
import uvicorn
from fastapi import (
    FastAPI,
    HTTPException,
    Query,
    Request,
    WebSocket,
    WebSocketDisconnect,
)
from fastapi.exceptions import RequestValidationError
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from .map_manager import MapManager, MapManagerError
from .process_manager import ProcessManager
from .platform_api import attach_platform_api
from .ros_bridge import MobileRosBridge
from .schemas import (
    ApiResponse,
    ChassisTestRequest,
    InitialPoseRequest,
    MapRenameRequest,
    MappingSaveRequest,
    NavigationGoalRequest,
    TaskCommand,
    TextCommand,
    VelocityCommand,
)

APP_SYSTEM_MODES = {'bringup', 'mapping'}
LOCAL_APP_HTTP_PATHS = {
    '/api/status',
    '/api/cmd_vel',
    '/api/text_command',
    '/api/task',
    '/api/stop',
}


def task_to_text(command: TaskCommand) -> str:
    if command.text:
        return command.text
    if command.command:
        return f'收到任务指令：{command.command}'
    return '收到通用任务指令'


def stable_robot_id(bridge: MobileRosBridge) -> str:
    configured = os.environ.get('YLHB_ROBOT_ID', '').strip() or str(
        getattr(bridge, 'robot_id', '') or ''
    ).strip()
    if configured:
        return configured
    try:
        seed = Path('/etc/machine-id').read_text(encoding='utf-8').strip()
    except OSError:
        seed = str(uuid.getnode())
    if not seed:
        seed = str(uuid.getnode())
    return str(uuid.uuid5(uuid.NAMESPACE_DNS, f'ylhb-robot:{seed}'))


def make_app(
    bridge: MobileRosBridge,
    process_manager: ProcessManager,
    default_map_path: Optional[str] = None,
) -> FastAPI:
    app = FastAPI(title='YLHB Mobile Bridge', version='0.1.0')
    robot_id = stable_robot_id(bridge)
    bridge_instance_id = str(uuid.uuid4())
    app.add_middleware(
        CORSMiddleware,
        allow_origins=['*'],
        allow_credentials=True,
        allow_methods=['*'],
        allow_headers=['*'],
    )

    def response_dict(response: ApiResponse) -> dict:
        return response.dict()

    def ok(message: str, data=None) -> ApiResponse:
        return ApiResponse(ok=True, message=message, data=data)

    def fail(error: str, exc: Exception | str) -> ApiResponse:
        message = str(exc)
        bridge.get_logger().error('%s: %s', error, message)
        return ApiResponse(ok=False, error=error, message=message)

    def status_payload() -> dict:
        return {
            **bridge.robot_status(),
            'apiVersion': '1.1',
            'robotId': robot_id,
            'bridgeInstanceId': bridge_instance_id,
        }

    def unauthorized_response() -> JSONResponse:
        return JSONResponse(
            status_code=401,
            content=response_dict(
                ApiResponse(
                    ok=False,
                    error='unauthorized',
                    message='unauthorized',
                )
            ),
        )

    def local_app_disabled_response() -> JSONResponse:
        return JSONResponse(
            status_code=503,
            content=response_dict(
                ApiResponse(
                    ok=False,
                    error='local_app_disabled',
                    message='本地 APP 服务已关闭',
                )
            ),
        )

    def local_app_enabled() -> bool:
        checker = getattr(bridge, 'is_local_app_enabled', None)
        return bool(checker()) if callable(checker) else True

    def is_local_app_path(path: str) -> bool:
        return path in LOCAL_APP_HTTP_PATHS or path.startswith('/api/debug/')

    async def wait_while_local_app_enabled(interval: float) -> bool:
        remaining = max(0.0, interval)
        while remaining > 0:
            step = min(0.25, remaining)
            await asyncio.sleep(step)
            if not local_app_enabled():
                return False
            remaining -= step
        return local_app_enabled()

    async def close_local_app_websocket(websocket: WebSocket) -> None:
        try:
            await websocket.close(code=1012)
        except Exception:
            pass

    def map_error_response(exc: MapManagerError) -> JSONResponse:
        return JSONResponse(
            status_code=exc.status_code,
            content=response_dict(
                ApiResponse(
                    ok=False,
                    error=exc.error,
                    message=str(exc),
                )
            ),
        )

    configured_map_path = (
        default_map_path
        or getattr(process_manager, 'default_map_path', None)
        or '/home/nvidia/ros2_DL/maps/my_map'
    )
    map_manager = MapManager(configured_map_path)
    map_mutation_lock = threading.Lock()

    def map_is_in_use() -> bool:
        is_running = getattr(process_manager, 'is_running', None)
        if callable(is_running) and is_running('mapping'):
            return True
        nodes = bridge.debug_status().get('nodes') or {}
        return bool(nodes.get('slam_toolbox') or nodes.get('map_server'))

    def token_from_authorization(value: Optional[str]) -> Optional[str]:
        if not value:
            return None
        scheme, _, token = value.partition(' ')
        if scheme.lower() == 'bearer' and token:
            return token
        return None

    def token_valid(token: Optional[str]) -> bool:
        return bool(token) and token == bridge.api_token

    def require_http_token(request: Request) -> bool:
        if not getattr(bridge, 'require_token', False):
            return True
        header_token = request.headers.get('x-api-token')
        bearer_token = token_from_authorization(
            request.headers.get('authorization')
        )
        return token_valid(header_token) or token_valid(bearer_token)

    def require_ws_token(websocket: WebSocket) -> bool:
        if not getattr(bridge, 'require_token', False):
            return True
        return token_valid(websocket.query_params.get('token'))

    @app.middleware('http')
    async def auth_middleware(request: Request, call_next):
        if is_local_app_path(request.url.path) and not local_app_enabled():
            return local_app_disabled_response()
        if (
            request.url.path.startswith('/api/')
            and not require_http_token(request)
        ):
            return unauthorized_response()
        return await call_next(request)

    @app.on_event('startup')
    async def mark_http_available():
        setter = getattr(bridge, 'set_local_app_http_available', None)
        if callable(setter):
            setter(True)

    @app.on_event('shutdown')
    async def mark_http_unavailable():
        setter = getattr(bridge, 'set_local_app_http_available', None)
        if callable(setter):
            setter(False)

    @app.exception_handler(RequestValidationError)
    async def validation_exception_handler(
        _request: Request,
        exc: RequestValidationError,
    ):
        return JSONResponse(
            status_code=422,
            content=response_dict(
                ApiResponse(
                    ok=False,
                    error='validation_error',
                    message='request validation failed',
                    data={'detail': exc.errors()},
                )
            ),
        )

    @app.exception_handler(HTTPException)
    async def http_exception(_request: Request, exc: HTTPException):
        if exc.status_code == 401:
            return unauthorized_response()
        return JSONResponse(
            status_code=exc.status_code,
            content=response_dict(
                ApiResponse(
                    ok=False,
                    error=(
                        'not_found'
                        if exc.status_code == 404
                        else 'http_error'
                    ),
                    message=str(exc.detail),
                )
            ),
        )

    @app.get('/api/status')
    def status():
        return ok('status', status_payload())

    @app.post('/api/cmd_vel')
    def cmd_vel(command: VelocityCommand):
        try:
            bridge.publish_velocity(
                command.linear_x,
                command.angular_z,
                command.duration_ms,
            )
            return ok('velocity command accepted')
        except Exception as exc:
            return fail('invalid_request', exc)

    @app.post('/api/text_command')
    def text_command(command: TextCommand):
        try:
            bridge.publish_text_command(command.text)
            return ok('text command published')
        except Exception as exc:
            return fail('invalid_request', exc)

    @app.post('/api/task')
    def task(command: TaskCommand):
        try:
            text = task_to_text(command)
            bridge.publish_text_command(text)
            return ok('task converted to text command', {'text': text})
        except Exception as exc:
            return fail('invalid_request', exc)

    @app.post('/api/stop')
    def stop():
        try:
            bridge.stop_all()
            return ok('stopped')
        except Exception as exc:
            return fail('internal_error', exc)

    @app.get('/api/debug/status')
    def debug_status():
        return ok('debug status', bridge.debug_status())

    @app.post('/api/debug/chassis/test')
    def chassis_test(command: ChassisTestRequest):
        try:
            bridge.publish_velocity(
                command.linear_x,
                command.angular_z,
                command.duration_ms,
            )
            return ok(f'chassis test {command.mode} accepted')
        except Exception as exc:
            return fail('invalid_request', exc)

    @app.post('/api/debug/chassis/stop')
    def chassis_stop():
        try:
            bridge.stop_motion()
            return ok('chassis stopped')
        except Exception as exc:
            return fail('internal_error', exc)

    @app.get('/api/debug/mapping/status')
    def mapping_status():
        try:
            process = process_manager.process_status('mapping')
            return ok('mapping status', bridge.mapping_status(process))
        except Exception as exc:
            return fail('process_error', exc)

    @app.get('/api/debug/maps')
    def maps_list():
        try:
            return ok('maps', map_manager.list_maps())
        except Exception as exc:
            return fail('map_operation_failed', exc)

    @app.get('/api/debug/maps/{map_name}/preview')
    def map_preview(
        map_name: str,
        max_size_px: int = Query(default=1024, ge=1, le=4096),
    ):
        try:
            return ok(
                'map preview',
                map_manager.preview_map(map_name, max_size_px),
            )
        except MapManagerError as exc:
            return map_error_response(exc)

    @app.post('/api/debug/maps/{map_name}/confirm_default')
    def map_confirm_default(map_name: str):
        if map_name != map_manager.default_name and map_is_in_use():
            return map_error_response(
                MapManagerError(
                    'map_in_use',
                    'maps cannot be changed while SLAM Toolbox '
                    'or map_server is running',
                    409,
                )
            )
        try:
            with map_mutation_lock:
                result = map_manager.confirm_default(map_name)
                worker = getattr(bridge, 'map_upload_worker', None)
                try:
                    if worker is None:
                        raise RuntimeError('map upload worker is not initialized')
                    upload = worker.enqueue(
                        map_manager.maps_dir / f'{map_manager.default_name}.yaml',
                        map_manager.maps_dir / f'{map_manager.default_name}.pgm',
                    )
                except Exception as exc:
                    bridge.get_logger().error(
                        'default map applied but upload task creation failed: %s',
                        exc,
                    )
                    upload = {
                        'task_created': False,
                        'task_id': '',
                        'status': 'FAILED_TO_CREATE',
                        'map_asset_id': '',
                        'error': str(exc)[:300],
                        'content_identity_sha256': '',
                    }
                return ok(
                    'default map applied',
                    {**result, 'default_applied': True, 'upload': upload},
                )
        except MapManagerError as exc:
            return map_error_response(exc)

    @app.post('/api/debug/maps/{map_name}/rename')
    def map_rename(map_name: str, request: MapRenameRequest):
        if map_is_in_use():
            return map_error_response(
                MapManagerError(
                    'map_in_use',
                    'maps cannot be changed while SLAM Toolbox '
                    'or map_server is running',
                    409,
                )
            )
        try:
            with map_mutation_lock:
                return ok(
                    'map renamed',
                    map_manager.rename_map(map_name, request.new_name),
                )
        except MapManagerError as exc:
            return map_error_response(exc)

    @app.delete('/api/debug/maps/{map_name}')
    def map_delete(map_name: str):
        if map_is_in_use():
            return map_error_response(
                MapManagerError(
                    'map_in_use',
                    'maps cannot be changed while SLAM Toolbox '
                    'or map_server is running',
                    409,
                )
            )
        try:
            with map_mutation_lock:
                return ok('map deleted', map_manager.delete_map(map_name))
        except MapManagerError as exc:
            return map_error_response(exc)

    @app.get('/api/debug/mapping/map_snapshot')
    def map_snapshot(downsample: int = Query(default=1, ge=1, le=16)):
        snapshot = bridge.map_snapshot(downsample=downsample)
        if snapshot is None:
            return ApiResponse(
                ok=False,
                error='no_map',
                message='no map has been received',
            )
        return ok('map snapshot', snapshot)

    @app.post('/api/debug/mapping/start')
    def mapping_start():
        try:
            bridge.reset_mapping_map()
            return ok(process_manager.start_mapping())
        except Exception as exc:
            return fail('process_error', exc)

    @app.post('/api/debug/mapping/save')
    def mapping_save(request: MappingSaveRequest):
        if not bridge.has_mapping_map():
            return ApiResponse(
                ok=False,
                error='no_map',
                message='no current SLAM map has been received',
            )
        try:
            return ok('map saved', process_manager.save_map(request.map_name))
        except Exception as exc:
            return fail('process_error', exc)

    @app.post('/api/debug/mapping/stop')
    def mapping_stop():
        try:
            message = process_manager.stop('mapping')
            bridge.reset_mapping_map()
            return ok(message)
        except Exception as exc:
            return fail('process_error', exc)

    @app.get('/api/debug/navigation/status')
    def navigation_status():
        return ok('navigation status', bridge.debug_status())

    @app.post('/api/debug/navigation/start')
    def navigation_start():
        try:
            return ok(process_manager.start_navigation())
        except Exception as exc:
            return fail('process_error', exc)

    @app.get('/api/debug/system/status')
    def system_status():
        try:
            if bridge.has_system_supervisor():
                return ok('system status', bridge.system_status())
            return ok(
                'system status',
                {
                    'bringup': process_manager.process_status('bringup'),
                    'mapping': process_manager.process_status('mapping'),
                },
            )
        except Exception as exc:
            return fail('process_error', exc)

    @app.post('/api/debug/system/start/{mode}')
    def system_start(mode: str):
        if mode not in APP_SYSTEM_MODES:
            raise HTTPException(status_code=404, detail='mode not found')
        try:
            if not bridge.has_system_supervisor():
                return ApiResponse(
                    ok=False,
                    error='supervisor_unavailable',
                    message='system supervisor is not online',
                )
            if mode == 'mapping':
                bridge.reset_mapping_map()
            command = f'start_{mode}'
            bridge.publish_system_command(command)
            return ok(f'system command sent: {command}')
        except Exception as exc:
            return fail('process_error', exc)

    @app.post('/api/debug/system/stop/{mode}')
    def system_stop(mode: str):
        if mode not in APP_SYSTEM_MODES:
            raise HTTPException(status_code=404, detail='mode not found')
        try:
            if not bridge.has_system_supervisor():
                return ApiResponse(
                    ok=False,
                    error='supervisor_unavailable',
                    message='system supervisor is not online',
                )
            command = f'stop_{mode}'
            bridge.publish_system_command(command)
            if mode == 'mapping':
                bridge.reset_mapping_map()
            return ok(f'system command sent: {command}')
        except Exception as exc:
            return fail('process_error', exc)

    @app.get('/api/debug/patrol/status')
    def patrol_status():
        status = bridge.patrol_status()
        if not status:
            supervisor_status = (
                bridge.system_status() if bridge.has_system_supervisor() else {}
            )
            executor_running = (
                supervisor_status.get('patrol_executor') == 'running'
            )
            status = {
                'state': 'idle' if executor_running else 'unavailable',
                'executor_running': executor_running,
                'message': (
                    'patrol executor running'
                    if executor_running
                    else 'patrol executor is not running'
                ),
            }
        else:
            status = dict(status)
            status.setdefault('executor_running', True)
        return ok('patrol status', status)

    def publish_patrol_api_command(command: str) -> ApiResponse:
        bridge.publish_patrol_command(command)
        return ok(f'patrol command sent: {command}')

    @app.post('/api/debug/patrol/start')
    def patrol_start():
        return publish_patrol_api_command('start')

    @app.post('/api/debug/patrol/pause')
    def patrol_pause():
        return publish_patrol_api_command('pause')

    @app.post('/api/debug/patrol/resume')
    def patrol_resume():
        return publish_patrol_api_command('resume')

    @app.post('/api/debug/patrol/cancel')
    def patrol_cancel():
        return publish_patrol_api_command('cancel')

    @app.post('/api/debug/patrol/reload')
    def patrol_reload():
        return publish_patrol_api_command('reload')

    @app.post('/api/debug/patrol/initialize')
    def patrol_initialize():
        return publish_patrol_api_command('initialize')

    @app.post('/api/debug/navigation/set_initial_pose')
    def set_initial_pose(request: InitialPoseRequest):
        try:
            bridge.publish_initial_pose(request.x, request.y, request.yaw)
            return ok('initial pose published')
        except Exception as exc:
            return fail('invalid_request', exc)

    @app.post('/api/debug/navigation/goal')
    def navigation_goal(request: NavigationGoalRequest):
        try:
            accepted = bridge.send_navigation_goal(
                request.x,
                request.y,
                request.yaw,
            )
            return ok(
                'goal accepted' if accepted else 'goal rejected',
                {'accepted': accepted},
            )
        except Exception as exc:
            return fail('ros_unavailable', exc)

    @app.post('/api/debug/navigation/cancel')
    def navigation_cancel():
        try:
            canceled = bridge.cancel_navigation()
            return ok('navigation cancel requested', {'canceled': canceled})
        except Exception as exc:
            return fail('ros_unavailable', exc)

    @app.websocket('/ws/status')
    async def ws_status(websocket: WebSocket):
        if not local_app_enabled():
            await close_local_app_websocket(websocket)
            return
        if not require_ws_token(websocket):
            await websocket.close(code=1008)
            return
        await websocket.accept()
        interval = 1.0 / max(
            0.1,
            float(getattr(bridge, 'status_rate_hz', 2.0)),
        )
        connected = getattr(bridge, 'local_app_client_connected', None)
        disconnected = getattr(bridge, 'local_app_client_disconnected', None)
        if callable(connected):
            connected('status')
        try:
            while True:
                if not local_app_enabled():
                    await close_local_app_websocket(websocket)
                    return
                await websocket.send_json(
                    response_dict(ok('status', status_payload()))
                )
                if not await wait_while_local_app_enabled(interval):
                    await close_local_app_websocket(websocket)
                    return
        except WebSocketDisconnect:
            return
        except Exception:
            return
        finally:
            if callable(disconnected):
                disconnected('status')

    @app.websocket('/ws/map')
    async def ws_map(websocket: WebSocket):
        if not local_app_enabled():
            await close_local_app_websocket(websocket)
            return
        if not require_ws_token(websocket):
            await websocket.close(code=1008)
            return
        await websocket.accept()
        try:
            downsample = max(
                1,
                min(16, int(websocket.query_params.get('downsample', '1'))),
            )
        except ValueError:
            downsample = 1
        interval = 1.0 / max(
            0.1,
            float(getattr(bridge, 'map_stream_rate_hz', 1.0)),
        )
        connected = getattr(bridge, 'local_app_client_connected', None)
        disconnected = getattr(bridge, 'local_app_client_disconnected', None)
        if callable(connected):
            connected('map')
        try:
            while True:
                if not local_app_enabled():
                    await close_local_app_websocket(websocket)
                    return
                snapshot = bridge.map_snapshot(downsample=downsample)
                if snapshot is None:
                    await websocket.send_json(
                        response_dict(
                            ApiResponse(
                                ok=False,
                                error='no_map',
                                message='no map has been received',
                            )
                        )
                    )
                else:
                    await websocket.send_json(
                        response_dict(ok('map snapshot', snapshot))
                    )
                if not await wait_while_local_app_enabled(interval):
                    await close_local_app_websocket(websocket)
                    return
        except WebSocketDisconnect:
            return
        except Exception:
            return
        finally:
            if callable(disconnected):
                disconnected('map')

    return app


def main() -> None:
    logging.basicConfig(level=logging.INFO)
    rclpy.init()
    bridge = MobileRosBridge()
    workspace_dir = bridge.get_parameter('workspace_dir').value
    default_map_path = bridge.get_parameter('default_map_path').value
    host = bridge.get_parameter('host').value
    port = int(bridge.get_parameter('port').value)
    process_manager = ProcessManager(workspace_dir, default_map_path)
    app = make_app(
        bridge,
        process_manager,
        default_map_path=default_map_path,
    )
    attach_platform_api(app, bridge)
    executor_thread = threading.Thread(
        target=rclpy.spin,
        args=(bridge,),
        daemon=True,
    )
    executor_thread.start()
    bridge.cloud_client.start()
    bridge.map_upload_worker.start()

    try:
        uvicorn.run(app, host=host, port=port, access_log=False)
    finally:
        bridge.map_upload_worker.stop()
        bridge.cloud_client.stop()
        bridge.stop_motion()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
