import asyncio
import logging
import threading

import rclpy
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

from .process_manager import ProcessManager
from .ros_bridge import MobileRosBridge
from .schemas import (
    ApiResponse,
    ChassisTestRequest,
    InitialPoseRequest,
    MappingSaveRequest,
    NavigationGoalRequest,
    TaskCommand,
    TextCommand,
    VelocityCommand,
)


def task_to_text(command: TaskCommand) -> str:
    if command.text:
        return command.text
    if command.command:
        return f'收到任务指令：{command.command}'
    return '收到通用任务指令'


def make_app(bridge: MobileRosBridge, process_manager: ProcessManager) -> FastAPI:
    app = FastAPI(title='YLHB Mobile Bridge', version='0.1.0')
    app.add_middleware(
        CORSMiddleware,
        allow_origins=['*'],
        allow_credentials=True,
        allow_methods=['*'],
        allow_headers=['*'],
    )

    def ok(message: str, data=None) -> ApiResponse:
        return ApiResponse(ok=True, message=message, data=data)

    def fail(error: str, exc: Exception) -> ApiResponse:
        bridge.get_logger().error('%s: %s', error, exc)
        return ApiResponse(ok=False, error=error, message=str(exc))

    @app.get('/api/status')
    def status():
        return bridge.robot_status()

    @app.post('/api/cmd_vel')
    def cmd_vel(command: VelocityCommand):
        try:
            bridge.publish_velocity(command.linear_x, command.angular_z, command.duration_ms)
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
        return bridge.debug_status()

    @app.post('/api/debug/chassis/test')
    def chassis_test(command: ChassisTestRequest):
        try:
            bridge.publish_velocity(command.linear_x, command.angular_z, command.duration_ms)
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
        return bridge.debug_status()

    @app.post('/api/debug/mapping/start')
    def mapping_start():
        try:
            return ok(process_manager.start_mapping())
        except Exception as exc:
            return fail('process_error', exc)

    @app.post('/api/debug/mapping/save')
    def mapping_save(request: MappingSaveRequest):
        try:
            return ok('map saved', process_manager.save_map(request.map_name))
        except Exception as exc:
            return fail('process_error', exc)

    @app.post('/api/debug/mapping/stop')
    def mapping_stop():
        try:
            return ok(process_manager.stop('mapping'))
        except Exception as exc:
            return fail('process_error', exc)

    @app.get('/api/debug/navigation/status')
    def navigation_status():
        return bridge.debug_status()

    @app.post('/api/debug/navigation/start')
    def navigation_start():
        try:
            return ok(process_manager.start_navigation())
        except Exception as exc:
            return fail('process_error', exc)

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
            accepted = bridge.send_navigation_goal(request.x, request.y, request.yaw)
            return ok('goal accepted' if accepted else 'goal rejected', {'accepted': accepted})
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
        await websocket.accept()
        try:
            while True:
                await websocket.send_json(bridge.robot_status())
                await asyncio.sleep(0.5)
        except WebSocketDisconnect:
            return
        except Exception:
            return

    return app


def main() -> None:
    logging.basicConfig(level=logging.INFO)
    rclpy.init()
    bridge = MobileRosBridge()
    executor_thread = threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True)
    executor_thread.start()

    workspace_dir = bridge.get_parameter('workspace_dir').value
    default_map_path = bridge.get_parameter('default_map_path').value
    host = bridge.get_parameter('host').value
    port = int(bridge.get_parameter('port').value)
    process_manager = ProcessManager(workspace_dir, default_map_path)
    app = make_app(bridge, process_manager)

    try:
        uvicorn.run(app, host=host, port=port)
    finally:
        bridge.stop_motion()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
