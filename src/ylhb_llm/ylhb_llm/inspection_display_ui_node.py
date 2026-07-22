import os
import re
import signal
import sys
import threading
import time
from typing import List, Optional

os.environ.setdefault('QT_QUICK_BACKEND', 'software')

import rclpy
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtCore import QTimer, QUrl
from PyQt5.QtGui import QGuiApplication, QIcon
from rclpy.executors import SingleThreadedExecutor

from .ui_backend import UiBackend
from .ui_models import UiState
from .ui_ros_bridge import InspectionDisplayRosBridge, UiSignals


def qml_main_path() -> str:
    try:
        installed = os.path.join(get_package_share_directory('ylhb_llm'), 'qml', 'Main.qml')
        if os.path.exists(installed):
            return installed
    except Exception:
        pass
    return os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'qml', 'Main.qml'))


def display_socket_path(display: str | None = None) -> str:
    match = re.fullmatch(r':(\d+)(?:\.\d+)?', str(display or os.environ.get('DISPLAY') or ''))
    return f'/tmp/.X11-unix/X{match.group(1)}' if match else ''


def display_is_local() -> bool:
    return bool(display_socket_path())


def display_is_ready() -> bool:
    xauthority = os.environ.get('XAUTHORITY', '')
    if not display_is_local() or not os.path.exists(display_socket_path()):
        return False
    if xauthority and not os.access(xauthority, os.R_OK):
        return False
    return os.system('timeout 3 xset q >/dev/null 2>&1') == 0


def wait_for_display() -> bool:
    if os.environ.get('YLHB_UI_WAIT_FOR_DISPLAY', 'true').lower() not in {'1', 'true', 'yes', 'on'}:
        return display_is_ready()
    retry = max(1.0, float(os.environ.get('YLHB_UI_DISPLAY_RETRY_SEC', '2')))
    timeout = max(0.0, float(os.environ.get('YLHB_UI_DISPLAY_WAIT_TIMEOUT_SEC', '0')))
    started = time.monotonic()
    last_log = 0.0
    while not display_is_ready():
        now = time.monotonic()
        if not last_log or now - last_log >= 10.0:
            print('正在等待本地图形会话恢复', file=sys.stderr, flush=True)
            last_log = now
        if timeout and now - started >= timeout:
            return False
        time.sleep(retry)
    return True


def main(args: Optional[List[str]] = None) -> None:
    try:
        from PyQt5.QtQml import QQmlApplicationEngine
    except ImportError as exc:
        print(
            '无法加载 QtQml。请安装: sudo apt install python3-pyqt5.qtquick '
            'qml-module-qtquick2 qml-module-qtquick-window2 qml-module-qtquick-controls2 '
            'qml-module-qtquick-layouts qml-module-qtqml-models2',
            file=sys.stderr,
        )
        print(f'QtQml import error: {exc}', file=sys.stderr)
        return

    if not wait_for_display():
        print('本地图形会话未在等待时限内恢复', file=sys.stderr)
        return
    rclpy.init(args=args)
    app = QGuiApplication(sys.argv[:1])
    icon_path = os.path.join(
        get_package_share_directory('ylhb_llm'), 'UI_sc', 'UI图标.png'
    )
    if os.path.exists(icon_path):
        app.setWindowIcon(QIcon(icon_path))
    shutdown_requested = threading.Event()
    signal.signal(signal.SIGINT, lambda *_args: shutdown_requested.set())
    signal.signal(signal.SIGTERM, lambda *_args: shutdown_requested.set())
    signal_timer = QTimer(app)
    signal_timer.timeout.connect(
        lambda: app.quit() if shutdown_requested.is_set() else None
    )
    signal_timer.start(100)
    signals = UiSignals()
    bridge = InspectionDisplayRosBridge(signals)
    backend = UiBackend(bridge, UiState())
    backend.shutdownRequested.connect(app.quit)
    signals.systemStatus.connect(backend.update_system_status)
    signals.localAppStatus.connect(backend.update_local_app_status)
    signals.cloudStatus.connect(backend.update_cloud_status)
    signals.bridgeAvailability.connect(backend.update_bridge_availability)
    signals.localAppControlResult.connect(backend.update_local_app_control_result)
    signals.cloudControlResult.connect(backend.update_cloud_control_result)
    signals.platformStartConfirmResult.connect(
        backend.update_platform_start_confirm_result
    )
    signals.taskContext.connect(backend.update_task_context)
    signals.taskEvent.connect(backend.on_task_event)
    signals.taskStatus.connect(backend.on_task_status)
    signals.sayText.connect(backend.on_say_text)
    signals.voiceStatus.connect(backend.on_voice_status)
    signals.voiceSessionStatus.connect(backend.update_voice_session_status)
    signals.voiceServiceResult.connect(backend.update_voice_service_result)
    signals.agentStatus.connect(backend.update_agent_status)
    signals.agentEvent.connect(backend.update_agent_event)
    signals.agentChat.connect(backend.update_agent_chat)
    signals.localizedObjects.connect(backend.on_localized_objects)
    signals.patrolStatus.connect(backend.update_patrol_status)
    signals.patrolEvent.connect(backend.update_patrol_event)
    signals.mapping3dStatus.connect(backend.update_mapping3d_status)
    signals.mapping3dResult.connect(backend.update_mapping3d_result)

    engine = QQmlApplicationEngine()
    engine.rootContext().setContextProperty('backend', backend)
    errors = []
    engine.warnings.connect(lambda warnings: errors.extend(str(item.toString()) for item in warnings))
    path = qml_main_path()
    engine.load(QUrl.fromLocalFile(path))
    if not engine.rootObjects():
        print(f'QML 启动失败: {path}', file=sys.stderr)
        for error in errors:
            print(error, file=sys.stderr)
        print(
            '请确认已安装 QtQuick/Window/Controls2/Layout/QML Models 依赖。',
            file=sys.stderr,
        )
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return

    root = engine.rootObjects()[0]
    if bool(bridge.get_parameter('fullscreen').value):
        root.showFullScreen()
    else:
        root.show()
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    try:
        app.exec_()
    finally:
        backend.shutdown()
        executor.shutdown(timeout_sec=1.0)
        executor_thread.join(timeout=1.0)
        try:
            executor.remove_node(bridge)
        except Exception:
            pass
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
