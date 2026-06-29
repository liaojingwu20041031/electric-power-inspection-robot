import json
import os
import threading
import time
from typing import Any, Callable, Dict, Optional

from PyQt5.QtCore import QObject, QTimer, pyqtProperty, pyqtSignal, pyqtSlot

from .route_preview import build_patrol_tasks, generate_route_preview
from .ui_models import UiState


STATUS_TEXT = {
    'running': '运行中',
    'stopped': '已停止',
    'http_ok': '连接正常',
    'http_error': '连接异常',
    'tcp_ok': '连接正常',
    'tcp_error': '连接异常',
    'embedded': '内嵌运行',
    'idle': '空闲',
    'paused': '已暂停',
    'waiting_initial_pose': '等待初始位姿',
    'waiting_nav2': '等待导航服务',
    'sending_goal': '发送导航目标',
    'retrying_goal': '导航目标重试',
    'target': '前往检查点',
    'return_home': '返回初始点',
    'waiting_localization': '等待定位',
    'returning_home': '返回初始点',
    'waiting_loop': '等待下一轮',
    'canceling': '正在取消',
    'succeeded': '已完成',
    'failed': '失败',
    'canceled': '已取消',
    'cancelled': '已取消',
    'unavailable': '不可用',
    'ready': '准备就绪',
    'fault': '故障',
    'mapping': '建图中',
}


class UiBackend(QObject):
    systemStatusChanged = pyqtSignal()
    logsChanged = pyqtSignal()
    robotModeChanged = pyqtSignal()
    controlUnlockedChanged = pyqtSignal()
    patrolStatusChanged = pyqtSignal()
    patrolEventsChanged = pyqtSignal()
    routePreviewChanged = pyqtSignal()
    patrolTasksChanged = pyqtSignal()
    routePreviewLoaded = pyqtSignal(dict, dict)

    def __init__(
        self,
        bridge,
        state: UiState,
        clock: Callable[[], float] = time.monotonic,
        route_preview_loader: Callable[..., Dict[str, Any]] = generate_route_preview,
        patrol_task_builder: Callable[[Any], Dict[str, Dict[str, Any]]] = build_patrol_tasks,
    ) -> None:
        super().__init__()
        self.bridge = bridge
        self.state = state
        self.clock = clock
        self.route_preview_loader = route_preview_loader
        self.patrol_task_builder = patrol_task_builder
        self._route_preview_thread: Optional[threading.Thread] = None
        self._control_unlocked = False
        self._last_control_activity = 0.0
        self._last_system_command_at: Dict[str, float] = {}
        self._last_status_log_key = ('', '')
        self._patrol_start_profile = 'navigation'
        self._has_patrol_status = False
        self.routePreviewLoaded.connect(self._apply_route_preview_result)
        self.safety_timer = QTimer(self)
        self.safety_timer.timeout.connect(self.checkSafetyTimeout)
        self.safety_timer.start(1000)
        self._start_route_preview_refresh(force=False)

    @pyqtProperty('QVariantMap', notify=systemStatusChanged)
    def systemStatus(self) -> Dict[str, Any]:
        return self.state.system_status

    @pyqtProperty('QVariantList', notify=logsChanged)
    def logs(self):
        return self.state.events

    @pyqtProperty(str, notify=robotModeChanged)
    def robotMode(self) -> str:
        return self.state.robot_mode

    @pyqtProperty(str, notify=systemStatusChanged)
    def jetsonIp(self) -> str:
        return str(self.state.system_status.get('jetson_ip', '-'))

    @pyqtProperty(str, notify=systemStatusChanged)
    def appUrl(self) -> str:
        return str(self.state.system_status.get('mobile_bridge_url', '-'))

    @pyqtProperty(bool, notify=controlUnlockedChanged)
    def controlUnlocked(self) -> bool:
        return self._control_unlocked

    @pyqtProperty('QVariantMap', notify=patrolStatusChanged)
    def patrolStatus(self) -> Dict[str, Any]:
        return self.state.patrol_status

    @pyqtProperty(str, notify=patrolStatusChanged)
    def patrolStatusText(self) -> str:
        return self.localizedStatus(str(self.state.patrol_status.get('state', 'idle')))

    @pyqtProperty('QVariantList', notify=patrolEventsChanged)
    def patrolEvents(self):
        return self.state.patrol_events

    @pyqtProperty('QVariantMap', notify=routePreviewChanged)
    def routePreview(self) -> Dict[str, Any]:
        return self.state.route_preview

    @pyqtProperty(str, notify=routePreviewChanged)
    def routePreviewImageUrl(self) -> str:
        return str(self.state.route_preview.get('image_url') or '')

    @pyqtProperty(str, notify=routePreviewChanged)
    def routePreviewImageSource(self) -> str:
        if not self._route_preview_has_overlay_image():
            return ''
        return self.routePreviewImageUrl

    @pyqtProperty(bool, notify=routePreviewChanged)
    def routePreviewOk(self) -> bool:
        return (
            bool(self.state.route_preview.get('ok'))
            and self.state.route_preview.get('preview_type') == 'route_overlay'
            and self.state.route_preview.get('overlay_ok') is True
            and self.state.route_preview.get('image_exists') is True
            and self.state.route_preview.get('image_valid') is True
            and bool(self.state.route_preview.get('image_url'))
        )

    def _route_preview_has_overlay_image(self) -> bool:
        preview = self.state.route_preview
        if not (
            bool(preview.get('ok'))
            and preview.get('preview_type') == 'route_overlay'
            and preview.get('overlay_ok') is True
            and preview.get('image_exists') is True
            and preview.get('image_valid') is True
        ):
            return False
        image_url = str(preview.get('image_url') or '')
        if not image_url:
            return False
        if preview.get('image_exists') is not None:
            return bool(preview.get('image_exists'))
        if image_url.startswith('file://'):
            return os.path.exists(image_url[len('file://'):])
        return True

    @pyqtProperty(str, notify=routePreviewChanged)
    def routePreviewMessage(self) -> str:
        return str(
            self.state.route_preview.get('image_error')
            or self.state.route_preview.get('message')
            or '路线预览图未生成'
        )

    @pyqtProperty(bool, notify=routePreviewChanged)
    def routePreviewLoading(self) -> bool:
        return bool(self.state.route_preview.get('loading'))

    @pyqtProperty(str, notify=systemStatusChanged)
    def patrolStartProfile(self) -> str:
        return self._patrol_start_profile

    @pyqtProperty('QVariantMap', notify=patrolTasksChanged)
    def patrolTasks(self) -> Dict[str, Dict[str, Any]]:
        return self.state.patrol_tasks

    @pyqtProperty(str, notify=systemStatusChanged)
    def patrolModeState(self) -> str:
        return str(self.state.system_status.get('patrol_mode_state') or 'idle')

    @pyqtProperty(str, notify=systemStatusChanged)
    def patrolStartupStep(self) -> str:
        return str(self.state.system_status.get('startup_step') or '')

    @pyqtProperty(str, notify=systemStatusChanged)
    def patrolError(self) -> str:
        return str(self.state.system_status.get('patrol_error') or '')

    @pyqtProperty(bool, notify=systemStatusChanged)
    def patrolReady(self) -> bool:
        readiness = self.state.system_status.get('patrol_readiness') or {}
        required = ('bringup', 'navigation', 'executor', 'route_file')
        return all(bool(readiness.get(key)) for key in required)

    @pyqtProperty(bool, notify=systemStatusChanged)
    def patrolControlsEnabled(self) -> bool:
        readiness = self.state.system_status.get('patrol_readiness') or {}
        return (
            bool(readiness.get('executor'))
            or self.state.system_status.get('patrol_executor') == 'running'
            or self._has_patrol_status
        )

    @pyqtProperty(str, notify=patrolStatusChanged)
    def patrolProgressLabel(self) -> str:
        return self._patrol_progress_label(self.state.patrol_status)

    @pyqtProperty(str, notify=patrolStatusChanged)
    def currentTargetLabel(self) -> str:
        status = self.state.patrol_status
        return str(
            status.get('current_target_label')
            or status.get('target_name')
            or status.get('target_id')
            or ''
        )

    @pyqtSlot(str, result=str)
    def localizedStatus(self, value: str) -> str:
        return STATUS_TEXT.get(str(value), str(value))

    @staticmethod
    def _patrol_progress_label(status: Dict[str, Any]) -> str:
        special_label = str(status.get('current_target_label') or '')
        if special_label:
            return special_label
        try:
            target_index = status.get('target_index')
            target_count = int(status.get('target_count') or 0)
            if target_index is None or target_count <= 0:
                return special_label
            return f'第 {int(target_index) + 1} / {target_count} 个检查点'
        except (TypeError, ValueError):
            return special_label

    @pyqtSlot(str, result=str)
    def assetPath(self, filename: str) -> str:
        root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
        path = os.path.join(root, 'UI_sc', filename)
        return 'file://' + path if os.path.exists(path) else ''

    @pyqtSlot(bool)
    def setControlUnlocked(self, unlocked: bool) -> None:
        self._control_unlocked = bool(unlocked)
        self._last_control_activity = self.clock() if unlocked else 0.0
        self.controlUnlockedChanged.emit()
        self.addLog('运动控制已解锁' if unlocked else '运动控制已锁定')

    @pyqtSlot()
    def checkSafetyTimeout(self) -> None:
        if self._control_unlocked and self.clock() - self._last_control_activity >= 10.0:
            self.setControlUnlocked(False)

    @pyqtSlot(str)
    def sendSystemCommand(self, command: str) -> None:
        if self._is_debounced(command):
            self.addLog(f'忽略重复系统命令: {command}')
            return
        self.bridge.publish_system_command(command)
        self.addLog(f'系统命令: {command}')

    @pyqtSlot(str)
    def sendPatrolCommand(self, command: str) -> None:
        value = str(command or '').strip()
        if value not in ('start', 'pause', 'resume', 'cancel', 'reload', 'initialize'):
            self.addLog(f'忽略未知巡逻命令: {value}')
            return
        if value == 'start':
            self.addLog('请使用一键启动巡逻模式')
            return
        debounce_key = f'patrol:{value}'
        if self._is_debounced(debounce_key):
            self.addLog(f'忽略重复巡逻命令: {value}')
            return
        system_command = {
            'pause': 'pause_patrol',
            'resume': 'resume_patrol',
            'cancel': 'cancel_patrol',
            'reload': 'reload_patrol_route',
            'initialize': 'reload_patrol_route',
        }[value]
        self.bridge.publish_system_command(system_command)
        self.addLog(f'巡逻命令: {value}')
        if value == 'reload':
            self.refreshRoutePreview()

    @pyqtSlot(str)
    def setPatrolStartProfile(self, profile: str) -> None:
        value = str(profile or '').strip()
        self._patrol_start_profile = value if value in ('navigation', 'inspection') else 'navigation'
        self.systemStatusChanged.emit()

    @pyqtSlot()
    def startPatrolMode(self) -> None:
        patrol_state = str(self.state.patrol_status.get('state') or '')
        if patrol_state in (
            'waiting_initial_pose',
            'waiting_nav2',
            'waiting_localization',
            'running',
            'paused',
            'returning_home',
        ):
            self.addLog(f'巡逻执行器已在工作: {self.localizedStatus(patrol_state)}')
            return
        if self._is_debounced('start_patrol_mode'):
            self.addLog('忽略重复系统命令: start_patrol_mode')
            return
        self.bridge.publish_system_command(
            'start_patrol_mode',
            profile=self._patrol_start_profile,
        )
        self.addLog('系统命令: start_patrol_mode')

    def _is_debounced(self, command: str) -> bool:
        debounced = {
            'start_patrol_mode',
            'patrol:start',
            'patrol:pause',
            'patrol:resume',
            'patrol:cancel',
            'patrol:reload',
            'patrol:initialize',
        }
        if command not in debounced:
            return False
        now = self.clock()
        previous = self._last_system_command_at.get(command)
        if previous is not None and now - previous < 0.8:
            return True
        self._last_system_command_at[command] = now
        return False

    @pyqtSlot()
    def refreshRoutePreview(self) -> None:
        self._start_route_preview_refresh(force=True)

    def _start_route_preview_refresh(self, force: bool) -> None:
        if self._route_preview_thread and self._route_preview_thread.is_alive():
            return
        self.state.route_preview = {
            **self.state.route_preview,
            'ok': False,
            'message': '正在生成路线预览...',
            'loading': True,
        }
        self.routePreviewChanged.emit()
        self._route_preview_thread = threading.Thread(
            target=self._refresh_route_preview_worker,
            args=(force,),
            daemon=True,
        )
        self._route_preview_thread.start()

    def _refresh_route_preview_worker(self, force: bool) -> None:
        try:
            preview = self.route_preview_loader(force=force)
        except Exception as exc:
            preview = {
                'ok': False,
                'message': f'路线预览失败: {exc}',
                'image_url': '',
                'source': 'route_preview_loader',
                'targets': [],
            }
        tasks = self.patrol_task_builder(preview.get('targets', []))
        preview = {**preview, 'loading': False}
        try:
            self.routePreviewLoaded.emit(preview, tasks)
        except RuntimeError:
            pass

    @pyqtSlot(dict, dict)
    def _apply_route_preview_result(
        self,
        preview: Dict[str, Any],
        tasks: Dict[str, Dict[str, Any]],
    ) -> None:
        self.state.route_preview = preview
        self.state.patrol_tasks = tasks
        self.routePreviewChanged.emit()
        self.patrolTasksChanged.emit()

    @pyqtSlot(str)
    def saveMap(self, map_name: str) -> None:
        name = map_name.strip() or time.strftime('inspection_map_%Y%m%d_%H%M')
        self.bridge.publish_system_command('save_map', map_name=name)
        self.addLog(f'保存地图: {name}')

    @pyqtSlot(str)
    def sendTextCommand(self, text: str) -> None:
        if text.strip():
            self.bridge.publish_text_command(text.strip())
            self.addLog(f'UI 指令: {text.strip()}')

    @pyqtSlot(str)
    def setRobotMode(self, mode: str) -> None:
        self.state.robot_mode = mode
        self.bridge.publish_system_mode(mode)
        self.robotModeChanged.emit()
        self.addLog(f'系统模式: {mode}')

    @pyqtSlot()
    def moveForward(self) -> None:
        self._move(0.12, 0.0, '前进')

    @pyqtSlot()
    def moveBackward(self) -> None:
        self._move(-0.12, 0.0, '后退')

    @pyqtSlot()
    def turnLeft(self) -> None:
        self._move(0.0, 0.45, '左转')

    @pyqtSlot()
    def turnRight(self) -> None:
        self._move(0.0, -0.45, '右转')

    @pyqtSlot()
    def stopMotion(self) -> None:
        self.bridge.publish_twist(0.0, 0.0)
        self.addLog('运动停止')

    @pyqtSlot()
    def emergencyStop(self) -> None:
        self.bridge.publish_twist(0.0, 0.0)
        self.bridge.publish_system_command('emergency_stop')
        if self._control_unlocked:
            self.setControlUnlocked(False)
        self.addLog('软件急停已发送')

    def _move(self, linear: float, angular: float, label: str) -> None:
        if not self._control_unlocked:
            self.addLog(f'控制已锁定，忽略{label}')
            return
        self._last_control_activity = self.clock()
        self.bridge.publish_twist(linear, angular)
        QTimer.singleShot(350, lambda: self.bridge.publish_twist(0.0, 0.0))
        self.addLog(f'短时运动: {label}')

    @pyqtSlot(str)
    def callVoiceService(self, name: str) -> None:
        self.bridge.call_voice_service(name)
        self.addLog(f'语音服务: {name}')

    @pyqtSlot(str)
    def addLog(self, message: str) -> None:
        self.state.add_event(message)
        self.logsChanged.emit()

    def update_system_status(self, payload: Dict[str, Any]) -> None:
        self.state.system_status = payload
        self.systemStatusChanged.emit()
        message = payload.get('message')
        log_key = (str(payload.get('last_command') or ''), str(message or ''))
        if message and log_key != self._last_status_log_key:
            self._last_status_log_key = log_key
            self.addLog(str(message))

    def update_patrol_status(self, payload: Dict[str, Any]) -> None:
        if not payload:
            payload = {
                'state': 'unavailable',
                'executor_running': False,
                'message': '巡逻执行器未运行，启动巡逻模式后再操作。',
            }
        else:
            self._has_patrol_status = True
        self.state.patrol_status = payload
        self.patrolStatusChanged.emit()
        self.systemStatusChanged.emit()

    def update_patrol_event(self, payload: Dict[str, Any]) -> None:
        payload = dict(payload)
        payload.setdefault('timestamp', time.strftime('%H:%M:%S'))
        self.state.patrol_events.append(payload)
        if len(self.state.patrol_events) > 100:
            del self.state.patrol_events[:-100]
        self.patrolEventsChanged.emit()

    def update_task_context(self, payload: Dict[str, Any]) -> None:
        self.state.task_context = payload

    def on_task_event(self, msg: Any) -> None:
        self.addLog(f'任务事件: {msg.intent} task={msg.task_id}')

    def on_task_status(self, msg: Any) -> None:
        self.addLog(f'任务状态: {msg.task_id} {msg.stage}/{msg.status} {msg.reason}')

    def on_say_text(self, msg: Any) -> None:
        self.addLog(f'播报: {msg.text}')

    def on_voice_status(self, msg: Any) -> None:
        if hasattr(msg, 'state') or hasattr(msg, 'text'):
            status = f"{getattr(msg, 'state', '')} {getattr(msg, 'text', '')}".strip()
        else:
            state = '播报中' if bool(getattr(msg, 'speaking', False)) else '空闲'
            task_id = str(getattr(msg, 'current_task_id', '') or '').strip()
            status = f'{state} {task_id}'.strip()
        self.state.voice_status = status
        self.state.system_status = dict(self.state.system_status)
        self.state.system_status['voice_status'] = status
        self.systemStatusChanged.emit()

    def on_localized_objects(self, text: str) -> None:
        self.state.localized_objects = text[:4000]
        try:
            count = len(json.loads(text))
            self.addLog(f'感知目标更新: {count}')
        except (json.JSONDecodeError, TypeError):
            pass

    def shutdown(self) -> None:
        self.safety_timer.stop()
        thread = self._route_preview_thread
        if thread and thread.is_alive():
            thread.join(timeout=1.0)
