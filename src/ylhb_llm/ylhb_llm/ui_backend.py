import json
import os
import threading
import time
import uuid
from typing import Any, Callable, Dict, Optional

from PyQt5.QtCore import QObject, QTimer, pyqtProperty, pyqtSignal, pyqtSlot

from .route_preview import build_patrol_tasks, generate_route_preview
from .ui_models import UiState
from .agent_chat_schema import dedupe_key, make_agent_chat


STATUS_TEXT = {
    'running': '运行中',
    'recording': '录制中',
    'reconstructing': '重建中',
    'stopped': '已停止',
    'http_ok': '连接正常',
    'http_error': '连接异常',
    'tcp_ok': '连接正常',
    'tcp_error': '连接异常',
    'embedded': '内嵌运行',
    'idle': '空闲',
    'command_sent': '等待巡逻执行器进入运行状态',
    'paused': '已暂停',
    'waiting_initial_pose': '等待初始位姿',
    'waiting_nav2': '等待导航服务',
    'waiting_after_bringup': '底盘启动后等待',
    'waiting_after_navigation': '导航启动后等待',
    'waiting_after_executor': '巡逻执行器启动后等待',
    'patrol_start_sent': '巡逻启动命令已发送',
    'waiting_executor_response': '等待巡逻执行器响应',
    'waiting_map_to_odom': '等待 map->odom TF',
    'waiting_nav2_active': '等待 Nav2 lifecycle active',
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
    'patrol_failed': '巡逻启动失败',
}

PATROL_ACTIVE_STATES = {
    'waiting_initial_pose',
    'waiting_nav2',
    'waiting_localization',
    'running',
    'paused',
    'returning_home',
    'waiting_loop',
    'canceling',
}

PATROL_NAVIGATION_ACTIVE_PHASES = {
    'waiting_nav2',
    'sending_goal',
    'retrying_goal',
    'target',
    'return_home',
}

PATROL_TERMINAL_LABELS = {
    'succeeded': '已完成',
    'failed': '失败',
    'canceled': '已取消',
    'cancelled': '已取消',
}


class UiBackend(QObject):
    systemStatusChanged = pyqtSignal()
    logsChanged = pyqtSignal()
    robotModeChanged = pyqtSignal()
    controlUnlockedChanged = pyqtSignal()
    patrolStatusChanged = pyqtSignal()
    patrolEventsChanged = pyqtSignal()
    mapping3dStatusChanged = pyqtSignal()
    routePreviewChanged = pyqtSignal()
    patrolTasksChanged = pyqtSignal()
    routePreviewLoaded = pyqtSignal(dict, dict)
    uiReadyChanged = pyqtSignal()
    agentStatusChanged = pyqtSignal()
    agentMessagesChanged = pyqtSignal()
    agentDebugVisibleChanged = pyqtSignal()
    voiceStatusChanged = pyqtSignal()

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
        self._route_preview_mode = 'route_focus'
        self._control_unlocked = False
        self._last_control_activity = 0.0
        self._last_system_command_at: Dict[str, float] = {}
        self._last_status_log_key = ('', '')
        self._patrol_start_profile = 'navigation'
        self._has_patrol_status = False
        self._agent_debug_visible = False
        self._agent_message_keys: set[str] = set()
        self._ui_ready = False
        self._startup_loading_text = '正在准备操控台...'
        self.routePreviewLoaded.connect(self._apply_route_preview_result)
        self.safety_timer = QTimer(self)
        self.safety_timer.timeout.connect(self.checkSafetyTimeout)
        self.safety_timer.start(1000)
        self.startup_timer = QTimer(self)
        self.startup_timer.setSingleShot(True)
        self.startup_timer.timeout.connect(self.finishStartup)
        self.startup_timer.start(2500)

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

    @pyqtProperty('QVariantMap', notify=mapping3dStatusChanged)
    def mapping3dStatus(self) -> Dict[str, Any]:
        return self.state.mapping3d_status

    @pyqtProperty('QVariantMap', notify=mapping3dStatusChanged)
    def mapping3dResult(self) -> Dict[str, Any]:
        return self.state.mapping3d_result

    @pyqtProperty(str, notify=mapping3dStatusChanged)
    def mapping3dStateText(self) -> str:
        status = self.state.mapping3d_status
        state = str(status.get('state') or self.state.system_status.get('3d_mapping') or 'stopped')
        message = str(status.get('message') or '')
        return f'{self.localizedStatus(state)} {message}'.strip()

    @pyqtProperty(str, notify=mapping3dStatusChanged)
    def mapping3dCaptureText(self) -> str:
        status = self.state.mapping3d_status
        latest = self.state.system_status.get('latest_3d_capture') or {}
        state = str(status.get('state') or self.state.system_status.get('3d_capture') or 'stopped')
        frames = status.get('svo_frame_count') or status.get('success_frames') or latest.get('svo_frame_count')
        suffix = f' {frames} 帧' if frames not in (None, '') else ''
        return f'{self.localizedStatus(state)}{suffix}'

    @pyqtProperty(str, notify=mapping3dStatusChanged)
    def mapping3dReconstructText(self) -> str:
        status = self.state.mapping3d_result or self.state.system_status.get('latest_3d_reconstruct') or {}
        state = str(status.get('state') or self.state.system_status.get('3d_reconstruct') or 'stopped')
        points = status.get('export_point_count')
        suffix = f' {points} 点' if points not in (None, '') else ''
        return f'{self.localizedStatus(state)}{suffix}'

    @pyqtProperty(str, notify=mapping3dStatusChanged)
    def latestSvoFile(self) -> str:
        latest = self.state.system_status.get('latest_3d_capture') or {}
        return str(
            latest.get('svo_file')
            or self.state.mapping3d_status.get('svo_file')
            or ''
        )

    @pyqtProperty(str, notify=mapping3dStatusChanged)
    def latestModelFile(self) -> str:
        latest = self.state.system_status.get('latest_3d_reconstruct') or {}
        return str(
            latest.get('output_file')
            or self.state.mapping3d_result.get('output_file')
            or ''
        )

    @pyqtProperty(bool, notify=mapping3dStatusChanged)
    def mapping3dCanStartCapture(self) -> bool:
        return self.state.system_status.get('3d_capture') != 'running'

    @pyqtProperty(bool, notify=mapping3dStatusChanged)
    def mapping3dCanStopCapture(self) -> bool:
        return self.state.system_status.get('3d_capture') == 'running'

    @pyqtProperty(bool, notify=mapping3dStatusChanged)
    def mapping3dCanReconstruct(self) -> bool:
        return bool(self.latestSvoFile) and self.state.system_status.get('3d_reconstruct') != 'running'

    @pyqtProperty(bool, notify=uiReadyChanged)
    def uiReady(self) -> bool:
        return self._ui_ready

    @pyqtProperty(str, notify=uiReadyChanged)
    def startupLoadingText(self) -> str:
        return self._startup_loading_text

    @pyqtProperty('QVariantMap', notify=agentStatusChanged)
    def agentStatus(self) -> Dict[str, Any]:
        return self.state.agent_status

    @pyqtProperty('QVariantList', notify=agentStatusChanged)
    def agentEvents(self):
        return self.state.agent_events

    @pyqtProperty('QVariantList', notify=agentMessagesChanged)
    def agentMessages(self):
        return self.state.agent_messages

    @pyqtProperty('QVariantMap', notify=agentStatusChanged)
    def agentSpecSummary(self) -> Dict[str, Any]:
        return self.state.agent_spec_summary

    @pyqtProperty(bool, notify=agentDebugVisibleChanged)
    def agentDebugVisible(self) -> bool:
        return self._agent_debug_visible

    @pyqtProperty('QVariantMap', notify=voiceStatusChanged)
    def voiceSessionStatus(self) -> Dict[str, Any]:
        return self.state.voice_session_status

    @pyqtProperty(bool, notify=voiceStatusChanged)
    def voiceSessionEnabled(self) -> bool:
        return bool(self.state.voice_session_status.get('enabled'))

    @pyqtProperty(str, notify=voiceStatusChanged)
    def voiceSessionState(self) -> str:
        return str(self.state.voice_session_status.get('state') or 'OFF')

    @pyqtProperty(str, notify=voiceStatusChanged)
    def voiceSessionStateText(self) -> str:
        label = str(self.state.voice_session_status.get('agent_voice_state_label') or '')
        if label:
            return label
        labels = {
            'OFF': '关闭',
            'WAIT_WAKE': '待唤醒',
            'LISTENING': '正在听',
            'AWAKENED_IDLE': '正在听',
            'CONTEXT_FOLLOWUP': '正在听',
            'RECORDING': '录音中',
            'ASR_PROCESSING': '识别中',
            'TTS_PAUSED': '播报中',
            'WAITING_RESPONSE': '等待响应',
        }
        return labels.get(self.voiceSessionState, self.voiceSessionState)

    @pyqtProperty(str, notify=voiceStatusChanged)
    def voiceStatusSummary(self) -> str:
        return f'{self.voiceSessionStateText} {self.voiceWaitingFor}'.strip()

    @pyqtProperty(str, notify=voiceStatusChanged)
    def voiceActivityText(self) -> str:
        state = str(self.state.voice_session_status.get('agent_voice_state') or '')
        if not state:
            state = self.voiceSessionState
        if state in ('off', 'OFF'):
            return '语音关闭'
        if state in ('waiting_wake', 'WAIT_WAKE'):
            return f'等待唤醒词：{self.voiceWakePhrase or "小零小零"}'
        if state in ('listening', 'LISTENING', 'AWAKENED_IDLE', 'CONTEXT_FOLLOWUP'):
            return '正在接收语音'
        if state in ('recording', 'RECORDING'):
            return '正在录音'
        if state in ('recognizing', 'ASR_PROCESSING'):
            return '正在识别'
        if state in ('responding', 'WAITING_RESPONSE', 'TTS_PAUSED'):
            return '等待响应'
        if state == 'error':
            return self.voiceLastError or '语音异常'
        return state

    @pyqtProperty(str, notify=voiceStatusChanged)
    def voiceActivityTone(self) -> str:
        state = str(self.state.voice_session_status.get('agent_voice_state') or '')
        if not state:
            state = self.voiceSessionState
        if state in ('listening', 'recording', 'LISTENING', 'AWAKENED_IDLE', 'CONTEXT_FOLLOWUP', 'RECORDING'):
            return 'active'
        if state in ('recognizing', 'responding', 'ASR_PROCESSING', 'WAITING_RESPONSE'):
            return 'busy'
        if state == 'TTS_PAUSED':
            return 'speaking'
        if state in ('waiting_wake', 'WAIT_WAKE'):
            return 'wake'
        if state == 'error':
            return 'busy'
        return 'off'

    @pyqtProperty(str, notify=voiceStatusChanged)
    def voiceWakePhrase(self) -> str:
        return str(self.state.voice_session_status.get('wake_phrase') or '')

    @pyqtProperty(str, notify=voiceStatusChanged)
    def voiceLastAsrText(self) -> str:
        return str(self.state.voice_session_status.get('last_asr_text') or '')

    @pyqtProperty(str, notify=voiceStatusChanged)
    def voiceLastPublishedText(self) -> str:
        return str(self.state.voice_session_status.get('last_published_text') or '')

    @pyqtProperty(str, notify=voiceStatusChanged)
    def voiceLastError(self) -> str:
        return str(self.state.voice_session_status.get('last_error') or '')

    @pyqtProperty(bool, notify=voiceStatusChanged)
    def voiceRecording(self) -> bool:
        return bool(self.state.voice_session_status.get('is_recording'))

    @pyqtProperty(bool, notify=voiceStatusChanged)
    def voiceSpeaking(self) -> bool:
        return bool(self.state.voice_session_status.get('is_tts_playing'))

    @pyqtProperty(int, notify=voiceStatusChanged)
    def voiceAsrFailCount(self) -> int:
        return int(self.state.voice_session_status.get('asr_fail_count') or 0)

    @pyqtProperty(str, notify=voiceStatusChanged)
    def voiceWaitingFor(self) -> str:
        return str(self.state.voice_session_status.get('waiting_for') or '')

    @pyqtProperty(str, notify=voiceStatusChanged)
    def voiceServiceStatus(self) -> str:
        return self.state.voice_service_status

    @pyqtProperty(str, notify=voiceStatusChanged)
    def voiceTtsStatus(self) -> str:
        return self.state.voice_status

    @pyqtProperty(str, notify=agentStatusChanged)
    def agentStatusText(self) -> str:
        status = self.state.agent_status
        last_tool = str(status.get('last_tool') or '')
        result = str(status.get('last_result_status') or status.get('state') or 'ready')
        return f'{result} {last_tool}'.strip()

    @pyqtProperty(str, notify=agentStatusChanged)
    def agentLastIntent(self) -> str:
        return str(self.state.agent_status.get('last_intent') or '')

    @pyqtProperty(str, notify=agentStatusChanged)
    def agentLastTool(self) -> str:
        return str(self.state.agent_status.get('last_tool') or '')

    @pyqtProperty(str, notify=agentStatusChanged)
    def agentLastResult(self) -> str:
        return str(self.state.agent_status.get('last_result_status') or '')

    @pyqtProperty(str, notify=agentStatusChanged)
    def agentLastError(self) -> str:
        return str(self.state.agent_status.get('last_error') or '')

    @pyqtProperty(str, notify=agentStatusChanged)
    def agentStatusSummary(self) -> str:
        parts = [self.agentLastIntent, self.agentLastTool, self.agentLastResult]
        return ' / '.join(part for part in parts if part) or 'ready'

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

    @pyqtProperty(str, notify=routePreviewChanged)
    def routePreviewMode(self) -> str:
        return self._route_preview_mode

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

    def _patrol_executor_running(self) -> bool:
        return self.state.system_status.get('patrol_executor') == 'running'

    @pyqtProperty(bool, notify=systemStatusChanged)
    def patrolStarting(self) -> bool:
        return self.patrolModeState in ('starting', 'command_sent')

    @pyqtProperty(bool, notify=systemStatusChanged)
    def patrolActive(self) -> bool:
        return (
            self._patrol_executor_running()
            and str(self.state.patrol_status.get('state') or '') in PATROL_ACTIVE_STATES
            or self.patrolModeState == 'running'
        )

    @pyqtProperty(bool, notify=systemStatusChanged)
    def patrolCanStart(self) -> bool:
        return not self.patrolStarting and not self.patrolActive

    @pyqtProperty(bool, notify=systemStatusChanged)
    def patrolCanPause(self) -> bool:
        state = str(self.state.patrol_status.get('state') or '')
        return self._patrol_executor_running() and state in (
            'running',
            'returning_home',
            'waiting_loop',
        )

    @pyqtProperty(bool, notify=systemStatusChanged)
    def patrolCanResume(self) -> bool:
        return (
            self._patrol_executor_running()
            and str(self.state.patrol_status.get('state') or '') == 'paused'
        )

    @pyqtProperty(bool, notify=systemStatusChanged)
    def patrolCanCancel(self) -> bool:
        status = self.state.patrol_status
        return (
            self._patrol_executor_running()
            and (
                str(status.get('state') or '') in (
                    'running',
                    'paused',
                    'returning_home',
                    'waiting_loop',
                    'canceling',
                )
                or str(status.get('navigation_phase') or '') in PATROL_NAVIGATION_ACTIVE_PHASES
            )
        )

    @pyqtProperty(str, notify=systemStatusChanged)
    def patrolStateLabel(self) -> str:
        error = self.patrolError
        if error:
            return f'异常: {error}'
        if self.patrolStarting:
            return '启动中: ' + str(
                self.state.system_status.get('startup_step_label')
                or self.localizedStatus(self.patrolStartupStep)
                or '准备依赖'
            )
        if self.patrolActive:
            return '运行中: ' + (self.patrolProgressLabel or self.patrolStatusText)
        state = str(self.state.patrol_status.get('state') or self.patrolModeState)
        if state in PATROL_TERMINAL_LABELS:
            return PATROL_TERMINAL_LABELS[state]
        return '就绪: 可启动巡逻' if self.patrolReady else '待命: 等待巡逻依赖'

    @pyqtProperty(str, notify=patrolStatusChanged)
    def patrolMainStatusLabel(self) -> str:
        state = str(self.state.patrol_status.get('state') or self.patrolModeState)
        labels = {
            'running': '运行中',
            'waiting_loop': '等待下一轮',
            'returning_home': '返回初始点',
            'paused': '已暂停',
            'canceled': '已取消',
            'cancelled': '已取消',
            'failed': '失败',
            'succeeded': '已完成',
        }
        return labels.get(state, self.localizedStatus(state))

    @pyqtProperty(str, notify=patrolStatusChanged)
    def patrolCycleLabel(self) -> str:
        status = self.state.patrol_status
        try:
            cycle_index = int(status.get('cycle_index') or 0)
        except (TypeError, ValueError):
            cycle_index = 0
        if cycle_index <= 0:
            return ''
        if bool(status.get('loop_is_infinite')):
            return f'第 {cycle_index} 轮 / 无限循环'
        try:
            max_cycles = int(status.get('loop_max_cycles') or 0)
        except (TypeError, ValueError):
            max_cycles = 0
        if max_cycles > 0:
            return f'第 {cycle_index} / {max_cycles} 轮'
        return f'第 {cycle_index} 轮'

    @pyqtProperty(str, notify=patrolStatusChanged)
    def patrolNextCycleLabel(self) -> str:
        status = self.state.patrol_status
        if str(status.get('state') or '') != 'waiting_loop':
            return ''
        try:
            remaining = int(status.get('loop_wait_remaining_sec'))
        except (TypeError, ValueError):
            return ''
        remaining = max(0, remaining)
        return f'距离下一轮 {remaining // 60:02d}:{remaining % 60:02d}'

    @pyqtProperty(str, notify=patrolStatusChanged)
    def patrolOverviewProgressLabel(self) -> str:
        return self._patrol_progress_label(self.state.patrol_status) or '未开始'

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

    @pyqtSlot()
    def start3dCapture(self) -> None:
        self.bridge.publish_system_command('start_3d_mapping')
        self.addLog('系统命令: start_3d_mapping')

    @pyqtSlot()
    def stop3dCapture(self) -> None:
        self.bridge.publish_system_command('stop_3d_mapping')
        self.addLog('系统命令: stop_3d_mapping')

    @pyqtSlot(str)
    def reconstructLatest3dMap(self, profile: str) -> None:
        command = {
            'fast_check': 'reconstruct_fast_3d_map',
            'quality_plus': 'reconstruct_quality_3d_map',
        }.get(str(profile or '').strip(), 'reconstruct_latest_3d_map')
        self.bridge.publish_system_command(command)
        self.addLog(f'系统命令: {command}')

    @pyqtSlot(str, str, str)
    def rename3dAsset(self, asset_type: str, session_id: str, display_name: str) -> None:
        self.bridge.publish_system_command(
            'rename_3d_asset',
            asset_type=str(asset_type or 'capture'),
            session_id=str(session_id or ''),
            display_name=str(display_name or ''),
        )

    @pyqtSlot(str, str)
    def delete3dAsset(self, asset_type: str, session_id: str) -> None:
        self.bridge.publish_system_command(
            'delete_3d_asset',
            asset_type=str(asset_type or 'capture'),
            session_id=str(session_id or ''),
        )

    @pyqtSlot(str)
    def setLatest3dCapture(self, session_id: str) -> None:
        self.bridge.publish_system_command('set_latest_3d_capture', session_id=str(session_id or ''))

    @pyqtSlot(str)
    def setLatest3dReconstruct(self, session_id: str) -> None:
        self.bridge.publish_system_command('set_latest_3d_reconstruct', session_id=str(session_id or ''))

    @pyqtSlot(str, str)
    def reconstruct3dCapture(self, session_id: str, profile: str) -> None:
        command = {
            'fast_check': 'reconstruct_fast_3d_map',
            'quality_plus': 'reconstruct_quality_3d_map',
        }.get(str(profile or '').strip(), 'reconstruct_latest_3d_map')
        self.bridge.publish_system_command(command, session_id=str(session_id or ''))

    def _is_debounced(self, command: str) -> bool:
        cooldowns = {
            'start_patrol_mode': 0.8,
            'pause_patrol': 0.8,
            'resume_patrol': 0.8,
            'reload_patrol_route': 0.8,
            'stop_robot_stack': 5.0,
            'stop_navigation': 5.0,
            'stop_bringup': 5.0,
            'stop_patrol_mode': 5.0,
            'patrol:start': 0.8,
            'patrol:pause': 0.8,
            'patrol:resume': 0.8,
            'patrol:cancel': 0.8,
            'patrol:reload': 0.8,
            'patrol:initialize': 0.8,
        }
        cooldown = cooldowns.get(command)
        if cooldown is None:
            return False
        now = self.clock()
        previous = self._last_system_command_at.get(command)
        if previous is not None and now - previous < cooldown:
            return True
        self._last_system_command_at[command] = now
        return False

    @pyqtSlot()
    def refreshRoutePreview(self) -> None:
        self._start_route_preview_refresh(force=True)

    @pyqtSlot(str)
    def setRoutePreviewMode(self, mode: str) -> None:
        if mode not in {'route_focus', 'full_map'} or mode == self._route_preview_mode:
            return
        self._route_preview_mode = mode
        self._start_route_preview_refresh(force=True)

    @pyqtSlot()
    def finishStartup(self) -> None:
        if self._ui_ready:
            return
        self._ui_ready = True
        self._startup_loading_text = '正在加载路线预览...'
        self.uiReadyChanged.emit()
        self._start_route_preview_refresh(force=False)

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
            try:
                preview = self.route_preview_loader(
                    force=force, preview_mode=self._route_preview_mode
                )
            except TypeError:
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
    def sendAgentText(self, text: str) -> None:
        value = text.strip()
        if value:
            client_msg_id = f'ui_{uuid.uuid4().hex[:12]}'
            self.update_agent_chat(make_agent_chat('user', value, client_msg_id, client_msg_id, source='ui'))
            self.bridge.publish_agent_request(value, client_msg_id)
            self.addLog(f'语言 Agent: {value}')

    @pyqtSlot()
    def clearAgentMessages(self) -> None:
        self.state.agent_messages.clear()
        self._agent_message_keys.clear()
        self.agentMessagesChanged.emit()

    @pyqtSlot()
    def toggleAgentDebugVisible(self) -> None:
        self._agent_debug_visible = not self._agent_debug_visible
        self.agentDebugVisibleChanged.emit()

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
        if (
            payload.get('patrol_mode_state') == 'idle'
            and payload.get('patrol_executor') == 'stopped'
        ):
            self.state.patrol_status = {'state': 'idle'}
            self.patrolStatusChanged.emit()
        latest_mapping3d = payload.get('latest_mapping3d_status')
        if isinstance(latest_mapping3d, dict) and latest_mapping3d:
            self.state.mapping3d_status = latest_mapping3d
            self.mapping3dStatusChanged.emit()
        latest_mapping3d_result = payload.get('latest_mapping3d_result')
        if isinstance(latest_mapping3d_result, dict) and latest_mapping3d_result:
            self.state.mapping3d_result = latest_mapping3d_result
            self.mapping3dStatusChanged.emit()
        self.systemStatusChanged.emit()
        self.mapping3dStatusChanged.emit()
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

    def update_mapping3d_status(self, payload: Dict[str, Any]) -> None:
        self.state.mapping3d_status = payload
        self.mapping3dStatusChanged.emit()

    def update_mapping3d_result(self, payload: Dict[str, Any]) -> None:
        self.state.mapping3d_result = payload
        self.mapping3dStatusChanged.emit()

    def update_task_context(self, payload: Dict[str, Any]) -> None:
        self.state.task_context = payload

    def update_agent_status(self, payload: Dict[str, Any]) -> None:
        self.state.agent_status = payload
        summary = payload.get('agent_spec_summary')
        if isinstance(summary, dict):
            self.state.agent_spec_summary = summary
        self.agentStatusChanged.emit()

    def update_agent_event(self, payload: Dict[str, Any]) -> None:
        payload = dict(payload)
        payload.setdefault('timestamp', time.strftime('%H:%M:%S'))
        self.state.agent_events.append(payload)
        if len(self.state.agent_events) > 100:
            del self.state.agent_events[:-100]
        message = payload.get('message')
        if message:
            self.addLog(f'Agent: {message}')
        self.state.agent_status = {
            **self.state.agent_status,
            'last_tool': str(payload.get('tool_name') or self.state.agent_status.get('last_tool') or ''),
            'last_result_status': str(payload.get('status') or self.state.agent_status.get('last_result_status') or ''),
            'last_error': str(payload.get('error_code') or self.state.agent_status.get('last_error') or ''),
        }
        self.agentStatusChanged.emit()

    def update_agent_chat(self, payload: Dict[str, Any]) -> None:
        payload = dict(payload)
        key = dedupe_key(payload)
        if key in self._agent_message_keys:
            return
        self._agent_message_keys.add(key)
        self.state.agent_messages.append(payload)
        if len(self.state.agent_messages) > self.state.max_agent_messages:
            removed = self.state.agent_messages[:-self.state.max_agent_messages]
            del self.state.agent_messages[:-self.state.max_agent_messages]
            for item in removed:
                self._agent_message_keys.discard(dedupe_key(item))
        self.agentMessagesChanged.emit()

    def update_voice_session_status(self, payload: Dict[str, Any]) -> None:
        self.state.voice_session_status = payload
        self.state.system_status = dict(self.state.system_status)
        self.state.system_status['voice_status'] = self.voiceStatusSummary
        self.voiceStatusChanged.emit()

    def update_voice_service_result(self, name: str, success: bool, message: str) -> None:
        self.state.voice_service_status = f'{name}: {"ok" if success else "failed"} {message}'.strip()
        self.addLog(f'语音服务: {self.state.voice_service_status}')
        self.voiceStatusChanged.emit()

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
        self.voiceStatusChanged.emit()

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
