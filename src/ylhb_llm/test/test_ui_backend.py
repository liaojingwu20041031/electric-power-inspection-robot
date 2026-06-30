import threading
import time
from pathlib import Path

from PyQt5.QtCore import QCoreApplication, QThread
from types import SimpleNamespace

from ylhb_mobile_bridge.patrol_route_store import load_route_file
from ylhb_llm.ui_backend import UiBackend
from ylhb_llm.ui_models import UiState


TEST_ROUTE_PATH = (
    Path(__file__).resolve().parents[2]
    / 'ylhb_mobile_bridge'
    / 'test'
    / 'fixtures'
    / 'patrol_routes.json'
)


class FakeBridge:
    def __init__(self):
        self.system_commands = []
        self.patrol_commands = []
        self.twists = []
        self.agent_requests = []

    def publish_system_command(self, command, **extra):
        self.system_commands.append((command, extra))

    def publish_patrol_command(self, command):
        self.patrol_commands.append(command)

    def publish_twist(self, linear=0.0, angular=0.0):
        self.twists.append((linear, angular))

    def publish_system_mode(self, _mode):
        pass

    def publish_text_command(self, _text):
        pass

    def publish_agent_request(self, text):
        self.agent_requests.append(text)

    def call_voice_service(self, _name):
        pass


def make_backend(clock):
    QCoreApplication.instance() or QCoreApplication([])
    return UiBackend(FakeBridge(), UiState(), clock=clock)


def route_loop_config():
    data = load_route_file(str(TEST_ROUTE_PATH))
    return dict(data['routes'][0]['loop'])


def process_events_until(predicate, timeout_sec=2.0):
    app = QCoreApplication.instance() or QCoreApplication([])
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        app.processEvents()
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


def test_route_preview_refresh_runs_in_background_thread():
    QCoreApplication.instance() or QCoreApplication([])
    calls = []
    release_loader = threading.Event()

    def preview_loader(force=False):
        release_loader.wait(timeout=2.0)
        calls.append(force)
        return {
            'ok': True,
            'preview_type': 'route_overlay',
            'overlay_ok': True,
            'image_url': 'file:///tmp/preview.png',
            'image_exists': True,
            'image_valid': True,
            'message': 'ok',
            'targets': [{'id': 'target_001', 'name': 'A'}],
        }

    backend = UiBackend(
        FakeBridge(),
        UiState(),
        clock=lambda: 100.0,
        route_preview_loader=preview_loader,
    )

    assert calls == []
    release_loader.set()
    backend._route_preview_thread.join(timeout=2.0)
    assert process_events_until(lambda: backend.routePreviewOk)
    assert calls == [False]
    assert backend.routePreviewOk is True
    assert 'target_001' in backend.patrolTasks


def test_route_preview_result_is_applied_on_qt_main_thread():
    QCoreApplication.instance() or QCoreApplication([])
    main_thread = QThread.currentThread()
    applied_threads = []

    backend = UiBackend(
        FakeBridge(),
        UiState(),
        clock=lambda: 100.0,
        route_preview_loader=lambda force=False: {'ok': True, 'preview_type': 'route_overlay', 'overlay_ok': True, 'image_url': 'file:///tmp/preview.png', 'image_exists': True, 'image_valid': True, 'targets': []},
    )
    backend.routePreviewChanged.connect(lambda: applied_threads.append(QThread.currentThread()))
    backend._route_preview_thread.join(timeout=2.0)

    assert process_events_until(lambda: backend.routePreviewOk)
    assert applied_threads[-1] == main_thread


def test_reload_patrol_route_command_uses_supervisor_and_refreshes_route_preview():
    QCoreApplication.instance() or QCoreApplication([])
    calls = []

    backend = UiBackend(
        FakeBridge(),
        UiState(),
        clock=lambda: 100.0,
        route_preview_loader=lambda force=False: calls.append(force) or {'ok': True, 'targets': []},
    )
    backend._route_preview_thread.join(timeout=2.0)

    backend.sendPatrolCommand('reload')
    backend._route_preview_thread.join(timeout=2.0)
    assert process_events_until(lambda: len(calls) == 2)

    assert backend.bridge.system_commands == [('reload_patrol_route', {})]
    assert backend.bridge.patrol_commands == []
    assert calls == [False, True]


def test_patrol_commands_use_supervisor_and_are_debounced():
    now = [100.0]
    backend = make_backend(lambda: now[0])

    backend.sendPatrolCommand('pause')
    backend.sendPatrolCommand('pause')
    now[0] = 100.81
    backend.sendPatrolCommand('pause')

    assert backend.bridge.system_commands == [
        ('pause_patrol', {}),
        ('pause_patrol', {}),
    ]
    assert backend.bridge.patrol_commands == []


def test_direct_patrol_system_commands_are_debounced():
    now = [100.0]
    backend = make_backend(lambda: now[0])

    backend.sendSystemCommand('pause_patrol')
    backend.sendSystemCommand('pause_patrol')
    now[0] = 100.81
    backend.sendSystemCommand('pause_patrol')
    backend.sendSystemCommand('resume_patrol')
    backend.sendSystemCommand('resume_patrol')

    assert backend.bridge.system_commands == [
        ('pause_patrol', {}),
        ('pause_patrol', {}),
        ('resume_patrol', {}),
    ]
    assert backend.bridge.patrol_commands == []


def test_direct_stop_robot_stack_command_uses_long_debounce():
    now = [100.0]
    backend = make_backend(lambda: now[0])

    backend.sendSystemCommand('stop_robot_stack')
    now[0] = 104.9
    backend.sendSystemCommand('stop_robot_stack')
    now[0] = 105.01
    backend.sendSystemCommand('stop_robot_stack')

    assert backend.bridge.system_commands == [
        ('stop_robot_stack', {}),
        ('stop_robot_stack', {}),
    ]


def test_start_patrol_mode_sends_system_command_to_supervisor():
    backend = make_backend(lambda: 100.0)
    backend.setPatrolStartProfile('inspection')

    backend.startPatrolMode()

    assert backend.bridge.system_commands == [('start_patrol_mode', {'profile': 'inspection'})]
    assert backend.bridge.patrol_commands == []


def test_patrol_readiness_properties_follow_system_status():
    backend = make_backend(lambda: 100.0)

    backend.update_system_status({
        'patrol_mode_state': 'starting',
        'startup_step': 'waiting_nav2_action',
        'patrol_error': '等待巡逻依赖: nav2_action',
        'patrol_readiness': {
            'bringup': True,
            'navigation': True,
            'executor': True,
            'route_file': True,
            'nav2_action': False,
        },
    })

    assert backend.patrolModeState == 'starting'
    assert backend.patrolStartupStep == 'waiting_nav2_action'
    assert backend.patrolError == '等待巡逻依赖: nav2_action'
    assert backend.patrolReady is True
    assert backend.patrolControlsEnabled is True


def test_patrol_display_properties_follow_starting_active_and_terminal_states():
    backend = make_backend(lambda: 100.0)

    backend.update_system_status({
        'patrol_mode_state': 'starting',
        'startup_step': 'waiting_executor_response',
        'startup_step_label': '等待巡逻执行器响应',
        'patrol_readiness': {},
    })
    assert backend.patrolStarting is True
    assert backend.patrolActive is False
    assert backend.patrolCanStart is False
    assert backend.patrolStateLabel == '启动中: 等待巡逻执行器响应'

    backend.update_system_status({
        'patrol_mode_state': 'running',
        'patrol_executor': 'running',
        'patrol_readiness': {},
    })
    backend.update_patrol_status({'state': 'running', 'target_index': 0, 'target_count': 2})
    assert backend.patrolStarting is False
    assert backend.patrolActive is True
    assert backend.patrolCanStart is False
    assert backend.patrolCanPause is True
    assert backend.patrolCanResume is False
    assert backend.patrolCanCancel is True
    assert '1 / 2' in backend.patrolStateLabel

    backend.update_system_status({'patrol_mode_state': 'failed', 'patrol_readiness': {}})
    backend.update_patrol_status({'state': 'failed'})
    assert backend.patrolActive is False
    assert backend.patrolCanStart is True
    assert backend.patrolStateLabel == '失败'

    backend.update_system_status({'patrol_mode_state': 'succeeded', 'patrol_readiness': {}})
    backend.update_patrol_status({'state': 'succeeded'})
    assert backend.patrolStateLabel == '已完成'

    backend.update_system_status({'patrol_mode_state': 'canceled', 'patrol_readiness': {}})
    backend.update_patrol_status({'state': 'canceled'})
    assert backend.patrolStateLabel == '已取消'


def test_patrol_display_properties_do_not_publish_commands():
    backend = make_backend(lambda: 100.0)
    backend.update_system_status({
        'patrol_mode_state': 'command_sent',
        'startup_step': 'patrol_start_sent',
        'startup_step_label': '巡逻启动命令已发送',
    })
    backend.update_patrol_status({'state': 'paused'})

    _ = (
        backend.patrolStarting,
        backend.patrolActive,
        backend.patrolCanStart,
        backend.patrolCanPause,
        backend.patrolCanResume,
        backend.patrolCanCancel,
        backend.patrolStateLabel,
    )

    assert backend.bridge.system_commands == []
    assert backend.bridge.patrol_commands == []


def test_stale_patrol_status_does_not_block_restart_after_executor_stops():
    backend = make_backend(lambda: 100.0)
    backend.update_system_status({
        'patrol_mode_state': 'idle',
        'patrol_executor': 'stopped',
        'patrol_readiness': {},
    })
    backend.update_patrol_status({'state': 'paused'})

    assert backend.patrolCanStart is True
    assert backend.patrolActive is False
    assert backend.patrolCanResume is False
    assert backend.patrolCanCancel is False


def test_patrol_controls_enabled_when_executor_running_or_status_seen():
    backend = make_backend(lambda: 100.0)

    backend.update_system_status({
        'patrol_executor': 'running',
        'patrol_readiness': {'executor': False},
    })

    assert backend.patrolControlsEnabled is True

    backend.update_system_status({
        'patrol_executor': 'stopped',
        'patrol_readiness': {'executor': False},
    })
    backend.update_patrol_status({'state': 'unavailable'})

    assert backend.patrolControlsEnabled is True


def test_ui_state_keeps_only_latest_200_events():
    state = UiState()
    for index in range(205):
        state.add_event(f'event-{index}', timestamp=f't-{index}')

    assert len(state.events) == 200
    assert state.events[0]['message'] == 'event-5'
    assert state.events[-1]['message'] == 'event-204'


def test_motion_is_blocked_until_control_is_unlocked():
    now = [100.0]
    backend = make_backend(lambda: now[0])

    backend.moveForward()
    assert backend.bridge.twists == []

    backend.setControlUnlocked(True)
    backend.moveForward()
    assert backend.bridge.twists == [(0.12, 0.0)]


def test_control_auto_locks_after_ten_seconds_idle():
    now = [100.0]
    backend = make_backend(lambda: now[0])
    backend.setControlUnlocked(True)

    now[0] = 111.0
    backend.checkSafetyTimeout()

    assert backend.controlUnlocked is False


def test_emergency_stop_is_always_published_and_locks_control():
    now = [100.0]
    backend = make_backend(lambda: now[0])
    backend.setControlUnlocked(True)

    backend.emergencyStop()

    assert backend.bridge.system_commands[-1][0] == 'emergency_stop'
    assert backend.bridge.twists[-1] == (0.0, 0.0)
    assert backend.controlUnlocked is False


def test_repeated_system_status_message_is_not_logged_twice():
    backend = make_backend(lambda: 100.0)

    backend.update_system_status({'last_command': 'start_mobile_bridge', 'message': 'ok'})
    backend.update_system_status({'last_command': 'start_mobile_bridge', 'message': 'ok'})

    assert [event['message'] for event in backend.logs] == ['ok']


def test_status_values_are_localized_for_qml():
    backend = make_backend(lambda: 100.0)

    assert backend.localizedStatus('running') == '运行中'
    assert backend.localizedStatus('http_ok') == '连接正常'
    assert backend.localizedStatus('embedded') == '内嵌运行'
    assert backend.localizedStatus('waiting_nav2') == '等待导航服务'
    assert backend.localizedStatus('sending_goal') == '发送导航目标'
    assert backend.localizedStatus('retrying_goal') == '导航目标重试'
    assert backend.localizedStatus('target') == '前往检查点'
    assert backend.localizedStatus('return_home') == '返回初始点'
    assert backend.localizedStatus('command_sent') == '等待巡逻执行器进入运行状态'
    assert backend.localizedStatus('waiting_after_bringup') == '底盘启动后等待'
    assert backend.localizedStatus('waiting_after_navigation') == '导航启动后等待'
    assert backend.localizedStatus('waiting_after_executor') == '巡逻执行器启动后等待'
    assert backend.localizedStatus('patrol_start_sent') == '巡逻启动命令已发送'


def test_patrol_status_and_event_update_ui_state():
    backend = make_backend(lambda: 100.0)

    backend.update_patrol_status({'state': 'running', 'active_route_id': 'route_patrol_001'})
    backend.update_patrol_event({'event': 'arrived', 'target_id': 'target_001'})

    assert backend.patrolStatus['state'] == 'running'
    assert backend.patrolStatusText == '运行中'
    assert backend.patrolEvents[-1]['event'] == 'arrived'


def test_route_preview_direct_properties_are_exposed_for_qml():
    backend = make_backend(lambda: 100.0)
    backend.state.route_preview = {
        'ok': True,
        'preview_type': 'route_overlay',
        'overlay_ok': True,
        'image_url': 'file:///tmp/preview.png',
        'image_exists': True,
        'image_valid': True,
        'image_mtime_ns': 12345,
        'message': 'ok',
    }

    assert backend.routePreviewOk is True
    assert backend.routePreviewImageUrl == 'file:///tmp/preview.png'
    assert backend.routePreviewImageSource == 'file:///tmp/preview.png'
    assert backend.routePreviewMessage == 'ok'


def test_route_preview_image_source_is_empty_for_non_overlay_or_missing_file():
    backend = make_backend(lambda: 100.0)
    backend.state.route_preview = {
        'ok': True,
        'preview_type': 'raw_map',
        'overlay_ok': False,
        'image_url': 'file:///tmp/raw-map.png',
        'image_exists': True,
        'image_valid': True,
    }

    assert backend.routePreviewImageSource == ''

    backend.state.route_preview = {
        'ok': True,
        'preview_type': 'route_overlay',
        'overlay_ok': True,
        'image_url': 'file:///tmp/missing-overlay.png',
        'image_exists': False,
        'image_valid': True,
    }

    assert backend.routePreviewImageSource == ''

    backend.state.route_preview = {
        'ok': True,
        'preview_type': 'route_overlay',
        'overlay_ok': True,
        'image_url': 'file:///tmp/bad-overlay.png',
        'image_exists': True,
        'image_valid': False,
        'image_error': 'bad png',
        'message': 'ok',
    }

    assert backend.routePreviewOk is False
    assert backend.routePreviewImageSource == ''
    assert backend.routePreviewMessage == 'bad png'


def test_route_preview_ok_requires_route_overlay():
    backend = make_backend(lambda: 100.0)
    backend.state.route_preview = {
        'ok': True,
        'preview_type': 'raw_map',
        'overlay_ok': False,
        'image_url': 'file:///tmp/raw-map.png',
    }

    assert backend.routePreviewOk is False


def test_start_patrol_mode_is_blocked_while_executor_waits_or_runs():
    backend = make_backend(lambda: 100.0)
    backend.update_patrol_status({'state': 'running', 'navigation_phase': 'waiting_nav2'})
    backend.startPatrolMode()

    assert backend.bridge.system_commands == []
    assert any('巡逻执行器已在工作' in event['message'] for event in backend.logs)


def test_start_patrol_mode_allows_terminal_and_unavailable_states():
    for state in ('failed', 'canceled', 'succeeded', 'unavailable'):
        backend = make_backend(lambda: 100.0)
        backend.update_patrol_status({'state': state})
        backend.startPatrolMode()

        assert backend.bridge.system_commands == [('start_patrol_mode', {'profile': 'navigation'})]


def test_patrol_status_progress_label_is_exposed_for_qml():
    backend = make_backend(lambda: 100.0)

    backend.update_patrol_status({
        'state': 'running',
        'target_index': 0,
        'target_count': 4,
        'target_name': '巡检点1',
        'cycle_index': 1,
    })

    assert backend.patrolProgressLabel == '第 1 / 4 个检查点'
    assert backend.currentTargetLabel == '巡检点1'

    backend.update_patrol_status({
        'state': 'running',
        'target_index': 0,
        'target_count': 4,
        'target_name': '巡检点1',
        'current_target_label': '巡检点1: 红外测温',
    })
    assert backend.patrolProgressLabel == '巡检点1: 红外测温'


def test_patrol_status_special_phases_are_exposed_for_qml():
    backend = make_backend(lambda: 100.0)

    backend.update_patrol_status({
        'state': 'returning_home',
        'navigation_phase': 'return_home',
        'current_target_label': '返回初始点',
        'target_count': 4,
        'cycle_index': 1,
    })
    assert backend.patrolProgressLabel == '返回初始点'
    assert backend.currentTargetLabel == '返回初始点'

    backend.update_patrol_status({
        'state': 'waiting_loop',
        'navigation_phase': 'waiting_next_cycle',
        'current_target_label': '等待下一轮',
        'target_count': 4,
        'cycle_index': 1,
    })
    assert backend.patrolProgressLabel == '等待下一轮'


def test_patrol_overview_labels_format_status_from_route_values():
    backend = make_backend(lambda: 100.0)
    loop = route_loop_config()
    wait_sec = int(loop['wait_sec'])

    backend.update_patrol_status({
        'state': 'waiting_loop',
        'target_index': 0,
        'target_count': 4,
        'cycle_index': 2,
        'next_cycle_index': 3,
        'loop_enabled': True,
        'loop_max_cycles': int(loop['max_cycles']),
        'loop_is_infinite': int(loop['max_cycles']) == 0,
        'loop_wait_remaining_sec': wait_sec,
        'current_target_label': '等待下一轮',
    })

    assert backend.patrolMainStatusLabel == '等待下一轮'
    assert backend.patrolCycleLabel == '第 2 轮 / 无限循环'
    assert f'00:{wait_sec:02d}' in backend.patrolNextCycleLabel
    assert '下一轮' in backend.patrolNextCycleLabel
    assert backend.patrolOverviewProgressLabel == '等待下一轮'


def test_patrol_overview_labels_do_not_publish_commands():
    backend = make_backend(lambda: 100.0)
    loop = route_loop_config()
    backend.update_patrol_status({
        'state': 'waiting_loop',
        'cycle_index': 1,
        'loop_enabled': True,
        'loop_is_infinite': int(loop['max_cycles']) == 0,
        'loop_wait_remaining_sec': int(loop['wait_sec']),
    })

    _ = (
        backend.patrolMainStatusLabel,
        backend.patrolCycleLabel,
        backend.patrolNextCycleLabel,
        backend.patrolOverviewProgressLabel,
    )

    assert backend.bridge.system_commands == []
    assert backend.bridge.patrol_commands == []


def test_missing_patrol_status_explains_executor_unavailable():
    backend = make_backend(lambda: 100.0)

    backend.update_patrol_status({})

    assert backend.patrolStatus['state'] == 'unavailable'
    assert '巡逻执行器未运行' in backend.patrolStatus['message']


def test_patrol_events_keep_latest_100_entries():
    backend = make_backend(lambda: 100.0)

    for index in range(105):
        backend.update_patrol_event({'event': f'e-{index}'})

    assert len(backend.patrolEvents) == 100
    assert backend.patrolEvents[0]['event'] == 'e-5'
    assert backend.patrolEvents[-1]['event'] == 'e-104'


def test_voice_status_uses_actual_ros_message_fields():
    backend = make_backend(lambda: 100.0)
    signals = []
    backend.voiceStatusChanged.connect(lambda: signals.append('voice'))

    backend.on_voice_status(SimpleNamespace(speaking=True, current_task_id='voice_session'))

    assert backend.state.voice_status == '播报中 voice_session'
    assert backend.systemStatus['voice_status'] == '播报中 voice_session'
    assert signals == ['voice']


def test_voice_session_status_updates_derived_properties():
    backend = make_backend(lambda: 100.0)
    signals = []
    backend.systemStatusChanged.connect(lambda: signals.append('system'))
    backend.voiceStatusChanged.connect(lambda: signals.append('voice'))

    backend.update_voice_session_status({
        'enabled': True,
        'state': 'WAIT_WAKE',
        'wake_phrase': '小零小零',
        'last_asr_text': '开始巡检',
        'last_published_text': '开始巡检',
        'asr_fail_count': 1,
        'waiting_for': 'wake_phrase',
    })

    assert backend.voiceSessionEnabled is True
    assert backend.voiceSessionStateText == '待唤醒'
    assert backend.voiceWakePhrase == '小零小零'
    assert backend.voiceLastAsrText == '开始巡检'
    assert backend.voiceAsrFailCount == 1
    assert '待唤醒' in backend.voiceStatusSummary
    assert backend.voiceActivityTone == 'wake'
    assert '等待唤醒词' in backend.voiceActivityText
    assert signals == ['voice']

    backend.update_voice_session_status({'enabled': True, 'state': 'RECORDING'})
    assert backend.voiceActivityTone == 'active'
    assert backend.voiceActivityText == '正在录音'

    backend.update_voice_session_status({'enabled': True, 'state': 'ASR_PROCESSING'})
    assert backend.voiceActivityTone == 'busy'
    assert backend.voiceActivityText == '正在识别'


def test_system_status_update_still_uses_system_signal():
    backend = make_backend(lambda: 100.0)
    signals = []
    backend.systemStatusChanged.connect(lambda: signals.append('system'))
    backend.voiceStatusChanged.connect(lambda: signals.append('voice'))

    backend.update_system_status({'mobile_bridge_http': 'http_ok'})

    assert backend.systemStatus['mobile_bridge_http'] == 'http_ok'
    assert signals == ['system']


def test_send_agent_text_publishes_agent_request():
    backend = make_backend(lambda: 100.0)

    backend.sendAgentText(' 开始巡检 ')

    assert backend.bridge.agent_requests == ['开始巡检']


def test_agent_event_updates_recent_tool_result_and_message():
    backend = make_backend(lambda: 100.0)

    backend.update_agent_event({'tool_name': 'start_patrol_mode', 'status': 'sent', 'message': '已发送'})

    assert backend.agentLastTool == 'start_patrol_mode'
    assert backend.agentLastResult == 'sent'
    assert backend.agentEvents[-1]['message'] == '已发送'


def test_voice_service_result_updates_status_and_log():
    backend = make_backend(lambda: 100.0)

    backend.update_voice_service_result('start', True, '语音模式已开启。')

    assert 'start: ok' in backend.voiceServiceStatus
    assert any('语音服务' in event['message'] for event in backend.logs)
