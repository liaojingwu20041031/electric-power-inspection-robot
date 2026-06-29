import threading
import time

from PyQt5.QtCore import QCoreApplication, QThread
from types import SimpleNamespace

from ylhb_llm.ui_backend import UiBackend
from ylhb_llm.ui_models import UiState


class FakeBridge:
    def __init__(self):
        self.system_commands = []
        self.patrol_commands = []
        self.twists = []

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

    def call_voice_service(self, _name):
        pass


def make_backend(clock):
    QCoreApplication.instance() or QCoreApplication([])
    return UiBackend(FakeBridge(), UiState(), clock=clock)


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

    def preview_loader():
        release_loader.wait(timeout=2.0)
        calls.append('loaded')
        return {
            'ok': True,
            'preview_type': 'route_overlay',
            'overlay_ok': True,
            'image_url': 'file:///tmp/preview.png',
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
    assert calls == ['loaded']
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
        route_preview_loader=lambda: {'ok': True, 'preview_type': 'route_overlay', 'overlay_ok': True, 'targets': []},
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
        route_preview_loader=lambda: calls.append('loaded') or {'ok': True, 'targets': []},
    )
    backend._route_preview_thread.join(timeout=2.0)

    backend.sendPatrolCommand('reload')
    backend._route_preview_thread.join(timeout=2.0)
    assert process_events_until(lambda: len(calls) == 2)

    assert backend.bridge.system_commands == [('reload_patrol_route', {})]
    assert backend.bridge.patrol_commands == []
    assert calls == ['loaded', 'loaded']


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
    }

    assert backend.routePreviewImageSource == ''

    backend.state.route_preview = {
        'ok': True,
        'preview_type': 'route_overlay',
        'overlay_ok': True,
        'image_url': 'file:///tmp/missing-overlay.png',
        'image_exists': False,
    }

    assert backend.routePreviewImageSource == ''


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
    backend.update_patrol_status({'state': 'waiting_nav2'})
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

    backend.on_voice_status(SimpleNamespace(speaking=True, current_task_id='voice_session'))

    assert backend.state.voice_status == '播报中 voice_session'
    assert backend.systemStatus['voice_status'] == '播报中 voice_session'
