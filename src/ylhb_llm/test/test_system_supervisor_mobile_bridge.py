import json
import threading
from pathlib import Path
from unittest.mock import Mock

import pytest
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from ylhb_mobile_bridge.patrol_qos import patrol_status_qos_profile

from ylhb_llm import system_supervisor_node
from ylhb_llm.system_supervisor_node import SystemSupervisorNode


class FakeProcess:
    def __init__(self, running=False):
        self._running = running

    def is_running(self):
        return self._running


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def allow_patrol_start_gates(node):
    node.wait_for_map_to_odom = Mock(return_value=True)
    node.wait_for_nav2_active_ready = Mock(return_value=True)
    node.wait_for_keepout_active_ready = Mock(return_value=True)
    node.log_patrol_start_readiness = Mock()


@pytest.fixture(autouse=True)
def skip_supervisor_sleeps(monkeypatch):
    monkeypatch.setattr(system_supervisor_node.time, 'sleep', lambda _sec: None)


def test_mobile_bridge_commands_start_stop_and_restart():
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.processes = {'mobile_bridge': FakeProcess()}
    node.start_process = Mock()
    node.stop_process = Mock()
    node.set_result = Mock()

    node.handle_command('start_mobile_bridge', {})
    node.start_process.assert_called_once_with('mobile_bridge')

    node.handle_command('stop_mobile_bridge', {})
    node.stop_process.assert_called_once_with('mobile_bridge')

    node.handle_command('restart_mobile_bridge', {})
    assert node.stop_process.call_args_list[-1].args == ('mobile_bridge',)
    assert node.start_process.call_args_list[-1].args == ('mobile_bridge',)


def test_start_patrol_mode_navigation_profile_starts_dependencies_in_ready_order(monkeypatch):
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.processes = {
        'bringup': FakeProcess(),
        'zed': FakeProcess(),
        'perception': FakeProcess(),
        'navigation': FakeProcess(),
        'llm': FakeProcess(),
        'patrol_executor': FakeProcess(),
    }
    node.start_process = Mock()
    node.publish_patrol_command = Mock()
    node.wait_for_patrol_command_subscriber = Mock(return_value=True)
    node.wait_for_patrol_status_heartbeat = Mock(return_value=True)
    node.last_patrol_status = {'state': 'idle'}
    node.last_patrol_status_received_at = 100.0
    node.set_result = Mock()
    node.log_info = Mock()
    sequence = []
    node.wait_for_core_sensors = Mock()
    node.wait_for_navigation_ready = Mock(return_value=True)
    node.wait_for_patrol_executor_ready = Mock()
    node.wait_for_initial_pose_published = Mock(return_value=True)
    node.wait_for_nav2_action_ready = Mock()
    node.wait_for_patrol_status = Mock()
    allow_patrol_start_gates(node)

    def fake_start_process(name):
        sequence.append(f'start_{name}')

    def fake_wait_navigation_ready(_timeout):
        sequence.append('wait_navigation_ready')
        return True

    def fake_wait_initial_pose(_timeout):
        sequence.append('wait_initial_pose')
        return True

    def fake_wait_map_to_odom(_timeout):
        sequence.append('wait_map_to_odom')
        return True

    def fake_wait_nav2_active(_timeout):
        sequence.append('wait_nav2_active')
        return True

    node.start_process.side_effect = fake_start_process
    node.wait_for_navigation_ready.side_effect = fake_wait_navigation_ready
    node.wait_for_initial_pose_published.side_effect = fake_wait_initial_pose
    node.wait_for_map_to_odom.side_effect = fake_wait_map_to_odom
    node.wait_for_nav2_active_ready.side_effect = fake_wait_nav2_active
    node.log_patrol_start_readiness.side_effect = lambda: sequence.append('log_start_readiness')
    node.publish_patrol_command.side_effect = lambda command, request_id='': sequence.append(
        f'publish_{command}'
    )
    sleeps = []

    def fake_sleep(sec):
        sleeps.append(sec)
        sequence.append(f'sleep_{sec:g}')

    monkeypatch.setattr(system_supervisor_node.time, 'time', lambda: 100.0)
    monkeypatch.setattr(system_supervisor_node.time, 'sleep', fake_sleep)

    node.handle_command('start_patrol_mode', {'profile': 'navigation'})

    assert sequence == [
        'start_bringup',
        'sleep_3',
        'start_navigation',
        'sleep_20',
        'wait_navigation_ready',
        'start_patrol_executor',
        'sleep_6',
        'wait_initial_pose',
        'wait_map_to_odom',
        'wait_nav2_active',
        'log_start_readiness',
        'publish_start',
        'sleep_5',
        'publish_start',
    ]
    node.wait_for_core_sensors.assert_not_called()
    node.wait_for_navigation_ready.assert_called_once_with(35.0)
    node.wait_for_patrol_executor_ready.assert_not_called()
    node.wait_for_initial_pose_published.assert_called_once_with(8.0)
    node.wait_for_map_to_odom.assert_called_once_with(10.0)
    node.wait_for_nav2_active_ready.assert_called_once_with(25.0)
    node.wait_for_nav2_action_ready.assert_not_called()
    node.wait_for_patrol_status.assert_not_called()
    node.wait_for_patrol_status_heartbeat.assert_called_once_with(100.0, 8.0)
    node.wait_for_patrol_command_subscriber.assert_called_once_with(5.0)
    assert node.publish_patrol_command.call_args_list[-1].args[0] == 'start'
    assert node.patrol_mode_state == 'command_sent'
    node.set_result.assert_called_with(
        'start_patrol_mode',
        True,
        '已按手动流程发送巡逻启动命令',
    )


def test_start_patrol_mode_inspection_profile_starts_perception_stack():
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.processes = {}
    node.start_process = Mock()
    node.publish_patrol_command = Mock()
    node.wait_for_patrol_command_subscriber = Mock(return_value=True)
    node.wait_for_patrol_status_heartbeat = Mock(return_value=True)
    node.wait_for_navigation_ready = Mock(return_value=True)
    node.wait_for_initial_pose_published = Mock(return_value=True)
    node.last_patrol_status = {'state': 'idle'}
    node.last_patrol_status_received_at = 100.0
    node.set_result = Mock()
    node.log_info = Mock()
    node.wait_for_core_sensors = Mock(return_value=True)
    node.wait_for_navigation_ready = Mock(return_value=True)
    node.wait_for_patrol_executor_ready = Mock(return_value=True)
    node.wait_for_initial_pose_published = Mock(return_value=True)
    node.wait_for_nav2_action_ready = Mock(return_value=True)
    node.wait_for_patrol_status = Mock(return_value='running')
    allow_patrol_start_gates(node)

    node.handle_command('start_patrol_mode', {'profile': 'inspection'})

    assert [call.args[0] for call in node.start_process.call_args_list] == [
        'bringup',
        'zed',
        'perception',
        'navigation',
        'patrol_executor',
    ]
    assert node.publish_patrol_command.call_count == 2


def test_start_patrol_mode_gates_start_on_nav2_lifecycle_not_action_server():
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.start_process = Mock()
    node.publish_patrol_command = Mock()
    node.wait_for_patrol_command_subscriber = Mock(return_value=True)
    node.wait_for_patrol_status_heartbeat = Mock(return_value=True)
    node.last_patrol_status = {'state': 'idle'}
    node.last_patrol_status_received_at = 100.0
    node.set_result = Mock()
    node.log_info = Mock()
    node.wait_for_core_sensors = Mock(return_value=True)
    node.wait_for_navigation_ready = Mock(return_value=True)
    node.wait_for_patrol_executor_ready = Mock(return_value=True)
    node.wait_for_initial_pose_published = Mock(return_value=True)
    node.wait_for_nav2_action_ready = Mock(return_value=False)
    allow_patrol_start_gates(node)

    node.handle_command('start_patrol_mode', {})

    assert [call.args[0] for call in node.start_process.call_args_list] == [
        'bringup',
        'navigation',
        'patrol_executor',
    ]
    node.wait_for_initial_pose_published.assert_called_once_with(8.0)
    node.wait_for_map_to_odom.assert_called_once_with(10.0)
    node.wait_for_nav2_active_ready.assert_called_once_with(25.0)
    node.wait_for_nav2_action_ready.assert_not_called()
    assert node.publish_patrol_command.call_args_list[-1].args[0] == 'start'
    node.set_result.assert_called_with(
        'start_patrol_mode',
        True,
        '已按手动流程发送巡逻启动命令',
    )


def test_navigation_command_adds_keepout_only_when_enabled():
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.default_navigation_map = '/ws/maps/my_map.yaml'
    node.keepout_mask_path = '/ws/maps/keepout/keepout_mask_power_room_a.yaml'
    node.enable_keepout_navigation = False

    assert node.navigation_launch_command() == 'ros2 launch ylhb_base navigation.launch.py map:=/ws/maps/my_map.yaml'

    node.enable_keepout_navigation = True
    assert node.navigation_launch_command() == (
        'ros2 launch ylhb_base navigation.launch.py map:=/ws/maps/my_map.yaml '
        'enable_keepout:=true keepout_mask:=/ws/maps/keepout/keepout_mask_power_room_a.yaml'
    )


def test_start_patrol_keepout_generation_failure_blocks_navigation_start():
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.enable_keepout_navigation = True
    node.patrol_error = ''
    node.start_process = Mock()
    node.publish_patrol_command = Mock()
    node.generate_keepout_mask = Mock(return_value=False)
    node.set_result = Mock()
    node.log_info = Mock()

    def fake_prepare():
        node.patrol_error = 'keepout mask generation failed: bad route'
        return False

    node.prepare_keepout_navigation = Mock(side_effect=fake_prepare)

    node.handle_command('start_patrol_mode', {})

    assert [call.args[0] for call in node.start_process.call_args_list] == ['bringup']
    node.publish_patrol_command.assert_not_called()
    node.set_result.assert_called_with(
        'start_patrol_mode',
        False,
        '巡逻启动失败: keepout mask generation failed: bad route',
    )


def test_start_patrol_keepout_lifecycle_failure_blocks_start_command():
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.enable_keepout_navigation = True
    node.start_process = Mock()
    node.publish_patrol_command = Mock()
    node.prepare_keepout_navigation = Mock(return_value=True)
    node.wait_for_patrol_command_subscriber = Mock(return_value=True)
    node.wait_for_patrol_status_heartbeat = Mock(return_value=True)
    node.wait_for_navigation_ready = Mock(return_value=True)
    node.wait_for_initial_pose_published = Mock(return_value=True)
    node.wait_for_map_to_odom = Mock(return_value=True)
    node.wait_for_nav2_active_ready = Mock(return_value=True)
    node.wait_for_keepout_active_ready = Mock(return_value=False)
    node.log_patrol_start_readiness = Mock()
    node.last_patrol_status = {'state': 'idle'}
    node.set_result = Mock()
    node.log_info = Mock()

    node.handle_command('start_patrol_mode', {})

    node.publish_patrol_command.assert_not_called()
    node.wait_for_keepout_active_ready.assert_called_once_with(12.0)
    node.set_result.assert_called_with(
        'start_patrol_mode',
        False,
        '巡逻启动门控失败: 未确认 Keepout lifecycle active',
    )


def test_start_patrol_mode_does_not_publish_start_before_nav2_active():
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.start_process = Mock()
    node.publish_patrol_command = Mock()
    node.wait_for_patrol_command_subscriber = Mock(return_value=True)
    node.wait_for_patrol_status_heartbeat = Mock(return_value=True)
    node.last_patrol_status = {'state': 'idle'}
    node.last_patrol_status_received_at = 100.0
    node.set_result = Mock()
    node.log_info = Mock()
    node.wait_for_core_sensors = Mock(return_value=True)
    node.wait_for_navigation_ready = Mock(return_value=True)
    node.wait_for_patrol_executor_ready = Mock(return_value=True)
    node.wait_for_initial_pose_published = Mock(return_value=True)
    node.wait_for_map_to_odom = Mock(return_value=True)
    node.wait_for_nav2_active_ready = Mock(return_value=False)
    node.log_patrol_start_readiness = Mock()
    node.wait_for_nav2_action_ready = Mock(return_value=True)

    node.handle_command('start_patrol_mode', {})

    node.wait_for_nav2_action_ready.assert_not_called()
    node.publish_patrol_command.assert_not_called()
    node.set_result.assert_called_with(
        'start_patrol_mode',
        False,
        '巡逻启动门控失败: 未确认 Nav2 lifecycle active',
    )


def test_start_patrol_mode_does_not_wait_for_patrol_status_but_forwards_start():
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.start_process = Mock()
    node.publish_patrol_command = Mock()
    node.wait_for_patrol_command_subscriber = Mock(return_value=True)
    node.wait_for_patrol_status_heartbeat = Mock(return_value=True)
    node.last_patrol_status = {'state': 'idle'}
    node.last_patrol_status_received_at = 100.0
    node.set_result = Mock()
    node.log_info = Mock()
    node.wait_for_core_sensors = Mock(return_value=True)
    node.wait_for_navigation_ready = Mock(return_value=True)
    node.wait_for_patrol_executor_ready = Mock(return_value=True)
    node.wait_for_initial_pose_published = Mock(return_value=True)
    node.wait_for_nav2_action_ready = Mock(return_value=True)
    node.wait_for_patrol_status = Mock(return_value='failed')
    allow_patrol_start_gates(node)

    node.handle_command('start_patrol_mode', {})

    node.wait_for_patrol_status.assert_not_called()
    assert node.publish_patrol_command.call_args_list[-1].args[0] == 'start'
    node.set_result.assert_called_with(
        'start_patrol_mode',
        True,
        '已按手动流程发送巡逻启动命令',
    )


def test_patrol_control_commands_are_forwarded_by_system_supervisor():
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.publish_patrol_command = Mock()
    node.set_result = Mock()
    node.is_patrol_executor_ready = Mock(return_value=False)

    node.handle_command('pause_patrol', {})
    node.handle_command('resume_patrol', {})
    node.handle_command('cancel_patrol', {})
    node.handle_command('reload_patrol_route', {})

    assert [call.args[0] for call in node.publish_patrol_command.call_args_list] == [
        'pause',
        'resume',
        'cancel',
        'reload',
    ]
    assert node.set_result.call_count == 4


def test_duplicate_inflight_long_command_does_not_start_second_handler(monkeypatch):
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.lock = threading.Lock()
    node.inflight_commands = set()
    node.handle_command = Mock()
    node.set_result = Mock()
    threads = []

    class FakeThread:
        def __init__(self, target, args, daemon):
            self.target = target
            self.args = args
            self.daemon = daemon
            threads.append(self)

        def start(self):
            pass

    monkeypatch.setattr(system_supervisor_node.threading, 'Thread', FakeThread)
    msg = String()
    msg.data = json.dumps({'command': 'stop_robot_stack'})

    node.command_callback(msg)
    node.command_callback(msg)

    assert len(threads) == 1
    node.handle_command.assert_not_called()
    node.set_result.assert_called_once_with('stop_robot_stack', True, '重复命令已忽略，命令正在执行')

    threads[0].target(*threads[0].args)
    node.handle_command.assert_called_once_with('stop_robot_stack', {'command': 'stop_robot_stack'})
    assert node.inflight_commands == set()


def test_cancel_patrol_only_forwards_cancel_without_stopping_processes():
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.publish_patrol_command = Mock()
    node.stop_process = Mock()
    node.set_result = Mock()

    node.handle_command('cancel_patrol', {})

    node.publish_patrol_command.assert_called_once_with('cancel')
    node.stop_process.assert_not_called()
    node.set_result.assert_called_with('cancel_patrol', True, '已发送巡逻命令: cancel')


def test_start_patrol_reuses_running_processes_and_republishes_start():
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.processes = {
        'bringup': FakeProcess(running=True),
        'navigation': FakeProcess(running=True),
        'patrol_executor': FakeProcess(running=True),
    }
    node.start_process = Mock()
    node.publish_patrol_command = Mock()
    node.wait_for_patrol_command_subscriber = Mock(return_value=True)
    node.wait_for_patrol_status_heartbeat = Mock(return_value=True)
    node.wait_for_navigation_ready = Mock(return_value=True)
    node.wait_for_initial_pose_published = Mock(return_value=True)
    node.last_patrol_status = {'state': 'idle'}
    node.last_patrol_status_received_at = 100.0
    node.set_result = Mock()
    node.log_info = Mock()
    allow_patrol_start_gates(node)

    node.handle_command('start_patrol_mode', {})

    assert [call.args[0] for call in node.start_process.call_args_list] == [
        'bringup',
        'navigation',
        'patrol_executor',
    ]
    assert node.publish_patrol_command.call_count == 2


def test_start_patrol_mode_warns_without_subscriber_or_heartbeat_and_never_sets_running():
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.start_process = Mock()
    node.publish_patrol_command = Mock()
    node.wait_for_patrol_command_subscriber = Mock(return_value=False)
    node.wait_for_patrol_status_heartbeat = Mock(return_value=False)
    node.wait_for_navigation_ready = Mock(return_value=True)
    node.wait_for_initial_pose_published = Mock(return_value=True)
    node.last_patrol_status = {}
    node.set_result = Mock()
    node.log_info = Mock()
    allow_patrol_start_gates(node)

    node.handle_command('start_patrol_mode', {})

    assert node.patrol_mode_state == 'command_sent'
    assert 'warning' in node.patrol_error
    assert node.publish_patrol_command.call_count == 1
    node.set_result.assert_called_once()
    assert node.set_result.call_args.args[1] is True


def test_patrol_status_callback_is_business_state_source():
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.patrol_mode_state = 'command_sent'
    node.startup_step = 'patrol_start_sent'
    node.patrol_error = 'warning: waiting'

    msg = type('Msg', (), {'data': json.dumps({'state': 'idle'})})()
    node.patrol_status_callback(msg)
    assert node.patrol_mode_state == 'command_sent'
    assert node.startup_step == 'waiting_executor_response'

    msg.data = json.dumps({'state': 'running', 'navigation_phase': 'waiting_nav2'})
    node.patrol_status_callback(msg)
    assert node.patrol_mode_state == 'running'
    assert node.startup_step == 'waiting_nav2'
    assert node.patrol_error == ''

    msg.data = json.dumps({'state': 'running', 'navigation_phase': 'sending_goal'})
    node.patrol_status_callback(msg)
    assert node.startup_step == 'sending_goal'

    msg.data = json.dumps({'state': 'running', 'navigation_phase': 'retrying_goal'})
    node.patrol_status_callback(msg)
    assert node.startup_step == 'retrying_goal'

    msg.data = json.dumps({'state': 'running', 'navigation_phase': 'target'})
    node.patrol_status_callback(msg)
    assert node.patrol_mode_state == 'running'
    assert node.startup_step == 'patrol_started'
    assert node.patrol_error == ''

    msg.data = json.dumps({'state': 'running', 'navigation_phase': 'return_home'})
    node.patrol_status_callback(msg)
    assert node.startup_step == 'returning_home'

    msg.data = json.dumps({'state': 'failed'})
    node.patrol_status_callback(msg)
    assert node.patrol_mode_state == 'failed'


def test_start_patrol_republishes_once_when_executor_still_idle_or_unavailable(monkeypatch):
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.start_process = Mock()
    node.publish_patrol_command = Mock()
    node.wait_for_patrol_command_subscriber = Mock(return_value=True)
    node.wait_for_patrol_status_heartbeat = Mock(return_value=True)
    node.wait_for_navigation_ready = Mock(return_value=True)
    node.wait_for_initial_pose_published = Mock(return_value=True)
    node.last_patrol_status = {'state': 'idle'}
    node.last_patrol_status_received_at = 100.0
    node.set_result = Mock()
    node.log_info = Mock()
    allow_patrol_start_gates(node)
    monkeypatch.setattr(system_supervisor_node.time, 'time', lambda: 100.0)
    monkeypatch.setattr(system_supervisor_node.time, 'sleep', lambda _sec: None)

    node.handle_command('start_patrol_mode', {})

    assert node.publish_patrol_command.call_count == 2

    node.publish_patrol_command.reset_mock()
    node.last_patrol_status = {'state': 'unavailable'}
    node.handle_command('start_patrol_mode', {})
    assert node.publish_patrol_command.call_count == 2


def test_start_patrol_does_not_republish_for_terminal_or_active_states(monkeypatch):
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.start_process = Mock()
    node.publish_patrol_command = Mock()
    node.wait_for_patrol_command_subscriber = Mock(return_value=True)
    node.wait_for_patrol_status_heartbeat = Mock(return_value=True)
    node.wait_for_navigation_ready = Mock(return_value=True)
    node.wait_for_initial_pose_published = Mock(return_value=True)
    node.set_result = Mock()
    node.log_info = Mock()
    allow_patrol_start_gates(node)
    monkeypatch.setattr(system_supervisor_node.time, 'time', lambda: 100.0)
    monkeypatch.setattr(system_supervisor_node.time, 'sleep', lambda _sec: None)

    no_retry_states = (
        'running',
        'waiting_nav2',
        'waiting_localization',
        'returning_home',
        'paused',
        'waiting_loop',
        'canceling',
        'failed',
        'succeeded',
        'canceled',
        'cancelled',
    )
    for state in no_retry_states:
        node.publish_patrol_command.reset_mock()
        node.patrol_mode_state = 'idle'
        node.last_patrol_status = {'state': state, 'navigation_phase': 'waiting_nav2'}
        node.handle_command('start_patrol_mode', {})
        assert node.publish_patrol_command.call_count == 1
        assert node.patrol_mode_state != 'running'


def test_publish_patrol_command_sends_json_to_patrol_command_topic():
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.patrol_command_pub = FakePublisher()
    node.last_patrol_start_request_id = ''

    node.publish_patrol_command('start')

    payload = json.loads(node.patrol_command_pub.messages[-1].data)
    assert payload['command'] == 'start'
    assert payload['schema_version'] == '1.0'
    assert payload['source'] == 'system_supervisor'
    assert isinstance(payload['timestamp'], float)
    assert payload['request_id'].startswith('patrol_start_')
    assert node.last_patrol_start_request_id == payload['request_id']


def test_3d_mapping_managed_process_is_configured():
    source = Path("src/ylhb_llm/ylhb_llm/system_supervisor_node.py").read_text(encoding="utf-8")

    assert "'3d_capture': ManagedProcess(" in source
    assert 'ros2 run ylhb_3d_mapping zed_svo_capture_node' in source
    assert '--ros-args -p output_root:={self.mapping3d_output_dir}' in source
    assert '-p auto_start:=true' in source
    assert '-p exit_on_finish:=true' in source
    assert "workspace_path('runs', '3d_capture')" in source
    assert 'ros2 launch ylhb_3d_mapping zed_spatial_mapping.launch.py' not in source


def test_export_3d_map_points_to_offline_svo_reconstruction():
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.set_result = Mock()

    node.handle_command('export_3d_map', {})

    node.set_result.assert_called_once()
    assert node.set_result.call_args.args[:2] == ('export_3d_map', False)
    assert 'zed_3d_reconstruct' in node.set_result.call_args.args[2]


def test_start_3d_mapping_refuses_zed_or_perception_running():
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.processes = {
        'zed': FakeProcess(running=True),
        'perception': FakeProcess(running=False),
        '3d_capture': FakeProcess(running=False),
    }
    node.start_process = Mock()
    node.publish_3d_mapping_command = Mock()
    node.set_result = Mock()

    node.handle_command('start_3d_mapping', {})

    node.start_process.assert_not_called()
    node.publish_3d_mapping_command.assert_not_called()
    assert node.set_result.call_args.args[0] == 'start_3d_mapping'
    assert node.set_result.call_args.args[1] is False
    assert 'zed' in node.set_result.call_args.args[2]


def test_start_3d_mapping_starts_process_then_publishes_start():
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.processes = {
        'zed': FakeProcess(running=False),
        'perception': FakeProcess(running=False),
        '3d_capture': FakeProcess(running=False),
    }
    node.start_process = Mock()
    node.publish_3d_mapping_command = Mock()
    node.set_result = Mock()

    node.handle_command('start_3d_mapping', {})

    node.start_process.assert_called_once_with('3d_capture')
    node.publish_3d_mapping_command.assert_not_called()
    node.set_result.assert_called_with('start_3d_mapping', True, '现场 SVO 采集已启动')


def test_stop_3d_mapping_stops_svo_capture_process():
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.processes = {'3d_capture': FakeProcess(running=True)}
    node.publish_3d_mapping_command = Mock()
    node.wait_for_mapping3d_terminal = Mock(return_value='stopped')
    node.stop_process = Mock()
    node.set_result = Mock()
    node.mapping3d_output_dir = '/tmp/missing'

    node.handle_command('stop_3d_mapping', {})

    node.publish_3d_mapping_command.assert_called_once_with('stop')
    node.stop_process.assert_not_called()
    node.set_result.assert_called_with('stop_3d_mapping', True, 'SVO 采集已停止')


def test_stop_3d_mapping_reports_latest_svo(tmp_path):
    (tmp_path / 'latest.json').write_text(
        json.dumps({'svo_file': '/tmp/capture.svo2'}),
        encoding='utf-8',
    )
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.processes = {'3d_capture': FakeProcess(running=True)}
    node.publish_3d_mapping_command = Mock()
    node.wait_for_mapping3d_terminal = Mock(return_value='succeeded')
    node.stop_process = Mock()
    node.set_result = Mock()
    node.mapping3d_output_dir = str(tmp_path)

    node.handle_command('stop_3d_mapping', {})

    assert '/tmp/capture.svo2' in node.set_result.call_args.args[2]


def test_reconstruct_3d_map_commands_use_latest_and_profiles(tmp_path):
    (tmp_path / 'latest.json').write_text(
        json.dumps({'svo_file': '/tmp/capture.svo2'}),
        encoding='utf-8',
    )
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.processes = {}
    node.mapping3d_output_dir = str(tmp_path)
    node.mapping3d_reconstruct_dir = str(tmp_path / 'recon')
    started_commands = []
    node.start_process = Mock(side_effect=lambda name: started_commands.append(node.processes[name].command))
    node.set_result = Mock()

    node.handle_command('reconstruct_latest_3d_map', {})
    node.handle_command('reconstruct_fast_3d_map', {})
    node.handle_command('reconstruct_quality_3d_map', {})

    assert node.start_process.call_args_list[0].args == ('3d_reconstruct',)
    assert all('input:=latest' in command for command in started_commands)
    assert all('capture_root:=' + str(tmp_path) in command for command in started_commands)
    assert 'profile:=quality_safe' in started_commands[0]
    assert 'profile:=fast_check' in started_commands[1]
    assert 'profile:=quality_plus' in started_commands[2]


def test_reconstruct_3d_map_accepts_capture_session_id(tmp_path):
    capture = tmp_path / 'capture_1'
    capture.mkdir()
    (capture / 'metadata.json').write_text(json.dumps({'svo_file': str(capture / 'capture.svo2')}), encoding='utf-8')
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.processes = {}
    node.mapping3d_output_dir = str(tmp_path)
    node.mapping3d_reconstruct_dir = str(tmp_path / 'recon')
    node.start_process = Mock()
    node.set_result = Mock()

    node.handle_command('reconstruct_fast_3d_map', {'session_id': 'capture_1'})

    command = node.processes['3d_reconstruct'].command
    assert 'session:=capture_1' in command
    assert 'input:=latest' not in command


def test_reconstruct_3d_map_requires_latest_capture(tmp_path):
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.processes = {}
    node.mapping3d_output_dir = str(tmp_path)
    node.start_process = Mock()
    node.set_result = Mock()

    node.handle_command('reconstruct_latest_3d_map', {})

    node.start_process.assert_not_called()
    node.set_result.assert_called_with('reconstruct_latest_3d_map', False, '请先完成一次现场采集')


def test_patrol_executor_launch_command_disables_auto_start():
    source = Path("src/ylhb_llm/ylhb_llm/system_supervisor_node.py").read_text(encoding="utf-8")

    assert 'auto_start:=false publish_initial_pose_on_startup:=true' in source
    assert 'auto_start:=true publish_initial_pose_on_startup:=true' not in source


def test_nav2_action_diagnostic_uses_topics_not_action_introspection():
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.topic_has_publishers = Mock(return_value=False)
    node.topic_has_subscribers = Mock(return_value=False)
    node.get_action_names_and_types = Mock(side_effect=AssertionError('must not be called'))

    assert node.has_nav2_action() is False
    node.get_action_names_and_types.assert_not_called()


def test_stop_patrol_mode_stops_executor_without_forwarding_cancel():
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.publish_patrol_command = Mock()
    node.set_result = Mock()
    node.stop_process = Mock()

    node.handle_command('stop_patrol_mode', {})

    node.publish_patrol_command.assert_not_called()
    node.stop_process.assert_called_once_with('patrol_executor')
    node.set_result.assert_called_with('stop_patrol_mode', True, '巡逻模式已停止')


def test_stop_robot_stack_fast_path_when_targets_already_stopped():
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.processes = {
        'patrol_executor': FakeProcess(),
        'navigation': FakeProcess(),
        'perception': FakeProcess(),
        'zed': FakeProcess(),
        'bringup': FakeProcess(),
    }
    node.stop_process = Mock()
    node.publish_mode = Mock()
    node.set_result = Mock()
    node.patrol_mode_state = 'running'
    node.startup_step = 'waiting_nav2'
    node.patrol_error = 'old error'

    node.stop_robot_stack()

    node.stop_process.assert_not_called()
    assert node.patrol_mode_state == 'idle'
    assert node.startup_step == ''
    assert node.patrol_error == ''
    node.publish_mode.assert_called_once_with('ready')
    node.set_result.assert_called_once_with(
        'stop_robot_stack',
        True,
        '巡检运动、导航和感知节点已停止，AI/UI 保持运行',
    )


def test_stopping_bringup_requests_lidar_motor_stop_first(monkeypatch):
    class RunningProcess:
        pid = 123

        def poll(self):
            return None

        def wait(self, timeout=None):
            return 0

    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    process = FakeProcess(running=True)
    process.process = RunningProcess()
    node.processes = {'bringup': process}
    node.lock = type(
        'Lock',
        (),
        {
            '__enter__': lambda _self: None,
            '__exit__': lambda _self, *_args: None,
        },
    )()
    sequence = []
    node.stop_lidar_motor = Mock(side_effect=lambda: sequence.append('stop_lidar_motor'))
    node.set_result_locked = Mock()
    monkeypatch.setattr(system_supervisor_node.os, 'getpgid', lambda _pid: 123)
    monkeypatch.setattr(system_supervisor_node.os, 'killpg', lambda *_args: sequence.append('killpg'))

    node.stop_process('bringup')

    node.stop_lidar_motor.assert_called_once()
    assert sequence[:2] == ['stop_lidar_motor', 'killpg']


def test_mobile_bridge_tcp_status_is_stopped_without_process():
    assert system_supervisor_node.mobile_bridge_tcp_status(False) == 'stopped'


def test_mobile_bridge_tcp_status_reports_connection_failure():
    def failing_connector(*_args, **_kwargs):
        raise OSError('connection refused')

    assert system_supervisor_node.mobile_bridge_tcp_status(True, connector=failing_connector) == 'tcp_error'


def test_mobile_bridge_tcp_status_reports_tcp_ok():
    class FakeSocket:
        def close(self):
            pass

    assert system_supervisor_node.mobile_bridge_tcp_status(
        True,
        connector=lambda *_args, **_kwargs: FakeSocket(),
    ) == 'tcp_ok'


def test_status_payload_contains_mobile_bridge_fields():
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.processes = {
        'bringup': FakeProcess(),
        'mobile_bridge': FakeProcess(running=True),
    }
    node.embedded_task_layer = False
    node.last_command = ''
    node.last_success = True
    node.last_message = 'ready'
    node.mobile_bridge_http = 'http_ok'
    node.mobile_bridge_url = 'http://192.168.1.50:8000'
    node.jetson_ip = '192.168.1.50'

    payload = node.build_status_payload()

    assert payload['mobile_bridge'] == 'running'
    assert payload['mobile_bridge_http'] == 'http_ok'
    assert payload['mobile_bridge_url'] == 'http://192.168.1.50:8000'
    assert payload['jetson_ip'] == '192.168.1.50'


def test_status_payload_contains_latest_mapping3d_status_and_result():
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.processes = {'3d_capture': FakeProcess(running=True)}
    node.embedded_task_layer = False
    node.last_command = ''
    node.last_success = True
    node.last_message = 'ready'
    node.mobile_bridge_http = 'stopped'
    node.mobile_bridge_url = 'http://127.0.0.1:8000'
    node.jetson_ip = '127.0.0.1'
    node.patrol_mode_state = 'idle'
    node.patrol_error = ''
    node.startup_step = ''
    node.latest_mapping3d_status = {'state': 'running', 'success_frames': 5}
    node.latest_mapping3d_result = {'output_file': '/tmp/map.ply'}
    node.build_light_patrol_readiness = Mock(return_value={})
    node.mapping3d_output_dir = '/tmp/missing'
    node.mapping3d_capture_dir = '/tmp/missing'
    node.mapping3d_reconstruct_dir = '/tmp/missing2'

    payload = node.build_status_payload()

    assert payload['latest_mapping3d_status']['success_frames'] == 5
    assert payload['latest_mapping3d_result']['output_file'] == '/tmp/map.ply'
    assert payload['latest_3d_capture'] == {}
    assert payload['latest_3d_reconstruct'] == {}
    assert payload['mapping3d_assets'] == {'captures': [], 'reconstructs': []}
    assert 'mapping3d_storage_summary' in payload


def test_asset_commands_route_to_asset_manager(tmp_path):
    capture = tmp_path / 'capture_1'
    capture.mkdir()
    (capture / 'metadata.json').write_text(json.dumps({'session_id': 'capture_1', 'output_dir': str(capture)}), encoding='utf-8')
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.mapping3d_output_dir = str(tmp_path)
    node.mapping3d_capture_dir = str(tmp_path)
    node.mapping3d_reconstruct_dir = str(tmp_path / 'recon')
    node.set_result = Mock()

    node.handle_command('rename_3d_asset', {'asset_type': 'capture', 'session_id': 'capture_1', 'display_name': 'A'})
    node.handle_command('set_latest_3d_capture', {'session_id': 'capture_1'})
    node.handle_command('delete_3d_asset', {'asset_type': 'capture', 'session_id': 'capture_1'})

    assert json.loads((tmp_path / 'latest.json').read_text())['display_name'] == 'A'
    assert not capture.exists()
    assert (tmp_path / '.trash').exists()


def test_status_payload_contains_patrol_orchestration_fields():
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.processes = {
        'bringup': FakeProcess(running=True),
        'navigation': FakeProcess(),
        'patrol_executor': FakeProcess(running=True),
    }
    node.embedded_task_layer = False
    node.last_command = ''
    node.last_success = True
    node.last_message = 'ready'
    node.mobile_bridge_http = 'stopped'
    node.mobile_bridge_url = 'http://127.0.0.1:8000'
    node.jetson_ip = '127.0.0.1'
    node.patrol_mode_state = 'starting'
    node.patrol_error = ''
    node.startup_step = 'waiting_navigation'
    node.last_patrol_status = {'state': 'running'}
    node.last_patrol_event = {'event': 'route_started'}
    node.build_patrol_readiness = Mock(return_value={
        'bringup': True,
        'navigation': False,
        'executor': True,
        'route_file': True,
        'nav2_action': False,
    })

    payload = node.build_status_payload()

    assert payload['patrol_mode_state'] == 'starting'
    assert payload['startup_step'] == 'waiting_navigation'
    assert payload['startup_step_label'] == '等待地图'
    assert payload['patrol_readiness']['navigation'] is False
    assert payload['patrol_error'] == ''
    assert payload['last_patrol_status'] == {'state': 'running'}
    assert payload['patrol_diagnostics']['last_patrol_event'] == {'event': 'route_started'}


def test_status_payload_does_not_duplicate_patrol_progress_labels():
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.processes = {}
    node.embedded_task_layer = False
    node.last_command = ''
    node.last_success = True
    node.last_message = 'ready'
    node.mobile_bridge_http = 'stopped'
    node.mobile_bridge_url = 'http://127.0.0.1:8000'
    node.jetson_ip = '127.0.0.1'
    node.patrol_mode_state = 'idle'
    node.patrol_error = ''
    node.startup_step = ''
    node.last_patrol_status = {
        'state': 'running',
        'target_index': 1,
        'target_count': 4,
        'target_name': '巡检点2',
        'cycle_index': 2,
    }
    node.build_light_patrol_readiness = Mock(return_value={})

    payload = node.build_status_payload()

    assert 'patrol_progress_label' not in payload
    assert 'current_target_label' not in payload
    assert 'patrol_cycle_label' not in payload


def test_supervisor_uses_same_patrol_status_qos_as_executor():
    qos = system_supervisor_node.patrol_status_qos_profile()
    executor_qos = patrol_status_qos_profile()

    assert qos.depth == executor_qos.depth == 10
    assert qos.reliability == executor_qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == executor_qos.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_readiness_errors_use_chinese_messages_for_navigation_dependencies():
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    states = [
        {
            'navigation': True,
            'map': True,
            'initialpose_subscribers': False,
        }
    ]
    node.build_patrol_readiness = Mock(side_effect=lambda: states[-1])

    assert node.wait_for_readiness_keys(
        ('navigation', 'map', 'initialpose_subscribers'),
        timeout_sec=0.01,
        error_prefix='',
    ) is False

    assert node.startup_step == 'waiting_initialpose_subscribers'
    assert node.patrol_error == '等待 /initialpose 订阅者超时'


def test_patrol_event_marks_initial_pose_published_before_nav2_action_wait():
    node = SystemSupervisorNode.__new__(SystemSupervisorNode)
    node.last_patrol_event = {}
    node.last_initial_pose_event = {}

    msg = type('Msg', (), {'data': '{"event":"initial_pose_published","remaining":2}'})()
    node.patrol_event_callback(msg)

    assert node.last_initial_pose_event['event'] == 'initial_pose_published'
    assert node.wait_for_initial_pose_published(timeout_sec=0.01) is True
