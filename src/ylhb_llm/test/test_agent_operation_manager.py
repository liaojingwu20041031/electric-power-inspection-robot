from ylhb_llm.agent_operation_manager import AgentOperationManager
from ylhb_llm.robot_status_aggregator import RobotStatusAggregator


def test_sent_operation_times_out_instead_of_becoming_successful():
    manager = AgentOperationManager(clock=lambda: 100.0)
    operation = manager.create('run_1', 'call_1', 'start_route', {'route_id': 'route_1'}, timeout_sec=5.0)
    manager.mark_sent(operation.operation_id, now=101.0)

    result = manager.get(operation.operation_id, now=106.0)

    assert result['state'] == 'timeout'
    assert result['result']['ok'] is False


def test_operation_feedback_advances_only_to_reported_state():
    manager = AgentOperationManager(clock=lambda: 100.0)
    operation = manager.create('run_1', 'call_1', 'move_relative', {'distance_m': 0.1}, timeout_sec=5.0)

    manager.update(operation.operation_id, 'accepted', {'message': 'accepted'}, now=101.0)
    manager.update(operation.operation_id, 'running', {'message': 'running'}, now=102.0)

    assert manager.get(operation.operation_id, now=102.0)['state'] == 'running'


def test_aggregator_marks_expired_observation_stale():
    aggregator = RobotStatusAggregator(default_max_age_sec=1.0, clock=lambda: 10.0)
    aggregator.update('odom', {'x': 1.0, 'y': 2.0}, now=8.0)

    observation = aggregator.get('odom', now=10.0)

    assert observation['fresh'] is False
    assert observation['state'] == 'stale'
