from ylhb_llm.agent_policy import authorize


def decision(tool, **arguments):
    return {
        'schema_version': '1.0',
        'decision_id': 'd1',
        'response_type': 'tool',
        'intent': tool,
        'safety_level': 'normal',
        'tool_call': {'name': tool, 'arguments': arguments},
        'speak': '',
        'final_answer': '',
        'need_confirm': False,
        'reason_cn': '',
    }


def test_emergency_stop_is_always_allowed():
    result = authorize(decision('emergency_stop'), {'patrol_state': 'unknown'})

    assert result.allowed is True
    assert result.priority == 10
    assert result.interrupt is True


def test_pause_patrol_requires_active_state():
    result = authorize(decision('pause_patrol'), {'patrol_state': 'idle'})

    assert result.allowed is False
    assert result.reason


def test_cancel_patrol_allowed_when_paused():
    result = authorize(decision('cancel_patrol'), {'patrol_state': 'paused'})

    assert result.allowed is True


def test_motion_stop_is_not_patrol_cancel():
    result = authorize(decision('send_motion_command', command='停止'), {'patrol_state': 'running'})

    assert result.allowed is True
    assert result.system_command == ''


def test_dangerous_tool_is_rejected_even_if_schema_was_bypassed():
    result = authorize(decision('/cmd_vel'), {'patrol_state': 'running'})

    assert result.allowed is False


def test_rotate_relative_range_is_enforced_by_policy_schema():
    schemas = {
        'rotate_relative': {
            'required': ['angle_deg'],
            'properties': {
                'angle_deg': {'type': 'number', 'minimum': -180, 'maximum': 180}
            },
        }
    }

    assert authorize(decision('rotate_relative', angle_deg=180), {}, schemas).allowed is True
    result = authorize(decision('rotate_relative', angle_deg=999), {}, schemas)

    assert result.allowed is False
    assert result.reason


def test_schema_preconditions_return_structured_recoverable_failure():
    schemas = {
        'rotate_relative': {
            'properties': {'angle_deg': {'type': 'number'}},
            'required': ['angle_deg'],
            'preconditions': ['bringup_running', 'chassis_online', 'sensor_fresh'],
        }
    }

    result = authorize(
        decision('rotate_relative', angle_deg=10),
        {
            'system_status': {'bringup': 'stopped'},
            'robot_summary': {
                'chassis': {'fresh': False},
                'sensors': {'lidar': 'stale', 'odom': 'stale'},
            },
        },
        schemas,
    )

    assert result.allowed is False
    assert result.error_code == 'precondition_failed'
    assert result.missing_preconditions == [
        'bringup_running', 'chassis_online', 'sensor_fresh',
    ]
    assert result.recoverable is True
    assert result.recovery_components == ['bringup']
    assert result.state_summary['components']['bringup'] == 'stopped'

    result = authorize(
        decision('rotate_relative', angle_deg=10),
        {
            'system_status': {'bringup': 'running'},
            'robot_summary': {
                'components': {'bringup': 'running'},
                'chassis': {'fresh': False},
                'sensors': {'lidar': 'stale', 'odom': 'stale'},
            },
        },
        schemas,
    )

    assert result.recoverable is False
    assert result.recovery_components == []


def test_start_component_schema_only_accepts_bringup():
    schema = {
        'properties': {'component': {'type': 'string', 'enum': ['bringup']}},
        'required': ['component'],
    }

    navigation = authorize(
        decision('start_component', component='navigation'),
        {},
        {'start_component': schema},
    )
    bringup = authorize(
        decision('start_component', component='bringup'),
        {},
        {'start_component': schema},
    )
    patrol = authorize(
        decision('start_component', component='patrol_executor'),
        {},
        {'start_component': schema},
    )

    assert navigation.allowed is False
    assert patrol.allowed is False
    assert bringup.allowed is True


def test_voice_session_tools_are_allowed_by_policy():
    assert authorize(decision('end_voice_conversation'), {}, {
        'end_voice_conversation': {'properties': {}, 'required': []},
    }).allowed is True


def test_robot_ready_precondition_recovers_only_with_bringup():
    result = authorize(
        decision('rotate_relative', angle_deg=10),
        {'system_status': {'bringup': 'stopped'}},
        {'rotate_relative': {
            'properties': {'angle_deg': {'type': 'number'}},
            'required': ['angle_deg'],
            'preconditions': ['robot_ready', 'chassis_online', 'sensor_fresh'],
        }},
    )

    assert result.recovery_components == ['bringup']


def test_fresh_offline_chassis_is_not_treated_as_online():
    result = authorize(
        decision('rotate_relative', angle_deg=10),
        {'robot_summary': {
            'components': {'bringup': 'running'},
            'chassis': {'state': 'offline', 'fresh': True},
            'sensors': {'lidar': 'ok', 'odom': 'ok'},
        }},
        {'rotate_relative': {
            'properties': {'angle_deg': {'type': 'number'}},
            'required': ['angle_deg'],
            'preconditions': ['robot_ready', 'chassis_online', 'sensor_fresh'],
        }},
    )

    assert result.missing_preconditions == ['chassis_online']
    assert result.recovery_components == []
    assert authorize(decision('close_voice_mode'), {}, {
        'close_voice_mode': {'properties': {}, 'required': []},
    }).allowed is True
