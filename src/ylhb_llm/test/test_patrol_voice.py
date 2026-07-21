import json
from pathlib import Path

from ylhb_llm.patrol_voice import PatrolVoice


CONFIG_PATH = Path(__file__).resolve().parents[1] / 'config' / 'patrol_voice.yaml'
AUDIO_DIR = Path(__file__).resolve().parents[1] / 'assets' / 'audio'


def make_voice():
    return PatrolVoice.from_file(str(CONFIG_PATH), str(AUDIO_DIR), '/inventory')


def event(name, **fields):
    return {
        'event': name,
        'boot_id': 'boot-1',
        'execution_id': 'execution-1',
        'route_id': 'route-1',
        'timestamp': 1784601700.0,
        **fields,
    }


def test_route_started_selects_fixed_audio():
    request = make_voice().request_for_event(event('route_started'))

    assert request.audio_path.endswith('patrol_route_started.wav')
    assert request.inventory_path == '/inventory/patrol_route_started.wav'
    assert request.priority == 40
    assert request.interrupt is False


def test_target_reached_inserts_target_name():
    request = make_voice().request_for_event(
        event('target_reached', target_id='target-1', target_name='一号开关柜')
    )

    assert '一号开关柜' in request.text
    assert request.audio_path == ''


def test_return_home_selects_normal_or_failure_rule():
    normal = make_voice().request_for_event(event('return_home_started'))
    failed = make_voice().request_for_event(
        event('return_home_started', after_failure=True, timestamp=1784601701.0)
    )

    assert normal.audio_path.endswith('patrol_return_home.wav')
    assert failed.audio_path.endswith('patrol_abnormal_return.wav')
    assert failed.priority == 85
    assert failed.interrupt is True


def test_route_failed_is_high_priority_and_interrupting():
    request = make_voice().request_for_event(event('route_failed'))

    assert request.priority == 95
    assert request.interrupt is True


def test_unknown_and_disabled_events_are_ignored(tmp_path):
    assert make_voice().request_for_event(event('target_task_finished')) is None

    config = tmp_path / 'disabled.yaml'
    config.write_text('version: 1\nenabled: false\nannouncements: {}\n', encoding='utf-8')
    voice = PatrolVoice.from_file(str(config), str(AUDIO_DIR))
    assert voice.request_for_event(event('route_started')) is None


def test_missing_template_field_uses_safe_default():
    request = make_voice().request_for_event(event('target_reached'))

    assert '检查点' in request.text


def test_invalid_json_is_ignored_without_exception():
    assert make_voice().request_for_json('{invalid json') is None


def test_duplicate_event_is_only_returned_once():
    voice = make_voice()
    raw = json.dumps(event('route_started'), ensure_ascii=False)

    assert voice.request_for_json(raw) is not None
    assert voice.request_for_json(raw) is None
