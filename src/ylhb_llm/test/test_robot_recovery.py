from pathlib import Path

from ylhb_llm.robot_recovery import RecoveryCatalog


def test_catalog_loads_only_valid_managed_process_recoveries(tmp_path: Path):
    path = tmp_path / 'recoveries.yaml'
    path.write_text(
        '''recoveries:
  perception:
    action_type: restart_managed_process
    process: perception
    timeout_sec: 25
    cooldown_sec: 60
    max_attempts_per_incident: 1
  unsafe:
    action_type: shell
    process: perception
  unknown:
    action_type: restart_managed_process
    process: navigation
''',
        encoding='utf-8',
    )

    catalog = RecoveryCatalog.from_file(path, managed_processes={'perception', 'mobile_bridge'})

    assert catalog.names() == ['perception']
    assert catalog.get('perception')['process'] == 'perception'
    assert catalog.invalid_items == {'unsafe': 'unsupported action_type', 'unknown': 'unmanaged process'}


def test_missing_or_invalid_file_keeps_read_only_agent_available(tmp_path: Path):
    assert RecoveryCatalog.from_file(tmp_path / 'missing.yaml').names() == []
