from pathlib import Path

from ylhb_mobile_bridge.patrol_route_store import (
    default_route_directory,
    resolve_route_file_path,
)


REPO_ROOT = Path(__file__).resolve().parents[3]


def read_repo_file(relative_path: str) -> str:
    return (REPO_ROOT / relative_path).read_text(encoding='utf-8')


def test_route_directory_is_resolved_from_ws_dir_at_runtime(monkeypatch, tmp_path):
    route = tmp_path / 'maps' / 'route_patrol_042.json'
    route.parent.mkdir()
    route.write_text('{}', encoding='utf-8')
    monkeypatch.setenv('WS_DIR', str(tmp_path))

    assert default_route_directory() == route.parent
    assert resolve_route_file_path('auto') == route


def test_agent_sources_do_not_embed_workspace_path():
    files = {
        'src/ylhb_llm/config/llm.yaml': read_repo_file('src/ylhb_llm/config/llm.yaml'),
        'src/ylhb_llm/launch/llm.launch.py': read_repo_file('src/ylhb_llm/launch/llm.launch.py'),
        'src/ylhb_llm/ylhb_llm/inspection_agent_node.py': read_repo_file(
            'src/ylhb_llm/ylhb_llm/inspection_agent_node.py'
        ),
        'src/ylhb_llm/ylhb_llm/system_supervisor_node.py': read_repo_file(
            'src/ylhb_llm/ylhb_llm/system_supervisor_node.py'
        ),
    }
    for path, text in files.items():
        assert '/home/nvidia/ros2_DL' not in text, path


def test_launch_does_not_override_planner_or_voice_model_defaults():
    launch = read_repo_file('src/ylhb_llm/launch/llm.launch.py')

    for parameter in (
        'chat_model', 'planner_base_url', 'asr_model', 'tts_model',
        'tts_voice', 'tts_language_type', 'vl_model',
    ):
        assert f"DeclareLaunchArgument('{parameter}'" not in launch
