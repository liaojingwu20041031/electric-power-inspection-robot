from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]


def read_repo_file(relative_path: str) -> str:
    return (REPO_ROOT / relative_path).read_text(encoding='utf-8')


def test_self_authored_map_paths_use_workspace_maps_dir():
    expected_yaml = '/home/nvidia/ros2_DL/maps/my_map.yaml'
    expected_prefix = '/home/nvidia/ros2_DL/maps/my_map'

    files = {
        'src/ylhb_llm/config/llm.yaml': read_repo_file('src/ylhb_llm/config/llm.yaml'),
        'src/ylhb_llm/launch/llm.launch.py': read_repo_file('src/ylhb_llm/launch/llm.launch.py'),
        'src/ylhb_llm/ylhb_llm/system_supervisor_node.py': read_repo_file(
            'src/ylhb_llm/ylhb_llm/system_supervisor_node.py'
        ),
        'src/ylhb_base/launch/navigation.launch.py': read_repo_file(
            'src/ylhb_base/launch/navigation.launch.py'
        ),
        'src/ylhb_mobile_bridge/config/mobile_bridge.yaml': read_repo_file(
            'src/ylhb_mobile_bridge/config/mobile_bridge.yaml'
        ),
        'src/ylhb_mobile_bridge/ylhb_mobile_bridge/ros_bridge.py': read_repo_file(
            'src/ylhb_mobile_bridge/ylhb_mobile_bridge/ros_bridge.py'
        ),
        'scripts/run_on_jetson.sh': read_repo_file('scripts/run_on_jetson.sh'),
    }

    for path, text in files.items():
        assert 'ros2_DL/src/maps' not in text, path
        assert 'ros2_DL/src/my_map' not in text, path

    assert expected_yaml in files['src/ylhb_llm/config/llm.yaml']
    assert "'maps', 'my_map.yaml'" in files['src/ylhb_base/launch/navigation.launch.py']
    assert expected_prefix in files['src/ylhb_mobile_bridge/config/mobile_bridge.yaml']
