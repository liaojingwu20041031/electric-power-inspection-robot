from pathlib import Path

import yaml


def test_launch_and_setup_do_not_register_voice_router():
    launch = Path('src/ylhb_llm/launch/llm.launch.py').read_text(encoding='utf-8')
    setup = Path('src/ylhb_llm/setup.py').read_text(encoding='utf-8')

    assert 'voice_command_router_node' not in launch
    assert 'voice_command_router_node' not in setup


def test_llm_config_uses_agent_motion_topic_without_router_vocab():
    config = yaml.safe_load(Path('src/ylhb_llm/config/llm.yaml').read_text(encoding='utf-8'))

    assert 'voice_command_router_node' not in config
    assert config['inspection_agent_node']['ros__parameters']['motion_command_topic'] == '/inspection_ai/motion_command'
    assert config['basic_motion_command_node']['ros__parameters']['motion_command_topic'] == '/inspection_ai/motion_command'
    router_vocab = {
        'motion_aliases',
        'general_qa_words',
        'inspection_words',
        'background_words',
        'followup_words',
        'system_commands',
    }
    for node in config.values():
        assert router_vocab.isdisjoint((node.get('ros__parameters') or {}).keys())


def test_agent_tools_config_renamed_motion_tool():
    config = yaml.safe_load(Path('src/ylhb_llm/config/agent_tools.yaml').read_text(encoding='utf-8'))
    names = {tool['name'] for tool in config['tools']}

    assert 'send_motion_command' in names
    assert 'send_text_motion' not in names
