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


def test_agent_config_uses_planner_flags_and_chat_topic():
    params = yaml.safe_load(Path('src/ylhb_llm/config/llm.yaml').read_text(encoding='utf-8'))['inspection_agent_node']['ros__parameters']

    assert params['agent_chat_topic'] == '/inspection_ai/agent_chat'
    assert params['planner_model'] == 'qwen3.7-plus'
    assert 'enable_llm_planner' in params
    assert 'offline_safe_mode' in params
    assert 'enable_llm_fallback' not in params


def test_voice_session_config_has_wake_phrase_and_threshold():
    params = yaml.safe_load(Path('src/ylhb_llm/config/llm.yaml').read_text(encoding='utf-8'))['voice_session_node']['ros__parameters']

    assert params['wake_phrase']
    assert 0.45 <= params['wake_match_threshold'] <= 0.8
    assert {'小林小林', '小玲小玲'} <= set(params['wake_aliases'])
    assert 1.2 <= params['kws_keywords_score'] <= 1.5
    assert params['kws_keywords_threshold'] >= 0.2
