import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('ylhb_llm')
    workspace_dir = os.environ.get('WS_DIR', os.path.expanduser('~/ros2_DL'))
    default_params = os.path.join(pkg_dir, 'config', 'llm.yaml')
    default_map_output_dir = os.path.join(workspace_dir, 'maps')
    default_perception_model = os.path.join(workspace_dir, 'src', 'ylhb_perception', 'models', 'yolo26.engine')

    params_file = LaunchConfiguration('params_file')
    dashscope_base_url = LaunchConfiguration('dashscope_base_url')
    vl_model = LaunchConfiguration('vl_model')
    chat_model = LaunchConfiguration('chat_model')
    asr_model = LaunchConfiguration('asr_model')
    tts_model = LaunchConfiguration('tts_model')
    audio_device = LaunchConfiguration('audio_device')
    audio_input_device = LaunchConfiguration('audio_input_device')
    audio_output_device = LaunchConfiguration('audio_output_device')
    tts_voice = LaunchConfiguration('tts_voice')
    tts_language_type = LaunchConfiguration('tts_language_type')
    enable_voice = LaunchConfiguration('enable_voice')
    enable_voice_session = LaunchConfiguration('enable_voice_session')
    enable_capture_voice = LaunchConfiguration('enable_capture_voice')
    enable_tts = LaunchConfiguration('enable_tts')
    enable_task_layer = LaunchConfiguration('enable_task_layer')
    enable_inspection_task = LaunchConfiguration('enable_inspection_task')
    enable_inspection_agent = LaunchConfiguration('enable_inspection_agent')
    enable_display_ui = LaunchConfiguration('enable_display_ui')
    enable_system_supervisor = LaunchConfiguration('enable_system_supervisor')
    initial_system_mode = LaunchConfiguration('initial_system_mode')
    fullscreen = LaunchConfiguration('fullscreen')
    display = LaunchConfiguration('display')
    xauthority = LaunchConfiguration('xauthority')
    force_local_display = LaunchConfiguration('force_local_display')
    workspace_dir_arg = LaunchConfiguration('workspace_dir')
    map_output_dir = LaunchConfiguration('map_output_dir')
    perception_model_path = LaunchConfiguration('perception_model_path')
    enable_llm_parse = LaunchConfiguration('enable_llm_parse')
    voice_energy_threshold = LaunchConfiguration('voice_energy_threshold')
    voice_command_vad_silence_sec = LaunchConfiguration('voice_command_vad_silence_sec')
    voice_command_min_voice_sec = LaunchConfiguration('voice_command_min_voice_sec')
    voice_wait_wake_threshold_multiplier = LaunchConfiguration('voice_wait_wake_threshold_multiplier')
    voice_tts_tail_pause_sec = LaunchConfiguration('voice_tts_tail_pause_sec')
    voice_debug_save_asr_audio = LaunchConfiguration('voice_debug_save_asr_audio')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('dashscope_base_url', default_value='https://dashscope.aliyuncs.com/compatible-mode/v1'),
        DeclareLaunchArgument('vl_model', default_value='qwen3.6-plus'),
        DeclareLaunchArgument('chat_model', default_value='qwen3.6-plus'),
        DeclareLaunchArgument('asr_model', default_value='qwen3-asr-flash'),
        DeclareLaunchArgument('tts_model', default_value='qwen3-tts-flash'),
        DeclareLaunchArgument('audio_device', default_value='default'),
        DeclareLaunchArgument('audio_input_device', default_value='default'),
        DeclareLaunchArgument('audio_output_device', default_value='default'),
        DeclareLaunchArgument('tts_voice', default_value='Serena'),
        DeclareLaunchArgument('tts_language_type', default_value='Chinese'),
        DeclareLaunchArgument('enable_voice', default_value='false'),
        DeclareLaunchArgument('enable_voice_session', default_value=enable_voice),
        DeclareLaunchArgument('enable_capture_voice', default_value=enable_voice),
        DeclareLaunchArgument('enable_tts', default_value='false'),
        DeclareLaunchArgument('enable_task_layer', default_value='true'),
        DeclareLaunchArgument('enable_inspection_task', default_value='false'),
        DeclareLaunchArgument('enable_inspection_agent', default_value='true'),
        DeclareLaunchArgument('enable_display_ui', default_value='true'),
        DeclareLaunchArgument('enable_system_supervisor', default_value='true'),
        DeclareLaunchArgument('workspace_dir', default_value=workspace_dir),
        DeclareLaunchArgument('map_output_dir', default_value=default_map_output_dir),
        DeclareLaunchArgument('perception_model_path', default_value=default_perception_model),
        DeclareLaunchArgument('initial_system_mode', default_value='ready'),
        DeclareLaunchArgument('fullscreen', default_value='true'),
        DeclareLaunchArgument('display', default_value=':0'),
        DeclareLaunchArgument('xauthority', default_value=''),
        DeclareLaunchArgument('force_local_display', default_value='true'),
        DeclareLaunchArgument('enable_llm_parse', default_value='false'),
        DeclareLaunchArgument('voice_energy_threshold', default_value='800'),
        DeclareLaunchArgument('voice_command_vad_silence_sec', default_value='1.25'),
        DeclareLaunchArgument('voice_command_min_voice_sec', default_value='0.8'),
        DeclareLaunchArgument('voice_wait_wake_threshold_multiplier', default_value='1.8'),
        DeclareLaunchArgument('voice_tts_tail_pause_sec', default_value='0.9'),
        DeclareLaunchArgument('voice_debug_save_asr_audio', default_value='false'),

        Node(
            package='ylhb_llm',
            executable='inspection_task_node',
            name='inspection_task_node',
            output='screen',
            condition=IfCondition(enable_inspection_task),
            parameters=[
                params_file,
                {
                    'dashscope_base_url': dashscope_base_url,
                    'chat_model': chat_model,
                    'enable_llm_parse': ParameterValue(enable_llm_parse, value_type=bool),
                },
            ],
        ),
        Node(
            package='ylhb_llm',
            executable='inspection_agent_node',
            name='inspection_agent_node',
            output='screen',
            condition=IfCondition(enable_inspection_agent),
            parameters=[
                params_file,
                {
                    'dashscope_base_url': dashscope_base_url,
                    'chat_model': chat_model,
                },
            ],
        ),
        Node(
            package='ylhb_llm',
            executable='basic_motion_command_node',
            name='basic_motion_command_node',
            output='screen',
            condition=IfCondition(enable_task_layer),
            parameters=[params_file],
        ),
        Node(
            package='ylhb_llm',
            executable='voice_input_node',
            name='voice_input_node',
            output='screen',
            condition=IfCondition(enable_task_layer),
            parameters=[params_file, {'dashscope_base_url': dashscope_base_url, 'asr_model': asr_model, 'audio_device': audio_device, 'audio_input_device': audio_input_device, 'enabled': ParameterValue(enable_capture_voice, value_type=bool)}],
        ),
        Node(
            package='ylhb_llm',
            executable='voice_session_node',
            name='voice_session_node',
            output='screen',
            condition=IfCondition(enable_task_layer),
            parameters=[params_file, {
                'dashscope_base_url': dashscope_base_url,
                'asr_model': asr_model,
                'audio_device': audio_device,
                'audio_input_device': audio_input_device,
                'enabled': ParameterValue(enable_voice_session, value_type=bool),
                'energy_threshold': ParameterValue(voice_energy_threshold, value_type=int),
                'command_vad_silence_sec': ParameterValue(voice_command_vad_silence_sec, value_type=float),
                'command_min_voice_sec': ParameterValue(voice_command_min_voice_sec, value_type=float),
                'wait_wake_threshold_multiplier': ParameterValue(voice_wait_wake_threshold_multiplier, value_type=float),
                'tts_tail_pause_sec': ParameterValue(voice_tts_tail_pause_sec, value_type=float),
                'debug_save_asr_audio': ParameterValue(voice_debug_save_asr_audio, value_type=bool),
            }],
        ),
        Node(
            package='ylhb_llm',
            executable='voice_output_node',
            name='voice_output_node',
            output='screen',
            condition=IfCondition(enable_task_layer),
            parameters=[params_file, {'dashscope_base_url': dashscope_base_url, 'tts_model': tts_model, 'audio_device': audio_device, 'audio_output_device': audio_output_device, 'tts_voice': tts_voice, 'tts_language_type': tts_language_type, 'enabled': ParameterValue(enable_voice, value_type=bool), 'tts_enabled': ParameterValue(enable_tts, value_type=bool)}],
        ),
        Node(
            package='ylhb_llm',
            executable='system_supervisor_node',
            name='system_supervisor_node',
            output='screen',
            condition=IfCondition(enable_system_supervisor),
            parameters=[params_file, {'workspace_dir': workspace_dir_arg, 'map_output_dir': map_output_dir, 'perception_model_path': perception_model_path, 'embedded_task_layer': ParameterValue(enable_task_layer, value_type=bool), 'enable_voice': ParameterValue(enable_voice, value_type=bool), 'enable_voice_session': ParameterValue(enable_voice_session, value_type=bool), 'enable_capture_voice': ParameterValue(enable_capture_voice, value_type=bool), 'enable_tts': ParameterValue(enable_tts, value_type=bool), 'audio_device': audio_device, 'audio_input_device': audio_input_device, 'audio_output_device': audio_output_device, 'asr_model': asr_model, 'tts_model': tts_model, 'tts_voice': tts_voice, 'tts_language_type': tts_language_type, 'dashscope_base_url': dashscope_base_url}],
        ),
        Node(
            package='ylhb_llm',
            executable='inspection_display_ui_node',
            name='inspection_display_ui_node',
            output='screen',
            condition=IfCondition(enable_display_ui),
            additional_env={'DISPLAY': display, 'XAUTHORITY': xauthority},
            parameters=[params_file, {'initial_system_mode': initial_system_mode, 'fullscreen': ParameterValue(fullscreen, value_type=bool), 'display': display, 'force_local_display': ParameterValue(force_local_display, value_type=bool)}],
        ),
    ])
