import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('ylhb_llm')
    workspace_dir = os.environ.get('WS_DIR', os.path.expanduser('~/ros2_DL'))
    default_params = os.path.join(pkg_dir, 'config', 'llm.yaml')
    default_map_output_dir = os.path.join(workspace_dir, 'maps')
    default_perception_model = os.path.join(workspace_dir, 'src', 'ylhb_perception', 'models', 'yolo26.engine')
    default_keepout_mask = os.path.join(workspace_dir, 'maps', 'keepout', 'keepout_mask_power_room_a.yaml')

    params_file = LaunchConfiguration('params_file')
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
    mobile_bridge_managed_externally = LaunchConfiguration('mobile_bridge_managed_externally')
    workspace_dir_arg = LaunchConfiguration('workspace_dir')
    route_directory = LaunchConfiguration('route_directory')
    map_output_dir = LaunchConfiguration('map_output_dir')
    perception_model_path = LaunchConfiguration('perception_model_path')
    enable_keepout_navigation = LaunchConfiguration('enable_keepout_navigation')
    keepout_mask_path = LaunchConfiguration('keepout_mask_path')
    patrol_route_path = LaunchConfiguration('patrol_route_path')
    keepout_route_path = LaunchConfiguration('keepout_route_path')
    enable_llm_parse = LaunchConfiguration('enable_llm_parse')
    voice_energy_threshold = LaunchConfiguration('voice_energy_threshold')
    voice_command_vad_silence_sec = LaunchConfiguration('voice_command_vad_silence_sec')
    voice_command_min_voice_sec = LaunchConfiguration('voice_command_min_voice_sec')
    voice_wait_wake_threshold_multiplier = LaunchConfiguration('voice_wait_wake_threshold_multiplier')
    voice_tts_tail_pause_sec = LaunchConfiguration('voice_tts_tail_pause_sec')
    voice_debug_save_asr_audio = LaunchConfiguration('voice_debug_save_asr_audio')
    shared_route_parameters = {
        'workspace_dir': workspace_dir_arg,
        'route_directory': route_directory,
        'patrol_route_path': patrol_route_path,
    }
    audio_environment_parameters = {
        parameter: value
        for parameter, value in {
            'audio_input_device': os.environ.get('YLHB_AUDIO_INPUT_DEVICE', ''),
            'audio_output_device': os.environ.get('YLHB_AUDIO_OUTPUT_DEVICE', ''),
            'tts_voice': os.environ.get('YLHB_TTS_VOICE', ''),
        }.items()
        if value
    }

    display_ui = Node(
        package='ylhb_llm',
        executable='inspection_display_ui_node',
        name='inspection_display_ui_node',
        output='screen',
        condition=IfCondition(enable_display_ui),
        additional_env={'DISPLAY': display, 'XAUTHORITY': xauthority},
        parameters=[params_file, {'initial_system_mode': initial_system_mode, 'fullscreen': ParameterValue(fullscreen, value_type=bool), 'display': display, 'force_local_display': ParameterValue(force_local_display, value_type=bool)}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
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
        DeclareLaunchArgument('route_directory', default_value=os.path.join(workspace_dir, 'maps')),
        DeclareLaunchArgument('map_output_dir', default_value=default_map_output_dir),
        DeclareLaunchArgument('perception_model_path', default_value=default_perception_model),
        DeclareLaunchArgument('enable_keepout_navigation', default_value='true'),
        DeclareLaunchArgument('keepout_mask_path', default_value=default_keepout_mask),
        DeclareLaunchArgument('patrol_route_path', default_value='auto'),
        DeclareLaunchArgument('keepout_route_path', default_value=''),
        DeclareLaunchArgument('initial_system_mode', default_value='ready'),
        DeclareLaunchArgument('fullscreen', default_value='true'),
        DeclareLaunchArgument('display', default_value=':0'),
        DeclareLaunchArgument('xauthority', default_value=''),
        DeclareLaunchArgument('force_local_display', default_value='true'),
        DeclareLaunchArgument('mobile_bridge_managed_externally', default_value='false'),
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
            parameters=[params_file, shared_route_parameters],
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
            executable='base_motion_skill_node',
            name='base_motion_skill_node',
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
            parameters=[params_file, audio_environment_parameters, {'enabled': ParameterValue(enable_capture_voice, value_type=bool)}],
        ),
        Node(
            package='ylhb_llm',
            executable='voice_session_node',
            name='voice_session_node',
            output='screen',
            condition=IfCondition(enable_task_layer),
            parameters=[params_file, audio_environment_parameters, {
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
            parameters=[params_file, audio_environment_parameters, {'enabled': ParameterValue(enable_voice, value_type=bool), 'tts_enabled': ParameterValue(enable_tts, value_type=bool)}],
        ),
        Node(
            package='ylhb_llm',
            executable='system_supervisor_node',
            name='system_supervisor_node',
            output='screen',
            condition=IfCondition(enable_system_supervisor),
            parameters=[params_file, shared_route_parameters, audio_environment_parameters, {'map_output_dir': map_output_dir, 'perception_model_path': perception_model_path, 'enable_keepout_navigation': ParameterValue(enable_keepout_navigation, value_type=bool), 'keepout_mask_path': keepout_mask_path, 'keepout_route_path': keepout_route_path, 'embedded_task_layer': ParameterValue(enable_task_layer, value_type=bool), 'enable_voice': ParameterValue(enable_voice, value_type=bool), 'enable_voice_session': ParameterValue(enable_voice_session, value_type=bool), 'enable_capture_voice': ParameterValue(enable_capture_voice, value_type=bool), 'enable_tts': ParameterValue(enable_tts, value_type=bool), 'mobile_bridge_managed_externally': ParameterValue(mobile_bridge_managed_externally, value_type=bool)}],
        ),
        RegisterEventHandler(
            # The UI is the inspection-console lifecycle anchor: never respawn it alone.
            # Its exit intentionally shuts down agent, voice and supervisor siblings.
            OnProcessExit(
                target_action=display_ui,
                on_exit=[EmitEvent(event=Shutdown(reason='inspection display UI exited'))],
            ),
            condition=IfCondition(enable_display_ui),
        ),
        display_ui,
    ])
