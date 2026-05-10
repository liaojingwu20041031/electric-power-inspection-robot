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
    workspace_dir = os.environ.get('WS_DIR', os.path.expanduser('~/ros2_ws'))
    default_params = os.path.join(pkg_dir, 'config', 'llm.yaml')
    default_products = os.path.join(pkg_dir, 'config', 'products.yaml')
    default_task_image_dir = os.path.join(pkg_dir, 'test_images')
    default_map_output_dir = os.path.join(workspace_dir, 'src', 'maps')
    default_navigation_map = os.path.join(workspace_dir, 'src', 'my_map.yaml')
    default_perception_model = os.path.join(
        workspace_dir, 'src', 'ylhb_perception', 'models', 'yolo26.engine')

    params_file = LaunchConfiguration('params_file')
    products_file = LaunchConfiguration('products_file')
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
    enable_tts = LaunchConfiguration('enable_tts')
    enable_task_layer = LaunchConfiguration('enable_task_layer')
    enable_display_ui = LaunchConfiguration('enable_display_ui')
    enable_system_supervisor = LaunchConfiguration('enable_system_supervisor')
    task_image_dir = LaunchConfiguration('task_image_dir')
    initial_system_mode = LaunchConfiguration('initial_system_mode')
    fullscreen = LaunchConfiguration('fullscreen')
    display = LaunchConfiguration('display')
    force_local_display = LaunchConfiguration('force_local_display')
    workspace_dir_arg = LaunchConfiguration('workspace_dir')
    map_output_dir = LaunchConfiguration('map_output_dir')
    default_navigation_map_arg = LaunchConfiguration('default_navigation_map')
    perception_model_path = LaunchConfiguration('perception_model_path')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('products_file', default_value=default_products),
        DeclareLaunchArgument('dashscope_base_url', default_value='https://dashscope.aliyuncs.com/compatible-mode/v1'),
        DeclareLaunchArgument('vl_model', default_value='qwen3.6-plus'),
        DeclareLaunchArgument('chat_model', default_value='qwen3.6-plus'),
        DeclareLaunchArgument('asr_model', default_value='qwen3-asr-flash'),
        DeclareLaunchArgument('tts_model', default_value='qwen3-tts-flash'),
        DeclareLaunchArgument('audio_device', default_value='default'),
        DeclareLaunchArgument('audio_input_device', default_value='default'),
        DeclareLaunchArgument('audio_output_device', default_value='default'),
        DeclareLaunchArgument('tts_voice', default_value='Cherry'),
        DeclareLaunchArgument('tts_language_type', default_value='Chinese'),
        DeclareLaunchArgument('enable_voice', default_value='false'),
        DeclareLaunchArgument('enable_tts', default_value='false'),
        DeclareLaunchArgument('enable_task_layer', default_value='true'),
        DeclareLaunchArgument('enable_display_ui', default_value='true'),
        DeclareLaunchArgument('enable_system_supervisor', default_value='true'),
        DeclareLaunchArgument('task_image_dir', default_value=default_task_image_dir),
        DeclareLaunchArgument('workspace_dir', default_value=workspace_dir),
        DeclareLaunchArgument('map_output_dir', default_value=default_map_output_dir),
        DeclareLaunchArgument('default_navigation_map', default_value=default_navigation_map),
        DeclareLaunchArgument('perception_model_path', default_value=default_perception_model),
        DeclareLaunchArgument('initial_system_mode', default_value='ready'),
        DeclareLaunchArgument('fullscreen', default_value='true'),
        DeclareLaunchArgument('display', default_value=':0'),
        DeclareLaunchArgument('force_local_display', default_value='true'),

        Node(
            package='ylhb_llm',
            executable='retail_task_node',
            name='retail_task_node',
            output='screen',
            condition=IfCondition(enable_task_layer),
            parameters=[
                params_file,
                {
                    'products_file': products_file,
                    'dashscope_base_url': dashscope_base_url,
                    'vl_model': vl_model,
                    'chat_model': chat_model,
                    'task_image_dir': task_image_dir,
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
            parameters=[
                params_file,
                {
                    'dashscope_base_url': dashscope_base_url,
                    'asr_model': asr_model,
                    'audio_device': audio_device,
                    'audio_input_device': audio_input_device,
                    'enabled': ParameterValue(enable_voice, value_type=bool),
                },
            ],
        ),
        Node(
            package='ylhb_llm',
            executable='voice_output_node',
            name='voice_output_node',
            output='screen',
            condition=IfCondition(enable_task_layer),
            parameters=[
                params_file,
                {
                    'dashscope_base_url': dashscope_base_url,
                    'tts_model': tts_model,
                    'audio_device': audio_device,
                    'audio_output_device': audio_output_device,
                    'tts_voice': tts_voice,
                    'tts_language_type': tts_language_type,
                    'enabled': ParameterValue(enable_voice, value_type=bool),
                    'tts_enabled': ParameterValue(enable_tts, value_type=bool),
                },
            ],
        ),
        Node(
            package='ylhb_llm',
            executable='system_supervisor_node',
            name='system_supervisor_node',
            output='screen',
            condition=IfCondition(enable_system_supervisor),
            parameters=[
                params_file,
                {
                    'workspace_dir': workspace_dir_arg,
                    'map_output_dir': map_output_dir,
                    'default_navigation_map': default_navigation_map_arg,
                    'perception_model_path': perception_model_path,
                },
            ],
        ),
        Node(
            package='ylhb_llm',
            executable='retail_display_ui_node',
            name='retail_display_ui_node',
            output='screen',
            condition=IfCondition(enable_display_ui),
            additional_env={'DISPLAY': display},
            parameters=[
                params_file,
                {
                    'task_image_dir': task_image_dir,
                    'initial_system_mode': initial_system_mode,
                    'fullscreen': ParameterValue(fullscreen, value_type=bool),
                    'display': display,
                    'force_local_display': ParameterValue(force_local_display, value_type=bool),
                },
            ],
        ),
    ])
