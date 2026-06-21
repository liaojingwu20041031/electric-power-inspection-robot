import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('ylhb_base')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    workspace_dir = os.environ.get('WS_DIR', os.path.expanduser('~/ros2_DL'))
    preferred_map = os.path.join(workspace_dir, 'maps', 'my_map.yaml')
    fallback_map = os.path.join(workspace_dir, 'src', 'my_map.yaml')
    default_map = preferred_map
    map_fallback_warning = None
    if not os.path.exists(preferred_map):
        default_map = fallback_map
        map_fallback_warning = LogInfo(
            msg=(
                f'WARN: recommended map {preferred_map} does not exist; '
                f'falling back to compatibility map {fallback_map}. '
                'This is not the recommended map path.'
            )
        )

    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    bringup_reminder = LogInfo(
        msg=(
            'INFO: navigation.launch.py starts Nav2 only. Start '
            'ylhb_base bringup.launch.py first so /odom, /scan, and TF are available. '
            'AMCL waits for an initial pose and does not force the map origin. '
            'Publish /initialpose in the map frame from RViz/Foxglove before '
            'sending navigation goals. For local scan-to-map correction, start '
            'scan_map_relocalization_node before publishing the coarse pose.'
        )
    )

    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': params_file,
        }.items()
    )

    ld = LaunchDescription()
    if map_fallback_warning is not None:
        ld.add_action(map_fallback_warning)
    ld.add_action(bringup_reminder)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(nav2_bringup_cmd)

    return ld
