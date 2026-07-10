import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from nav2_common.launch import RewrittenYaml


def fail_if_keepout_mask_missing(context):
    enable_keepout = LaunchConfiguration('enable_keepout').perform(context).lower()
    if enable_keepout not in ('true', '1', 'yes', 'on'):
        return []
    mask_path = os.path.expanduser(LaunchConfiguration('keepout_mask').perform(context))
    if os.path.exists(mask_path):
        return []
    return [
        LogInfo(msg=f'ERROR: keepout mask {mask_path} does not exist. Run scripts/generate_keepout_mask.py first.'),
        Shutdown(reason=f'keepout mask missing: {mask_path}'),
    ]


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
    enable_keepout = LaunchConfiguration('enable_keepout')
    keepout_mask = LaunchConfiguration('keepout_mask')

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'global_costmap.global_costmap.ros__parameters.keepout_filter.enabled': enable_keepout,
        },
        convert_types=True,
    )

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

    declare_enable_keepout_cmd = DeclareLaunchArgument(
        'enable_keepout',
        default_value='true',
        description='Enable global Nav2 keepout filter and keepout mask lifecycle nodes')

    declare_keepout_mask_cmd = DeclareLaunchArgument(
        'keepout_mask',
        default_value=os.path.join(workspace_dir, 'maps', 'keepout', 'keepout_mask_power_room_a.yaml'),
        description='Full path to keepout mask yaml file')

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
            'params_file': configured_params,
        }.items()
    )

    ld = LaunchDescription()
    if map_fallback_warning is not None:
        ld.add_action(map_fallback_warning)
    ld.add_action(bringup_reminder)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_enable_keepout_cmd)
    ld.add_action(declare_keepout_mask_cmd)
    ld.add_action(OpaqueFunction(function=fail_if_keepout_mask_missing))
    ld.add_action(nav2_bringup_cmd)
    ld.add_action(LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='keepout_filter_mask_server',
        output='screen',
        condition=IfCondition(enable_keepout),
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': keepout_mask,
            'topic_name': 'keepout_filter_mask',
        }],
    ))
    ld.add_action(LifecycleNode(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        condition=IfCondition(enable_keepout),
        parameters=[{
            'use_sim_time': use_sim_time,
            'filter_info_topic': 'keepout_costmap_filter_info',
            'type': 0,
            'mask_topic': 'keepout_filter_mask',
            'base': 0.0,
            'multiplier': 1.0,
        }],
    ))
    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_keepout',
        output='screen',
        condition=IfCondition(enable_keepout),
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'keepout_filter_mask_server',
                'costmap_filter_info_server',
            ],
        }],
    ))

    return ld
