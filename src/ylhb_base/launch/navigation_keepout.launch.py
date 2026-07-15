import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def fail_if_keepout_masks_missing(context):
    mask_paths = [
        os.path.expanduser(LaunchConfiguration(name).perform(context))
        for name in ('keepout_global_mask', 'keepout_local_mask')
    ]
    missing = [path for path in mask_paths if not os.path.exists(path)]
    if not missing:
        return []
    paths = ', '.join(missing)
    return [
        LogInfo(msg=f'ERROR: keepout masks missing: {paths}. Run scripts/generate_keepout_mask.py first.'),
        Shutdown(reason=f'keepout masks missing: {paths}'),
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
    autostart = LaunchConfiguration('autostart')
    keepout_global_mask = LaunchConfiguration('keepout_global_mask')
    keepout_local_mask = LaunchConfiguration('keepout_local_mask')

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
        default_value=os.path.join(pkg_dir, 'config', 'nav2_params_keepout.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='false',
        description='Let the supervisor activate localization and navigation in order')

    declare_keepout_global_mask_cmd = DeclareLaunchArgument(
        'keepout_global_mask',
        default_value=os.path.join(workspace_dir, 'maps', 'keepout', 'keepout_global_mask.yaml'),
        description='Global planner keepout mask yaml')

    declare_keepout_local_mask_cmd = DeclareLaunchArgument(
        'keepout_local_mask',
        default_value=os.path.join(workspace_dir, 'maps', 'keepout', 'keepout_local_mask.yaml'),
        description='Local controller keepout mask yaml')

    bringup_reminder = LogInfo(
        msg=(
            'INFO: navigation_keepout.launch.py starts Nav2 with keepout filter. '
            'Start ylhb_base bringup.launch.py first so /odom, /scan, and TF are available. '
            'AMCL waits for an initial pose and does not force the map origin.'
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
            'autostart': autostart,
        }.items()
    )

    ld = LaunchDescription()
    if map_fallback_warning is not None:
        ld.add_action(map_fallback_warning)
    ld.add_action(bringup_reminder)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_keepout_global_mask_cmd)
    ld.add_action(declare_keepout_local_mask_cmd)
    ld.add_action(OpaqueFunction(function=fail_if_keepout_masks_missing))
    ld.add_action(nav2_bringup_cmd)
    ld.add_action(LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='keepout_global_mask_server',
        namespace='',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': keepout_global_mask,
            'topic_name': '/keepout_global_mask',
        }],
    ))
    ld.add_action(LifecycleNode(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='keepout_global_filter_info_server',
        namespace='',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'filter_info_topic': '/keepout_global_filter_info',
            'type': 0,
            'mask_topic': '/keepout_global_mask',
            'base': 0.0,
            'multiplier': 1.0,
        }],
    ))
    ld.add_action(LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='keepout_local_mask_server',
        namespace='',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': keepout_local_mask,
            'topic_name': '/keepout_local_mask',
        }],
    ))
    ld.add_action(LifecycleNode(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='keepout_local_filter_info_server',
        namespace='',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'filter_info_topic': '/keepout_local_filter_info',
            'type': 0,
            'mask_topic': '/keepout_local_mask',
            'base': 0.0,
            'multiplier': 1.0,
        }],
    ))
    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_keepout',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': False,
            'node_names': [
                'keepout_global_mask_server',
                'keepout_global_filter_info_server',
                'keepout_local_mask_server',
                'keepout_local_filter_info_server',
            ],
        }],
    ))

    return ld
