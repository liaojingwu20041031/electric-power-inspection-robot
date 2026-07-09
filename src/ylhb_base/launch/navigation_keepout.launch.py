import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from nav2_common.launch import RewrittenYaml


def warn_if_mask_missing(context):
    mask_path = LaunchConfiguration("keepout_mask").perform(context)
    if os.path.exists(os.path.expanduser(mask_path)):
        return []
    return [
        LogInfo(
            msg=(
                f"ERROR: keepout mask {mask_path} does not exist. "
                "Run scripts/generate_keepout_mask.py first."
            )
        )
    ]


def generate_launch_description():
    pkg_dir = get_package_share_directory("ylhb_base")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    workspace_dir = os.environ.get("WS_DIR", os.path.expanduser("~/ros2_DL"))

    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    keepout_mask = LaunchConfiguration("keepout_mask")
    enable_local_keepout = LaunchConfiguration("enable_local_keepout")

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            "local_costmap.local_costmap.ros__parameters.keepout_filter.enabled": enable_local_keepout,
        },
        convert_types=True,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(workspace_dir, "maps", "my_map.yaml"),
                description="Full path to map yaml file to load",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(pkg_dir, "config", "nav2_params_keepout.yaml"),
                description="Nav2 params with keepout filter configured",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "keepout_mask",
                default_value="/tmp/keepout_mask_power_room_a.yaml",
            ),
            DeclareLaunchArgument("enable_local_keepout", default_value="false"),
            OpaqueFunction(function=warn_if_mask_missing),
            LogInfo(
                msg=(
                    "INFO: keepout navigation starts global keepout first. "
                    "Set enable_local_keepout:=true only after low-speed robot validation."
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "map": map_yaml_file,
                    "params_file": configured_params,
                }.items(),
            ),
            LifecycleNode(
                package="nav2_map_server",
                executable="map_server",
                name="keepout_filter_mask_server",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "yaml_filename": keepout_mask,
                        "topic_name": "keepout_filter_mask",
                    }
                ],
            ),
            LifecycleNode(
                package="nav2_map_server",
                executable="costmap_filter_info_server",
                name="costmap_filter_info_server",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "filter_info_topic": "keepout_costmap_filter_info",
                        "type": 0,
                        "mask_topic": "keepout_filter_mask",
                        "base": 0.0,
                        "multiplier": 1.0,
                    }
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_keepout",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": [
                            "keepout_filter_mask_server",
                            "costmap_filter_info_server",
                        ],
                    }
                ],
            ),
        ]
    )
