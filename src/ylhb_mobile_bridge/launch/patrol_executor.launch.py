from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    route_file_path = LaunchConfiguration("route_file_path")
    route_directory = LaunchConfiguration("route_directory")
    execution_id = LaunchConfiguration("execution_id")
    deployment_id = LaunchConfiguration("deployment_id")
    platform_request_id = LaunchConfiguration("platform_request_id")
    platform_command_id = LaunchConfiguration("platform_command_id")
    auto_start = LaunchConfiguration("auto_start")
    publish_initial_pose = LaunchConfiguration(
        "publish_initial_pose_on_startup"
    )
    startup_id = LaunchConfiguration("startup_id")
    inspection_enabled = LaunchConfiguration("inspection_enabled")
    localized_objects_topic = LaunchConfiguration("localized_objects_topic")
    default_inspection_window_sec = LaunchConfiguration(
        "default_inspection_window_sec"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "route_file_path",
                default_value="auto",
            ),
            DeclareLaunchArgument("route_directory", default_value=""),
            DeclareLaunchArgument("execution_id", default_value=""),
            DeclareLaunchArgument("deployment_id", default_value=""),
            DeclareLaunchArgument("platform_request_id", default_value=""),
            DeclareLaunchArgument("platform_command_id", default_value=""),
            DeclareLaunchArgument("auto_start", default_value="false"),
            DeclareLaunchArgument(
                "publish_initial_pose_on_startup",
                default_value="true",
            ),
            DeclareLaunchArgument("startup_id", default_value=""),
            DeclareLaunchArgument("inspection_enabled", default_value="false"),
            DeclareLaunchArgument(
                "localized_objects_topic",
                default_value="/perception/localized_objects",
            ),
            DeclareLaunchArgument(
                "default_inspection_window_sec",
                default_value="5.0",
            ),
            Node(
                package="ylhb_mobile_bridge",
                executable="patrol_executor_node",
                name="patrol_executor",
                output="screen",
                parameters=[
                    {
                        "route_file_path": route_file_path,
                        "route_directory": route_directory,
                        "execution_id": execution_id,
                        "deployment_id": deployment_id,
                        "platform_request_id": platform_request_id,
                        "platform_command_id": platform_command_id,
                        "auto_start": ParameterValue(
                            auto_start,
                            value_type=bool,
                        ),
                        "publish_initial_pose_on_startup": ParameterValue(
                            publish_initial_pose,
                            value_type=bool,
                        ),
                        "startup_id": startup_id,
                        "inspection_enabled": ParameterValue(
                            inspection_enabled,
                            value_type=bool,
                        ),
                        "localized_objects_topic": localized_objects_topic,
                        "default_inspection_window_sec": ParameterValue(
                            default_inspection_window_sec,
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
