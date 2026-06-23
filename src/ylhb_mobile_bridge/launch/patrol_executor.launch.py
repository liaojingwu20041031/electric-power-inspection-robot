from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    route_file_path = LaunchConfiguration("route_file_path")
    auto_start = LaunchConfiguration("auto_start")
    publish_initial_pose = LaunchConfiguration(
        "publish_initial_pose_on_startup"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "route_file_path",
                default_value="auto",
            ),
            DeclareLaunchArgument("auto_start", default_value="false"),
            DeclareLaunchArgument(
                "publish_initial_pose_on_startup",
                default_value="true",
            ),
            Node(
                package="ylhb_mobile_bridge",
                executable="patrol_executor_node",
                name="patrol_executor",
                output="screen",
                parameters=[
                    {
                        "route_file_path": route_file_path,
                        "auto_start": ParameterValue(
                            auto_start,
                            value_type=bool,
                        ),
                        "publish_initial_pose_on_startup": ParameterValue(
                            publish_initial_pose,
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
