from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _bridge_executable():
    share_dir = Path(get_package_share_directory("dg_hvs_bridge"))
    executable = share_dir.parents[1] / "lib" / "dg_hvs_bridge" / "dg_hvs_bridge_node"
    if not executable.exists():
        raise RuntimeError(
            "dg_hvs_bridge_node is unavailable in this build. "
            "Build dg_hvs_bridge on RDK X5 with HV_TOOLKIT_ROOT set to the "
            "complete shimetapi_hybrid_vision_toolkit tree."
        )
    return "dg_hvs_bridge_node"


def generate_launch_description():
    default_params = str(
        Path(get_package_share_directory("dg_hvs_bridge"))
        / "config"
        / "rdk_x5_apx003cc.yaml"
    )
    params_file = LaunchConfiguration("params_file")
    recording_enabled = LaunchConfiguration("recording_enabled")
    record_directory = LaunchConfiguration("record_directory")
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("recording_enabled", default_value="false"),
            DeclareLaunchArgument(
                "record_directory", default_value=""
            ),
            Node(
                package="dg_hvs_bridge",
                executable=_bridge_executable(),
                name="dg_hvs_bridge_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "recording_enabled": ParameterValue(
                            recording_enabled, value_type=bool
                        ),
                        "record_directory": record_directory,
                    },
                ],
            ),
        ]
    )
