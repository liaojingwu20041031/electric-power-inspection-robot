from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('ylhb_mobile_bridge')
    config_path = os.path.join(package_dir, 'config', 'mobile_bridge.yaml')

    return LaunchDescription([
        Node(
            package='ylhb_mobile_bridge',
            executable='mobile_bridge_server',
            name='mobile_bridge',
            output='screen',
            parameters=[config_path],
        )
    ])
