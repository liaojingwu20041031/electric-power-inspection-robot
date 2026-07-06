import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ylhb_3d_mapping')
    config_file = os.path.join(pkg_dir, 'config', 'zed_spatial_mapping.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('output_root', default_value='/home/nvidia/ros2_DL/runs/3d_mapping'),
        DeclareLaunchArgument('map_type', default_value='fused_point_cloud'),
        DeclareLaunchArgument('resolution_preset', default_value='medium'),
        DeclareLaunchArgument('range_preset', default_value='medium'),
        DeclareLaunchArgument('save_texture', default_value='false'),
        Node(
            package='ylhb_3d_mapping',
            executable='zed_spatial_mapping_node',
            name='zed_spatial_mapping_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'output_root': LaunchConfiguration('output_root'),
                    'map_type': LaunchConfiguration('map_type'),
                    'resolution_preset': LaunchConfiguration('resolution_preset'),
                    'range_preset': LaunchConfiguration('range_preset'),
                    'save_texture': LaunchConfiguration('save_texture'),
                },
            ],
        ),
    ])
