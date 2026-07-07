import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('ylhb_3d_mapping')
    config_file = os.path.join(pkg_dir, 'config', 'zed_spatial_mapping.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('output_root', default_value='/home/nvidia/ros2_DL/runs/3d_mapping'),
        DeclareLaunchArgument('map_type', default_value='fused_point_cloud'),
        DeclareLaunchArgument('resolution_preset', default_value='high'),
        DeclareLaunchArgument('range_preset', default_value='near'),
        DeclareLaunchArgument('save_texture', default_value='false'),
        DeclareLaunchArgument('publish_rate_hz', default_value='2.0'),
        DeclareLaunchArgument('preview_rate_hz', default_value='1.0'),
        DeclareLaunchArgument('preview_frame_id', default_value='zed_3d_map'),
        DeclareLaunchArgument('preview_max_points', default_value='1000000'),
        DeclareLaunchArgument('camera_resolution', default_value='HD720'),
        DeclareLaunchArgument('camera_fps', default_value='15'),
        DeclareLaunchArgument('depth_mode', default_value='NEURAL'),
        DeclareLaunchArgument('depth_minimum_distance', default_value='0.25'),
        DeclareLaunchArgument('depth_maximum_distance', default_value='4.0'),
        DeclareLaunchArgument('confidence_threshold', default_value='60'),
        DeclareLaunchArgument('texture_confidence_threshold', default_value='60'),
        DeclareLaunchArgument('spatial_mapping_max_memory_mb', default_value='1024'),
        DeclareLaunchArgument('mesh_filter_preset', default_value='high'),
        DeclareLaunchArgument('max_duration_sec', default_value='0.0'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('auto_export_on_shutdown', default_value='true'),
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
                    'save_texture': ParameterValue(LaunchConfiguration('save_texture'), value_type=bool),
                    'publish_rate_hz': ParameterValue(LaunchConfiguration('publish_rate_hz'), value_type=float),
                    'preview_rate_hz': ParameterValue(LaunchConfiguration('preview_rate_hz'), value_type=float),
                    'preview_frame_id': LaunchConfiguration('preview_frame_id'),
                    'preview_max_points': ParameterValue(LaunchConfiguration('preview_max_points'), value_type=int),
                    'camera_resolution': LaunchConfiguration('camera_resolution'),
                    'camera_fps': ParameterValue(LaunchConfiguration('camera_fps'), value_type=int),
                    'depth_mode': LaunchConfiguration('depth_mode'),
                    'depth_minimum_distance': ParameterValue(
                        LaunchConfiguration('depth_minimum_distance'),
                        value_type=float,
                    ),
                    'depth_maximum_distance': ParameterValue(
                        LaunchConfiguration('depth_maximum_distance'),
                        value_type=float,
                    ),
                    'confidence_threshold': ParameterValue(
                        LaunchConfiguration('confidence_threshold'),
                        value_type=int,
                    ),
                    'texture_confidence_threshold': ParameterValue(
                        LaunchConfiguration('texture_confidence_threshold'),
                        value_type=int,
                    ),
                    'spatial_mapping_max_memory_mb': ParameterValue(
                        LaunchConfiguration('spatial_mapping_max_memory_mb'),
                        value_type=int,
                    ),
                    'mesh_filter_preset': LaunchConfiguration('mesh_filter_preset'),
                    'max_duration_sec': ParameterValue(LaunchConfiguration('max_duration_sec'), value_type=float),
                    'auto_start': ParameterValue(LaunchConfiguration('auto_start'), value_type=bool),
                    'auto_export_on_shutdown': ParameterValue(
                        LaunchConfiguration('auto_export_on_shutdown'),
                        value_type=bool,
                    ),
                },
            ],
        ),
    ])
