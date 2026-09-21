"""Optional standalone launch for the injector and evaluator nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scenario_file = LaunchConfiguration("scenario_file")
    output_dir = LaunchConfiguration("output_dir")
    return LaunchDescription([
        DeclareLaunchArgument("scenario_file"),
        DeclareLaunchArgument("output_dir"),
        Node(package="dg_synthetic_validation", executable="synthetic_injector_node", arguments=[scenario_file]),
        Node(package="dg_synthetic_validation", executable="synthetic_evaluator_node", arguments=[scenario_file, output_dir]),
    ])
