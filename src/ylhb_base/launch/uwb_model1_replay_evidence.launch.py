"""Isolated, motion-free UWB replay proving MODEL-1 input consumption.

The identity transform is intentionally applied in a dedicated
``uwb_replay_test`` frame.  It proves the software consumption path only and
must not be interpreted as a surveyed UWB-to-map alignment.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("ylhb_base")
    default_input = os.path.join(
        package_share,
        "data",
        "uwb",
        "20260905",
        "static_raw_20260905_165208.csv",
    )
    default_params = os.path.join(
        package_share,
        "config",
        "uwb_bringup_temporary.yaml",
    )

    input_file = LaunchConfiguration("input_file")
    params_file = LaunchConfiguration("uwb_params_file")
    playback_rate_hz = LaunchConfiguration("playback_rate_hz")
    loop = LaunchConfiguration("loop")
    output_frame_id = LaunchConfiguration("output_frame_id")
    output_child_frame_id = LaunchConfiguration("output_child_frame_id")

    replay = Node(
        package="ylhb_base",
        executable="uwb_replay_node",
        name="dg_uwb_evidence_replay_node",
        output="screen",
        parameters=[
            params_file,
            {
                "input_file": input_file,
                "playback_rate_hz": ParameterValue(playback_rate_hz, value_type=float),
                "loop": ParameterValue(loop, value_type=bool),
            },
        ],
    )

    fusion = Node(
        package="ylhb_base",
        executable="multisource_fusion_node",
        name="dg_uwb_evidence_fusion_node",
        output="screen",
        parameters=[
            params_file,
            {
                # Deliberately unused inputs isolate UWB from live map/odom
                # sources. No publisher is created for any of these names.
                "local_odom_topic": "/dg/evidence/no_local_odom",
                "gnss_fix_topic": "/dg/evidence/no_gnss_fix",
                "gnss_quality_topic": "/dg/evidence/no_gnss_quality",
                "lidar_quality_topic": "/dg/evidence/no_lidar_quality",
                "amcl_pose_topic": "/dg/evidence/no_amcl_pose",
                "scan_match_pose_topic": "/dg/evidence/no_scan_match_pose",
                "match_quality_topic": "/dg/evidence/no_match_quality",
                "uwb_pose_topic": "/dg/uwb/pose",
                "uwb_quality_topic": "/dg/uwb/quality",
                "uwb_enabled": True,
                "uwb_alignment_enabled": True,
                "uwb_to_map_yaw": 0.0,
                "uwb_to_map_offset_x": 0.0,
                "uwb_to_map_offset_y": 0.0,
                "output_frame_id": output_frame_id,
                "output_child_frame_id": output_child_frame_id,
                "fusion_odom_topic": "/dg/fusion/odom",
                "fusion_pose_topic": "/dg/fusion/pose",
                "fusion_status_topic": "/dg/fusion/status",
                "publish_tf": False,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("input_file", default_value=default_input),
            DeclareLaunchArgument("uwb_params_file", default_value=default_params),
            DeclareLaunchArgument("playback_rate_hz", default_value="25.0"),
            DeclareLaunchArgument("loop", default_value="false"),
            DeclareLaunchArgument("output_frame_id", default_value="uwb_replay_test"),
            DeclareLaunchArgument(
                "output_child_frame_id",
                default_value="uwb_replay_tag_test",
            ),
            replay,
            fusion,
        ]
    )
