"""Safe DG-202611 three-model and measured-UWB integration entry point.

Start the existing robot bringup separately so that /scan, /odom and TF are
available. This launch adds only the DG observation/fusion/recovery layer.
Command arbitration and UWB-to-map fusion stay disabled until explicitly
enabled after on-robot interface checks.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("ylhb_base")
    workspace_dir = os.environ.get("WS_DIR", os.path.expanduser("~/ros2_DL"))
    preferred_map = os.path.join(workspace_dir, "maps", "my_map.yaml")
    fallback_map = os.path.join(workspace_dir, "src", "my_map.yaml")
    default_map = preferred_map if os.path.exists(preferred_map) else fallback_map
    default_uwb_params = os.path.join(package_share, "config", "uwb_bringup_temporary.yaml")
    default_uwb_replay = os.path.join(
        package_share, "data", "uwb", "20260905", "static_raw_20260905_165208.csv"
    )

    enable_nav2 = LaunchConfiguration("enable_nav2")
    enable_cmd_vel_arbiter = LaunchConfiguration("enable_cmd_vel_arbiter")
    enable_multisource_fusion = LaunchConfiguration("enable_multisource_fusion")
    enable_scan_map = LaunchConfiguration("enable_scan_map")
    enable_uwb_replay = LaunchConfiguration("enable_uwb_replay")
    enable_event_camera_replay = LaunchConfiguration("enable_event_camera_replay")
    enable_event_feature_processing = LaunchConfiguration("enable_event_feature_processing")
    enable_hybrid_vision_assist = LaunchConfiguration("enable_hybrid_vision_assist")
    event_camera_intrinsics_valid = LaunchConfiguration("event_camera_intrinsics_valid")
    event_camera_lidar_extrinsics_valid = LaunchConfiguration(
        "event_camera_lidar_extrinsics_valid"
    )
    event_camera_motion_compensation_ready = LaunchConfiguration(
        "event_camera_motion_compensation_ready"
    )
    enable_auxiliary_transport_monitor = LaunchConfiguration("enable_auxiliary_transport_monitor")
    enable_mmwave_replay = LaunchConfiguration("enable_mmwave_replay")
    enable_mmwave_observation = LaunchConfiguration("enable_mmwave_observation")
    enable_mmwave_dynamic_mask = LaunchConfiguration("enable_mmwave_dynamic_mask")
    enable_relative_height = LaunchConfiguration("enable_relative_height")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    uwb_params_file = LaunchConfiguration("uwb_params_file")
    uwb_replay_file = LaunchConfiguration("uwb_replay_file")
    uwb_replay_rate_hz = LaunchConfiguration("uwb_replay_rate_hz")
    uwb_replay_loop = LaunchConfiguration("uwb_replay_loop")
    event_camera_input_file = LaunchConfiguration("event_camera_input_file")
    event_camera_replay_rate_hz = LaunchConfiguration("event_camera_replay_rate_hz")
    event_camera_window_us = LaunchConfiguration("event_camera_window_us")
    event_feature_match_distance_px = LaunchConfiguration("event_feature_match_distance_px")
    mmwave_input_file = LaunchConfiguration("mmwave_input_file")
    mmwave_replay_rate_hz = LaunchConfiguration("mmwave_replay_rate_hz")
    mmwave_expected_frame_id = LaunchConfiguration("mmwave_expected_frame_id")
    mmwave_timeout_sec = LaunchConfiguration("mmwave_timeout_sec")
    mmwave_to_lidar_x = LaunchConfiguration("mmwave_to_lidar_x")
    mmwave_to_lidar_y = LaunchConfiguration("mmwave_to_lidar_y")
    mmwave_to_lidar_yaw = LaunchConfiguration("mmwave_to_lidar_yaw")
    mmwave_min_abs_velocity = LaunchConfiguration("mmwave_min_abs_velocity")
    mmwave_min_snr = LaunchConfiguration("mmwave_min_snr")
    mmwave_min_confidence = LaunchConfiguration("mmwave_min_confidence")
    mmwave_angular_half_width = LaunchConfiguration("mmwave_angular_half_width")
    mmwave_radial_margin = LaunchConfiguration("mmwave_radial_margin")
    relative_height_odom_topic = LaunchConfiguration("relative_height_odom_topic")
    uwb_alignment_enabled = LaunchConfiguration("uwb_alignment_enabled")
    uwb_to_map_yaw = LaunchConfiguration("uwb_to_map_yaw")
    uwb_to_map_offset_x = LaunchConfiguration("uwb_to_map_offset_x")
    uwb_to_map_offset_y = LaunchConfiguration("uwb_to_map_offset_y")
    required_signals = LaunchConfiguration("required_signals")
    post_recovery_stable_sec = LaunchConfiguration("post_recovery_stable_sec")
    mission_gate_enabled = LaunchConfiguration("mission_gate_enabled")
    mission_status_timeout_sec = LaunchConfiguration("mission_status_timeout_sec")
    mission_activation_grace_sec = LaunchConfiguration("mission_activation_grace_sec")
    cmd_vel_output_topic = LaunchConfiguration("cmd_vel_output_topic")
    gnss_origin_latitude = LaunchConfiguration("gnss_origin_latitude")
    gnss_origin_longitude = LaunchConfiguration("gnss_origin_longitude")
    gnss_origin_altitude = LaunchConfiguration("gnss_origin_altitude")
    map_enu_yaw = LaunchConfiguration("map_enu_yaw")
    map_enu_offset_x = LaunchConfiguration("map_enu_offset_x")
    map_enu_offset_y = LaunchConfiguration("map_enu_offset_y")

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_share, "launch", "navigation.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": map_file,
            "params_file": params_file,
            "autostart": "true",
        }.items(),
    )

    lidar = Node(
        package="ylhb_base",
        executable="lidar_robust_node",
        name="dg_lidar_robust_node",
        output="screen",
        parameters=[
            {
                "scan_topic": "/scan",
                "odom_topic": "/odom",
                "filtered_scan_topic": "/dg/lidar/filtered_scan",
                "stable_features_topic": "/dg/lidar/stable_features",
                "quality_topic": "/dg/lidar/quality",
                "event_assist_enabled": ParameterValue(
                    enable_hybrid_vision_assist, value_type=bool
                ),
                "event_assist_quality_topic": "/dg/event_camera/assist_quality",
                "mmwave_dynamic_mask_enabled": ParameterValue(
                    enable_mmwave_dynamic_mask, value_type=bool
                ),
                "mmwave_observation_enabled": ParameterValue(
                    enable_mmwave_observation, value_type=bool
                ),
                "mmwave_points_topic": "/dg/mmwave/points",
                "mmwave_expected_frame_id": mmwave_expected_frame_id,
                "mmwave_timeout_sec": ParameterValue(mmwave_timeout_sec, value_type=float),
                "mmwave_to_lidar_x": ParameterValue(mmwave_to_lidar_x, value_type=float),
                "mmwave_to_lidar_y": ParameterValue(mmwave_to_lidar_y, value_type=float),
                "mmwave_to_lidar_yaw": ParameterValue(mmwave_to_lidar_yaw, value_type=float),
                "mmwave_min_abs_velocity": ParameterValue(
                    mmwave_min_abs_velocity, value_type=float
                ),
                "mmwave_min_snr": ParameterValue(mmwave_min_snr, value_type=float),
                "mmwave_min_confidence": ParameterValue(
                    mmwave_min_confidence, value_type=float
                ),
                "mmwave_angular_half_width": ParameterValue(
                    mmwave_angular_half_width, value_type=float
                ),
                "mmwave_radial_margin": ParameterValue(
                    mmwave_radial_margin, value_type=float
                ),
            }
        ],
    )
    gnss = Node(
        package="ylhb_base",
        executable="gnss_quality_node",
        name="dg_gnss_quality_node",
        output="screen",
        parameters=[
            {
                "fix_topic": "/gps/fix",
                "status_topic": "/gps/rtk_status",
                "quality_topic": "/dg/gnss/quality",
            }
        ],
    )
    uwb_replay = Node(
        package="ylhb_base",
        executable="uwb_replay_node",
        name="dg_uwb_replay_node",
        output="screen",
        condition=IfCondition(enable_uwb_replay),
        parameters=[
            uwb_params_file,
            {
                "input_file": uwb_replay_file,
                "playback_rate_hz": ParameterValue(uwb_replay_rate_hz, value_type=float),
                "loop": ParameterValue(uwb_replay_loop, value_type=bool),
            },
        ],
    )
    event_camera_replay = Node(
        package="ylhb_base",
        executable="auxiliary_sensor_replay_node",
        name="dg_event_camera_replay_node",
        output="screen",
        condition=IfCondition(enable_event_camera_replay),
        parameters=[
            {
                "sensor_kind": "event_camera",
                "input_file": event_camera_input_file,
                "replay_rate_hz": ParameterValue(
                    event_camera_replay_rate_hz, value_type=float
                ),
                "event_window_us": ParameterValue(event_camera_window_us, value_type=int),
                "frame_id": "event_camera_optical_frame",
                "data_topic": "/dg/event_camera/events",
                "quality_topic": "/dg/event_camera/quality",
            }
        ],
    )
    event_feature = Node(
        package="ylhb_base",
        executable="event_feature_node",
        name="dg_event_feature_node",
        output="screen",
        condition=IfCondition(enable_event_feature_processing),
        parameters=[
            {
                "events_topic": "/dg/event_camera/events",
                "features_topic": "/dg/event_camera/features",
                "quality_topic": "/dg/event_camera/feature_quality",
                "expected_frame_id": "event_camera_optical_frame",
                "match_distance_px": ParameterValue(
                    event_feature_match_distance_px, value_type=float
                ),
            }
        ],
    )
    hybrid_vision_assist = Node(
        package="ylhb_base",
        executable="hybrid_vision_assist_node",
        name="dg_hybrid_vision_assist_node",
        output="screen",
        condition=IfCondition(enable_hybrid_vision_assist),
        parameters=[
            {
                "camera_quality_topic": "/dg/event_camera/quality",
                "intrinsics_valid": ParameterValue(event_camera_intrinsics_valid, value_type=bool),
                "event_features_topic": "/dg/event_camera/features",
                "motion_mask_topic": "/dg/event_camera/motion_mask",
                "camera_info_topic": "/dg/event_camera/camera_info",
                "odom_topic": "/odom",
                "imu_topic": "/imu/data",
                "assist_quality_topic": "/dg/event_camera/assist_quality",
                "lidar_extrinsics_valid": ParameterValue(
                    event_camera_lidar_extrinsics_valid, value_type=bool
                ),
                "motion_compensation_ready": ParameterValue(
                    event_camera_motion_compensation_ready, value_type=bool
                ),
            }
        ],
    )
    auxiliary_transport_monitor = Node(
        package="ylhb_base",
        executable="auxiliary_transport_monitor_node",
        name="dg_auxiliary_transport_monitor_node",
        output="screen",
        condition=IfCondition(enable_auxiliary_transport_monitor),
        parameters=[
            {
                "event_quality_topic": "/dg/event_camera/quality",
                "event_features_topic": "/dg/event_camera/features",
                "event_mask_topic": "/dg/event_camera/motion_mask",
                "mmwave_points_topic": "/dg/mmwave/points",
                "mmwave_quality_topic": "/dg/mmwave/quality",
                "output_topic": "/dg/auxiliary/transport_quality",
            }
        ],
    )
    mmwave_replay = Node(
        package="ylhb_base",
        executable="auxiliary_sensor_replay_node",
        name="dg_mmwave_replay_node",
        output="screen",
        condition=IfCondition(enable_mmwave_replay),
        parameters=[
            {
                "sensor_kind": "mmwave",
                "input_file": mmwave_input_file,
                "replay_rate_hz": ParameterValue(mmwave_replay_rate_hz, value_type=float),
                "frame_id": "mmwave_link",
                "data_topic": "/dg/mmwave/points",
                "quality_topic": "/dg/mmwave/quality",
            }
        ],
    )
    scan_map = Node(
        package="ylhb_base",
        executable="scan_map_relocalization_node",
        name="dg_scan_map_relocalization_node",
        output="screen",
        condition=IfCondition(enable_scan_map),
        parameters=[
            {
                "map_topic": "/map",
                "scan_topic": "/dg/lidar/filtered_scan",
                "initialpose_topic": "/initialpose",
                "seed_topic": "/dg/relocalization/seed",
                "scan_match_pose_topic": "/scan_match_pose",
                "match_quality_topic": "/dg/relocalization/match_quality",
            }
        ],
    )
    relocalization = Node(
        package="ylhb_base",
        executable="active_relocalization_node",
        name="dg_active_relocalization_node",
        output="screen",
        parameters=[
            {
                "scan_topic": "/dg/lidar/filtered_scan",
                "odom_topic": "/odom",
                "imu_topic": "/imu/data",
                "amcl_pose_topic": "/amcl_pose",
                "lidar_quality_topic": "/dg/lidar/quality",
                "gnss_quality_topic": "/dg/gnss/quality",
                "match_quality_topic": "/dg/relocalization/match_quality",
                "cmd_vel_topic": "/cmd_vel_recovery",
                "required_signals": required_signals,
                "system_status_topic": "/inspection_ai/system_status",
                "mission_gate_enabled": ParameterValue(mission_gate_enabled, value_type=bool),
                "mission_status_timeout_sec": ParameterValue(mission_status_timeout_sec, value_type=float),
                "mission_activation_grace_sec": ParameterValue(mission_activation_grace_sec, value_type=float),
                "recovery_motion_armed": ParameterValue(enable_cmd_vel_arbiter, value_type=bool),
            }
        ],
    )
    health = Node(
        package="ylhb_base",
        executable="navigation_health_node",
        name="dg_navigation_health_node",
        output="screen",
        parameters=[
            {
                "required_signals": required_signals,
                "uwb_quality_topic": "/dg/uwb/quality",
                "odom_topic": "/odom",
                "scan_topic": "/scan",
                "imu_topic": "/imu/data",
                "status_topic": "/dg/navigation/status",
                "post_recovery_stable_sec": ParameterValue(
                    post_recovery_stable_sec, value_type=float
                ),
                "system_status_topic": "/inspection_ai/system_status",
                "mission_gate_enabled": ParameterValue(mission_gate_enabled, value_type=bool),
                "mission_status_timeout_sec": ParameterValue(mission_status_timeout_sec, value_type=float),
                "mission_activation_grace_sec": ParameterValue(mission_activation_grace_sec, value_type=float),
                "recovery_motion_armed": ParameterValue(enable_cmd_vel_arbiter, value_type=bool),
            }
        ],
    )
    arbiter = Node(
        package="ylhb_base",
        executable="cmd_vel_arbiter_node",
        name="dg_cmd_vel_arbiter_node",
        output="screen",
        condition=IfCondition(enable_cmd_vel_arbiter),
        parameters=[
            {
                "navigation_topic": "/cmd_vel_nav",
                "recovery_topic": "/cmd_vel_recovery",
                "manual_command_topic": "/cmd_vel_manual",
                "manual_authorization_topic": "/dg/control/manual_authorized",
                "output_topic": cmd_vel_output_topic,
                "navigation_status_topic": "/dg/navigation/status",
            }
        ],
    )
    multisource_fusion = Node(
        package="ylhb_base",
        executable="multisource_fusion_node",
        name="dg_multisource_fusion_node",
        output="screen",
        condition=IfCondition(enable_multisource_fusion),
        parameters=[
            uwb_params_file,
            {
                "local_odom_topic": "/odom",
                "gnss_fix_topic": "/dg/gnss/accepted_fix",
                "gnss_quality_topic": "/dg/gnss/quality",
                "lidar_quality_topic": "/dg/lidar/quality",
                "amcl_pose_topic": "/amcl_pose",
                "scan_match_pose_topic": "/scan_match_pose",
                "match_quality_topic": "/dg/relocalization/match_quality",
                "uwb_pose_topic": "/dg/uwb/pose",
                "uwb_quality_topic": "/dg/uwb/quality",
                "uwb_enabled": ParameterValue(enable_uwb_replay, value_type=bool),
                "uwb_alignment_enabled": ParameterValue(uwb_alignment_enabled, value_type=bool),
                "uwb_to_map_yaw": ParameterValue(uwb_to_map_yaw, value_type=float),
                "uwb_to_map_offset_x": ParameterValue(uwb_to_map_offset_x, value_type=float),
                "uwb_to_map_offset_y": ParameterValue(uwb_to_map_offset_y, value_type=float),
                "relative_height_enabled": ParameterValue(enable_relative_height, value_type=bool),
                "relative_height_odom_topic": relative_height_odom_topic,
                "fusion_odom_topic": "/dg/fusion/odom",
                "fusion_pose_topic": "/dg/fusion/pose",
                "fusion_status_topic": "/dg/fusion/status",
                "publish_tf": False,
                "gnss_origin_latitude": ParameterValue(gnss_origin_latitude, value_type=float),
                "gnss_origin_longitude": ParameterValue(gnss_origin_longitude, value_type=float),
                "gnss_origin_altitude": ParameterValue(gnss_origin_altitude, value_type=float),
                "map_enu_yaw": ParameterValue(map_enu_yaw, value_type=float),
                "map_enu_offset_x": ParameterValue(map_enu_offset_x, value_type=float),
                "map_enu_offset_y": ParameterValue(map_enu_offset_y, value_type=float),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_nav2", default_value="false", description="Start existing Nav2 launch"),
            DeclareLaunchArgument(
                "enable_cmd_vel_arbiter",
                default_value="false",
                description="Allow DG command output; disabled for observation-only startup",
            ),
            DeclareLaunchArgument(
                "enable_multisource_fusion",
                default_value="true",
                description="Start the DG side-channel multi-source fusion layer",
            ),
            DeclareLaunchArgument(
                "enable_scan_map",
                default_value="true",
                description="Start the real scan-to-map candidate producer",
            ),
            DeclareLaunchArgument(
                "enable_uwb_replay",
                default_value="false",
                description="Replay the saved measured UWB stream into MODEL-1",
            ),
            DeclareLaunchArgument("uwb_replay_file", default_value=default_uwb_replay),
            DeclareLaunchArgument("uwb_replay_rate_hz", default_value="25.0"),
            DeclareLaunchArgument("uwb_replay_loop", default_value="false"),
            DeclareLaunchArgument(
                "enable_event_camera_replay",
                default_value="false",
                description="Replay measured Shimeta event CSV; observation only",
            ),
            DeclareLaunchArgument("event_camera_input_file", default_value=""),
            DeclareLaunchArgument("event_camera_replay_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("event_camera_window_us", default_value="20000"),
            DeclareLaunchArgument(
                "enable_event_feature_processing",
                default_value="false",
                description="Extract event-image features and adjacent-packet repeatability evidence",
            ),
            DeclareLaunchArgument("event_feature_match_distance_px", default_value="12.0"),
            DeclareLaunchArgument(
                "enable_hybrid_vision_assist",
                default_value="false",
                description="Feed live HVS diagnostics to MODEL-2 without bypassing calibration gates",
            ),
            DeclareLaunchArgument(
                "event_camera_intrinsics_valid",
                default_value="false",
                description="Reserved evidence flag; CameraInfo values remain the source of truth",
            ),
            DeclareLaunchArgument(
                "event_camera_lidar_extrinsics_valid", default_value="false"
            ),
            DeclareLaunchArgument(
                "event_camera_motion_compensation_ready", default_value="false"
            ),
            DeclareLaunchArgument(
                "enable_auxiliary_transport_monitor",
                default_value="false",
                description="Record RDK-to-Jetson auxiliary topic receipt without changing robot control",
            ),
            DeclareLaunchArgument(
                "enable_mmwave_replay",
                default_value="false",
                description="Replay vendor-normalized measured mmWave CSV; observation only",
            ),
            DeclareLaunchArgument("mmwave_input_file", default_value=""),
            DeclareLaunchArgument("mmwave_replay_rate_hz", default_value="20.0"),
            DeclareLaunchArgument(
                "enable_mmwave_dynamic_mask",
                default_value="false",
                description="Use fresh normalized mmWave detections to mask dynamic LiDAR sectors",
            ),
            DeclareLaunchArgument(
                "enable_mmwave_observation",
                default_value="false",
                description="Subscribe to normalized mmWave detections and report receipt without changing LiDAR",
            ),
            DeclareLaunchArgument("mmwave_expected_frame_id", default_value="mmwave_link"),
            DeclareLaunchArgument("mmwave_timeout_sec", default_value="0.25"),
            DeclareLaunchArgument("mmwave_to_lidar_x", default_value="0.0"),
            DeclareLaunchArgument("mmwave_to_lidar_y", default_value="0.0"),
            DeclareLaunchArgument("mmwave_to_lidar_yaw", default_value="0.0"),
            DeclareLaunchArgument("mmwave_min_abs_velocity", default_value="0.15"),
            DeclareLaunchArgument("mmwave_min_snr", default_value="5.0"),
            DeclareLaunchArgument("mmwave_min_confidence", default_value="0.5"),
            DeclareLaunchArgument("mmwave_angular_half_width", default_value="0.08"),
            DeclareLaunchArgument("mmwave_radial_margin", default_value="0.35"),
            DeclareLaunchArgument(
                "enable_relative_height",
                default_value="false",
                description="Use gravity-aligned ZED visual odometry as MODEL-1 relative height input",
            ),
            DeclareLaunchArgument(
                "relative_height_odom_topic",
                default_value="/zed/zed_node/odom",
            ),
            DeclareLaunchArgument("uwb_params_file", default_value=default_uwb_params),
            DeclareLaunchArgument("uwb_alignment_enabled", default_value="false"),
            DeclareLaunchArgument("uwb_to_map_yaw", default_value="0.0"),
            DeclareLaunchArgument("uwb_to_map_offset_x", default_value="0.0"),
            DeclareLaunchArgument("uwb_to_map_offset_y", default_value="0.0"),
            DeclareLaunchArgument("gnss_origin_latitude", default_value="nan"),
            DeclareLaunchArgument("gnss_origin_longitude", default_value="nan"),
            DeclareLaunchArgument("gnss_origin_altitude", default_value="0.0"),
            DeclareLaunchArgument("map_enu_yaw", default_value="nan"),
            DeclareLaunchArgument("map_enu_offset_x", default_value="0.0"),
            DeclareLaunchArgument("map_enu_offset_y", default_value="0.0"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(package_share, "config", "nav2_params.yaml"),
            ),
            DeclareLaunchArgument(
                "required_signals",
                default_value="['']",
                description="Optional required health signals, for example ['lidar']",
            ),
            DeclareLaunchArgument(
                "post_recovery_stable_sec",
                default_value="1.0",
                description="Continuous healthy time before recovery hands control back to navigation",
            ),
            DeclareLaunchArgument(
                "mission_gate_enabled",
                default_value="true",
                description="Gate MODEL-3 and navigation health by live patrol lifecycle",
            ),
            DeclareLaunchArgument(
                "mission_status_timeout_sec",
                default_value="3.0",
                description="Maximum age of /inspection_ai/system_status before safe standby",
            ),
            DeclareLaunchArgument(
                "mission_activation_grace_sec",
                default_value="2.0",
                description="Continuous ready time before active recovery supervision",
            ),
            DeclareLaunchArgument(
                "cmd_vel_output_topic",
                default_value="/dg/test_cmd_vel",
                description="Set /cmd_vel only during supervised robot motion validation",
            ),
            GroupAction(
                [SetRemap(src="/cmd_vel", dst="/cmd_vel_nav"), nav2_launch],
                condition=IfCondition(enable_nav2),
            ),
            lidar,
            gnss,
            uwb_replay,
            event_camera_replay,
            event_feature,
            hybrid_vision_assist,
            mmwave_replay,
            auxiliary_transport_monitor,
            scan_map,
            relocalization,
            health,
            arbiter,
            multisource_fusion,
        ]
    )
