import ast
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
LAUNCH = ROOT / "launch" / "dg_navigation_integration.launch.py"
CMAKE = ROOT / "CMakeLists.txt"

if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from ros_shutdown import cleanup_ros_node  # noqa: E402

from auxiliary_sensor_data import (  # noqa: E402
    iter_event_packets,
    iter_mmwave_frames,
    summarize_event_packet,
    summarize_mmwave_frame,
)


class MultisourceFusionProtocolTests(unittest.TestCase):
    def test_new_python_modules_parse_without_ros_imports_at_module_scope(self) -> None:
        for name in (
            "multisource_fusion_core.py",
            "multisource_fusion_node.py",
            "auxiliary_sensor_data.py",
            "auxiliary_sensor_replay_node.py",
            "event_feature_core.py",
            "event_feature_node.py",
            "auxiliary_transport_monitor_node.py",
        ):
            ast.parse((SCRIPTS / name).read_text(encoding="utf-8"))

    def test_measured_auxiliary_sensor_csv_contracts(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            event_csv = root / "events.csv"
            event_csv.write_text(
                "x,y,polarity,timestamp\n10,20,1,1000\n11,20,0,2000\n12,21,1,22000\n",
                encoding="utf-8",
            )
            event_packets = list(iter_event_packets(event_csv, window_us=20_000))
            self.assertEqual([len(packet.samples) for packet in event_packets], [2, 1])
            event_summary = summarize_event_packet(event_packets[0])
            self.assertEqual(event_summary["event_count"], 2)
            self.assertEqual(event_summary["positive_fraction"], 0.5)

            mmwave_csv = root / "mmwave.csv"
            mmwave_csv.write_text(
                "timestamp_us,frame_id,track_id,x_m,y_m,z_m,radial_velocity_mps,snr_db,confidence\n"
                "1000,7,1,1.0,0.2,0.1,0.4,18.0,0.9\n"
                "1000,7,2,1.4,-0.2,0.1,0.0,12.0,0.7\n"
                "2000,8,1,0.9,0.2,0.1,-0.2,17.0,0.8\n",
                encoding="utf-8",
            )
            mmwave_frames = list(iter_mmwave_frames(mmwave_csv))
            self.assertEqual([frame.frame_id for frame in mmwave_frames], [7, 8])
            radar_summary = summarize_mmwave_frame(mmwave_frames[0])
            self.assertEqual(radar_summary["detection_count"], 2)
            self.assertEqual(radar_summary["moving_detection_count"], 1)

    def test_hvs_bridge_recording_matches_event_replay_csv_contract(self) -> None:
        bridge = (
            ROOT.parents[1]
            / "src"
            / "dg_hvs_bridge"
            / "src"
            / "dg_hvs_bridge_node.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn('csv_file_ << "x,y,polarity,timestamp\\n";', bridge)
        self.assertIn(
            "csv_file_ << event.x << ',' << event.y << ','",
            bridge,
        )
        self.assertIn(
            "<< (event.polarity ? 1 : 0) << ',' << event.t << '\\n';",
            bridge,
        )

    def test_hvs_bridge_publishes_feature_quality_with_repeatability_gate(self) -> None:
        bridge_root = ROOT.parents[1] / "src" / "dg_hvs_bridge"
        bridge = (bridge_root / "src" / "dg_hvs_bridge_node.cpp").read_text(encoding="utf-8")
        config = (bridge_root / "config" / "rdk_x5_apx003cc.yaml").read_text(encoding="utf-8")
        readme = (bridge_root / "README.md").read_text(encoding="utf-8")
        self.assertIn('"/dg/event_camera/feature_quality"', bridge)
        self.assertIn('"feature_min_unique_pixels"', bridge)
        self.assertIn('"feature_match_distance_px"', bridge)
        self.assertIn('"repeatability_status"', bridge)
        self.assertIn('"unique_pixel_count"', bridge)
        self.assertIn('"observation_only", "true"', bridge)
        self.assertIn('"publishes_cmd_vel", "false"', bridge)
        self.assertIn("feature_min_unique_pixels: 3", config)
        self.assertIn("feature_match_distance_px: 12.0", config)
        self.assertIn("ZERO_FEATURE_PAIR", readme)
        self.assertIn("NO_PREVIOUS_PACKET", readme)

    def test_transport_monitor_exposes_auditable_timing_fields(self) -> None:
        import auxiliary_transport_monitor_node as monitor

        snapshot = monitor._timing_snapshot(
            {
                "received_monotonic": 10.0,
                "received_wall_time": 1_700_000_000.25,
                "received_ros_time": 20.0,
                "source_stamp": 19.75,
            },
            now_monotonic=10.4,
            now_ros_time=20.4,
        )
        self.assertEqual(snapshot["header_stamp_sec"], 19.75)
        self.assertEqual(snapshot["received_wall_time_sec"], 1_700_000_000.25)
        self.assertEqual(snapshot["received_ros_time_sec"], 20.0)
        self.assertAlmostEqual(snapshot["age_sec"], 0.4)
        self.assertAlmostEqual(snapshot["header_age_sec"], 0.65)
        self.assertAlmostEqual(snapshot["header_to_receive_delta_sec"], 0.25)
        self.assertAlmostEqual(snapshot["estimated_latency_sec"], 0.25)

    def test_replay_source_kind_cannot_silently_label_fixture_as_measured(self) -> None:
        text = (SCRIPTS / "auxiliary_sensor_replay_node.py").read_text(encoding="utf-8")
        self.assertIn('self.declare_parameter("source_kind", "MEASURED_REPLAY")', text)
        self.assertIn('"SYNTHETIC_CONTRACT_FIXTURE"', text)
        self.assertIn('"source_kind": self._source_kind', text)
        self.assertIn('quality_state = "GOOD" if has_data else "FAULT"', text)
        self.assertIn('"data_valid": has_data', text)

    def test_runtime_nodes_tolerate_rclpy_external_shutdown(self) -> None:
        for name in (
            "active_relocalization_node.py",
            "gnss_quality_node.py",
            "navigation_health_node.py",
            "lidar_robust_node.py",
            "multisource_fusion_node.py",
            "scan_map_relocalization_node.py",
            "auxiliary_sensor_replay_node.py",
            "event_feature_node.py",
        ):
            text = (SCRIPTS / name).read_text(encoding="utf-8")
            self.assertIn("ExternalShutdownException", text, name)
            self.assertIn("cleanup_ros_node(node, rclpy)", text, name)

    def test_cleanup_tolerates_sigint_during_node_and_context_shutdown(self) -> None:
        calls: list[str] = []

        class InterruptingNode:
            def destroy_node(self) -> None:
                calls.append("destroy_node")
                raise KeyboardInterrupt

        class InterruptingRclpy:
            @staticmethod
            def try_shutdown() -> None:
                calls.append("try_shutdown")
                raise KeyboardInterrupt

        cleanup_ros_node(InterruptingNode(), InterruptingRclpy())
        self.assertEqual(calls, ["destroy_node", "try_shutdown"])

    def test_core_defines_required_modes_and_outputs(self) -> None:
        text = (SCRIPTS / "multisource_fusion_core.py").read_text(encoding="utf-8")
        for value in (
            "LOCAL_ODOM",
            "GNSS_POSITION",
            "LIDAR_AIDED",
            "INITIALIZING",
            "GNSS_AIDED",
            "DEAD_RECKONING",
            "REJECTED_UPDATE",
            "WAITING_ALIGNMENT",
            "position_uncertainty",
            "compute_gnss_confidence",
            "compute_lidar_confidence",
            "measurement_confidence",
            "adaptive_position_uncertainty",
            "adaptive_yaw_uncertainty",
        ):
            self.assertIn(value, text)

    def test_wrapper_uses_real_repository_topics(self) -> None:
        text = (SCRIPTS / "multisource_fusion_node.py").read_text(encoding="utf-8")
        for topic in (
            '"/odom"',
            '"/dg/gnss/accepted_fix"',
            '"/dg/gnss/quality"',
            '"/dg/lidar/quality"',
            '"/amcl_pose"',
            '"/scan_match_pose"',
            '"/dg/relocalization/match_quality"',
            '"/dg/fusion/odom"',
            '"/dg/fusion/pose"',
            '"/dg/fusion/status"',
            '"/zed/zed_node/odom"',
        ):
            self.assertIn(topic, text)

    def test_wrapper_does_not_publish_tf(self) -> None:
        text = (SCRIPTS / "multisource_fusion_node.py").read_text(encoding="utf-8")
        self.assertIn('self.declare_parameter("publish_tf", False)', text)
        self.assertNotIn("TransformBroadcaster", text)

    def test_wrapper_tracks_and_consumes_message_sequences(self) -> None:
        text = (SCRIPTS / "multisource_fusion_node.py").read_text(encoding="utf-8")
        for field in (
            "_odom_sequence",
            "_gnss_sequence",
            "_amcl_sequence",
            "_scan_match_sequence",
            "_odom_consumed_sequence",
            "_gnss_consumed_sequence",
            "_amcl_consumed_sequence",
            "_scan_match_consumed_sequence",
        ):
            self.assertIn(field, text)

    def test_launch_has_optional_multisource_fusion_switch(self) -> None:
        text = LAUNCH.read_text(encoding="utf-8")
        self.assertIn('"enable_multisource_fusion"', text)
        self.assertIn('default_value="true"', text)
        self.assertIn('executable="multisource_fusion_node"', text)
        self.assertIn('condition=IfCondition(enable_multisource_fusion)', text)
        self.assertIn('enable_auxiliary_transport_monitor', text)
        self.assertIn('executable="auxiliary_transport_monitor_node"', text)
        for argument in (
            "gnss_origin_latitude",
            "gnss_origin_longitude",
            "gnss_origin_altitude",
            "map_enu_yaw",
            "map_enu_offset_x",
            "map_enu_offset_y",
            "enable_relative_height",
            "relative_height_odom_topic",
            "enable_mmwave_dynamic_mask",
            "enable_mmwave_observation",
            "mmwave_expected_frame_id",
            "mmwave_to_lidar_x",
            "mmwave_to_lidar_y",
            "mmwave_to_lidar_yaw",
            "enable_event_feature_processing",
            "event_feature_match_distance_px",
        ):
            self.assertIn(f'"{argument}"', text)

        for sensor, data_topic, quality_topic in (
            ("event_camera", "/dg/event_camera/events", "/dg/event_camera/quality"),
            ("mmwave", "/dg/mmwave/points", "/dg/mmwave/quality"),
        ):
            self.assertIn(f'"sensor_kind": "{sensor}"', text)
            self.assertIn(f'"data_topic": "{data_topic}"', text)
            self.assertIn(f'"quality_topic": "{quality_topic}"', text)
            self.assertIn(f'"enable_{sensor}_replay"', text)

        self.assertIn('"mmwave_dynamic_mask_enabled": ParameterValue(', text)
        self.assertIn('"mmwave_points_topic": "/dg/mmwave/points"', text)
        self.assertIn('executable="event_feature_node"', text)
        self.assertIn('"features_topic": "/dg/event_camera/features"', text)
        self.assertIn('"quality_topic": "/dg/event_camera/feature_quality"', text)

    def test_mmwave_dynamic_mask_is_safe_by_default(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")
        marker = '"enable_mmwave_dynamic_mask",\n                default_value="false"'
        self.assertIn(marker, launch)
        node = (SCRIPTS / "lidar_robust_node.py").read_text(encoding="utf-8")
        self.assertIn('self.declare_parameter("mmwave_dynamic_mask_enabled", False)', node)
        self.assertIn('self.declare_parameter("mmwave_observation_enabled", False)', node)
        self.assertIn('"mmwave_dynamic_mask_applied"', node)
        self.assertIn('self._mmwave_enabled and external_dynamic_sectors', node)
        self.assertIn('self.declare_parameter("mmwave_points_topic", "/dg/mmwave/points")', node)
        self.assertIn('(\"x\", \"y\", \"radial_velocity\", \"snr_db\", \"confidence\")', node)
        self.assertNotIn('create_publisher(Twist', node)

    def test_launch_alignment_parameters_are_explicit_float_values(self) -> None:
        text = LAUNCH.read_text(encoding="utf-8")
        self.assertIn("from launch_ros.parameter_descriptions import ParameterValue", text)
        for argument in (
            "gnss_origin_latitude",
            "gnss_origin_longitude",
            "gnss_origin_altitude",
            "map_enu_yaw",
            "map_enu_offset_x",
            "map_enu_offset_y",
        ):
            self.assertIn(f'ParameterValue({argument}, value_type=float)', text)

    def test_launch_keeps_fusion_as_side_channel(self) -> None:
        text = LAUNCH.read_text(encoding="utf-8")
        self.assertIn('"publish_tf": False', text)
        self.assertIn(
            '"relative_height_enabled": ParameterValue(enable_relative_height, value_type=bool)',
            text,
        )
        self.assertIn('"fusion_odom_topic": "/dg/fusion/odom"', text)
        self.assertIn('"fusion_status_topic": "/dg/fusion/status"', text)
        self.assertNotIn('"map_to_odom_topic"', text)

    def test_cmake_installs_wrapper_and_registers_new_tests(self) -> None:
        text = CMAKE.read_text(encoding="utf-8")
        self.assertIn("scripts/ros_shutdown.py", text)
        self.assertIn("scripts/multisource_fusion_core.py", text)
        self.assertIn("scripts/multisource_fusion_node.py", text)
        self.assertIn("scripts/auxiliary_sensor_data.py", text)
        self.assertIn("scripts/auxiliary_sensor_replay_node.py", text)
        self.assertIn("scripts/event_feature_core.py", text)
        self.assertIn("scripts/event_feature_node.py", text)
        self.assertIn("test_multisource_fusion_core", text)
        self.assertIn("test_multisource_fusion_protocols", text)
        self.assertIn("test_event_feature_core", text)
        self.assertIn("auxiliary_transport_monitor_node.py", text)

    def test_default_ekf_and_bringup_are_not_wired_to_fusion(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")
        bringup = (ROOT / "launch" / "bringup.launch.py").read_text(encoding="utf-8")
        self.assertNotIn("ekf.yaml", launch)
        self.assertNotIn("multisource_fusion", bringup)
        self.assertNotIn("TransformBroadcaster", launch)


if __name__ == "__main__":
    unittest.main()
