import ast
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
LAUNCH = ROOT / "launch" / "dg_navigation_integration.launch.py"
UWB_EVIDENCE_LAUNCH = ROOT / "launch" / "uwb_model1_replay_evidence.launch.py"
CMAKE = ROOT / "CMakeLists.txt"
REPOSITORY_ROOT = ROOT.parents[1]
DG_RUNNER = REPOSITORY_ROOT / "scripts" / "run_dg_integration.sh"
REMOTE_INSTALLER = REPOSITORY_ROOT / "scripts" / "install_release_on_robot.sh"
RELEASE_ACTIVATOR = REPOSITORY_ROOT / "scripts" / "activate_dg_release.sh"
WINDOWS_DEPLOYER = REPOSITORY_ROOT / "scripts" / "deploy_release_from_windows.ps1"
PREFLIGHT = REPOSITORY_ROOT / "scripts" / "run_dg202611_preflight.sh"
DG_SERVICE_RUNNER = REPOSITORY_ROOT / "scripts" / "run_dg_models_service.sh"
DG_SERVICE_INSTALLER = REPOSITORY_ROOT / "scripts" / "install_dg_models_service.sh"
DG_INSPECTION_RUNNER = REPOSITORY_ROOT / "scripts" / "run_dg_inspection.sh"
GIT_ATTRIBUTES = REPOSITORY_ROOT / ".gitattributes"

if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from mission_activity import MissionActivityGate  # noqa: E402


class NavigationProtocolAndLaunchTests(unittest.TestCase):
    def test_all_new_python_files_parse_without_ros_imports_at_module_scope(self) -> None:
        for path in (
            SCRIPTS / "navigation_health_core.py",
            SCRIPTS / "navigation_health_node.py",
            SCRIPTS / "mission_activity.py",
            SCRIPTS / "cmd_vel_arbiter_core.py",
            SCRIPTS / "cmd_vel_arbiter_node.py",
            SCRIPTS / "uwb_input.py",
            SCRIPTS / "uwb_replay_node.py",
            SCRIPTS / "invalid_candidate_injector_node.py",
            LAUNCH,
            UWB_EVIDENCE_LAUNCH,
        ):
            ast.parse(path.read_text(encoding="utf-8"))

    def test_launch_contains_all_integration_nodes(self) -> None:
        text = LAUNCH.read_text(encoding="utf-8")
        for executable in (
            "lidar_robust_node",
            "gnss_quality_node",
            "scan_map_relocalization_node",
            "active_relocalization_node",
            "navigation_health_node",
            "cmd_vel_arbiter_node",
            "uwb_replay_node",
        ):
            self.assertIn(f'executable="{executable}"', text)

    def test_launch_has_explicit_topic_boundaries(self) -> None:
        text = LAUNCH.read_text(encoding="utf-8")
        for topic in (
            "/scan",
            "/odom",
            "/gps/fix",
            "/gps/rtk_status",
            "/dg/lidar/quality",
            "/dg/lidar/filtered_scan",
            "/dg/lidar/stable_features",
            "/dg/gnss/quality",
            "/dg/relocalization/seed",
            "/dg/relocalization/match_quality",
            "/cmd_vel_nav",
            "/cmd_vel_recovery",
            "/cmd_vel",
            "/dg/navigation/status",
        ):
            self.assertIn(topic, text)
        self.assertEqual(text.count('"scan_topic": "/dg/lidar/filtered_scan"'), 2)
        self.assertRegex(
            text,
            r'DeclareLaunchArgument\(\s*"enable_scan_map"',
        )

    def test_launch_defaults_nav2_disabled_and_remaps_cmd_vel(self) -> None:
        text = LAUNCH.read_text(encoding="utf-8")
        self.assertIn('DeclareLaunchArgument("enable_nav2", default_value="false"', text)
        self.assertIn('SetRemap(src="/cmd_vel", dst="/cmd_vel_nav")', text)
        self.assertIn('default_map = preferred_map if os.path.exists(preferred_map) else fallback_map', text)
        self.assertIn('DeclareLaunchArgument("map", default_value=default_map)', text)
        self.assertNotIn('DeclareLaunchArgument("map", default_value="")', text)

    def test_launch_defaults_motion_output_and_uwb_alignment_disabled(self) -> None:
        text = LAUNCH.read_text(encoding="utf-8")
        self.assertIn('"enable_cmd_vel_arbiter",', text)
        self.assertIn('default_value="/dg/test_cmd_vel"', text)
        self.assertIn('condition=IfCondition(enable_cmd_vel_arbiter)', text)
        self.assertIn('"enable_uwb_replay",', text)
        self.assertIn('DeclareLaunchArgument("uwb_alignment_enabled", default_value="false")', text)

    def test_uwb_evidence_launch_is_isolated_and_marks_test_frame(self) -> None:
        text = UWB_EVIDENCE_LAUNCH.read_text(encoding="utf-8")
        self.assertIn('executable="uwb_replay_node"', text)
        self.assertIn('executable="multisource_fusion_node"', text)
        self.assertIn('"uwb_alignment_enabled": True', text)
        self.assertIn('default_value="uwb_replay_test"', text)
        self.assertIn('default_value="uwb_replay_tag_test"', text)
        self.assertIn('"publish_tf": False', text)
        self.assertNotIn("cmd_vel", text)
        for disabled_input in (
            "/dg/evidence/no_local_odom",
            "/dg/evidence/no_gnss_fix",
            "/dg/evidence/no_amcl_pose",
            "/dg/evidence/no_scan_match_pose",
        ):
            self.assertIn(disabled_input, text)

    def test_uwb_evidence_wrapper_has_scenarios_and_no_motion_command(self) -> None:
        text = (REPOSITORY_ROOT / "scripts" / "run_uwb_model1_evidence.sh").read_text(
            encoding="utf-8"
        )
        for scenario in ("static", "move", "anchor_fault"):
            self.assertIn(scenario, text)
        self.assertIn("dg_uwb_evidence", text)
        self.assertNotIn("topic pub", text)
        self.assertNotIn("/cmd_vel", text)

    def test_fusion_wrapper_publishes_configured_output_frame(self) -> None:
        text = (SCRIPTS / "multisource_fusion_node.py").read_text(encoding="utf-8")
        self.assertIn('self.declare_parameter("output_frame_id", "map")', text)
        self.assertIn('self.declare_parameter("unanchored_output_frame_id", "odom")', text)
        self.assertIn("odom.header.frame_id = frame_id", text)
        self.assertIn("odom.child_frame_id = self._output_child_frame_id", text)
        self.assertIn('"output_frame_id": frame_id', text)

    def test_cmake_installs_new_wrappers_and_registers_tests(self) -> None:
        text = CMAKE.read_text(encoding="utf-8")
        for script in (
            "navigation_health_core.py",
            "navigation_health_node.py",
            "mission_activity.py",
            "cmd_vel_arbiter_core.py",
            "cmd_vel_arbiter_node.py",
            "uwb_input.py",
            "uwb_replay_node.py",
            "invalid_candidate_injector_node.py",
        ):
            self.assertIn(f"scripts/{script}", text)
        for test_name in (
            "test_navigation_health_core",
            "test_cmd_vel_arbiter_core",
            "test_navigation_integration_core",
            "test_navigation_protocols",
        ):
            self.assertIn(test_name, text)

    def test_default_bringup_does_not_include_competition_launch(self) -> None:
        text = (ROOT / "launch" / "bringup.launch.py").read_text(encoding="utf-8")
        self.assertNotIn("dg_navigation_integration", text)

    def test_dg_runner_loads_base_then_isolated_overlay(self) -> None:
        text = DG_RUNNER.read_text(encoding="utf-8")
        base_source = 'source "${base_ws}/install/setup.bash"'
        integration_source = 'source "${integration_ws}/install/setup.bash"'
        self.assertIn(base_source, text)
        self.assertIn(integration_source, text)
        self.assertLess(text.index(base_source), text.index(integration_source))
        self.assertIn("dg_navigation_integration.launch.py", text)
        self.assertNotIn("topic pub", text)

    def test_release_deployment_is_versioned_verified_and_motion_free(self) -> None:
        installer = REMOTE_INSTALLER.read_text(encoding="utf-8")
        activator = RELEASE_ACTIVATOR.read_text(encoding="utf-8")
        deployer = WINDOWS_DEPLOYER.read_text(encoding="utf-8")
        self.assertIn("sha256sum", installer)
        self.assertIn("Refusing to overwrite existing release directory", installer)
        self.assertIn("motion_test_executed", installer)
        self.assertIn("run_dg202611_preflight.sh", installer)
        self.assertIn("Refusing to replace non-symlink path", activator)
        self.assertIn("DEPLOYMENT_RECORD.json", activator)
        self.assertIn("mv -Tf", activator)
        self.assertIn("/home/nvidia/ros2_DL was not changed", activator)
        self.assertIn("Get-FileHash", deployer)
        self.assertIn("Test-NetConnection", deployer)
        self.assertIn("Cannot infer the commit from archive name", deployer)
        self.assertIn("does not match archive filename commit", deployer)
        self.assertIn("staging_dir", installer)
        self.assertIn("source_root", installer)
        for text in (installer, deployer, activator):
            self.assertNotIn("/home/nvidia/ros2_DL/src", text)
            self.assertNotIn("topic pub", text)
            self.assertNotIn("cmd_vel", text)

    def test_dg_models_service_is_single_owner_and_observation_only(self) -> None:
        runner = DG_SERVICE_RUNNER.read_text(encoding="utf-8")
        installer = DG_SERVICE_INSTALLER.read_text(encoding="utf-8")
        self.assertIn("run_dg_integration.sh", runner)
        self.assertIn("enable_nav2:=false", runner)
        self.assertIn("enable_cmd_vel_arbiter:=false", runner)
        self.assertIn("enable_uwb_replay:=false", runner)
        self.assertIn("cmd_vel_output_topic:=/dg/test_cmd_vel", runner)
        self.assertIn("mission_gate_enabled:=true", runner)
        self.assertIn("dg202611-models.service", installer)
        self.assertIn("KillSignal=SIGINT", installer)
        self.assertIn("KillMode=control-group", installer)
        self.assertIn("Restart=on-failure", installer)
        self.assertIn("Refusing to start a second DG integration instance", installer)
        self.assertNotIn("ros2 topic pub", runner)
        self.assertNotIn("/home/nvidia/ros2_DL/src", installer)

    def test_cross_board_runner_forces_non_localhost_transport(self) -> None:
        runner = DG_RUNNER.read_text(encoding="utf-8")
        self.assertIn('export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"', runner)
        self.assertIn("export ROS_LOCALHOST_ONLY=0", runner)
        self.assertNotIn('export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"', runner)
        for sidecar in (
            REPOSITORY_ROOT / "scripts" / "run_rdk_event_feature_sidecar.sh",
            REPOSITORY_ROOT / "scripts" / "run_rdk_event_csv_replay.sh",
        ):
            text = sidecar.read_text(encoding="utf-8")
            self.assertIn("export ROS_LOCALHOST_ONLY=0", text)

    def test_dg_inspection_uses_base_ui_and_single_bridge_owner(self) -> None:
        runner = DG_INSPECTION_RUNNER.read_text(encoding="utf-8")
        jetson = (REPOSITORY_ROOT / "scripts" / "run_on_jetson.sh").read_text(encoding="utf-8")
        preflight = PREFLIGHT.read_text(encoding="utf-8")
        self.assertIn('export WS_DIR="${base_ws}"', runner)
        self.assertIn('export DG_INTEGRATION_WS="${integration_ws}"', runner)
        self.assertIn('base_runner="${base_ws}/scripts/run_on_jetson.sh"', runner)
        self.assertIn('base_ui="${base_ws}/install/ylhb_llm/lib/ylhb_llm/inspection_display_ui_node"', runner)
        self.assertIn("拒绝启动第二套 inspection UI/Supervisor", runner)
        self.assertIn("mobile_bridge_managed_externally:=true", runner)
        self.assertIn("auto_start_mobile_bridge:=false", runner)
        self.assertNotIn('export YLHB_OVERLAY_WS="${integration_ws}"', runner)
        self.assertIn('source_ros_setup "${OVERLAY_WS_DIR}/install/setup.bash"', jetson)
        self.assertIn("--packages-select ylhb_base ylhb_llm", preflight)
        self.assertNotIn("topic pub", runner)

    def test_preflight_counter_does_not_trip_errexit_after_first_pass(self) -> None:
        text = PREFLIGHT.read_text(encoding="utf-8")
        self.assertIn("PASS_COUNT=$((PASS_COUNT + 1))", text)
        self.assertIn("FAIL_COUNT=$((FAIL_COUNT + 1))", text)
        self.assertNotIn("((PASS_COUNT++))", text)
        self.assertNotIn("((FAIL_COUNT++))", text)

    def test_release_line_endings_are_declared_for_linux_entry_points(self) -> None:
        text = GIT_ATTRIBUTES.read_text(encoding="utf-8")
        self.assertIn("*.sh text eol=lf", text)
        self.assertIn("*.py text eol=lf", text)
        self.assertIn("*.sha256 text eol=lf", text)
        self.assertIn("LONG_RUN_TERMINAL_RECORD.md text eol=lf", text)

    def test_active_relocalization_default_feeds_recovery_channel(self) -> None:
        text = (SCRIPTS / "active_relocalization_node.py").read_text(encoding="utf-8")
        self.assertIn('"cmd_vel_topic": "/cmd_vel_recovery"', text)

    def test_production_integration_enables_mission_gate_but_core_defaults_stay_compatible(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")
        active = (SCRIPTS / "active_relocalization_node.py").read_text(encoding="utf-8")
        health = (SCRIPTS / "navigation_health_node.py").read_text(encoding="utf-8")
        runner = (
            REPOSITORY_ROOT
            / "src"
            / "dg_synthetic_validation"
            / "dg_synthetic_validation"
            / "scenario_runner.py"
        ).read_text(encoding="utf-8")
        self.assertIn('"mission_gate_enabled",\n                default_value="true"', launch)
        self.assertIn('"mission_gate_enabled": ParameterValue(mission_gate_enabled', launch)
        self.assertIn('"mission_gate_enabled": False', active)
        self.assertIn('self.declare_parameter("mission_gate_enabled", False)', health)
        self.assertIn('"mission_gate_enabled:=false"', runner)

    def test_mission_status_contract_is_shared_by_recovery_and_health(self) -> None:
        for path in (
            SCRIPTS / "active_relocalization_node.py",
            SCRIPTS / "navigation_health_node.py",
        ):
            text = path.read_text(encoding="utf-8")
            self.assertIn('"system_status_topic": "/inspection_ai/system_status"' if "active" in path.name else '"system_status_topic", "/inspection_ai/system_status"', text)
            for field in (
                "mission_phase",
                "mission_active",
                "mission_status_fresh",
                "standby_reason",
                "last_terminal_state",
                "last_failure_reason",
                "recovery_motion_armed",
            ):
                self.assertIn(field, text)

    def test_mission_gate_only_activates_for_a_fresh_ready_patrol(self) -> None:
        def status(state: str = "running", **overrides):
            payload = {
                "patrol_mode_state": state,
                "bringup": "running",
                "navigation": "running",
                "patrol_executor": "running",
            }
            payload.update(overrides)
            return payload

        gate = MissionActivityGate(status_timeout_sec=3.0, activation_grace_sec=2.0)
        self.assertEqual(gate.evaluate(None, None, 0.0).standby_reason, "NO_SYSTEM_STATUS")
        self.assertEqual(gate.evaluate(status(), 0.0, 10.0).standby_reason, "SYSTEM_STATUS_STALE")
        self.assertEqual(
            gate.evaluate(status(navigation="stopped"), 11.0, 11.0).standby_reason,
            "COMPONENTS_NOT_READY",
        )
        self.assertEqual(gate.evaluate(status(), 12.0, 12.0).phase, "WARMUP")
        self.assertEqual(gate.evaluate(status(), 14.0, 14.0).phase, "ACTIVE")
        terminal = gate.evaluate(status("succeeded"), 15.0, 15.0)
        self.assertEqual(terminal.phase, "STANDBY")
        self.assertEqual(terminal.terminal_state, "succeeded")

    def test_gnss_is_not_wired_into_ekf_by_integration_launch(self) -> None:
        text = LAUNCH.read_text(encoding="utf-8")
        self.assertNotIn("ekf.yaml", text)
        self.assertNotIn("NavSatTransform", text)

    def test_controlled_invalid_candidate_tool_has_no_motion_interface(self) -> None:
        text = (SCRIPTS / "invalid_candidate_injector_node.py").read_text(encoding="utf-8")
        self.assertIn("DiagnosticArray", text)
        self.assertIn("CONTROLLED_INVALID_CANDIDATE", text)
        self.assertNotIn("Twist", text)
        self.assertNotIn("/cmd_vel", text)

    def test_poc_publishers_and_health_parser_share_real_field_names(self) -> None:
        lidar = (SCRIPTS / "lidar_robust_features.py").read_text(encoding="utf-8")
        gnss = (SCRIPTS / "gnss_quality_node.py").read_text(encoding="utf-8")
        scan_map = (SCRIPTS / "scan_map_relocalization_node.py").read_text(encoding="utf-8")
        active = (SCRIPTS / "active_relocalization_node.py").read_text(encoding="utf-8")
        health = (SCRIPTS / "navigation_health_node.py").read_text(encoding="utf-8")
        for field in ("valid_ratio", "geometry_score", "temporal_status"):
            self.assertIn(field, lidar)
        for field in ("current_state", "decision"):
            self.assertIn(field, gnss)
        for field in ("accepted", "score", "mean_distance", "inlier_ratio", "candidate_x", "candidate_y", "candidate_yaw", "reason"):
            self.assertIn(field, scan_map)
        for field in ("state", "attempt_id", "request_candidate", "seed_source"):
            self.assertIn(field, active)
        for field in ("gnss_state", "lidar_state", "uwb_state", "amcl_state", "scan_match_state", "relocalization_state", "overall_state", "reasons", "timestamp"):
            self.assertIn(field, health)

    def test_health_and_arbiter_have_freshness_guards(self) -> None:
        health = (SCRIPTS / "navigation_health_node.py").read_text(encoding="utf-8")
        arbiter = (SCRIPTS / "cmd_vel_arbiter_node.py").read_text(encoding="utf-8")
        core = (SCRIPTS / "cmd_vel_arbiter_core.py").read_text(encoding="utf-8")
        self.assertIn('"min_lidar_quality", 0.2', health)
        self.assertIn("relocalization_fresh", health)
        self.assertIn('"status_timeout", 1.0', arbiter)
        self.assertIn("navigation_status_received_at", arbiter)
        self.assertIn("NAVIGATION_STATUS_STALE", core)


if __name__ == "__main__":
    unittest.main()
