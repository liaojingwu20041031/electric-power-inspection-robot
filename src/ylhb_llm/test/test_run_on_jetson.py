import os
import subprocess
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT = REPO_ROOT / 'scripts' / 'run_on_jetson.sh'
PREFLIGHT = REPO_ROOT / 'scripts' / 'check_agent_setup.py'


class RunOnJetsonTest(unittest.TestCase):
    def run_script(self, *args, agent_env='', robot_env='', mobile_bridge_state='active', owner='auto'):
        with tempfile.TemporaryDirectory() as tmp:
            fake_ros2 = Path(tmp) / 'ros2'
            fake_ros2.write_text(
                '#!/usr/bin/env bash\n'
                'printf "XAUTHORITY=%s\\n" "${XAUTHORITY:-}"\n'
                'printf "AGENT_KEY_LOADED=%s\\n" "${DASHSCOPE_API_KEY:+yes}"\n'
                'printf "YLHB_AUDIO_INPUT_DEVICE=%s\\n" "${YLHB_AUDIO_INPUT_DEVICE:-}"\n'
                'printf "YLHB_AUDIO_OUTPUT_DEVICE=%s\\n" "${YLHB_AUDIO_OUTPUT_DEVICE:-}"\n'
                'printf "YLHB_TTS_VOICE=%s\\n" "${YLHB_TTS_VOICE:-}"\n'
                'printf "%s\\n" "$@"\n',
                encoding='utf-8',
            )
            fake_ros2.chmod(0o755)
            fake_systemctl = Path(tmp) / 'systemctl'
            fake_systemctl.write_text(
                '#!/usr/bin/env bash\n'
                'case "$1" in\n'
                '  is-active) [ "${FAKE_MOBILE_BRIDGE_STATE}" = active ] ;;\n'
                '  is-enabled) [ "${FAKE_MOBILE_BRIDGE_STATE}" = enabled-inactive ] ;;\n'
                '  list-unit-files) [ "${FAKE_MOBILE_BRIDGE_STATE}" != missing ] && printf "ylhb-mobile-bridge.service enabled\\n" ;;\n'
                'esac\n',
                encoding='utf-8',
            )
            fake_systemctl.chmod(0o755)
            fake_home = Path(tmp) / 'home'
            fake_home.mkdir()
            if agent_env:
                agent_env_path = fake_home / '.config' / 'ylhb' / 'agent.env'
                agent_env_path.parent.mkdir(parents=True)
                agent_env_path.write_text(agent_env, encoding='utf-8')
            if robot_env:
                robot_env_path = fake_home / '.config' / 'ylhb' / 'robot.env'
                robot_env_path.parent.mkdir(parents=True, exist_ok=True)
                robot_env_path.write_text(robot_env, encoding='utf-8')
            fake_xauthority = fake_home / '.Xauthority'
            fake_xauthority.write_text('cookie', encoding='utf-8')
            env = os.environ.copy()
            env['PATH'] = f'{tmp}:{env["PATH"]}'
            env['WS_DIR'] = str(REPO_ROOT)
            env['HOME'] = str(fake_home)
            env['DISPLAY'] = 'localhost:10.0'
            env['ENABLE_CHINESE_IME'] = 'false'
            env['FAKE_MOBILE_BRIDGE_STATE'] = mobile_bridge_state
            env['YLHB_MOBILE_BRIDGE_OWNER'] = owner
            env.pop('DASHSCOPE_API_KEY', None)
            return subprocess.run(
                ['bash', str(SCRIPT), *args],
                cwd=REPO_ROOT,
                env=env,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )

    def test_inspection_forces_formal_console_voice_and_tts(self):
        result = self.run_script('inspection')

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('enable_display_ui:=true', result.stdout)
        self.assertIn('enable_system_supervisor:=true', result.stdout)
        self.assertIn('enable_keepout_navigation:=true', result.stdout)
        self.assertIn('enable_voice:=true', result.stdout)
        self.assertIn('enable_voice_session:=true', result.stdout)
        self.assertIn('enable_tts:=true', result.stdout)
        self.assertIn('enable_capture_voice:=false', result.stdout)
        self.assertIn('display:=:', result.stdout)
        self.assertIn('xauthority:=', result.stdout)
        self.assertIn('XAUTHORITY=', result.stdout)
        self.assertIn('mobile_bridge_managed_externally:=true', result.stdout)
        self.assertIn('Mobile Bridge owner: systemd', result.stderr)

    def test_inspection_auto_owner_falls_back_to_supervisor_without_unit(self):
        result = self.run_script('inspection', mobile_bridge_state='missing')

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('mobile_bridge_managed_externally:=false', result.stdout)
        self.assertIn('auto_start_mobile_bridge:=true', result.stdout)
        self.assertIn('Mobile Bridge owner: supervisor', result.stderr)

    def test_inspection_auto_owner_keeps_enabled_but_inactive_systemd_ownership(self):
        result = self.run_script('inspection', mobile_bridge_state='enabled-inactive')

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('mobile_bridge_managed_externally:=true', result.stdout)
        self.assertIn('enabled but not active', result.stderr)

    def test_explicit_mobile_bridge_launch_argument_has_priority(self):
        result = self.run_script(
            'inspection', 'mobile_bridge_managed_externally:=true',
            mobile_bridge_state='missing', owner='supervisor',
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.count('mobile_bridge_managed_externally:=true'), 1)
        self.assertIn('Mobile Bridge owner: systemd', result.stderr)

    def test_inspection_rejects_disabling_voice_session_or_tts(self):
        for disabled_arg in ('enable_voice:=false', 'enable_voice_session:=false', 'enable_tts:=false'):
            with self.subTest(disabled_arg=disabled_arg):
                result = self.run_script('inspection', disabled_arg)

                self.assertNotEqual(result.returncode, 0)
                self.assertIn('inspection mode is the formal robot console', result.stderr)

    def test_inspection_loads_default_agent_env_without_printing_key(self):
        result = self.run_script('inspection', agent_env='DASHSCOPE_API_KEY=not-for-output\n')

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('AGENT_KEY_LOADED=yes', result.stdout)
        self.assertNotIn('not-for-output', result.stdout + result.stderr)

    def test_inspection_warns_when_agent_key_is_missing(self):
        result = self.run_script('inspection')

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('WARN: DASHSCOPE_API_KEY is missing;', result.stderr)

    def test_inspection_reads_audio_devices_from_robot_env(self):
        result = self.run_script(
            'inspection',
            robot_env=(
                'YLHB_AUDIO_INPUT_DEVICE=plughw:CARD=USB,DEV=0\n'
                'YLHB_AUDIO_OUTPUT_DEVICE=hw:CARD=USB,DEV=0\n'
                'YLHB_TTS_VOICE=CustomVoice\n'
            ),
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('YLHB_AUDIO_INPUT_DEVICE=plughw:CARD=USB,DEV=0', result.stdout)
        self.assertIn('YLHB_AUDIO_OUTPUT_DEVICE=hw:CARD=USB,DEV=0', result.stdout)
        self.assertIn('YLHB_TTS_VOICE=CustomVoice', result.stdout)

    def test_agent_preflight_reports_missing_key_without_echoing_environment(self):
        env = os.environ.copy()
        env.pop('DASHSCOPE_API_KEY', None)
        env['PYTHONPATH'] = os.pathsep.join((
            str(REPO_ROOT / 'src' / 'ylhb_llm'), str(REPO_ROOT / 'src' / 'ylhb_mobile_bridge'),
        ))
        result = subprocess.run(
            [
                'python3', str(PREFLIGHT),
                '--skip-endpoint',
                '--skip-ros',
                '--route-file', str(REPO_ROOT / 'maps' / 'route_patrol_001.json'),
                '--capabilities-file', str(REPO_ROOT / 'src' / 'ylhb_llm' / 'config' / 'robot_capabilities.yaml'),
            ],
            cwd=REPO_ROOT,
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )

        payload = __import__('json').loads(result.stdout)
        self.assertEqual(result.returncode, 1)
        self.assertFalse(payload['planner_available'])
        self.assertTrue(payload['route_catalog_available'])
        self.assertTrue(payload['capability_catalog_available'])
        self.assertNotIn('DASHSCOPE_API_KEY=', result.stdout + result.stderr)

    def test_agent_preflight_accepts_keyless_local_provider_from_config(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / 'llm.yaml'
            config.write_text(
                'inspection_agent_node:\n'
                '  ros__parameters:\n'
                '    planner_provider_name: local\n'
                '    planner_base_url: http://127.0.0.1:1234/v1\n'
                '    planner_model: local-model\n'
                '    planner_api_key_env: ""\n'
                '    planner_api_key_required: false\n'
                '    planner_models_path: /models\n'
                '    planner_chat_path: /chat/completions\n'
                '    route_file_path: auto\n',
                encoding='utf-8',
            )
            result = subprocess.run(
                [
                    'python3', str(PREFLIGHT), '--config', str(config),
                    '--skip-endpoint', '--skip-ros',
                    '--route-directory', str(REPO_ROOT / 'maps'),
                    '--capabilities-file', str(REPO_ROOT / 'src' / 'ylhb_llm' / 'config' / 'robot_capabilities.yaml'),
                ],
                cwd=REPO_ROOT,
                env={**os.environ, 'PYTHONPATH': os.pathsep.join((
                    str(REPO_ROOT / 'src' / 'ylhb_llm'), str(REPO_ROOT / 'src' / 'ylhb_mobile_bridge'),
                ))},
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )

        payload = __import__('json').loads(result.stdout)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(payload['provider'], 'local')
        self.assertTrue(payload['planner_available'])
        self.assertEqual(payload['model'], 'local-model')

    def test_zed_3d_mapping_mode_is_removed_from_help(self):
        help_result = self.run_script('help')
        self.assertNotIn('zed_3d_mapping', help_result.stdout)
        self.assertIn('zed_3d_capture', help_result.stdout)
        self.assertIn('zed_3d_reconstruct', help_result.stdout)
        self.assertNotIn('navigation_keepout', help_result.stdout)

    def test_zed_3d_capture_and_reconstruct_routes(self):
        capture = self.run_script('zed_3d_capture', 'duration_sec:=1')
        self.assertEqual(capture.returncode, 0, capture.stderr)
        self.assertIn('run', capture.stdout)
        self.assertIn('ylhb_3d_mapping', capture.stdout)
        self.assertIn('zed_svo_capture', capture.stdout)
        self.assertIn('duration_sec:=1', capture.stdout)

        reconstruct = self.run_script('zed_3d_reconstruct', 'input:=/tmp/capture.svo2')
        self.assertEqual(reconstruct.returncode, 0, reconstruct.stderr)
        self.assertIn('run', reconstruct.stdout)
        self.assertIn('ylhb_3d_mapping', reconstruct.stdout)
        self.assertIn('zed_svo_reconstruct', reconstruct.stdout)
        self.assertIn('input:=/tmp/capture.svo2', reconstruct.stdout)

        help_result = self.run_script('help')
        self.assertIn('zed_3d_capture', help_result.stdout)
        self.assertIn('zed_3d_reconstruct', help_result.stdout)
        self.assertIn('zed_3d_reconstruct latest', help_result.stdout)
        self.assertIn('input:=latest profile:=quality_safe', help_result.stdout)
        self.assertIn('session:=capture_YYYYmmdd_HHMMSS', help_result.stdout)
        self.assertNotIn('input:=${WS_DIR}/runs/3d_capture/capture_YYYYmmdd_HHMMSS/capture.svo2', help_result.stdout)


if __name__ == '__main__':
    unittest.main()
