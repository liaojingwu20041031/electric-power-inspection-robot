import os
import subprocess
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT = REPO_ROOT / 'scripts' / 'run_on_jetson.sh'


class RunOnJetsonTest(unittest.TestCase):
    def run_script(self, *args):
        with tempfile.TemporaryDirectory() as tmp:
            fake_ros2 = Path(tmp) / 'ros2'
            fake_ros2.write_text(
                '#!/usr/bin/env bash\n'
                'printf "XAUTHORITY=%s\\n" "${XAUTHORITY:-}"\n'
                'printf "%s\\n" "$@"\n',
                encoding='utf-8',
            )
            fake_ros2.chmod(0o755)
            fake_home = Path(tmp) / 'home'
            fake_home.mkdir()
            fake_xauthority = fake_home / '.Xauthority'
            fake_xauthority.write_text('cookie', encoding='utf-8')
            env = os.environ.copy()
            env['PATH'] = f'{tmp}:{env["PATH"]}'
            env['WS_DIR'] = str(REPO_ROOT)
            env['HOME'] = str(fake_home)
            env['DISPLAY'] = 'localhost:10.0'
            env['ENABLE_CHINESE_IME'] = 'false'
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

    def test_inspection_rejects_disabling_voice_session_or_tts(self):
        for disabled_arg in ('enable_voice:=false', 'enable_voice_session:=false', 'enable_tts:=false'):
            with self.subTest(disabled_arg=disabled_arg):
                result = self.run_script('inspection', disabled_arg)

                self.assertNotEqual(result.returncode, 0)
                self.assertIn('inspection mode is the formal robot console', result.stderr)

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
