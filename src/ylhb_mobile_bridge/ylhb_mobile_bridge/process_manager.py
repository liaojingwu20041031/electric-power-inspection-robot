import logging
import os
import signal
import subprocess
from pathlib import Path
from typing import Dict, Optional


class ProcessManager:
    def __init__(self, workspace_dir: str, default_map_path: str) -> None:
        self.workspace_dir = Path(os.path.expanduser(workspace_dir))
        self.default_map_path = Path(os.path.expanduser(default_map_path))
        self._processes: Dict[str, subprocess.Popen] = {}
        self._logger = logging.getLogger('ylhb_mobile_bridge.process_manager')

    def _script(self) -> Path:
        return self.workspace_dir / 'scripts' / 'run_on_jetson.sh'

    def is_running(self, name: str) -> bool:
        process = self._processes.get(name)
        return process is not None and process.poll() is None

    def start_mapping(self) -> str:
        return self._start('mapping', [str(self._script()), 'mapping'])

    def start_navigation(self) -> str:
        return self._start('navigation', [str(self._script()), 'navigation'])

    def _start(self, name: str, command: list[str]) -> str:
        if self.is_running(name):
            return f'{name} already running'
        if name not in {'mapping', 'navigation'}:
            raise ValueError('not_allowed')
        process = subprocess.Popen(
            command,
            cwd=str(self.workspace_dir),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            shell=False,
            preexec_fn=os.setsid if hasattr(os, 'setsid') else None,
        )
        self._processes[name] = process
        self._logger.info('started %s pid=%s command=%s', name, process.pid, command)
        return f'{name} started pid={process.pid}'

    def stop(self, name: str) -> str:
        process = self._processes.get(name)
        if process is None:
            return f'{name} was not started by bridge'
        if process.poll() is not None:
            self._processes.pop(name, None)
            return f'{name} already stopped'
        self._logger.info('stopping %s pid=%s', name, process.pid)
        try:
            if hasattr(os, 'killpg'):
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            else:
                process.terminate()
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            self._logger.warning('force killing %s pid=%s', name, process.pid)
            if hasattr(os, 'killpg'):
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            else:
                process.kill()
        finally:
            self._processes.pop(name, None)
        return f'{name} stopped'

    def save_map(self, map_name: Optional[str]) -> dict:
        safe_name = (map_name or self.default_map_path.name).strip()
        if not safe_name.replace('_', '').replace('-', '').isalnum():
            raise ValueError('map_name must contain only letters, numbers, underscore or hyphen')
        target = self.default_map_path.with_name(safe_name)
        command = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(target)]
        self._logger.info('saving map command=%s', command)
        process = subprocess.Popen(
            command,
            cwd=str(self.workspace_dir),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            shell=False,
        )
        output, _ = process.communicate(timeout=60)
        if process.returncode != 0:
            raise RuntimeError(output)
        return {
            'yaml_path': str(target.with_suffix('.yaml')),
            'pgm_path': str(target.with_suffix('.pgm')),
            'output': output,
        }
