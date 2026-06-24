import logging
import os
import signal
import subprocess
import time
from pathlib import Path
from typing import Dict, IO, Optional


class ProcessManager:
    ALLOWED_MODES = {'bringup', 'mapping', 'navigation'}

    def __init__(self, workspace_dir: str, default_map_path: str) -> None:
        self.workspace_dir = Path(os.path.expanduser(workspace_dir))
        self.default_map_path = Path(os.path.expanduser(default_map_path))
        self._processes: Dict[str, subprocess.Popen] = {}
        self._records: Dict[str, dict] = {}
        self._log_handles: Dict[str, IO[str]] = {}
        self._logger = logging.getLogger('ylhb_mobile_bridge.process_manager')

    def _script(self) -> Path:
        return self.workspace_dir / 'scripts' / 'run_on_jetson.sh'

    def _log_path(self, name: str) -> Path:
        return self.workspace_dir / 'logs' / 'mobile_bridge' / f'{name}.log'

    def is_running(self, name: str) -> bool:
        process = self._processes.get(name)
        return process is not None and process.poll() is None

    def start_mapping(self) -> str:
        return self.start('mapping')

    def start_navigation(self) -> str:
        return self.start('navigation')

    def start(self, mode: str) -> str:
        if mode not in self.ALLOWED_MODES:
            raise ValueError('not_allowed')
        return self._start(mode, [str(self._script()), mode])

    def _start(self, name: str, command: list[str]) -> str:
        if self.is_running(name):
            return f'{name} already running'
        if name not in self.ALLOWED_MODES:
            raise ValueError('not_allowed')
        self._close_log(name)
        log_path = self._log_path(name)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_handle = log_path.open('a', encoding='utf-8')
        process = subprocess.Popen(
            command,
            cwd=str(self.workspace_dir),
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            text=True,
            shell=False,
            preexec_fn=os.setsid if hasattr(os, 'setsid') else None,
        )
        self._processes[name] = process
        self._log_handles[name] = log_handle
        self._records[name] = {
            'command': list(command),
            'pid': process.pid,
            'started_at': time.time(),
            'log_path': str(log_path),
        }
        self._logger.info(
            'started %s pid=%s command=%s',
            name,
            process.pid,
            command,
        )
        return f'{name} started pid={process.pid}'

    def stop(self, name: str) -> str:
        if name not in self.ALLOWED_MODES:
            raise ValueError('not_allowed')
        process = self._processes.get(name)
        if process is None:
            return f'{name} was not started by bridge'
        if process.poll() is not None:
            self._close_log(name)
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
            self._close_log(name)
        return f'{name} stopped'

    def _close_log(self, name: str) -> None:
        handle = self._log_handles.pop(name, None)
        if handle is not None and not handle.closed:
            handle.close()

    def _read_log_tail(
        self,
        log_path: Optional[str],
        max_bytes: int = 8192,
    ) -> str:
        if not log_path:
            return ''
        path = Path(log_path)
        if not path.exists():
            return ''
        with path.open('rb') as handle:
            handle.seek(0, os.SEEK_END)
            size = handle.tell()
            handle.seek(max(0, size - max_bytes))
            data = handle.read()
        return data.decode('utf-8', errors='replace').strip()

    def process_status(self, name: str) -> dict:
        if name not in self.ALLOWED_MODES:
            raise ValueError('not_allowed')
        process = self._processes.get(name)
        record = self._records.get(name)
        returncode = process.poll() if process is not None else None
        if returncode is not None:
            self._close_log(name)
        return {
            'mode': name,
            'command': record['command'] if record is not None else None,
            'pid': record['pid'] if record is not None else None,
            'started_at': record['started_at'] if record is not None else None,
            'running': process is not None and returncode is None,
            'returncode': returncode,
            'log_path': record['log_path'] if record is not None else None,
            'log_tail': self._read_log_tail(
                record['log_path'] if record is not None else None
            ),
            'managed_by_bridge': record is not None,
        }

    def save_map(self, map_name: Optional[str]) -> dict:
        safe_name = (map_name or self.default_map_path.name).strip()
        if not safe_name.replace('_', '').replace('-', '').isalnum():
            raise ValueError(
                'map_name must contain only letters, numbers, '
                'underscore or hyphen'
            )
        target = self.default_map_path.with_name(safe_name)
        command = [
            'ros2',
            'run',
            'nav2_map_server',
            'map_saver_cli',
            '-f',
            str(target),
            '--ros-args',
            '-p',
            'save_map_timeout:=10.0',
        ]
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
