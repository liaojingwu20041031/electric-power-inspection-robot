import signal
import subprocess
from pathlib import Path
from unittest.mock import MagicMock, call, patch

import pytest

from ylhb_mobile_bridge.process_manager import ProcessManager


def make_manager(tmp_path: Path):
    return ProcessManager(
        str(tmp_path),
        str(tmp_path / "maps" / "my_map"),
    )


@pytest.mark.parametrize("mode", ["bringup", "mapping", "navigation"])
def test_process_manager_starts_allowed_modes_with_argv_and_shell_false(
    tmp_path,
    mode,
):
    manager = make_manager(tmp_path)
    fake_process = MagicMock()
    fake_process.pid = 123
    fake_process.poll.return_value = None

    with patch("ylhb_mobile_bridge.process_manager.subprocess.Popen") as popen:
        popen.return_value = fake_process
        message = manager.start(mode)

    assert message == f"{mode} started pid=123"
    command = popen.call_args.args[0]
    kwargs = popen.call_args.kwargs
    assert isinstance(command, list)
    assert command[-1] == mode
    assert kwargs["shell"] is False
    assert kwargs["stderr"] is not subprocess.PIPE
    assert kwargs["stdout"] is not subprocess.PIPE
    assert kwargs["stdout"].name.endswith(f"{mode}.log")
    kwargs["stdout"].close()


def test_process_manager_rejects_unknown_mode(tmp_path):
    with pytest.raises(ValueError, match="not_allowed"):
        make_manager(tmp_path).start("localization")


def test_process_manager_rejects_stopping_unknown_mode(tmp_path):
    with pytest.raises(ValueError, match="not_allowed"):
        make_manager(tmp_path).stop("localization")


def test_process_manager_stops_with_sigint_before_forceful_signals(tmp_path):
    manager = make_manager(tmp_path)
    fake_process = MagicMock()
    fake_process.pid = 456
    fake_process.poll.return_value = None

    with patch(
        "ylhb_mobile_bridge.process_manager.subprocess.Popen",
        return_value=fake_process,
    ):
        manager.start("bringup")

    with (
        patch("ylhb_mobile_bridge.process_manager.os.getpgid", return_value=456),
        patch("ylhb_mobile_bridge.process_manager.os.killpg") as killpg,
    ):
        message = manager.stop("bringup")

    assert message == "bringup stopped"
    killpg.assert_called_once_with(456, signal.SIGINT)
    fake_process.wait.assert_called_once_with(timeout=8)


def test_process_manager_escalates_if_sigint_times_out(tmp_path):
    manager = make_manager(tmp_path)
    fake_process = MagicMock()
    fake_process.pid = 789
    fake_process.poll.return_value = None
    fake_process.wait.side_effect = [
        subprocess.TimeoutExpired("bringup", 8),
        None,
    ]

    with patch(
        "ylhb_mobile_bridge.process_manager.subprocess.Popen",
        return_value=fake_process,
    ):
        manager.start("bringup")

    with (
        patch("ylhb_mobile_bridge.process_manager.os.getpgid", return_value=789),
        patch("ylhb_mobile_bridge.process_manager.os.killpg") as killpg,
    ):
        message = manager.stop("bringup")

    assert message == "bringup stopped"
    assert killpg.call_args_list == [
        call(789, signal.SIGINT),
        call(789, signal.SIGTERM),
    ]
    assert fake_process.wait.call_args_list == [
        call(timeout=8),
        call(timeout=4),
    ]


def test_process_status_reports_metadata_and_log_tail(tmp_path):
    manager = make_manager(tmp_path)
    fake_process = MagicMock()
    fake_process.pid = 456
    fake_process.poll.return_value = None

    with patch(
        "ylhb_mobile_bridge.process_manager.subprocess.Popen",
        return_value=fake_process,
    ):
        manager.start("bringup")

    log_path = Path(manager.process_status("bringup")["log_path"])
    log_path.write_text("line one\nline two\n", encoding="utf-8")
    status = manager.process_status("bringup")

    assert status["mode"] == "bringup"
    assert status["command"][-1] == "bringup"
    assert status["pid"] == 456
    assert status["started_at"] is not None
    assert status["running"] is True
    assert status["returncode"] is None
    assert status["managed_by_bridge"] is True
    assert status["log_tail"] == "line one\nline two"
    manager._log_handles["bringup"].close()


def test_process_status_preserves_exit_code_after_process_exits(tmp_path):
    manager = make_manager(tmp_path)
    fake_process = MagicMock()
    fake_process.pid = 789
    fake_process.poll.return_value = 2
    manager._processes["mapping"] = fake_process
    manager._records["mapping"] = {
        "command": ["/tmp/run_on_jetson.sh", "mapping"],
        "pid": 789,
        "started_at": 123.0,
        "log_path": str(tmp_path / "mapping.log"),
    }

    status = manager.process_status("mapping")

    assert status["running"] is False
    assert status["pid"] == 789
    assert status["returncode"] == 2
    assert status["managed_by_bridge"] is True


def test_process_status_marks_unmanaged_mode(tmp_path):
    status = make_manager(tmp_path).process_status("mapping")

    assert status == {
        "mode": "mapping",
        "command": None,
        "pid": None,
        "started_at": None,
        "running": False,
        "returncode": None,
        "log_path": None,
        "log_tail": "",
        "managed_by_bridge": False,
    }
