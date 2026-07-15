import os
import subprocess
from pathlib import Path


REPO = Path(__file__).resolve().parents[3]
SCRIPT = REPO / "scripts" / "wifi_reconnect_once.sh"


def run_reconnect(tmp_path, scenario):
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir(parents=True)
    calls = tmp_path / "calls"
    (bin_dir / "flock").write_text(
        "#!/bin/sh\nshift 4\nexec \"$@\"\n", encoding="utf-8"
    )
    (bin_dir / "nmcli").write_text(
        """#!/bin/sh
printf '%s\n' "$*" >> "$CALLS"
case "$*" in
  "general status") echo "connected" ;;
  "radio wifi") [ "$SCENARIO" = radio_off ] && echo disabled || echo enabled ;;
  "-t -f DEVICE,TYPE,STATE device")
    [ "$SCENARIO" = active ] && echo "wlan0:wifi:connected" || echo "wlan0:wifi:disconnected"
    ;;
  "-g GENERAL.CONNECTION device show wlan0")
    [ "$SCENARIO" = active ] && echo hotspot
    [ "$SCENARIO" = other ] && echo other
    ;;
  "-g IP4.ADDRESS device show wlan0") echo "192.168.137.100/24" ;;
  "-g 802-11-wireless.ssid connection show hotspot") echo "robot-hotspot" ;;
  "device wifi rescan ifname wlan0") ;;
  "-g SSID device wifi list ifname wlan0")
    [ "$SCENARIO" = visible ] && echo "robot-hotspot" || echo "other-ssid"
    ;;
  "--wait 20 connection up hotspot ifname wlan0") ;;
  *) echo "unexpected nmcli call: $*" >&2; exit 1 ;;
esac
""",
        encoding="utf-8",
    )
    for path in bin_dir.iterdir():
        path.chmod(0o755)
    env = {
        **os.environ,
        "PATH": f"{bin_dir}:{os.environ['PATH']}",
        "CALLS": str(calls),
        "SCENARIO": scenario,
        "XDG_RUNTIME_DIR": str(tmp_path),
        "YLHB_WIFI_CONNECTION": "hotspot",
        "YLHB_WIFI_INTERFACE": "wlan0",
    }
    result = subprocess.run(
        [str(SCRIPT)], capture_output=True, text=True, env=env, check=False
    )
    return result, calls.read_text(encoding="utf-8")


def test_active_target_is_not_reactivated(tmp_path):
    result, calls = run_reconnect(tmp_path, "active")

    assert result.returncode == 0
    assert "connection up" not in calls


def test_invisible_hotspot_waits_without_error(tmp_path):
    result, calls = run_reconnect(tmp_path, "invisible")

    assert result.returncode == 0
    assert "connection up" not in calls
    assert "error" not in result.stderr.lower()


def test_visible_hotspot_only_activates_target_profile(tmp_path):
    result, calls = run_reconnect(tmp_path, "visible")

    assert result.returncode == 0
    assert "--wait 20 connection up hotspot ifname wlan0" in calls
    assert "connection down" not in calls


def test_radio_off_or_other_wifi_does_not_take_over_interface(tmp_path):
    for scenario in ("radio_off", "other"):
        result, calls = run_reconnect(tmp_path / scenario, scenario)

        assert result.returncode == 0
        assert "connection up" not in calls
        assert "connection down" not in calls
        assert "radio wifi on" not in calls
