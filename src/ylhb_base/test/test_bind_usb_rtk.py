from pathlib import Path


BIND_USB_PATH = Path(__file__).resolve().parents[2] / "bind_usb.sh"


def test_rtk_udev_rule_matches_data_interface():
    script = BIND_USB_PATH.read_text(encoding="utf-8")

    assert 'ATTRS{idVendor}=="19d1"' in script
    assert 'ATTRS{idProduct}=="0001"' in script
    assert 'ENV{ID_USB_INTERFACE_NUM}=="06"' in script
    assert 'bInterfaceNumber' in script
    assert 'SYMLINK+="rtk_4g"' in script


def test_hardware_guard_refreshes_rtk_alias():
    script = BIND_USB_PATH.read_text(encoding="utf-8")

    assert "RTK_VID=\"19d1\"" in script
    assert "RTK_PID=\"0001\"" in script
    assert "RTK_INTERFACE=\"06\"" in script
    assert "refresh_rtk_alias /dev/rtk_4g" in script
