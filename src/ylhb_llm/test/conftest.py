import sys
import types
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))
MOBILE_BRIDGE_ROOT = PACKAGE_ROOT.parent / 'ylhb_mobile_bridge'
if str(MOBILE_BRIDGE_ROOT) not in sys.path:
    sys.path.insert(0, str(MOBILE_BRIDGE_ROOT))

if 'ylhb_interfaces.msg' not in sys.modules:
    fake_interfaces = types.ModuleType('ylhb_interfaces')
    fake_msg = types.ModuleType('ylhb_interfaces.msg')
    fake_msg.SayText = type('SayText', (), {})
    fake_msg.TaskEvent = type('TaskEvent', (), {})
    fake_msg.TaskStatus = type('TaskStatus', (), {})
    fake_msg.VoiceStatus = type('VoiceStatus', (), {})
    fake_interfaces.msg = fake_msg
    sys.modules['ylhb_interfaces'] = fake_interfaces
    sys.modules['ylhb_interfaces.msg'] = fake_msg
