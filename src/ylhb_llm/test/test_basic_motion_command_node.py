import json
import time
from types import SimpleNamespace

from geometry_msgs.msg import Twist

from ylhb_llm.basic_motion_command_node import BasicMotionCommandNode


class FakePub:
    def __init__(self, subscribers=1):
        self.messages = []
        self.subscribers = subscribers

    def publish(self, msg):
        self.messages.append(msg)

    def get_subscription_count(self):
        return self.subscribers


def make_node():
    node = BasicMotionCommandNode.__new__(BasicMotionCommandNode)
    node.linear_speed = 0.12
    node.angular_speed = 0.45
    node.motion_duration_sec = 1.0
    node.require_chassis_online = False
    node.system_mode = 'ready'
    node.active_twist = None
    node.stop_at = 0.0
    node.zlac_status = ''
    node.zlac_status_received_at = 0.0
    node.cmd_pub = FakePub()
    node.say_pub = FakePub()
    node.get_logger = lambda: SimpleNamespace(info=lambda _msg: None, warn=lambda _msg: None)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None))
    return node


def msg(data):
    return SimpleNamespace(data=data)


def test_structured_backward_command_moves():
    node = make_node()

    BasicMotionCommandNode.motion_command_callback(
        node,
        msg(json.dumps({'command': '后退'}, ensure_ascii=False)),
    )

    assert isinstance(node.cmd_pub.messages[-1], Twist)
    assert node.cmd_pub.messages[-1].linear.x == -0.12
    assert node.stop_at > time.monotonic()


def test_free_text_and_composite_command_do_not_move():
    node = make_node()

    BasicMotionCommandNode.motion_command_callback(node, msg('后退旋转'))
    BasicMotionCommandNode.motion_command_callback(
        node,
        msg(json.dumps({'command': '后退旋转'}, ensure_ascii=False)),
    )

    assert node.cmd_pub.messages == []


def test_unknown_json_command_does_not_move():
    node = make_node()

    BasicMotionCommandNode.motion_command_callback(
        node,
        msg(json.dumps({'command': '旋转'}, ensure_ascii=False)),
    )

    assert node.cmd_pub.messages == []
