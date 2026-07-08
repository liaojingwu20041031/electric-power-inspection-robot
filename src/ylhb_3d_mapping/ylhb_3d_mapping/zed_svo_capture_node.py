import json
import threading
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .zed_svo_tools import capture_svo


def latched_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class ZedSvoCaptureNode(Node):
    def __init__(self) -> None:
        super().__init__('zed_svo_capture_node')
        self.declare_parameter('output_root', '/home/nvidia/ros2_DL/runs/3d_capture')
        self.declare_parameter('duration_sec', 0.0)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('exit_on_finish', False)
        self.declare_parameter('command_topic', '/inspection_ai/mapping3d_capture_command')
        self.declare_parameter('status_topic', '/inspection_ai/mapping3d_status')
        self.declare_parameter('result_topic', '/inspection_ai/mapping3d_result')
        self.stop_requested = False
        self.worker: Optional[threading.Thread] = None
        self.status_pub = self.create_publisher(String, str(self.get_parameter('status_topic').value), latched_qos())
        self.result_pub = self.create_publisher(String, str(self.get_parameter('result_topic').value), latched_qos())
        self.create_subscription(String, str(self.get_parameter('command_topic').value), self.command_callback, 10)
        if bool(self.get_parameter('auto_start').value):
            self.start_capture()

    def command_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {'command': msg.data}
        command = str(payload.get('command') or '').strip()
        if command == 'start':
            self.start_capture()
        elif command == 'stop':
            self.stop_requested = True
            self.publish_status({'state': 'stopping', 'message': 'stop requested'})

    def capture_args(self) -> List[str]:
        return [
            f'output_root:={self.get_parameter("output_root").value}',
            f'duration_sec:={self.get_parameter("duration_sec").value}',
        ]

    def start_capture(self) -> None:
        if self.worker and self.worker.is_alive():
            self.publish_status({'state': 'recording', 'message': 'capture already running'})
            return
        self.stop_requested = False
        self.worker = threading.Thread(target=self._run_capture, daemon=True)
        self.worker.start()

    def publish_status(self, payload: Dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)

    def publish_result(self, payload: Dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.result_pub.publish(msg)

    def _run_capture(self) -> None:
        try:
            metadata = capture_svo(
                self.capture_args(),
                stop_requested=lambda: self.stop_requested,
                status_callback=self.publish_status,
                final_state_on_stop='succeeded',
            )
            self.publish_status(metadata)
            self.publish_result(metadata)
        except Exception as exc:
            payload = {'schema_version': '1.0', 'state': 'failed', 'message': str(exc)}
            self.publish_status(payload)
            self.publish_result(payload)
        finally:
            if bool(self.get_parameter('exit_on_finish').value):
                rclpy.shutdown()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = ZedSvoCaptureNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
