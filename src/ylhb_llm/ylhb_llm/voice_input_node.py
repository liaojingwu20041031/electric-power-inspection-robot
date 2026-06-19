import os
import subprocess
import tempfile
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .qwen_client import QwenClient, QwenClientError


class VoiceInputNode(Node):
    def __init__(self) -> None:
        super().__init__('voice_input_node')
        self.declare_parameter('text_command_topic', '/inspection_ai/text_command')
        self.declare_parameter('capture_voice_service_name', '/inspection_ai/capture_voice')
        self.declare_parameter('audio_device', 'default')
        self.declare_parameter('audio_input_device', 'default')
        self.declare_parameter('record_sec', 4.0)
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('enabled', False)
        self.declare_parameter('dashscope_base_url', 'https://dashscope.aliyuncs.com/compatible-mode/v1')
        self.declare_parameter('asr_model', 'qwen3-asr-flash')
        self.declare_parameter('request_timeout_sec', 15.0)

        self.enabled = bool(self.get_parameter('enabled').value)
        input_device = str(self.get_parameter('audio_input_device').value)
        legacy_device = str(self.get_parameter('audio_device').value)
        self.audio_device = input_device if input_device and input_device != 'default' else legacy_device
        self.record_sec = float(self.get_parameter('record_sec').value)
        self.sample_rate = int(self.get_parameter('sample_rate').value)
        self.asr_model = self.get_parameter('asr_model').value
        self.request_timeout_sec = float(self.get_parameter('request_timeout_sec').value)
        self.qwen = QwenClient(self.get_parameter('dashscope_base_url').value)
        self.text_pub = self.create_publisher(String, self.get_parameter('text_command_topic').value, 10)
        self.capture_lock = threading.Lock()
        self.create_service(
            Trigger,
            self.get_parameter('capture_voice_service_name').value,
            self.capture_voice_callback,
        )
        self.get_logger().info(
            f'Voice input started: enabled={self.enabled}, device={self.audio_device}, '
            f'record_sec={self.record_sec}'
        )

    def capture_voice_callback(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if not self.enabled:
            response.success = False
            response.message = '语音输入未启用，请用 enable_voice:=true 启动。'
            return response
        if not self.qwen.available():
            response.success = False
            response.message = 'DASHSCOPE_API_KEY 未设置，无法调用云端 ASR。'
            return response
        if not self.capture_lock.acquire(blocking=False):
            response.success = False
            response.message = '上一段语音仍在录制或识别中，请稍后再试。'
            return response

        try:
            text = self.capture_once()
        finally:
            self.capture_lock.release()
        if not text:
            response.success = False
            response.message = '未识别到有效语音。'
            return response
        msg = String()
        msg.data = text
        self.text_pub.publish(msg)
        self.get_logger().info(f'ASR command: {text}')
        response.success = True
        response.message = text
        return response

    def capture_once(self) -> str:
        with tempfile.NamedTemporaryFile(prefix='ylhb_asr_', suffix='.wav', delete=False) as f:
            audio_path = f.name
        try:
            if not self.record_audio(audio_path):
                return ''
            try:
                return self.qwen.transcribe_audio(
                    audio_path=audio_path,
                    model=self.asr_model,
                    timeout_sec=self.request_timeout_sec,
                ).strip()
            except QwenClientError as exc:
                self.get_logger().warn(f'语音识别失败：{exc}')
                return ''
        finally:
            try:
                os.unlink(audio_path)
            except OSError:
                pass

    def record_audio(self, audio_path: str) -> bool:
        cmd = [
            'arecord',
            '-q',
            '-f', 'S16_LE',
            '-r', str(self.sample_rate),
            '-c', '1',
            '-d', str(max(1, int(round(self.record_sec)))),
        ]
        if self.audio_device and self.audio_device != 'default':
            cmd.extend(['-D', self.audio_device])
        cmd.append(audio_path)
        try:
            subprocess.run(cmd, check=True, timeout=self.record_sec + 2.0)
            return True
        except Exception as exc:
            self.get_logger().warn(f'arecord failed: {exc}')
            return False

    def destroy_node(self) -> bool:
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoiceInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
