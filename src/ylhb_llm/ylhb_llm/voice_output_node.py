import itertools
import os
import queue
import subprocess
import tempfile
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from ylhb_interfaces.msg import SayText, VoiceStatus

from .patrol_voice import PatrolVoice, VoiceRequest
from .qwen_client import QwenClient, QwenClientError
from .voice_stability import safe_wav_duration_sec


def patrol_event_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class VoiceOutputNode(Node):
    def __init__(self) -> None:
        super().__init__('voice_output_node')
        self.declare_parameter('say_text_topic', '/inspection_ai/say_text')
        self.declare_parameter('voice_status_topic', '/inspection_ai/voice_status')
        self.declare_parameter('patrol_event_topic', '/patrol/event')
        self.declare_parameter('patrol_voice_enabled', True)
        package_share = get_package_share_directory('ylhb_llm')
        self.declare_parameter(
            'patrol_voice_config_file', os.path.join(package_share, 'config', 'patrol_voice.yaml'))
        self.declare_parameter(
            'patrol_voice_inventory_dir', '~/.local/share/ylhb/patrol_voice')
        self.declare_parameter('enabled', False)
        self.declare_parameter('tts_enabled', False)
        self.declare_parameter('audio_device', 'default')
        self.declare_parameter('audio_output_device', 'default')
        self.declare_parameter('dashscope_base_url', 'https://dashscope.aliyuncs.com/compatible-mode/v1')
        self.declare_parameter('tts_model', 'qwen3-tts-flash')
        self.declare_parameter('tts_voice', 'Serena')
        self.declare_parameter('tts_language_type', 'Chinese')
        self.declare_parameter('request_timeout_sec', 5.0)
        self.declare_parameter('enable_tts_cache', True)
        self.declare_parameter('split_long_tts', True)
        self.declare_parameter('tts_segment_max_chars', 70)
        self.declare_parameter('preserve_long_task_tts_single_request', True)
        self.declare_parameter('interrupt_current_playback', True)

        self.enabled = bool(self.get_parameter('enabled').value)
        self.tts_enabled = bool(self.get_parameter('tts_enabled').value)
        output_device = str(self.get_parameter('audio_output_device').value)
        legacy_device = str(self.get_parameter('audio_device').value)
        self.audio_device = output_device if output_device and output_device != 'default' else legacy_device
        self.tts_model = self.get_parameter('tts_model').value
        self.tts_voice = str(self.get_parameter('tts_voice').value)
        self.tts_language_type = str(self.get_parameter('tts_language_type').value)
        self.request_timeout_sec = float(self.get_parameter('request_timeout_sec').value)
        self.enable_tts_cache = bool(self.get_parameter('enable_tts_cache').value)
        self.split_long_tts = bool(self.get_parameter('split_long_tts').value)
        self.tts_segment_max_chars = int(self.get_parameter('tts_segment_max_chars').value)
        self.preserve_long_task_tts_single_request = bool(
            self.get_parameter('preserve_long_task_tts_single_request').value)
        self.interrupt_current_playback = bool(
            self.get_parameter('interrupt_current_playback').value)
        self.qwen = QwenClient(self.get_parameter('dashscope_base_url').value)
        self.queue: 'queue.PriorityQueue[tuple[int, int, VoiceRequest, float]]' = queue.PriorityQueue()
        self._queue_sequence = itertools.count()
        self._queued_say_keys: set[tuple[str, str]] = set()
        self.stop_event = threading.Event()
        self.current_task_id = ''
        self.tts_cache: dict[tuple[str, str, str, str], bytes] = {}
        self.tts_cache_order: list[tuple[str, str, str, str]] = []
        self.playback_lock = threading.Lock()
        self.current_playback: subprocess.Popen | None = None
        self.playback_generation = 0

        self.status_pub = self.create_publisher(
            VoiceStatus, self.get_parameter('voice_status_topic').value, 10)
        self.create_subscription(
            SayText, self.get_parameter('say_text_topic').value, self.say_callback, 10)
        self.patrol_voice = None
        if bool(self.get_parameter('patrol_voice_enabled').value):
            config_file = str(self.get_parameter('patrol_voice_config_file').value)
            if not os.path.isabs(config_file):
                config_file = os.path.join(package_share, 'config', config_file)
            try:
                self.patrol_voice = PatrolVoice.from_file(
                    config_file,
                    os.path.join(package_share, 'assets', 'audio'),
                    os.path.expanduser(str(
                        self.get_parameter('patrol_voice_inventory_dir').value)),
                )
                self.create_subscription(
                    String,
                    self.get_parameter('patrol_event_topic').value,
                    self.patrol_event_callback,
                    patrol_event_qos(),
                )
            except Exception as exc:
                self.get_logger().warn(f'巡逻语音配置不可用，已禁用巡逻播报：{exc}')

        self.worker = threading.Thread(target=self.play_loop, daemon=True)
        self.worker.start()
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info(
            f'语音输出节点已启动：enabled={self.enabled}, tts_enabled={self.tts_enabled}, '
            f'播放设备={self.audio_device}'
        )

    def say_callback(self, msg: SayText) -> None:
        self.enqueue_request(VoiceRequest(
            task_id=str(msg.task_id),
            text=str(msg.text),
            priority=int(msg.priority),
            interrupt=bool(msg.interrupt),
        ))

    def patrol_event_callback(self, msg: String) -> None:
        try:
            request = self.patrol_voice.request_for_json(msg.data) if self.patrol_voice else None
            if request is not None:
                self.enqueue_request(request)
        except Exception as exc:
            self.get_logger().warn(f'忽略无效巡逻语音事件：{exc}')

    def enqueue_request(self, request: VoiceRequest) -> None:
        if request.interrupt:
            self.clear_queue()
            self.interrupt_playback()
        text = request.text.strip()
        key = (request.task_id, text)
        if key in self._queued_say_keys:
            return
        self._queued_say_keys.add(key)
        self.queue.put((
            -request.priority,
            next(self._queue_sequence),
            request,
            time.monotonic(),
        ))

    def play_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                _priority, _sequence, request, enqueued_at = self.queue.get(timeout=0.2)
            except queue.Empty:
                continue
            self._queued_say_keys.discard((request.task_id, request.text.strip()))
            try:
                text = request.text.strip()
                if text:
                    self.get_logger().info(f'播报请求[{request.task_id}]：{text}')
                self.play_request(request, enqueued_at)
            finally:
                self.current_task_id = ''
                self.publish_status_once()
                self.queue.task_done()

    def play_request(self, request: VoiceRequest, enqueued_at: float | None = None) -> None:
        if not self.enabled:
            return
        generation = self.playback_generation
        enqueued_at = time.monotonic() if enqueued_at is None else enqueued_at
        deadline = (
            enqueued_at + request.max_delay_sec
            if request.max_delay_sec > 0.0 else 0.0
        )
        if not self.playback_request_valid(generation, deadline):
            return
        if request.audio_path:
            if os.path.isfile(request.audio_path):
                self.play_audio_file(
                    request.audio_path, request.task_id, False, generation, deadline)
                return
            if request.inventory_path and os.path.isfile(request.inventory_path):
                self.play_audio_file(
                    request.inventory_path, request.task_id, False, generation, deadline)
                return
            self.get_logger().warn(
                f'本地巡逻语音不存在，回退 TTS：task_id={request.task_id}, '
                f'path={request.audio_path}')
        text = request.text.strip()
        if not self.tts_enabled or not text:
            self.get_logger().warn(f'TTS 不可用，丢弃播报：task_id={request.task_id}')
            return
        segments = (
            self.split_tts_segments(text, self.tts_segment_max_chars)
            if self.should_split_tts(request.task_id, text)
            else [text]
        )
        for segment in segments:
            if not self.playback_request_valid(generation, deadline):
                break
            self.speak(
                segment, request.task_id, request.inventory_path, generation, deadline)

    def speak(
        self,
        text: str,
        task_id: str,
        inventory_path: str = '',
        generation: int | None = None,
        deadline: float = 0.0,
    ) -> None:
        generation = self.playback_generation if generation is None else generation
        self.current_task_id = task_id
        self.publish_status_once()
        audio_path = ''
        delete_after = True
        try:
            if not self.qwen.available():
                self.get_logger().warn('DASHSCOPE_API_KEY 未设置，跳过 TTS 播放。')
                return
            cache_key = (self.tts_model, self.tts_voice, self.tts_language_type, text)
            audio = self.tts_cache.get(cache_key) if self.enable_tts_cache else None
            if audio is None:
                self.get_logger().info(
                    f'TTS 开始合成：task_id={task_id}, model={self.tts_model}, '
                    f'voice={self.tts_voice}, text_len={len(text)}'
                )
                audio = self.qwen.synthesize_speech_bytes(
                    text=text,
                    model=self.tts_model,
                    timeout_sec=self.request_timeout_sec,
                    voice=self.tts_voice,
                    language_type=self.tts_language_type,
                )
                if not self.playback_request_valid(generation, deadline):
                    return
                self.get_logger().info(
                    f'TTS 合成完成：task_id={task_id}, audio_bytes={len(audio) if audio else 0}'
                )
                if audio and self.enable_tts_cache:
                    self.remember_tts_cache(cache_key, audio)
            else:
                self.get_logger().info(
                    f'TTS 命中缓存：task_id={task_id}, text_len={len(text)}, audio_bytes={len(audio)}'
                )
            if not audio:
                self.get_logger().warn(f'TTS 未返回音频：task_id={task_id}')
                return
            if not self.playback_request_valid(generation, deadline):
                return
            if inventory_path:
                try:
                    audio_path = self.save_audio_inventory(audio, inventory_path)
                    delete_after = False
                    self.get_logger().info(f'巡逻语音已加入本地库存：{audio_path}')
                except OSError as exc:
                    self.get_logger().warn(f'巡逻语音库存写入失败，改用临时文件：{exc}')
            if not audio_path:
                if not self.playback_request_valid(generation, deadline):
                    return
                with tempfile.NamedTemporaryFile(
                    prefix='ylhb_tts_', suffix='.wav', delete=False
                ) as f:
                    f.write(audio)
                    audio_path = f.name
            time.sleep(0.25)
            if not self.playback_request_valid(generation, deadline):
                return
            self.play_audio_file(
                audio_path, task_id, delete_after, generation, deadline)
        except QwenClientError as exc:
            self.get_logger().warn(f'TTS 合成失败：task_id={task_id}, error={exc}')
        except subprocess.TimeoutExpired:
            self.terminate_current_playback()
            self.get_logger().warn(f'音频播放超时：task_id={task_id}, path={audio_path}')
        except Exception as exc:
            self.get_logger().warn(f'TTS 或音频播放异常：task_id={task_id}, error={exc}')
        finally:
            with self.playback_lock:
                if self.current_playback is not None and self.current_playback.poll() is not None:
                    self.current_playback = None
            self.current_task_id = ''
            self.publish_status_once()
            if audio_path and (
                delete_after
                or (inventory_path and not self.playback_request_valid(generation, deadline))
            ):
                try:
                    os.unlink(audio_path)
                except OSError:
                    pass

    def save_audio_inventory(self, audio: bytes, inventory_path: str) -> str:
        directory = os.path.dirname(inventory_path)
        os.makedirs(directory, exist_ok=True)
        temporary_path = ''
        try:
            with tempfile.NamedTemporaryFile(
                prefix='.ylhb_patrol_', suffix='.wav', dir=directory, delete=False
            ) as stream:
                stream.write(audio)
                temporary_path = stream.name
            os.replace(temporary_path, inventory_path)
            return inventory_path
        except OSError:
            if temporary_path:
                try:
                    os.unlink(temporary_path)
                except OSError:
                    pass
            raise

    def play_audio_file(
        self,
        audio_path: str,
        task_id: str,
        delete_after: bool,
        generation: int | None = None,
        deadline: float = 0.0,
    ) -> None:
        generation = self.playback_generation if generation is None else generation
        if not self.playback_request_valid(generation, deadline):
            return
        self.current_task_id = task_id
        self.publish_status_once()
        try:
            with self.playback_lock:
                if not self.playback_request_valid(generation, deadline):
                    return
                proc, play_timeout, _duration = self._start_audio_process(audio_path, task_id)
                self.current_playback = proc
            returncode = proc.wait(timeout=play_timeout)
            if returncode not in (0, -15):
                self.get_logger().warn(
                    f'音频播放失败：task_id={task_id}, aplay_exit={returncode}, '
                    f'device={self.audio_device}')
            elif returncode == 0:
                self.get_logger().info(f'音频播放完成：task_id={task_id}')
        except subprocess.TimeoutExpired:
            self.terminate_current_playback()
            self.get_logger().warn(f'音频播放超时：task_id={task_id}, path={audio_path}')
        except Exception as exc:
            self.get_logger().warn(f'音频播放异常：task_id={task_id}, error={exc}')
        finally:
            with self.playback_lock:
                if self.current_playback is not None and self.current_playback.poll() is not None:
                    self.current_playback = None
            self.current_task_id = ''
            self.publish_status_once()
            if delete_after:
                try:
                    os.unlink(audio_path)
                except OSError:
                    pass

    def _start_audio_process(
        self, audio_path: str, task_id: str
    ) -> tuple[subprocess.Popen, float, float]:
        cmd = ['aplay', '-q']
        if self.audio_device and self.audio_device != 'default':
            cmd.extend(['-D', self.audio_device])
        cmd.append(audio_path)
        duration = safe_wav_duration_sec(audio_path)
        play_timeout = min(45.0, max(8.0, duration + 5.0))
        self.get_logger().info(
            f'音频播放开始：task_id={task_id}, device={self.audio_device}, '
            f'duration={duration:.2f}s, timeout={play_timeout:.2f}s, cmd={" ".join(cmd)}')
        return subprocess.Popen(cmd), play_timeout, duration

    def should_split_tts(self, task_id: str, text: str) -> bool:
        if task_id.startswith('assistant_chat_'):
            return False
        if not self.split_long_tts:
            return False
        if self.preserve_long_task_tts_single_request and task_id.startswith(('text_', 'inspection_', 'inspect_')):
            return False
        return len(text.strip()) > self.tts_segment_max_chars

    def split_tts_segments(self, text: str, max_chars: int) -> list[str]:
        text = text.strip()
        if not text:
            return []
        max_chars = max(10, int(max_chars))
        raw_segments = []
        buf = ''
        for ch in text:
            buf += ch
            if ch in '。！？；;':
                raw_segments.append(buf.strip())
                buf = ''
        if buf.strip():
            raw_segments.append(buf.strip())

        merged = []
        current = ''
        for seg in raw_segments:
            if not current:
                current = seg
                continue
            if len(current) + len(seg) <= max_chars:
                current += seg
            else:
                merged.append(current)
                current = seg
        if current:
            merged.append(current)

        final_segments = []
        for seg in merged:
            while len(seg) > max_chars:
                final_segments.append(seg[:max_chars])
                seg = seg[max_chars:]
            if seg:
                final_segments.append(seg)
        return final_segments

    def remember_tts_cache(self, key: tuple[str, str, str, str], audio: bytes) -> None:
        self.tts_cache[key] = audio
        self.tts_cache_order.append(key)
        while len(self.tts_cache_order) > 64:
            old = self.tts_cache_order.pop(0)
            self.tts_cache.pop(old, None)

    def playback_request_valid(self, generation: int, deadline: float = 0.0) -> bool:
        return (
            not self.stop_event.is_set()
            and generation == self.playback_generation
            and (deadline <= 0.0 or time.monotonic() <= deadline)
        )

    def publish_status(self) -> None:
        msg = VoiceStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.speaking = bool(self.current_task_id)
        msg.current_task_id = self.current_task_id
        self.status_pub.publish(msg)

    def publish_status_once(self) -> None:
        self.publish_status()

    def clear_queue(self) -> None:
        self._queued_say_keys.clear()
        while True:
            try:
                self.queue.get_nowait()
                self.queue.task_done()
            except queue.Empty:
                return

    def interrupt_playback(self) -> None:
        with self.playback_lock:
            self.playback_generation += 1
        if self.interrupt_current_playback:
            self.terminate_current_playback()

    def terminate_current_playback(self) -> None:
        with self.playback_lock:
            proc = self.current_playback
        if proc is None or proc.poll() is not None:
            return
        proc.terminate()
        try:
            proc.wait(timeout=0.5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=0.5)

    def destroy_node(self) -> bool:
        self.stop_event.set()
        self.interrupt_playback()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoiceOutputNode()
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
