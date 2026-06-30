import audioop
from collections import deque
import json
import os
import subprocess
import tempfile
import threading
import time
import wave
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ylhb_interfaces.msg import SayText, VoiceStatus

from .qwen_client import QwenClient, QwenClientError
from .voice_stability import normalize_voice_text
from .ros_params import declare_string_array_parameter


ASR_TIMEOUT_MARKER = '__ASR_TIMEOUT__'


def transient_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class VoiceSessionNode(Node):
    def __init__(self) -> None:
        super().__init__('voice_session_node')
        self.declare_parameter('agent_request_topic', '/inspection_ai/agent_request')
        self.declare_parameter('voice_session_status_topic', '/inspection_ai/voice_session_status')
        self.declare_parameter('say_text_topic', '/inspection_ai/say_text')
        self.declare_parameter('voice_status_topic', '/inspection_ai/voice_status')
        self.declare_parameter('task_context_status_topic', '/inspection_ai/task_context_status')
        self.declare_parameter('start_voice_session_service_name', '/inspection_ai/start_voice_session')
        self.declare_parameter('stop_voice_session_service_name', '/inspection_ai/stop_voice_session')
        self.declare_parameter('audio_device', 'default')
        self.declare_parameter('audio_input_device', 'default')
        self.declare_parameter('enabled', False)
        self.declare_parameter('dashscope_base_url', 'https://dashscope.aliyuncs.com/compatible-mode/v1')
        self.declare_parameter('asr_model', 'qwen3-asr-flash')
        self.declare_parameter('request_timeout_sec', 15.0)
        self.declare_parameter('wake_phrase', '小零小零')
        self.declare_parameter('wake_aliases', ['小零小零', '小玲小玲', '小灵小灵', '小林小林', '小零', '小玲'])
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('frame_ms', 30)
        self.declare_parameter('energy_threshold', 550)
        self.declare_parameter('vad_silence_sec', 1.0)
        self.declare_parameter('min_voice_sec', 0.5)
        self.declare_parameter('max_utterance_sec', 8.0)
        self.declare_parameter('session_idle_timeout_sec', 35.0)
        self.declare_parameter('max_listen_wait_sec', 8.0)
        self.declare_parameter('voice_start_frames_required', 3)
        self.declare_parameter('asr_empty_silent_first', True)
        self.declare_parameter('asr_fail_prompt_threshold', 2)
        self.declare_parameter('asr_fail_standby_threshold', 3)
        self.declare_parameter('post_event_listen_pause_sec', 3.0)
        self.declare_parameter('ignore_empty_asr_after_event_sec', 6.0)
        self.declare_parameter('context_followup_timeout_sec', 8.0)
        self.declare_parameter('start_prompt_cooldown_sec', 8.0)
        self.declare_parameter('repeat_start_feedback', False)
        self.declare_parameter('pre_roll_sec', 0.4)
        self.declare_parameter('command_vad_silence_sec', 1.25)
        self.declare_parameter('command_min_voice_sec', 0.8)
        self.declare_parameter('wait_wake_threshold_multiplier', 1.8)
        self.declare_parameter('tts_tail_pause_sec', 0.9)
        self.declare_parameter('debug_save_asr_audio', False)
        self.declare_parameter('debug_audio_dir', '/tmp/ylhb_voice_debug')
        self.declare_parameter('debug_audio_keep', 20)
        self.declare_parameter('debug_state_transitions', False)
        declare_string_array_parameter(self, 'voice_close_words')

        self.enabled = bool(self.get_parameter('enabled').value)
        input_device = str(self.get_parameter('audio_input_device').value)
        legacy_device = str(self.get_parameter('audio_device').value)
        self.audio_device = input_device if input_device and input_device != 'default' else legacy_device
        self.asr_model = str(self.get_parameter('asr_model').value)
        self.request_timeout_sec = float(self.get_parameter('request_timeout_sec').value)
        self.wake_phrase = str(self.get_parameter('wake_phrase').value)
        self.wake_aliases = [str(v) for v in self.get_parameter('wake_aliases').value]
        self.sample_rate = int(self.get_parameter('sample_rate').value)
        self.frame_ms = int(self.get_parameter('frame_ms').value)
        self.energy_threshold = int(self.get_parameter('energy_threshold').value)
        self.vad_silence_sec = float(self.get_parameter('vad_silence_sec').value)
        self.min_voice_sec = float(self.get_parameter('min_voice_sec').value)
        self.max_utterance_sec = float(self.get_parameter('max_utterance_sec').value)
        self.session_idle_timeout_sec = float(self.get_parameter('session_idle_timeout_sec').value)
        self.max_listen_wait_sec = float(self.get_parameter('max_listen_wait_sec').value)
        self.voice_start_frames_required = max(
            1,
            int(self.get_parameter('voice_start_frames_required').value),
        )
        self.asr_empty_silent_first = bool(self.get_parameter('asr_empty_silent_first').value)
        self.asr_fail_prompt_threshold = int(self.get_parameter('asr_fail_prompt_threshold').value)
        self.asr_fail_standby_threshold = int(self.get_parameter('asr_fail_standby_threshold').value)
        self.post_event_listen_pause_sec = float(self.get_parameter('post_event_listen_pause_sec').value)
        self.ignore_empty_asr_after_event_sec = float(
            self.get_parameter('ignore_empty_asr_after_event_sec').value)
        self.context_followup_timeout_sec = float(
            self.get_parameter('context_followup_timeout_sec').value)
        self.start_prompt_cooldown_sec = float(self.get_parameter('start_prompt_cooldown_sec').value)
        self.repeat_start_feedback = bool(self.get_parameter('repeat_start_feedback').value)
        self.pre_roll_sec = float(self.get_parameter('pre_roll_sec').value)
        self.command_vad_silence_sec = float(self.get_parameter('command_vad_silence_sec').value)
        self.command_min_voice_sec = float(self.get_parameter('command_min_voice_sec').value)
        self.wait_wake_threshold_multiplier = float(
            self.get_parameter('wait_wake_threshold_multiplier').value)
        self.tts_tail_pause_sec = float(self.get_parameter('tts_tail_pause_sec').value)
        self.debug_save_asr_audio = bool(self.get_parameter('debug_save_asr_audio').value)
        self.debug_audio_dir = str(self.get_parameter('debug_audio_dir').value)
        self.debug_audio_keep = int(self.get_parameter('debug_audio_keep').value)
        self.debug_state_transitions = bool(self.get_parameter('debug_state_transitions').value)
        self.voice_close_words = tuple(
            str(value)
            for value in self.get_parameter('voice_close_words').value
            if str(value)
        )

        self.qwen = QwenClient(str(self.get_parameter('dashscope_base_url').value))
        self.agent_request_pub = self.create_publisher(String, self.get_parameter('agent_request_topic').value, 10)
        self.status_pub = self.create_publisher(
            String, self.get_parameter('voice_session_status_topic').value, transient_qos())
        self.say_pub = self.create_publisher(SayText, self.get_parameter('say_text_topic').value, 10)
        self.create_subscription(
            VoiceStatus,
            self.get_parameter('voice_status_topic').value,
            self.voice_status_callback,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter('task_context_status_topic').value,
            self.task_context_status_callback,
            transient_qos(),
        )
        self.create_service(
            Trigger,
            self.get_parameter('start_voice_session_service_name').value,
            self.start_callback,
        )
        self.create_service(
            Trigger,
            self.get_parameter('stop_voice_session_service_name').value,
            self.stop_callback,
        )

        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.session_enabled = False
        self.awakened = False
        self.session_id = ''
        self.utterance_seq = 0
        self.state = 'OFF'
        self.last_logged_state = ''
        self.is_tts_playing = False
        self.last_tts_speaking = False
        self.is_recording = False
        self.asr_fail_count = 0
        self.last_asr_text = ''
        self.last_published_text = ''
        self.last_error = ''
        self.last_active_at = 0.0
        self.pause_listen_until = 0.0
        self.last_request_published_at = 0.0
        self.context_followup_until = 0.0
        self.in_context_followup = False
        self.last_start_prompt_at = 0.0
        self.last_say_text = ''
        self.last_say_at = 0.0
        self.last_recording_stats = {}
        self.last_status_payload_json = ''

        self.worker = threading.Thread(target=self.session_loop, daemon=True)
        self.worker.start()
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info(
            f'连续语音节点已启动：enabled={self.enabled}, 录音设备={self.audio_device}, '
            f'唤醒词={self.wake_phrase}'
        )

    def start_callback(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if not self.enabled:
            response.success = False
            response.message = '连续语音模式未启用，请用 enable_voice:=true 启动。'
            return response
        if not self.qwen.available():
            response.success = False
            response.message = 'DASHSCOPE_API_KEY 未设置，无法调用云端 ASR。'
            return response
        with self.lock:
            if self.session_enabled:
                now = time.monotonic()
                if self.repeat_start_feedback and now - self.last_start_prompt_at >= self.start_prompt_cooldown_sec:
                    self.say('voice_session', '语音模式已经开启。', priority=5)
                    self.last_start_prompt_at = now
                response.success = True
                response.message = '语音模式已经开启。'
                return response
            self.session_enabled = True
            self.awakened = False
            self.session_id = time.strftime('voice_%Y%m%d_%H%M%S')
            self.utterance_seq = 0
            self.asr_fail_count = 0
            self.last_error = ''
            self.last_active_at = time.monotonic()
            self.pause_listen_until = 0.0
            self.last_request_published_at = 0.0
            self.context_followup_until = 0.0
            self.in_context_followup = False
            self.set_state('WAIT_WAKE')
        self.say('voice_session', f'语音模式已开启，请先说{self.wake_phrase}。', priority=6)
        self.last_start_prompt_at = time.monotonic()
        self.publish_status(force=True)
        response.success = True
        response.message = '语音模式已开启。'
        return response

    def stop_callback(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.stop_session('语音模式已关闭。', say=True)
        response.success = True
        response.message = '语音模式已关闭。'
        return response

    def voice_status_callback(self, msg: VoiceStatus) -> None:
        speaking = bool(msg.speaking)
        now = time.monotonic()
        if speaking:
            self.pause_listen_until = max(self.pause_listen_until, now + 0.5)
        if self.last_tts_speaking and not speaking:
            self.last_active_at = now
            self.pause_listen_until = max(self.pause_listen_until, now + self.tts_tail_pause_sec)
            if self.in_context_followup:
                self.context_followup_until = now + self.context_followup_timeout_sec
        self.is_tts_playing = speaking
        self.last_tts_speaking = speaking

    def task_context_status_callback(self, msg: String) -> None:
        if not self.session_enabled:
            self.in_context_followup = False
            self.context_followup_until = 0.0
            return
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        state = str(status.get('state') or '')
        waiting_for = str(status.get('waiting_for') or '')
        active_task_id = str(status.get('active_task_id') or '')
        active = state not in ('', 'idle', 'ready') or bool(active_task_id)
        if active and (waiting_for or active_task_id or state):
            now = time.monotonic()
            self.context_followup_until = now + self.context_followup_timeout_sec
            self.in_context_followup = True
            self.last_active_at = now
            self.set_state('CONTEXT_FOLLOWUP')
            return
        self.in_context_followup = False
        self.context_followup_until = 0.0

    def set_state(self, state: str) -> None:
        if self.state == state:
            return
        self.state = state
        log_states = {'OFF', 'WAIT_WAKE', 'RECORDING', 'ASR_PROCESSING'}
        if self.last_logged_state != state and (self.debug_state_transitions or state in log_states):
            self.get_logger().info(f'连续语音状态切换：{state}')
            self.last_logged_state = state
        self.publish_status(force=True)

    def session_loop(self) -> None:
        while not self.stop_event.is_set():
            if not self.session_enabled:
                time.sleep(0.1)
                continue
            if self.is_tts_playing:
                self.set_state('TTS_PAUSED')
                time.sleep(0.1)
                continue
            if time.monotonic() < self.pause_listen_until:
                self.set_state('WAITING_RESPONSE')
                time.sleep(0.05)
                continue
            if self.awakened and time.monotonic() - self.last_active_at > self.session_idle_timeout_sec:
                with self.lock:
                    self.awakened = False
                    self.set_state('WAIT_WAKE')
                self.say('voice_session', f'语音会话已待机，请说{self.wake_phrase}重新唤醒。', priority=4)
                continue

            self.update_followup_window()
            listen_without_wake = self.awakened or self.in_context_followup
            self.set_state('LISTENING' if listen_without_wake else 'WAIT_WAKE')
            idle_deadline = (
                self.last_active_at + self.session_idle_timeout_sec
                if listen_without_wake else 0.0
            )
            audio = self.wait_and_record_by_vad(idle_deadline=idle_deadline)
            if not audio:
                continue
            self.set_state('ASR_PROCESSING')
            text = self.transcribe_pcm(audio)
            if text == ASR_TIMEOUT_MARKER:
                self.save_debug_asr_audio(audio, '', 'timeout')
                self.set_state('AWAKENED_IDLE' if self.awakened else 'WAIT_WAKE')
                self.get_logger().info('Ignoring ASR timeout audio segment.')
                continue
            self.save_debug_asr_audio(audio, text, 'asr')
            if not text:
                self.handle_empty_asr()
                continue
            self.asr_fail_count = 0
            self.last_asr_text = text
            self.handle_asr_text(text)

    def handle_empty_asr(self) -> None:
        now = time.monotonic()
        if self.last_request_published_at > 0.0 and (
            now - self.last_request_published_at < self.ignore_empty_asr_after_event_sec
        ):
            self.set_state('AWAKENED_IDLE')
            self.get_logger().info('Ignoring empty ASR shortly after valid voice event.')
            return
        if not self.awakened and not self.in_context_followup:
            self.asr_fail_count = 0
            self.set_state('WAIT_WAKE')
            return
        self.asr_fail_count += 1
        if self.asr_fail_count >= self.asr_fail_standby_threshold:
            self.awakened = False
            self.in_context_followup = False
            self.context_followup_until = 0.0
            self.set_state('WAIT_WAKE')
            self.say('voice_session', f'多次没有听清，已回到待唤醒。请说{self.wake_phrase}。', priority=5)
            return
        if self.asr_empty_silent_first and self.asr_fail_count < self.asr_fail_prompt_threshold:
            self.set_state('AWAKENED_IDLE')
            return
        self.say('voice_session', '我没有听清，请再说一遍。', priority=5)

    def wait_and_record_by_vad(self, idle_deadline: float = 0.0) -> bytes:
        frame_bytes = int(self.sample_rate * 2 * self.frame_ms / 1000)
        max_frames = max(1, int(self.max_utterance_sec * 1000 / self.frame_ms))
        listen_without_wake = self.awakened or self.in_context_followup
        min_voice_sec, silence_sec = self.effective_vad_timing(listen_without_wake)
        min_frames = max(1, int(min_voice_sec * 1000 / self.frame_ms))
        silence_frames = max(1, int(silence_sec * 1000 / self.frame_ms))
        pre_roll_frames = max(0, int(self.pre_roll_sec * 1000 / self.frame_ms))
        listen_started_at = time.monotonic()
        cmd = [
            'arecord',
            '-q',
            '-f', 'S16_LE',
            '-r', str(self.sample_rate),
            '-c', '1',
            '-t', 'raw',
        ]
        if self.audio_device and self.audio_device != 'default':
            cmd.extend(['-D', self.audio_device])
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        except Exception as exc:
            self.last_error = f'arecord 启动失败: {exc}'
            self.get_logger().warn(self.last_error)
            time.sleep(1.0)
            return b''

        frames: List[bytes] = []
        pending_loud_frames: List[bytes] = []
        pre_speech_frames = deque(maxlen=pre_roll_frames)
        active = False
        quiet = 0
        loud = 0
        energy_sum = 0
        energy_peak = 0
        energy_count = 0
        effective_energy_threshold = self.energy_threshold
        effective_start_frames = self.voice_start_frames_required
        if not self.awakened and not self.in_context_followup:
            effective_energy_threshold = int(self.energy_threshold * self.wait_wake_threshold_multiplier)
            effective_start_frames = max(self.voice_start_frames_required, 6)
        try:
            while self.session_enabled and not self.stop_event.is_set():
                now = time.monotonic()
                if idle_deadline > 0.0 and now >= idle_deadline:
                    return b''
                if (
                    idle_deadline > 0.0
                    and not active
                    and self.max_listen_wait_sec > 0.0
                    and now - listen_started_at > self.max_listen_wait_sec
                ):
                    return b''
                if self.is_tts_playing:
                    return b''
                chunk = proc.stdout.read(frame_bytes) if proc.stdout else b''
                if len(chunk) < frame_bytes:
                    return b''
                energy = audioop.rms(chunk, 2)
                energy_sum += energy
                energy_peak = max(energy_peak, energy)
                energy_count += 1
                if not active:
                    if energy >= effective_energy_threshold:
                        loud += 1
                        pending_loud_frames.append(chunk)
                        if loud >= effective_start_frames:
                            active = True
                            quiet = 0
                            frames.extend(pre_speech_frames)
                            frames.extend(pending_loud_frames)
                            pending_loud_frames = []
                    else:
                        if pending_loud_frames:
                            pre_speech_frames.extend(pending_loud_frames)
                        loud = 0
                        pending_loud_frames = []
                        pre_speech_frames.append(chunk)
                    continue
                if energy >= effective_energy_threshold:
                    quiet = 0
                else:
                    quiet += 1
                if active:
                    self.set_state('RECORDING')
                    self.is_recording = True
                    frames.append(chunk)
                    if len(frames) >= max_frames or quiet >= silence_frames:
                        break
            self.last_recording_stats = {
                'duration_sec': len(frames) * self.frame_ms / 1000.0,
                'rms_avg': int(energy_sum / energy_count) if energy_count else 0,
                'rms_peak': int(energy_peak),
                'threshold': int(effective_energy_threshold),
                'phase': 'command' if listen_without_wake else 'wake',
            }
            if len(frames) < min_frames:
                return b''
            return b''.join(frames)
        finally:
            self.is_recording = False
            proc.terminate()
            try:
                proc.wait(timeout=0.5)
            except subprocess.TimeoutExpired:
                proc.kill()

    def effective_vad_timing(self, listen_without_wake: bool) -> Tuple[float, float]:
        if listen_without_wake:
            return self.command_min_voice_sec, self.command_vad_silence_sec
        return self.min_voice_sec, self.vad_silence_sec

    def transcribe_pcm(self, pcm: bytes) -> str:
        with tempfile.NamedTemporaryFile(prefix='ylhb_voice_session_', suffix='.wav', delete=False) as f:
            path = f.name
        try:
            with wave.open(path, 'wb') as wav:
                wav.setnchannels(1)
                wav.setsampwidth(2)
                wav.setframerate(self.sample_rate)
                wav.writeframes(pcm)
            try:
                return self.qwen.transcribe_audio(path, self.asr_model, self.request_timeout_sec).strip()
            except QwenClientError as exc:
                error_text = str(exc).lower()
                if 'response has no text' in error_text and not self.awakened:
                    self.last_error = ''
                    return ''
                self.last_error = f'语音识别失败：{exc}'
                self.get_logger().warn(self.last_error)
                if 'timed out' in error_text or 'timeout' in error_text:
                    return ASR_TIMEOUT_MARKER
                return ''
        finally:
            try:
                os.unlink(path)
            except OSError:
                pass

    def save_debug_asr_audio(self, pcm: bytes, text: str, phase: str) -> None:
        if not self.debug_save_asr_audio or not pcm:
            return
        try:
            os.makedirs(self.debug_audio_dir, exist_ok=True)
            stats = self.last_recording_stats or {}
            safe_text = ''.join(ch if ch.isalnum() else '_' for ch in text)[:24] or 'empty'
            filename = (
                f"{time.strftime('%Y%m%d_%H%M%S')}_{phase}_"
                f"{stats.get('phase', 'unknown')}_"
                f"{float(stats.get('duration_sec') or 0.0):.2f}s_"
                f"rms{int(stats.get('rms_avg') or 0)}_peak{int(stats.get('rms_peak') or 0)}_"
                f"{safe_text}.wav"
            )
            path = os.path.join(self.debug_audio_dir, filename)
            with wave.open(path, 'wb') as wav:
                wav.setnchannels(1)
                wav.setsampwidth(2)
                wav.setframerate(self.sample_rate)
                wav.writeframes(pcm)
            self.prune_debug_audio()
            self.get_logger().info(f'已保存 ASR 调试音频：{path}')
        except Exception as exc:
            self.get_logger().warn(f'保存 ASR 调试音频失败：{exc}')

    def prune_debug_audio(self) -> None:
        keep = max(1, int(self.debug_audio_keep))
        try:
            files = [
                os.path.join(self.debug_audio_dir, name)
                for name in os.listdir(self.debug_audio_dir)
                if name.endswith('.wav')
            ]
        except OSError:
            return
        files.sort(key=lambda path: os.path.getmtime(path), reverse=True)
        for path in files[keep:]:
            try:
                os.unlink(path)
            except OSError:
                pass

    def handle_asr_text(self, raw_text: str) -> None:
        normalized = normalize_voice_text(raw_text)
        contains_wake = self.has_wake_phrase(normalized)
        command = self.strip_wake_phrase(normalized) if contains_wake else normalized
        if any(word in command for word in self.voice_close_words):
            self.stop_session('语音模式已关闭。', say=True)
            return

        self.update_followup_window()
        interaction_phase = 'wake_command'
        if self.in_context_followup and not contains_wake:
            interaction_phase = 'context_followup'
            self.last_active_at = time.monotonic()
        elif not self.awakened:
            if not contains_wake:
                self.set_state('WAIT_WAKE')
                return
            self.awakened = True
            self.last_active_at = time.monotonic()
            if not command:
                self.set_state('AWAKENED_IDLE')
                self.say('voice_session', '我在，请说。', priority=6)
                self.publish_status(force=True)
                return
        else:
            self.last_active_at = time.monotonic()

        if not command:
            self.set_state('AWAKENED_IDLE')
            return
        self.publish_agent_request(command, raw_text, contains_wake, interaction_phase)
        self.set_state('AWAKENED_IDLE')

    def publish_agent_request(
        self,
        text: str,
        raw_text: str,
        contains_wake: bool,
        interaction_phase: str,
    ) -> None:
        self.utterance_seq += 1
        utterance_id = f'utt_{self.utterance_seq:04d}'
        payload = {
            'source': 'voice',
            'schema_version': '1.0',
            'session_id': self.session_id,
            'request_id': utterance_id,
            'utterance_id': utterance_id,
            'text': text,
            'raw_asr_text': raw_text,
            'awakened': bool(self.awakened),
            'contains_wake_phrase': bool(contains_wake),
            'interaction_phase': interaction_phase,
            'confidence': 0.8,
            'timestamp': time.time(),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.agent_request_pub.publish(msg)
        self.last_published_text = text
        self.get_logger().info(f'发布语音 Agent 请求：{msg.data}')
        now = time.monotonic()
        self.last_request_published_at = now
        self.pause_listen_until = now + self.post_event_listen_pause_sec
        self.asr_fail_count = 0

    def stop_session(self, text: str, say: bool) -> None:
        with self.lock:
            self.session_enabled = False
            self.awakened = False
            self.set_state('OFF')
            self.is_recording = False
            self.pause_listen_until = 0.0
            self.last_request_published_at = 0.0
            self.context_followup_until = 0.0
            self.in_context_followup = False
        if say:
            self.say('voice_session', text, priority=6, interrupt=True)
        self.publish_status(force=True)

    def has_wake_phrase(self, text: str) -> bool:
        return any(alias and alias in text for alias in self.wake_aliases)

    def strip_wake_phrase(self, text: str) -> str:
        command = text
        for alias in sorted(self.wake_aliases, key=len, reverse=True):
            command = command.replace(alias, '')
        return command.strip()

    def update_followup_window(self) -> None:
        if self.in_context_followup and time.monotonic() >= self.context_followup_until:
            self.in_context_followup = False
            self.context_followup_until = 0.0

    def say(
        self,
        task_id: str,
        text: str,
        priority: int = 5,
        interrupt: bool = False,
    ) -> None:
        if task_id == 'voice_session':
            now = time.monotonic()
            if (
                not interrupt
                and text == self.last_say_text
                and now - self.last_say_at < self.start_prompt_cooldown_sec
            ):
                return
            self.last_say_text = text
            self.last_say_at = now
            self.pause_listen_until = max(self.pause_listen_until, now + 2.0)
        msg = SayText()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id = task_id
        msg.priority = int(priority)
        msg.interrupt = bool(interrupt)
        msg.text = text
        self.say_pub.publish(msg)

    def publish_status(self, force: bool = False) -> None:
        agent_voice_state, agent_voice_state_label = self.agent_voice_state()
        payload = {
            'enabled': bool(self.session_enabled),
            'state': self.state,
            'agent_voice_state': agent_voice_state,
            'agent_voice_state_label': agent_voice_state_label,
            'wake_phrase': self.wake_phrase,
            'awakened': bool(self.awakened),
            'session_id': self.session_id,
            'active_module': '',
            'waiting_for': (
                'context_followup'
                if self.in_context_followup
                else 'wake_phrase'
                if self.session_enabled and not self.awakened
                else ''
            ),
            'interaction_phase': 'context_followup' if self.in_context_followup else 'wake_command',
            'is_tts_playing': bool(self.is_tts_playing),
            'is_recording': bool(self.is_recording),
            'asr_fail_count': int(self.asr_fail_count),
            'last_asr_text': self.last_asr_text,
            'last_published_text': self.last_published_text,
            'last_intent': '',
            'last_target': '',
            'last_confidence': 0.0,
            'last_error': self.last_error,
        }
        payload_json = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        if not force and payload_json == self.last_status_payload_json:
            return
        self.last_status_payload_json = payload_json
        payload['last_update_time'] = time.time()
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)

    def agent_voice_state(self) -> Tuple[str, str]:
        if self.last_error:
            return 'error', '异常'
        labels = {
            'OFF': ('off', '关闭'),
            'WAIT_WAKE': ('waiting_wake', '待唤醒'),
            'LISTENING': ('listening', '正在听'),
            'AWAKENED_IDLE': ('listening', '正在听'),
            'CONTEXT_FOLLOWUP': ('listening', '正在听'),
            'RECORDING': ('recording', '录音中'),
            'ASR_PROCESSING': ('recognizing', '识别中'),
            'WAITING_RESPONSE': ('responding', '等待响应'),
            'TTS_PAUSED': ('responding', '播报中'),
        }
        return labels.get(self.state, (self.state.lower(), self.state))

    def destroy_node(self) -> bool:
        self.stop_event.set()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoiceSessionNode()
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
