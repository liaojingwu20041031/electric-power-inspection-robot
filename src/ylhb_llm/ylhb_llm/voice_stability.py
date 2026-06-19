import os
import wave
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple


@dataclass(frozen=True)
class VoiceIntent:
    route: str
    text: str
    feedback: str = ''
    system_command: str = ''


@dataclass(frozen=True)
class VoiceRoutingPolicy:
    system_commands: Dict[str, Tuple[str, str]] = field(default_factory=dict)
    voice_close_words: Tuple[str, ...] = ()
    safety_words: Tuple[str, ...] = ()
    cancel_words: Tuple[str, ...] = ()
    system_feedback_words: Tuple[str, ...] = ()
    general_qa_words: Tuple[str, ...] = ()
    inspection_words: Tuple[str, ...] = ()
    background_words: Tuple[str, ...] = ()
    followup_words: Tuple[str, ...] = ()
    motion_aliases: Tuple[Tuple[str, str], ...] = ()
    incomplete_motion_words: Tuple[str, ...] = ()


def normalize_voice_text(text: str) -> str:
    table = str.maketrans('', '', ' ，。！？!?、,. ')
    cleaned = text.strip().translate(table)
    for filler in ('呃', '嗯', '啊'):
        cleaned = cleaned.replace(filler, '')
    return cleaned


def contains_any(text: str, words: Tuple[str, ...]) -> bool:
    return any(word in text for word in words)


def is_context_followup_text(text: str, policy: VoiceRoutingPolicy) -> bool:
    normalized = normalize_voice_text(text)
    if not normalized:
        return False
    return contains_any(normalized, policy.followup_words)


def classify_voice_intent(
    text: str,
    policy: VoiceRoutingPolicy,
    interaction_phase: str = 'wake_command',
    ignore_unknown_voice: bool = True,
) -> VoiceIntent:
    normalized = normalize_voice_text(text)
    if not normalized:
        return VoiceIntent('ignore', '')

    if is_close_voice_session(normalized, policy):
        return VoiceIntent('voice_close', normalized, '已关闭语音模式。')

    for phrase in sorted(policy.system_commands, key=len, reverse=True):
        if phrase == normalized or phrase in normalized:
            command, feedback = policy.system_commands[phrase]
            return VoiceIntent('system_command', normalized, feedback, command)

    if interaction_phase == 'context_followup' and not is_context_followup_text(normalized, policy):
        return VoiceIntent('ignore', normalized)

    if contains_any(normalized, policy.safety_words):
        return VoiceIntent('global_safety', '停止', '已停止。')

    motion_text = normalize_motion_command(normalized, policy.motion_aliases)
    if motion_text:
        return VoiceIntent('motion', motion_text)

    if any(word == normalized or word in normalized for word in policy.incomplete_motion_words):
        return VoiceIntent('unsupported_motion', normalized, '请说左转或右转。')

    if contains_any(normalized, policy.cancel_words):
        return VoiceIntent('global_cancel', normalized, '已取消当前任务。')
    if contains_any(normalized, policy.system_feedback_words):
        return VoiceIntent('system_feedback', normalized)
    if contains_any(normalized, policy.general_qa_words):
        return VoiceIntent('general_qa', normalized)
    if contains_any(normalized, policy.inspection_words):
        return VoiceIntent('inspection_command', normalized)
    if is_background_or_debug_talk(normalized, policy):
        return VoiceIntent('ignore', normalized)
    if ignore_unknown_voice:
        return VoiceIntent('ignore', normalized)
    return VoiceIntent('inspection_command', normalized)


def normalize_motion_command(text: str, aliases: Tuple[Tuple[str, str], ...]) -> str:
    for alias, canonical in aliases:
        if alias == text or alias in text:
            return canonical
    return ''


def is_close_voice_session(text: str, policy: VoiceRoutingPolicy) -> bool:
    return any(word == text or word in text for word in policy.voice_close_words)


def is_background_or_debug_talk(text: str, policy: VoiceRoutingPolicy) -> bool:
    if contains_any(text, policy.background_words):
        return True
    if len(text) >= 18 and not (
        contains_any(text, policy.inspection_words)
        or contains_any(text, policy.general_qa_words)
    ):
        return True
    return False


def safe_wav_duration_sec(
    audio_path: str,
    default_sec: float = 8.0,
    sample_rate: int = 16000,
    sample_width: int = 2,
    channels: int = 1,
) -> float:
    try:
        with wave.open(audio_path, 'rb') as wav:
            frames = wav.getnframes()
            rate = wav.getframerate()
            duration = frames / float(rate) if rate > 0 else 0.0
            if 0.0 < duration < 600.0:
                return duration
    except Exception:
        pass
    estimate = estimate_pcm_duration(audio_path, sample_rate, sample_width, channels)
    if estimate is not None:
        return estimate
    return float(default_sec)


def estimate_pcm_duration(audio_path: str, sample_rate: int, sample_width: int, channels: int) -> Optional[float]:
    try:
        size = os.path.getsize(audio_path)
    except OSError:
        return None
    bytes_per_second = sample_rate * sample_width * channels
    if bytes_per_second <= 0 or size <= 0:
        return None
    return max(0.1, (max(0, size - 44) / float(bytes_per_second)))
