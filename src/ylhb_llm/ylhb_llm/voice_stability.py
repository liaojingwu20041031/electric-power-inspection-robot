import wave
from pathlib import Path
from typing import Optional


def normalize_voice_text(text: str) -> str:
    table = str.maketrans('', '', ' ，。！？!?、,. ')
    cleaned = text.strip().translate(table)
    for filler in ('呃', '嗯', '啊'):
        cleaned = cleaned.replace(filler, '')
    return cleaned


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
        size = Path(audio_path).stat().st_size
    except OSError:
        return None
    bytes_per_second = sample_rate * sample_width * channels
    if bytes_per_second <= 0 or size <= 0:
        return None
    return max(0.1, (max(0, size - 44) / float(bytes_per_second)))
