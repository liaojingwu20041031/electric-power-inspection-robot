from __future__ import annotations

import os
import subprocess
import threading
import wave
from pathlib import Path
from typing import Callable


class LocalWakeWord:
    def __init__(
        self,
        *,
        encoder: str,
        decoder: str,
        joiner: str,
        tokens: str,
        keywords: str,
        audio_device: str = 'default',
        sample_rate: int = 16000,
        threshold: float = 0.25,
        score: float = 1.0,
    ) -> None:
        self.paths = {
            'encoder': str(Path(encoder).expanduser()),
            'decoder': str(Path(decoder).expanduser()),
            'joiner': str(Path(joiner).expanduser()),
            'tokens': str(Path(tokens).expanduser()),
            'keywords': str(Path(keywords).expanduser()),
        }
        self.audio_device = audio_device
        self.sample_rate = int(sample_rate)
        self.threshold = float(threshold)
        self.score = float(score)
        self.spotter = None
        self.error = ''
        self._proc = None
        self._lock = threading.Lock()

    @property
    def ready(self) -> bool:
        return self.spotter is not None and not self.error

    def load(self) -> bool:
        missing = [f'{name}: {path}' for name, path in self.paths.items() if not os.path.isfile(path)]
        if missing:
            self.error = 'KWS 文件不存在：' + '；'.join(missing) + '；请运行 scripts/install_local_kws.sh'
            return False
        try:
            import sherpa_onnx  # Delayed so the ROS package still imports before KWS installation.

            self.spotter = sherpa_onnx.KeywordSpotter(
                tokens=self.paths['tokens'],
                encoder=self.paths['encoder'],
                decoder=self.paths['decoder'],
                joiner=self.paths['joiner'],
                num_threads=1,
                provider='cpu',
                max_active_paths=4,
                num_trailing_blanks=1,
                keywords_file=self.paths['keywords'],
                keywords_score=self.score,
                keywords_threshold=self.threshold,
            )
        except Exception as exc:
            self.spotter = None
            self.error = f'KWS 模型加载失败：{exc}；请运行 scripts/install_local_kws.sh'
            return False
        self.error = ''
        return True

    def listen(self, should_stop: Callable[[], bool]) -> bool:
        if not self.ready:
            return False
        cmd = ['arecord', '-q', '-f', 'S16_LE', '-r', str(self.sample_rate), '-c', '1', '-t', 'raw']
        if self.audio_device and self.audio_device != 'default':
            cmd.extend(['-D', self.audio_device])
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        except Exception as exc:
            self.error = f'KWS arecord 启动失败：{exc}'
            return False
        with self._lock:
            self._proc = proc
        stream = self.spotter.create_stream()
        frame_bytes = int(self.sample_rate * 0.1) * 2
        try:
            import numpy as np

            while not should_stop():
                chunk = proc.stdout.read(frame_bytes) if proc.stdout else b''
                if len(chunk) != frame_bytes:
                    if not should_stop():
                        self.error = 'KWS 麦克风音频流中断'
                    return False
                samples = np.frombuffer(chunk, dtype=np.int16).astype(np.float32) / 32768.0
                stream.accept_waveform(self.sample_rate, samples)
                while self.spotter.is_ready(stream):
                    self.spotter.decode_stream(stream)
                    if self.spotter.get_result(stream):
                        self.spotter.reset_stream(stream)
                        return True
            return False
        finally:
            self.stop()

    def detect_wav(self, path: str) -> bool:
        if not self.ready:
            return False
        import numpy as np

        with wave.open(path, 'rb') as wav:
            if wav.getnchannels() != 1 or wav.getsampwidth() != 2:
                raise ValueError('WAV 必须是单声道 16-bit PCM')
            sample_rate = wav.getframerate()
            samples = np.frombuffer(wav.readframes(wav.getnframes()), dtype=np.int16).astype(np.float32) / 32768.0
        stream = self.spotter.create_stream()
        step = max(1, int(sample_rate * 0.1))
        for offset in range(0, len(samples), step):
            stream.accept_waveform(sample_rate, samples[offset:offset + step])
            while self.spotter.is_ready(stream):
                self.spotter.decode_stream(stream)
                if self.spotter.get_result(stream):
                    return True
        stream.accept_waveform(sample_rate, np.zeros(int(sample_rate * 0.5), dtype=np.float32))
        while self.spotter.is_ready(stream):
            self.spotter.decode_stream(stream)
            if self.spotter.get_result(stream):
                return True
        return False

    def stop(self) -> None:
        with self._lock:
            proc = self._proc
            self._proc = None
        if proc is None or proc.poll() is not None:
            return
        proc.terminate()
        try:
            proc.wait(timeout=0.5)
        except subprocess.TimeoutExpired:
            proc.kill()
