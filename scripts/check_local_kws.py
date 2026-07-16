#!/usr/bin/env python3
import argparse
import sys
from pathlib import Path


MODEL_NAME = 'sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20'


def main() -> int:
    parser = argparse.ArgumentParser(description='检查小零本地唤醒模型')
    parser.add_argument('--model-dir', default=f'~/.local/share/ylhb/kws/{MODEL_NAME}')
    parser.add_argument('--audio-device', default='default')
    parser.add_argument('--wav', help='用单声道 16-bit PCM WAV 验证唤醒')
    parser.add_argument('--microphone', action='store_true', help='交互式监听麦克风直到检出唤醒词')
    args = parser.parse_args()

    workspace = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(workspace / 'src' / 'ylhb_llm'))
    from ylhb_llm.local_wake_word import LocalWakeWord

    model = Path(args.model_dir).expanduser()
    kws = LocalWakeWord(
        encoder=str(model / 'encoder-epoch-13-avg-2-chunk-8-left-64.int8.onnx'),
        decoder=str(model / 'decoder-epoch-13-avg-2-chunk-8-left-64.onnx'),
        joiner=str(model / 'joiner-epoch-13-avg-2-chunk-8-left-64.int8.onnx'),
        tokens=str(model / 'tokens.txt'),
        keywords=str(model / 'keywords.txt'),
        audio_device=args.audio_device,
    )
    if not kws.load():
        print(kws.error, file=sys.stderr)
        return 1
    print('KWS 模型加载成功')
    if args.wav:
        detected = kws.detect_wav(args.wav)
        print('WAV 检出小零小零' if detected else 'WAV 未检出小零小零')
        return 0 if detected else 2
    if args.microphone:
        print('请说“小零小零”（Ctrl-C 退出）')
        detected = kws.listen(lambda: False)
        print('麦克风检出小零小零' if detected else f'监听结束：{kws.error}')
        return 0 if detected else 2
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
