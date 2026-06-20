#!/usr/bin/env python3

import argparse
import os
import shutil
import subprocess
from pathlib import Path


def workspace_path(*parts):
    workspace_dir = os.environ.get('WS_DIR', os.path.expanduser('~/ros2_DL'))
    return str(Path(workspace_dir).joinpath(*parts))


def parse_args():
    parser = argparse.ArgumentParser(
        description='Compile a PC-exported YOLO ONNX model into a TensorRT FP16 engine on Jetson.'
    )
    parser.add_argument(
        '--onnx',
        default=workspace_path('src', 'ylhb_perception', 'models', 'yolo26.onnx'),
        help='Input ONNX model path copied from the PC.',
    )
    parser.add_argument(
        '--output',
        default=workspace_path('src', 'ylhb_perception', 'models', 'yolo26.engine'),
        help='Output TensorRT .engine path for Jetson runtime.',
    )
    parser.add_argument(
        '--trtexec',
        default=None,
        help='Path to trtexec. Defaults to PATH lookup, then /usr/src/tensorrt/bin/trtexec.',
    )
    parser.add_argument(
        '--workspace',
        type=int,
        default=2048,
        help='TensorRT workspace memory in MiB.',
    )
    parser.add_argument(
        '--input-shape',
        default='',
        help='Optional dynamic input shape, for example images:1x3x960x960.',
    )
    parser.add_argument(
        '--fp32',
        action='store_true',
        help='Disable FP16 and build an FP32 engine. FP16 is the default.',
    )
    parser.add_argument(
        '--verbose',
        action='store_true',
        help='Enable verbose TensorRT builder logs.',
    )
    return parser.parse_args()


def resolve_trtexec(path_arg):
    if path_arg:
        path = Path(path_arg).expanduser().resolve()
        if path.exists():
            return str(path)
        raise FileNotFoundError(f'trtexec does not exist: {path}')

    path = shutil.which('trtexec')
    if path:
        return path

    default_path = Path('/usr/src/tensorrt/bin/trtexec')
    if default_path.exists():
        return str(default_path)

    raise FileNotFoundError(
        'Could not find trtexec. Install TensorRT tools or pass --trtexec /path/to/trtexec.'
    )


def main():
    args = parse_args()
    onnx_path = Path(args.onnx).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    trtexec = resolve_trtexec(args.trtexec)

    if not onnx_path.exists():
        raise FileNotFoundError(f'Input ONNX model does not exist: {onnx_path}')

    output_path.parent.mkdir(parents=True, exist_ok=True)

    cmd = [
        trtexec,
        f'--onnx={onnx_path}',
        f'--saveEngine={output_path}',
        f'--memPoolSize=workspace:{args.workspace}',
    ]
    if not args.fp32:
        cmd.append('--fp16')
    if args.input_shape:
        cmd.append(f'--shapes={args.input_shape}')
    if args.verbose:
        cmd.append('--verbose')

    print('[TRT] Compile configuration:')
    print(f'  onnx:      {onnx_path}')
    print(f'  output:    {output_path}')
    print(f'  trtexec:   {trtexec}')
    print(f'  workspace: {args.workspace} MiB')
    print(f'  precision: {"FP32" if args.fp32 else "FP16"}')
    if args.input_shape:
        print(f'  shapes:    {args.input_shape}')

    subprocess.run(cmd, check=True)

    if not output_path.exists():
        raise RuntimeError(f'TensorRT build finished but engine was not created: {output_path}')

    print('[TRT] Compile finished.')
    print(f'[TRT] Engine path: {output_path}')
    print('[TRT] Runtime example:')
    print(
        '  ros2 launch ylhb_perception perception.launch.py '
        f'model_path:={output_path} backend:=tensorrt imgsz:=960 half:=true'
    )


if __name__ == '__main__':
    main()
