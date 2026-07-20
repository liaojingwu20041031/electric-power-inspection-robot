#!/usr/bin/env bash
set -euo pipefail

MODEL_NAME=sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20
MODEL_ROOT="${HOME}/.local/share/ylhb/kws"
ARCHIVE="${MODEL_ROOT}/${MODEL_NAME}.tar.bz2"
MODEL_DIR="${MODEL_ROOT}/${MODEL_NAME}"
URL="https://github.com/k2-fsa/sherpa-onnx/releases/download/kws-models/${MODEL_NAME}.tar.bz2"
SHA256=68447f4fbc67e70eee3a93961f36e81e98f47aef73ce7e7ca00885c6cd3616a6

if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "仅支持 Jetson aarch64，当前架构：$(uname -m)" >&2
  exit 1
fi

python3 -m pip install --user --only-binary=:all: \
  sherpa-onnx==1.13.4 \
  sentencepiece==0.2.2 \
  pypinyin==0.55.0

mkdir -p "${MODEL_ROOT}"
if [[ ! -d "${MODEL_DIR}" ]]; then
  curl -L --fail --retry 3 -o "${ARCHIVE}" "${URL}"
  echo "${SHA256}  ${ARCHIVE}" | sha256sum --check --status
  tar -xjf "${ARCHIVE}" -C "${MODEL_ROOT}"
  rm -f "${ARCHIVE}"
fi

printf '%s\n' \
  'x iǎo l íng x iǎo l íng @小零小零' \
  'x iǎo l ín x iǎo l ín @小林小林' \
  > "${MODEL_DIR}/keywords.txt"
python3 "$(dirname "$0")/check_local_kws.py"
