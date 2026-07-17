#!/usr/bin/env bash
set -euo pipefail

DEFAULT_ENV_DIR="${HOME}/.config/ylhb"
ENV_FILE="${AGENT_ENV_FILE:-${DEFAULT_ENV_DIR}/agent.env}"
ENV_DIR="$(dirname "${ENV_FILE}")"

fail() {
  echo "ERROR: $*" >&2
  exit 2
}

validate_key() {
  local key="$1"
  [ -n "${key}" ] || fail 'DASHSCOPE_API_KEY 不能为空'
  [[ "${key}" != *$'\r'* && "${key}" != *$'\n'* ]] || fail 'DASHSCOPE_API_KEY 包含 CRLF 或换行'
  [[ "${key}" =~ ^[A-Za-z0-9._-]{8,}$ ]] || fail 'DASHSCOPE_API_KEY 格式无效'
}

validate_file() {
  [ -d "${ENV_DIR}" ] || fail "缺少 ${ENV_DIR}"
  [ "$(stat -c '%a' "${ENV_DIR}")" = 700 ] || fail "${ENV_DIR} 权限必须为 700"
  [ -f "${ENV_FILE}" ] && [ ! -L "${ENV_FILE}" ] || fail "缺少安全的 ${ENV_FILE}"
  [ "$(stat -c '%a' "${ENV_FILE}")" = 600 ] || fail "${ENV_FILE} 权限必须为 600"
  [ "$(stat -c '%u' "${ENV_FILE}")" = "$(id -u)" ] || fail "${ENV_FILE} 所有者不正确"
  mapfile -t lines <"${ENV_FILE}"
  [ "${#lines[@]}" -eq 1 ] || fail "${ENV_FILE} 必须只包含一行"
  [[ "${lines[0]}" == DASHSCOPE_API_KEY=* ]] || fail "${ENV_FILE} 缺少 DASHSCOPE_API_KEY"
  validate_key "${lines[0]#DASHSCOPE_API_KEY=}"
}

if [ "${1:-}" = '--check' ]; then
  validate_file
  echo "agent.env 只读验证通过"
  exit 0
fi
[ "$#" -eq 0 ] || fail '用法: configure_agent_env.sh [--check]'

if [ -t 0 ]; then
  IFS= read -r -s -p '请输入 DASHSCOPE_API_KEY: ' key || fail '未读取到 DASHSCOPE_API_KEY'
  printf '\n' >&2
else
  IFS= read -r -s key || fail '未读取到 DASHSCOPE_API_KEY'
fi
validate_key "${key}"

umask 077
mkdir -p "${ENV_DIR}"
chmod 700 "${ENV_DIR}"
tmp_file="$(mktemp "${ENV_DIR}/agent.env.tmp.XXXXXX")"
trap 'rm -f "${tmp_file}"' EXIT
printf 'DASHSCOPE_API_KEY=%s\n' "${key}" >"${tmp_file}"
chmod 600 "${tmp_file}"
mv -f "${tmp_file}" "${ENV_FILE}"
trap - EXIT
validate_file
echo "已安全配置 ${ENV_FILE}，只读验证通过"
