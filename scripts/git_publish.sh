#!/usr/bin/env bash
set -euo pipefail

usage() {
  printf '用法: %s "提交说明"\n' "$0"
}

die() {
  printf '错误: %s\n' "$*" >&2
  exit 1
}

confirm() {
  local answer
  printf '\n确认执行 git add -A、commit 并推送到远程仓库吗？[y/N] '
  read -r answer
  case "$answer" in
    y|Y|yes|YES) return 0 ;;
    *) return 1 ;;
  esac
}

git rev-parse --is-inside-work-tree >/dev/null 2>&1 || die "当前目录不在 git 仓库中"

repo_root="$(git rev-parse --show-toplevel)"
cd "$repo_root"

branch="$(git branch --show-current)"
[ -n "$branch" ] || die "当前处于 detached HEAD 状态，脚本不支持直接发布"

remote_url="$(git remote get-url origin 2>/dev/null || true)"
[ -n "$remote_url" ] || die "未配置远程仓库 origin"

message="${1:-}"
if [ -z "$message" ]; then
  printf '提交说明: '
  read -r message
fi
[ -n "$message" ] || die "必须提供提交说明"

if git diff --quiet && git diff --cached --quiet; then
  printf '没有需要发布的改动。\n'
  exit 0
fi

printf '仓库:   %s\n' "$repo_root"
printf '分支:   %s\n' "$branch"
printf '远程:   %s\n' "$remote_url"
printf '\n工作区状态:\n'
git status --short

printf '\n改动摘要:\n'
git diff --stat
git diff --cached --stat

if ! confirm; then
  printf '已取消。脚本没有暂存、提交或推送任何改动。\n'
  exit 0
fi

git add -A

if git diff --cached --quiet; then
  printf '执行 git add -A 后没有可提交的暂存改动。\n'
  exit 0
fi

git commit -m "$message"
git push origin "$branch"

printf '\n已发布到 origin/%s。\n' "$branch"
