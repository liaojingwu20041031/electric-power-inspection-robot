#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 /home/nvidia/dg202611_ws_<commit>" >&2
  exit 2
fi

requested="$1"
release_dir="$(readlink -f "${requested}")"
active_link="/home/nvidia/dg202611_ws"

case "${release_dir}" in
  /home/nvidia/dg202611_ws_*) ;;
  *)
    echo "Refusing release outside /home/nvidia/dg202611_ws_*: ${release_dir}" >&2
    exit 2
    ;;
esac

for required in \
  DEPLOYMENT_RECORD.json \
  deploy_preflight.log \
  install/setup.bash \
  scripts/run_dg_models_service.sh; do
  if [ ! -f "${release_dir}/${required}" ]; then
    echo "Release is not deploy-ready; missing ${release_dir}/${required}" >&2
    exit 3
  fi
done

if [ -e "${active_link}" ] && [ ! -L "${active_link}" ]; then
  echo "Refusing to replace non-symlink path: ${active_link}" >&2
  exit 4
fi

previous="$(readlink -f "${active_link}" 2>/dev/null || true)"
temporary="${active_link}.next.$$"
trap 'rm -f "${temporary}"' EXIT
ln -s "${release_dir}" "${temporary}"
mv -Tf "${temporary}" "${active_link}"
trap - EXIT

echo "ACTIVE_DG_RELEASE=${release_dir}"
echo "PREVIOUS_DG_RELEASE=${previous:-none}"
echo "The protected base workspace /home/nvidia/ros2_DL was not changed."
if systemctl is-active --quiet dg202611-models.service; then
  echo "Restart required to load this release: sudo systemctl restart dg202611-models.service"
fi
