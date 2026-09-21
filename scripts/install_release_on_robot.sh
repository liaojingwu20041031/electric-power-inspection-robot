#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 <archive.tar.gz> <release_name> <commit> <expected_sha256>" >&2
}

if [ "$#" -ne 4 ]; then
  usage
  exit 2
fi

archive_path="$1"
release_name="$2"
release_commit="$3"
expected_sha256="${4,,}"
base_ws="${DG_BASE_WS:-/home/nvidia/ros2_DL}"

[[ "${release_name}" =~ ^dg202611_ws_[A-Za-z0-9._-]+$ ]] || {
  echo "Invalid release name: ${release_name}" >&2
  exit 2
}
[[ "${release_commit}" =~ ^[0-9a-f]{7,40}$ ]] || {
  echo "Invalid commit id: ${release_commit}" >&2
  exit 2
}
[[ "${expected_sha256}" =~ ^[0-9a-f]{64}$ ]] || {
  echo "Invalid SHA256: ${expected_sha256}" >&2
  exit 2
}

archive_path="$(readlink -f "${archive_path}")"
release_dir="/home/nvidia/${release_name}"

if [ ! -f "${archive_path}" ]; then
  echo "Release archive is missing: ${archive_path}" >&2
  exit 2
fi
if [ -e "${release_dir}" ] || [ -L "${release_dir}" ]; then
  echo "Refusing to overwrite existing release directory: ${release_dir}" >&2
  exit 3
fi
if [ ! -f "${base_ws}/install/setup.bash" ]; then
  echo "Robot base workspace is not built: ${base_ws}" >&2
  exit 2
fi

actual_sha256="$(sha256sum "${archive_path}" | awk '{print tolower($1)}')"
if [ "${actual_sha256}" != "${expected_sha256}" ]; then
  echo "Archive SHA256 mismatch" >&2
  echo "expected=${expected_sha256}" >&2
  echo "actual=${actual_sha256}" >&2
  exit 4
fi

staging_dir="$(mktemp -d "/home/nvidia/.dg202611-release.XXXXXX")"
cleanup_staging() {
  rm -rf -- "${staging_dir}"
}
trap cleanup_staging EXIT

# Reject archive entries that could escape the staging directory before
# extraction. The release archive is produced locally, but this check keeps a
# corrupted or substituted archive from writing outside the release area.
while IFS= read -r archive_entry; do
  case "${archive_entry}" in
    /*|../*|*/../*|..)
      echo "Refusing unsafe archive entry: ${archive_entry}" >&2
      exit 5
      ;;
  esac
done < <(tar -tzf "${archive_path}")

tar -xzf "${archive_path}" -C "${staging_dir}"

# Accept both archive layouts used by the local packaging tools:
#   1. files at archive root (scripts/, src/, ...)
#   2. one top-level workspace directory containing those files.
source_root="${staging_dir}"
if [ ! -f "${source_root}/scripts/run_dg202611_preflight.sh" ]; then
  mapfile -t top_level_dirs < <(find "${staging_dir}" -mindepth 1 -maxdepth 1 -type d -print)
  if [ "${#top_level_dirs[@]}" -eq 1 ] \
    && [ -f "${top_level_dirs[0]}/scripts/run_dg202611_preflight.sh" ]; then
    source_root="${top_level_dirs[0]}"
  else
    echo "Release archive has no recognized workspace root" >&2
    exit 5
  fi
fi

if [ -e "${release_dir}" ] || [ -L "${release_dir}" ]; then
  echo "Refusing to overwrite existing release directory: ${release_dir}" >&2
  exit 3
fi

for required in \
  scripts/run_dg202611_preflight.sh \
  scripts/activate_dg_release.sh \
  scripts/run_dg_integration.sh \
  scripts/run_dg_inspection.sh \
  scripts/run_dg_models_service.sh \
  scripts/install_dg_models_service.sh \
  scripts/run_real_robot_capture.sh \
  src/ylhb_base/CMakeLists.txt; do
  if [ ! -f "${source_root}/${required}" ]; then
    echo "Release is incomplete; missing ${source_root}/${required}" >&2
    exit 5
  fi
done

# Move the verified tree into its final versioned path only after all required
# files are present. This prevents a failed extraction from leaving a partial
# release that blocks retries.
mv -- "${source_root}" "${release_dir}"
if [ ! -d "${release_dir}" ]; then
  echo "Unable to install release directory: ${release_dir}" >&2
  exit 5
fi
trap - EXIT
trap 'rm -rf -- "${staging_dir}"' EXIT

cd "${release_dir}"
export DG_BASE_WS="${base_ws}"
set -o pipefail
./scripts/run_dg202611_preflight.sh 2>&1 | tee deploy_preflight.log

deployed_utc="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
python3 - "${release_dir}" "${release_commit}" "${actual_sha256}" "${deployed_utc}" "${base_ws}" <<'PY'
import json
from pathlib import Path
import sys

release_dir, commit, archive_sha256, deployed_utc, base_workspace = sys.argv[1:]
payload = {
    "schema_version": 1,
    "release_dir": release_dir,
    "commit": commit,
    "archive_sha256": archive_sha256,
    "deployed_utc": deployed_utc,
    "base_workspace": base_workspace,
    "base_workspace_modified": False,
    "motion_test_executed": False,
    "preflight_log": "deploy_preflight.log",
}
(Path(release_dir) / "DEPLOYMENT_RECORD.json").write_text(
    json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
    encoding="utf-8",
)
PY

find . -type f \
  -not -path './build/*' \
  -not -path './install/*' \
  -not -path './log/*' \
  -not -name 'DEPLOYED_SOURCE_MANIFEST.sha256' \
  -print0 | sort -z | xargs -0 sha256sum > DEPLOYED_SOURCE_MANIFEST.sha256

echo "DEPLOY_READY=${release_dir}"
echo "COMMIT=${release_commit}"
echo "ARCHIVE_SHA256=${actual_sha256}"
echo "No motion test was executed."
