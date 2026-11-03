#!/usr/bin/env bash
# Build only this repository in an isolated overlay. No package installation.
set -euo pipefail

if [[ $# -ne 1 ]]; then
  printf 'Usage: bash scripts/build_ros2.sh /absolute/path/to/workspace\n' >&2
  exit 2
fi
if [[ -z "${ROS_DISTRO:-}" ]]; then
  printf 'Source the existing robot ROS2 environment first.\n' >&2
  exit 2
fi
for tool in colcon python3; do
  if ! command -v "$tool" >/dev/null 2>&1; then
    printf 'Required tool missing: %s\n' "$tool" >&2
    exit 2
  fi
done

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
workspace_dir="$1"
if [[ "$workspace_dir" != /* ]]; then
  printf 'Workspace must be an absolute path.\n' >&2
  exit 2
fi
mkdir -p -- "$workspace_dir"
workspace_dir="$(cd -- "$workspace_dir" && pwd -P)"

colcon --log-base "$workspace_dir/log/ros2_worker" build \
  --base-paths "$repo_dir/ai_worker_interfaces" "$repo_dir/ai_worker_sorting" \
  --packages-select ai_worker_interfaces ai_worker_sorting \
  --build-base "$workspace_dir/build/ros2_worker" \
  --install-base "$workspace_dir/install/ros2_worker" \
  --symlink-install

printf '\nBuild complete. Source this overlay:\nsource "%s/install/ros2_worker/setup.bash"\n' "$workspace_dir"
