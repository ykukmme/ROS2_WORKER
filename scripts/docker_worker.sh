#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
export WORKER_UID="$(id -u)" WORKER_GID="$(id -g)"
export WORKER_ROS_DISTRO="${WORKER_ROS_DISTRO:-${ROS_DISTRO:-humble}}"
case "$WORKER_ROS_DISTRO" in humble|jazzy) ;; *) printf 'Supported ROS2 distributions: humble, jazzy\n' >&2; exit 2 ;; esac
mkdir -p worker-data/config
args=(-f compose.yaml)
if [[ "${WORKER_GPU:-0}" == 1 ]]; then
  args+=(-f compose.gpu.yaml)
  export WORKER_TORCH_INDEX_URL="${WORKER_TORCH_INDEX_URL:-https://pypi.org/simple}"
fi
case "${1:-menu}" in
  build) exec docker compose "${args[@]}" build worker ;;
  menu) printf 'ROS2=%s DOMAIN=%s DEVICE=%s\n' "$WORKER_ROS_DISTRO" "${ROS_DOMAIN_ID:-0}" "${WORKER_DEVICE:-auto}"
        exec docker compose "${args[@]}" run --rm --no-deps worker ;;
  check) exec docker compose "${args[@]}" run --rm --no-deps worker /opt/worker_venv/bin/python -c 'import torch,ultralytics,rclpy,vision_msgs.msg,robotis_interfaces.msg; from pathlib import Path; from ai_worker_sorting.runtime_defaults import inference_device; assert Path("/opt/worker/models/yolo26n.pt").is_file(); print("Dependencies/model OK; device:",inference_device())' ;;
  *) printf 'Usage: bash scripts/docker_worker.sh {build|menu|check}\n' >&2; exit 2 ;;
esac
