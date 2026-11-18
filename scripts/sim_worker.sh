#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
cd "$repo_dir"
case "${1:-menu}" in
  build)
    docker build --build-arg ROS_DISTRO=humble -t ros2-worker:humble .
    docker build -f docker/Dockerfile.sim \
      --build-arg "NOVNC_BASE=${WORKER_NOVNC_BASE:-tiryoh/ros2-desktop-vnc:humble}" \
      -t ros2-worker:humble-sim .
    ;;
  menu)
    if [[ $# == 0 ]]; then set -- menu; fi
    docker compose -p ros2-worker-simulation -f compose.sim.yaml run --rm --service-ports simulation "$@"
    ;;
  test|train|inspect)
    docker compose -p ros2-worker-simulation -f compose.sim.yaml run --rm simulation "$@"
    ;;
  stop) docker compose -p ros2-worker-simulation -f compose.sim.yaml down --remove-orphans ;;
  *) printf 'Usage: bash scripts/sim_worker.sh [build|menu|inspect|train|test|stop]\n' >&2; exit 2 ;;
esac
