#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
export WORKER_ROS_DISTRO="${WORKER_ROS_DISTRO:-jazzy}"
case "$WORKER_ROS_DISTRO" in humble|jazzy) ;; *) printf 'Supported ROS2 distributions: humble, jazzy\n' >&2; exit 2 ;; esac
args=(-f compose.yaml)
if [[ "${WORKER_GPU:-0}" == 1 ]]; then
  args+=(-f compose.gpu.yaml)
  export WORKER_TORCH_INDEX_URL="${WORKER_TORCH_INDEX_URL:-https://pypi.org/simple}"
fi
case "${1:-menu}" in
  build) exec docker compose "${args[@]}" build worker ;;
  menu|check|import-data) exec bash scripts/run_worker.sh "$@" ;;
  export)
    image="ros2-worker:$WORKER_ROS_DISTRO"
    docker image inspect "$image" >/dev/null
    destination="${2:-dist/ros2-worker-$WORKER_ROS_DISTRO}"
    mkdir -p -- "$destination"
    destination="$(cd -- "$destination" && pwd)"
    cp scripts/run_worker.sh "$destination/worker.sh"
    cp docker/SHARED_PC.md "$destination/README.md"
    printf '%s\n' "$image" > "$destination/image.txt"
    docker save --output "$destination/ros2-worker.tar.partial" "$image"
    mv -f -- "$destination/ros2-worker.tar.partial" "$destination/ros2-worker.tar"
    (cd -- "$destination" && sha256sum ros2-worker.tar > SHA256SUMS)
    printf 'Copy this entire directory to the Ubuntu PC: %s\n' "$destination" ;;
  *) printf 'Usage: bash scripts/docker_worker.sh {build|menu|check|import-data DIRECTORY|export [DIRECTORY]}\n' >&2; exit 2 ;;
esac
