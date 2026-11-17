#!/usr/bin/env bash
# Standalone Ubuntu launcher; only Bash and Docker Engine are required.
set -euo pipefail
bundle_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
image="${WORKER_IMAGE:-}"
bundle_image=""
if [[ -f "$bundle_dir/image.txt" ]]; then
  IFS= read -r bundle_image < "$bundle_dir/image.txt"
fi
if [[ -z "$image" ]]; then
  if [[ -n "$bundle_image" ]]; then
    image="$bundle_image"
  elif [[ "${WORKER_ROS_DISTRO:-jazzy}" == jazzy ]]; then
    image=ghcr.io/ykukmme/ros2-worker:jazzy
  else
    image="ros2-worker:${WORKER_ROS_DISTRO}"
  fi
fi
volume="${WORKER_VOLUME:-ros2-worker-data}"
container="${WORKER_CONTAINER:-ros2-worker}"
if [[ ! "$volume" =~ ^[a-zA-Z0-9][a-zA-Z0-9_.-]+$ ]]; then
  printf 'Invalid WORKER_VOLUME: use a Docker volume name, not a path.\n' >&2
  exit 2
fi
load_image() {
  local archive="$bundle_dir/ros2-worker.tar"
  [[ -f "$archive" ]] || { printf 'Image archive missing: %s\n' "$archive" >&2; return 1; }
  (cd -- "$bundle_dir" && sha256sum --check SHA256SUMS)
  docker load --input "$archive"
  docker image inspect "$image" >/dev/null
}
ensure_stopped() {
  if [[ "$(docker container inspect --format '{{.State.Running}}' "$container" 2>/dev/null || true)" == true ]]; then
    printf 'Close the menu after confirming the robot has stopped before updating images.\n' >&2
    return 1
  fi
}
pull_image() {
  if [[ "$image" != */* ]]; then
    printf 'Local image missing: %s. Build it or use an offline bundle.\n' "$image" >&2
    return 1
  fi
  if ! docker pull -- "$image"; then
    printf 'Image download failed. Check network access and GHCR package visibility (Public).\n' >&2
    return 1
  fi
}
command="${1:-menu}"
case "$command" in menu|check|load|pull|update|import-data) ;; *) printf 'Usage: bash worker.sh {menu|check|pull|update|load|import-data DIRECTORY}\n' >&2; exit 2 ;; esac
docker_os="$(docker info --format '{{.OSType}}')"
[[ "$docker_os" == linux ]] || { printf 'Linux containers are required.\n' >&2; exit 2; }
if [[ "$command" == load ]]; then
  ensure_stopped
  load_image
  exit
fi
if [[ "$command" == pull || "$command" == update ]]; then
  ensure_stopped
  pull_image
  exit
fi
if ! docker image inspect "$image" >/dev/null 2>&1; then
  if [[ -n "$bundle_image" && "$image" == "$bundle_image" && -f "$bundle_dir/ros2-worker.tar" ]]; then
    load_image
  else
    pull_image
  fi
fi
args=(run --rm --pull never --name "$container" --stop-timeout 30
      --network host --ipc host --user 1000:1000
      --mount "type=volume,src=$volume,dst=/data"
      -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
      -e "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
      -e "WORKER_DEVICE=${WORKER_DEVICE:-auto}"
      -e "WORKER_WORKSPACE=${WORKER_WORKSPACE:-}"
      -e "WORKER_ROBOT_DESCRIPTION_NODE=${WORKER_ROBOT_DESCRIPTION_NODE:-/robot_state_publisher}")
if [[ "${WORKER_GPU:-0}" == 1 ]]; then args+=(--gpus all); fi
case "$command" in
  menu)
    args+=(-i)
    if [[ -t 0 && -t 1 ]]; then args+=(-t); fi
    printf 'IMAGE=%s DOMAIN=%s DATA=%s\n' "$image" "${ROS_DOMAIN_ID:-0}" "$volume"
    exec docker "${args[@]}" "$image" menu ;;
  check)
    exec docker "${args[@]}" "$image" /opt/worker_venv/bin/python -c 'import os,sys,torch,ultralytics,rclpy,vision_msgs.msg,robotis_interfaces.msg; from pathlib import Path; from ai_worker_sorting.runtime_defaults import inference_device; assert Path("/opt/worker/models/yolo26n.pt").is_file(); print("ROS2:",os.environ["ROS_DISTRO"],"Python:",sys.version.split()[0],"Dependencies/model OK; device:",inference_device())' ;;
  import-data)
    [[ -d "${2:-}" ]] || { printf 'Provide the existing worker-data directory.\n' >&2; exit 2; }
    source_dir="$(cd -- "$2" && pwd)"
    [[ "$source_dir" != *,* ]] || { printf 'Import path must not contain a comma.\n' >&2; exit 2; }
    exec docker "${args[@]}" --mount "type=bind,src=$source_dir,dst=/import,readonly" "$image" bash -euo pipefail -c '
      if find /data -mindepth 1 ! -type d -print -quit | grep -q .; then
        printf "Destination already contains files; import refused.\n" >&2; exit 1
      fi
      cp -R --no-clobber /import/. /data/
      printf "Data imported. Custom model paths must use /data/models/.\n"
    ' ;;
esac
