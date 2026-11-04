#!/usr/bin/env bash
# Run from the connected PC's existing ROS2 environment.
set -eo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
workspace_dir="${WORKER_WS:-$HOME/ros2_worker_ws}"
config_file="${WORKER_CONFIG:-$HOME/ros2_worker_config/live.local.yaml}"
python_bin="${WORKER_PYTHON:-python3}"
command="${1:-help}"
if [[ $# -gt 0 ]]; then shift; fi
if [[ "$command" == help ]]; then
  printf '%s\n' 'Usage: bash scripts/worker.sh {check|build|configure|calibrate|teach|doctor|run|prepare|start|status|cancel|session} [arguments]' \
    'Source the existing ROS2 and ROBOTIS environment first.' \
    'WORKER_WS: overlay workspace; WORKER_CONFIG: local live YAML; WORKER_PYTHON: existing Python executable.'
  exit 0
fi
if [[ -z "${ROS_DISTRO:-}" ]]; then
  printf 'Source the existing robot-compatible ROS2 environment first.\n' >&2
  exit 2
fi
case "$command" in
  check)
    command -v ros2 >/dev/null
    command -v colcon >/dev/null
    ros2 pkg prefix robotis_interfaces
    "$python_bin" -c 'import rclpy, cv_bridge, tf2_ros, message_filters, vision_msgs.msg, robotis_interfaces.msg, yaml, numpy, cv2, ultralytics; print("Dependencies import OK; no motion commands sent")'
    exit 0 ;;
  build) exec bash "$repo_dir/scripts/build_ros2.sh" "$workspace_dir" ;;
esac
overlay="$workspace_dir/install/ros2_worker/setup.bash"
if [[ ! -f "$overlay" ]]; then
  printf 'Overlay missing; run: bash scripts/worker.sh build\n' >&2
  exit 2
fi
source "$overlay"
case "$command" in
  configure) exec "$python_bin" -m ai_worker_sorting.configure_live --output "$config_file" "$@" ;;
  teach) exec "$python_bin" -m ai_worker_sorting.configure_auto "$@" ;;
  calibrate)
    prefix=()
    if [[ -n "${WORKER_PYTHON:-}" ]]; then prefix+=("python_prefix:=$WORKER_PYTHON"); fi
    exec ros2 launch ai_worker_sorting calibration.launch.py "${prefix[@]}" "$@" ;;
  doctor) exec "$python_bin" -m ai_worker_sorting.worker_cli doctor --config "$config_file" "$@" ;;
  run)
    prefix=()
    if [[ -n "${WORKER_PYTHON:-}" ]]; then prefix+=("python_prefix:=$WORKER_PYTHON"); fi
    exec ros2 launch ai_worker_sorting ai_worker_live.launch.py "config:=$config_file" "${prefix[@]}" "$@" ;;
  prepare|start|status|cancel|session)
    exec "$python_bin" -m ai_worker_sorting.worker_cli "$command" "$@" ;;
  *) printf 'Unknown command: %s\n' "$command" >&2; exit 2 ;;
esac
