#!/usr/bin/env bash
set -eo pipefail
source "/opt/ros/$ROS_DISTRO/setup.bash"
source /opt/vendor_ws/install/setup.bash
source /opt/worker_ws/install/ros2_worker/setup.bash
mkdir -p /data/config /data/.ros /data/ultralytics
export ROS_LOG_DIR=/data/.ros/log
export YOLO_CONFIG_DIR=/data/ultralytics
if [[ "${1:-menu}" == menu ]]; then
  shift || true
  exec "$WORKER_PYTHON" -m ai_worker_sorting.terminal_menu --config "$WORKER_CONFIG" "$@"
fi
exec "$@"
