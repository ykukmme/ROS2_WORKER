#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/vendor_ws/install/setup.bash
source /opt/sim_ws/install/ros2_worker/setup.bash
export PYTHONPATH="/opt/worker/ai_worker_sorting:${PYTHONPATH:-}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}" OPENBLAS_NUM_THREADS=1
export MPLCONFIGDIR=/sim-data/matplotlib YOLO_CONFIG_DIR=/sim-data/ultralytics
mkdir -p /sim-data
if [[ "${1:-menu}" == menu ]]; then
    export USER=root DISPLAY=:1
    /bin/bash /entrypoint.sh > /sim-data/desktop.log 2>&1 &
    desktop_pid=$!
    trap 'kill "$desktop_pid" 2>/dev/null || true' EXIT
    for attempt in {1..100}; do
        [[ -S /tmp/.X11-unix/X1 ]] && break
        sleep .2
    done
    /opt/worker_venv/bin/python -m ai_worker_sorting.sim_runtime "$@"
else
    exec /opt/worker_venv/bin/python -m ai_worker_sorting.sim_runtime "$@"
fi
