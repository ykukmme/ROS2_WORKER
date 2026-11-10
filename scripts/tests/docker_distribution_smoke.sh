#!/usr/bin/env bash
# Integration checks against the real Docker Engine; sends no robot commands.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.."
export WORKER_IMAGE="${WORKER_IMAGE:-ros2-worker:${WORKER_ROS_DISTRO:-jazzy}}"
export WORKER_CONTAINER="ros2-worker-test-$$"
export WORKER_VOLUME="ros2-worker-test-$$"
export ROS_DOMAIN_ID=88
import_volume="$WORKER_VOLUME-import"
fixture="$(mktemp -d)"
cleanup() {
  docker volume rm "$WORKER_VOLUME" "$import_volume" >/dev/null 2>&1 || true
  rm -f -- "$fixture/config/live.local.yaml"
  rmdir -- "$fixture/config" "$fixture" 2>/dev/null || true
}
trap cleanup EXIT
bash -n scripts/run_worker.sh scripts/docker_worker.sh
docker compose config --quiet
bash scripts/run_worker.sh check
docker run --rm --network none --mount "type=volume,src=$WORKER_VOLUME,dst=/data" \
  "$WORKER_IMAGE" bash -c 'test "$(id -u)" = 1000 && printf persisted > /data/config/probe'
docker run --rm --network none --mount "type=volume,src=$WORKER_VOLUME,dst=/data" \
  "$WORKER_IMAGE" bash -c 'test "$(cat /data/config/probe)" = persisted'
mkdir -p "$fixture/config"
printf 'migration_probe: preserved\n' > "$fixture/config/live.local.yaml"
WORKER_VOLUME="$import_volume" bash scripts/run_worker.sh import-data "$fixture"
docker run --rm --network none --mount "type=volume,src=$import_volume,dst=/data" \
  "$WORKER_IMAGE" bash -c 'grep -q "migration_probe: preserved" /data/config/live.local.yaml'
printf 'migration_probe: overwritten\n' > "$fixture/config/live.local.yaml"
if WORKER_VOLUME="$import_volume" bash scripts/run_worker.sh import-data "$fixture"; then
  printf 'FAIL: nonempty volume was overwritten\n' >&2; exit 1
fi
docker run --rm --network none --mount "type=volume,src=$import_volume,dst=/data" \
  "$WORKER_IMAGE" bash -c 'grep -q "migration_probe: preserved" /data/config/live.local.yaml'
if WORKER_VOLUME=/tmp/invalid bash scripts/run_worker.sh check; then
  printf 'FAIL: host path accepted as a volume name\n' >&2; exit 1
fi
if [[ -n "${1:-}" ]]; then
  bundle="$(cd -- "$1" && pwd)"
  (cd /tmp && bash "$bundle/worker.sh" load && bash "$bundle/worker.sh" check)
  export WORKER_TEST_BUNDLE="$bundle"
  printf '0\n' | script --quiet --return --command 'bash "$WORKER_TEST_BUNDLE/worker.sh"' /dev/null
  # Exercise first-run auto-load without removing any existing image tag.
  real_docker="$(command -v docker)"
  probe_file="$fixture/load-requested"
  export real_docker probe_file
  docker() {
    if [[ "${1:-}" == image && "${2:-}" == inspect && ! -f "$probe_file" ]]; then
      return 1
    fi
    if [[ "${1:-}" == load ]]; then touch "$probe_file"; fi
    "$real_docker" "$@"
  }
  export -f docker
  (cd /tmp && bash "$bundle/worker.sh" check)
  test -f "$probe_file"
  rm -f -- "$probe_file"
  unset -f docker
fi
printf 'PASS: volume permissions/persistence, migration/refusal, launcher and optional bundle load\n'
