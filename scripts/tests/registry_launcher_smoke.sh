#!/usr/bin/env bash
# Exercise first download, cached/offline starts, update and failure paths without a registry.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
fixture="$(mktemp -d)"
mkdir -p "$fixture/bin" "$fixture/bundle"
cleanup() {
  rm -f -- "$fixture/bin/docker" "$fixture/log" "$fixture/present" "$fixture/fail" "$fixture/running" \
    "$fixture/bundle/worker.sh" "$fixture/bundle/image.txt" "$fixture/bundle/ros2-worker.tar" "$fixture/bundle/SHA256SUMS"
  rmdir -- "$fixture/bin" "$fixture/bundle" "$fixture"
}
trap cleanup EXIT
export MOCK_DOCKER_DIR="$fixture"
export PATH="$fixture/bin:$PATH"
unset WORKER_IMAGE WORKER_ROS_DISTRO WORKER_GPU WORKER_VOLUME WORKER_CONTAINER
cat > "$fixture/bin/docker" <<'MOCK'
#!/usr/bin/env bash
set -euo pipefail
printf '%s\n' "$*" >> "$MOCK_DOCKER_DIR/log"
case "${1:-} ${2:-}" in
  'info --format') printf 'linux\n' ;;
  'image inspect') test -f "$MOCK_DOCKER_DIR/present" ;;
  'container inspect') if [[ -f "$MOCK_DOCKER_DIR/running" ]]; then printf 'true\n'; else printf 'false\n'; fi ;;
  'pull --') test ! -f "$MOCK_DOCKER_DIR/fail" && touch "$MOCK_DOCKER_DIR/present" ;;
  'load --input') touch "$MOCK_DOCKER_DIR/present" ;;
  'run --rm') : ;;
  *) printf 'Unexpected mock Docker command: %s\n' "$*" >&2; exit 99 ;;
esac
MOCK
chmod +x "$fixture/bin/docker"
reset() { rm -f -- "$fixture/present" "$fixture/fail" "$fixture/running"; : > "$fixture/log"; }
reset
bash "$root/worker.sh" check
grep -qx 'pull -- ghcr.io/ykukmme/ros2-worker:jazzy' "$fixture/log"
grep -q '^run --rm --pull never' "$fixture/log"
: > "$fixture/log"
bash "$root/worker.sh" check
! grep -q '^pull ' "$fixture/log"
reset
touch "$fixture/fail"
if bash "$root/worker.sh" check; then printf 'FAIL: pull failure accepted\n' >&2; exit 1; fi
! grep -q '^run ' "$fixture/log"
reset
touch "$fixture/present"
bash "$root/worker.sh" update
grep -q '^pull ' "$fixture/log"
! grep -q '^run ' "$fixture/log"
reset
touch "$fixture/running"
if bash "$root/worker.sh" update; then printf 'FAIL: running container updated\n' >&2; exit 1; fi
! grep -q '^pull ' "$fixture/log"
reset
if WORKER_IMAGE=ros2-worker:jazzy bash "$root/worker.sh" check; then printf 'FAIL: absent local image accepted\n' >&2; exit 1; fi
! grep -q '^pull ' "$fixture/log"
reset
WORKER_IMAGE=ghcr.io/ykukmme/ros2-worker:sha-example bash "$root/worker.sh" check
grep -qx 'pull -- ghcr.io/ykukmme/ros2-worker:sha-example' "$fixture/log"
cp "$root/scripts/run_worker.sh" "$fixture/bundle/worker.sh"
printf 'ros2-worker:jazzy\n' > "$fixture/bundle/image.txt"
printf 'offline-test-archive\n' > "$fixture/bundle/ros2-worker.tar"
(cd -- "$fixture/bundle" && sha256sum ros2-worker.tar > SHA256SUMS)
reset
bash "$fixture/bundle/worker.sh" check
grep -q '^load ' "$fixture/log"
! grep -q '^pull ' "$fixture/log"
reset
printf 'corrupted\n' >> "$fixture/bundle/ros2-worker.tar"
if bash "$fixture/bundle/worker.sh" check; then printf 'FAIL: bad checksum accepted\n' >&2; exit 1; fi
! grep -qE '^(pull|load|run) ' "$fixture/log"
printf 'PASS: registry launcher download/cache/update/failure/offline checks\n'
