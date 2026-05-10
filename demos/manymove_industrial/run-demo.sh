#!/bin/bash
set -euo pipefail

cd "$(dirname "$0")"

PROFILE="cpu"
NO_CACHE=0

while [ $# -gt 0 ]; do
  case "$1" in
    --ci)        PROFILE="ci"; shift ;;
    --no-cache)  NO_CACHE=1; shift ;;
    -h|--help)
      cat <<EOF
manymove_industrial demo

Usage:
  $0 [--ci] [--no-cache]

  --ci         Run the headless CI profile (no web UI).
  --no-cache   Force docker compose build --no-cache.
EOF
      exit 0
      ;;
    *) echo "unknown arg: $1"; exit 1 ;;
  esac
done

if ! command -v docker >/dev/null 2>&1; then
  echo "docker is required" >&2
  exit 1
fi

# Allow X11 access from the container so RViz / Groot / manymove HMI render
# on the host display when running the cpu profile.
if [ "$PROFILE" = "cpu" ] && command -v xhost >/dev/null 2>&1; then
  xhost +local:root >/dev/null 2>&1 || true
fi

if [ "$NO_CACHE" = "1" ]; then
  docker compose --profile "$PROFILE" build --no-cache
fi

docker compose --profile "$PROFILE" up -d --build

cat <<EOF

Started manymove_industrial (profile: $PROFILE).

  Gateway:  http://localhost:8080/api/v1/health
  Web UI:   http://localhost:3000  (cpu profile only)

Live BT view (Groot, in the running container):
  docker compose exec manymove-sim bash -lc "source install/setup.bash && src/Groot/build/Groot"

Inject scripts (also runnable from the medkit Web UI):
  docker compose exec manymove-sim bash -lc "/var/lib/ros2_medkit/scripts/manymove-planning/inject-collision/script.bash"

Other helpers:
  docker compose --profile $PROFILE logs -f manymove-sim
  ./check-demo.sh
  ./stop-demo.sh
EOF
