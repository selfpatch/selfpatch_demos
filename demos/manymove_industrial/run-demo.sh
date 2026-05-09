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

if [ "$NO_CACHE" = "1" ]; then
  docker compose --profile "$PROFILE" build --no-cache
fi

docker compose --profile "$PROFILE" up -d --build

cat <<EOF

Started manymove_industrial (profile: $PROFILE).

  Gateway:  http://localhost:8080/api/v1/health
  Web UI:   http://localhost:3000  (cpu profile only)

Try:
  docker compose --profile $PROFILE logs -f manymove-sim
  ./check-demo.sh
  ./stop-demo.sh
EOF
