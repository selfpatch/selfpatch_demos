#!/bin/bash
set -euo pipefail

cd "$(dirname "$0")"

PROFILE="cpu"
VOLUMES=0

while [ $# -gt 0 ]; do
  case "$1" in
    --ci)       PROFILE="ci"; shift ;;
    --volumes)  VOLUMES=1; shift ;;
    -h|--help)
      cat <<EOF
Stop manymove_industrial.

Usage: $0 [--ci] [--volumes]

  --ci        Tear down the CI profile.
  --volumes   Also remove anonymous volumes.
EOF
      exit 0
      ;;
    *) echo "unknown arg: $1"; exit 1 ;;
  esac
done

if [ "$VOLUMES" = "1" ]; then
  docker compose --profile "$PROFILE" down -v
else
  docker compose --profile "$PROFILE" down
fi
