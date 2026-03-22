#!/usr/bin/env bash
# Stop the OpenPLC Tank Demo
set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEMO_DIR="$(dirname "$SCRIPT_DIR")"
cd "$DEMO_DIR"

VOLUMES=false

while [[ $# -gt 0 ]]; do
  case $1 in
    --volumes|-v) VOLUMES=true; shift ;;
    *)            shift ;;
  esac
done

if docker compose version &> /dev/null; then
  COMPOSE_CMD="docker compose"
else
  COMPOSE_CMD="docker-compose"
fi

DOWN_ARGS=""
if [[ "$VOLUMES" == "true" ]]; then
  DOWN_ARGS="-v"
fi

echo "Stopping OpenPLC Tank Demo..."
$COMPOSE_CMD down $DOWN_ARGS
echo "Done."
