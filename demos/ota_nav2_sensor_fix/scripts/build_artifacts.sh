#!/usr/bin/env bash
#
# Optional dev-convenience: build artefact tarballs + catalog.json on
# the host so a maintainer can iterate on broken_lidar / fixed_lidar /
# obstacle_classifier_v2 without going through `docker compose build`
# every time.
#
# This script is NOT load-bearing for CI or distribution. The
# reproducible path is `docker compose build ota_update_server`, which
# multi-stage-builds the same artefacts inside ros:jazzy. If you don't
# want to think about ROS env on your host, use compose.
#
# Prerequisites for running locally:
#   - /opt/ros/jazzy on the prefix path
#   - ros2_medkit_msgs sourced (e.g. via a colcon overlay built from
#     a local clone of ros2_medkit; the gateway image embeds this).

set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DEMO_DIR="$(dirname "$SCRIPT_DIR")"
WS="$DEMO_DIR/ros2_ws"
ARTIFACTS="$DEMO_DIR/artifacts"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash

if ! ros2 pkg prefix ros2_medkit_msgs > /dev/null 2>&1; then
  echo "ros2_medkit_msgs not found on the prefix path." >&2
  echo "" >&2
  echo "fixed_lidar / broken_lidar depend on the SOVD ReportFault service" >&2
  echo "definition that lives in ros2_medkit. Either:" >&2
  echo "  - source an overlay that has it built, or" >&2
  echo "  - run 'docker compose build ota_update_server' instead - that" >&2
  echo "    path is reproducible and bundles the msgs build internally." >&2
  exit 1
fi

set -u

mkdir -p "$WS/src"
for pkg in broken_lidar fixed_lidar broken_lidar_legacy obstacle_classifier_v2; do
  ln -sfn "$DEMO_DIR/ros2_packages/$pkg" "$WS/src/$pkg"
done

(cd "$WS" && colcon build --packages-select fixed_lidar obstacle_classifier_v2)

mkdir -p "$ARTIFACTS"
rm -f "$ARTIFACTS/catalog.json"

PACK=("$SCRIPT_DIR/.venv/bin/python" "$SCRIPT_DIR/pack_artifact.py")

env -i PATH=/usr/bin:/bin HOME="$HOME" "${PACK[@]}" \
  --package fixed_lidar --version 2.1.0 \
  --kind update --target-component scan_sensor_node \
  --executable fixed_lidar_node \
  --replaces-executable broken_lidar_node \
  --notes "Fix /scan noise filter" \
  --skip-build --workspace "$WS" \
  --out-dir "$ARTIFACTS" --catalog "$ARTIFACTS/catalog.json"

env -i PATH=/usr/bin:/bin HOME="$HOME" "${PACK[@]}" \
  --package obstacle_classifier_v2 --version 1.0.0 \
  --kind install --target-component obstacle_classifier \
  --executable obstacle_classifier_node \
  --notes "Extra safety layer for nav2" \
  --skip-build --workspace "$WS" \
  --out-dir "$ARTIFACTS" --catalog "$ARTIFACTS/catalog.json"

env -i PATH=/usr/bin:/bin HOME="$HOME" "${PACK[@]}" \
  --package broken_lidar_legacy --version "" \
  --kind uninstall --target-component broken_lidar_legacy \
  --notes "Clean up deprecated package" \
  --skip-build --workspace "$WS" \
  --out-dir "$ARTIFACTS" --catalog "$ARTIFACTS/catalog.json"

if command -v jq >/dev/null 2>&1; then
  echo "Built catalog with $(jq length "$ARTIFACTS/catalog.json") entries"
else
  echo "Built catalog: $(wc -l < "$ARTIFACTS/catalog.json") lines"
fi
