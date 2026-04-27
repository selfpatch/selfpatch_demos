#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DEMO_DIR="$(dirname "$SCRIPT_DIR")"
WS="$DEMO_DIR/ros2_ws"
ARTIFACTS="$DEMO_DIR/artifacts"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
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
