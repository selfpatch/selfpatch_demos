#!/usr/bin/env bash
#
# Optional dev-convenience: build artefact tarballs + catalog.json on
# the host so a maintainer can iterate on broken_lidar / fixed_lidar
# without going through `docker compose build` every time.
#
# This script is NOT load-bearing for CI or distribution. The
# reproducible path is `docker compose build ota_update_server`, which
# multi-stage-builds the same artefacts inside ros:jazzy. If you don't
# want to think about ROS env on your host, use compose.
#
# broken_lidar / fixed_lidar are pure rclcpp + sensor_msgs
# (+ visualization_msgs) republishers - Nav2's own log + action-status
# bridges turn its failure into SOVD faults, not a ReportFault call
# from these nodes.
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
  echo "This script still gates on ros2_medkit_msgs being sourced (legacy of" >&2
  echo "when fixed_lidar / broken_lidar called ReportFault directly; they are" >&2
  echo "now pure /scan_sim republishers with no ros2_medkit dependency, but" >&2
  echo "the gate is left in place here). Either:" >&2
  echo "  - source an overlay that has it built, or" >&2
  echo "  - run 'docker compose build ota_update_server' instead - that" >&2
  echo "    path is reproducible and bundles the msgs build internally." >&2
  exit 1
fi

set -u

mkdir -p "$WS/src"
for pkg in broken_lidar fixed_lidar; do
  ln -sfn "$DEMO_DIR/ros2_packages/$pkg" "$WS/src/$pkg"
done

(cd "$WS" && colcon build --packages-select broken_lidar fixed_lidar)

mkdir -p "$ARTIFACTS"
rm -f "$ARTIFACTS/catalog.json" "$ARTIFACTS/catalog_pending.json"

PACK=("$SCRIPT_DIR/.venv/bin/python" "$SCRIPT_DIR/pack_artifact.py")

# The BOOT catalog (catalog.json) holds only the bad update broken_lidar_3_0_0
# - that is what was pushed and auto-applied. The remediation build
# fixed_lidar_3_0_1 is packed into catalog_pending.json instead (its tarball
# still ships), so it is NOT in the boot catalog - the operator publishes it
# at diagnose time with publish-fix.sh (SOVD POST /updates). This mirrors
# ota_update_server/Dockerfile's in-image build exactly.
env -i PATH=/usr/bin:/bin HOME="$HOME" "${PACK[@]}" \
  --package broken_lidar --version 3.0.0 \
  --kind update --target-component scan_sensor_node \
  --executable broken_lidar_node \
  --replaces-executable fixed_lidar_node \
  --notes "Perception: /scan noise-filter tuning" \
  --skip-build --workspace "$WS" \
  --out-dir "$ARTIFACTS" --catalog "$ARTIFACTS/catalog.json"

env -i PATH=/usr/bin:/bin HOME="$HOME" "${PACK[@]}" \
  --package fixed_lidar --version 3.0.1 \
  --kind update --target-component scan_sensor_node \
  --executable fixed_lidar_node \
  --replaces-executable broken_lidar_node \
  --notes "Fix regressed /scan noise filter (3.0.0 hotfix)" \
  --skip-build --workspace "$WS" \
  --out-dir "$ARTIFACTS" --catalog "$ARTIFACTS/catalog_pending.json"

if command -v jq >/dev/null 2>&1; then
  echo "Built boot catalog with $(jq length "$ARTIFACTS/catalog.json") entries"
  echo "Built pending catalog with $(jq length "$ARTIFACTS/catalog_pending.json") entries"
else
  echo "Built boot catalog: $(wc -l < "$ARTIFACTS/catalog.json") lines"
  echo "Built pending catalog: $(wc -l < "$ARTIFACTS/catalog_pending.json") lines"
fi
