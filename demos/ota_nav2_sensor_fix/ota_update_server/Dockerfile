# Copyright 2026 bburda
# Apache 2.0
#
# Multi-stage build: stage 1 produces the SOVD catalog + tarballs from
# the demo's ros2_packages/ (broken_lidar / fixed_lidar are pure rclcpp +
# sensor_msgs republishers - Nav2's own log + action-status bridges turn
# its failure into SOVD faults, not a ReportFault call from these nodes);
# stage 2 ships a slim FastAPI server
# that serves those artefacts. This keeps `docker compose build`
# self-contained and reproducible on CI - no separate "build artifacts on
# host" step required.
#
# Build context is the demo root (so we can pull in ros2_packages/ +
# scripts/ + ota_update_server/). docker-compose.yml wires that up.

# =============================================================================
# Stage 1: build artefacts + catalog inside ros:jazzy
# =============================================================================
FROM ros:jazzy AS artefact_builder


RUN apt-get update && apt-get install -y --no-install-recommends \
      git \
      python3-colcon-common-extensions \
      python3-catkin-pkg \
      python3-venv \
      python3-pip \
      build-essential \
      cmake \
      ros-jazzy-rclcpp \
      ros-jazzy-sensor-msgs \
      ros-jazzy-geometry-msgs \
      ros-jazzy-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*

# broken_lidar / fixed_lidar are pure /scan_sim republishers (Nav2's own
# failure is surfaced via the log/action-status bridges), so neither
# depends on ros2_medkit_msgs - no gateway clone here.
#
# Bring in the demo's source packages + pack_artifact.py.
WORKDIR /demo
COPY ros2_packages /demo/ros2_packages
COPY scripts /demo/scripts

# Build broken_lidar + fixed_lidar. Tarballs + catalog.json land in
# /demo/artifacts.
RUN mkdir -p /demo/ros2_ws/src && \
    for pkg in broken_lidar fixed_lidar; do \
      ln -sfn /demo/ros2_packages/$pkg /demo/ros2_ws/src/$pkg; \
    done && \
    . /opt/ros/jazzy/setup.sh && \
    cd /demo/ros2_ws && \
    colcon build --packages-select broken_lidar fixed_lidar

# pack_artifact.py uses pure stdlib (json/tarfile/argparse) so we don't
# need the .venv/pytest dance build_artifacts.sh does locally.
#
# The BOOT catalog (catalog.json, served at GET /catalog) holds only the bad
# update broken_lidar_3_0_0 - that is what was pushed and auto-applied. The
# remediation build fixed_lidar_3_0_1 is packed into catalog_pending.json
# instead (its tarball still ships), so it is NOT in the boot catalog. The
# operator publishes it at diagnose time with publish-fix.sh (SOVD
# POST /updates), the way a real hotfix is released in response to an incident.
RUN mkdir -p /demo/artifacts && \
    rm -f /demo/artifacts/catalog.json && \
    PACK="python3 /demo/scripts/pack_artifact.py" && \
    $PACK --package broken_lidar --version 3.0.0 \
          --kind update --target-component scan_sensor_node \
          --executable broken_lidar_node \
          --replaces-executable fixed_lidar_node \
          --notes "Perception: /scan noise-filter tuning" \
          --skip-build --workspace /demo/ros2_ws \
          --out-dir /demo/artifacts --catalog /demo/artifacts/catalog.json && \
    $PACK --package fixed_lidar --version 3.0.1 \
          --kind update --target-component scan_sensor_node \
          --executable fixed_lidar_node \
          --replaces-executable broken_lidar_node \
          --notes "Fix regressed /scan noise filter (3.0.0 hotfix)" \
          --skip-build --workspace /demo/ros2_ws \
          --out-dir /demo/artifacts --catalog /demo/artifacts/catalog_pending.json

# =============================================================================
# Stage 2: slim runtime image
# =============================================================================
FROM python:3.11-slim

WORKDIR /app
COPY ota_update_server/pyproject.toml ./
COPY ota_update_server/ota_update_server ./ota_update_server
RUN pip install --no-cache-dir .

COPY --from=artefact_builder /demo/artifacts /artifacts

ENV OTA_ARTIFACTS_DIR=/artifacts \
    OTA_HOST=0.0.0.0 \
    OTA_PORT=9000
EXPOSE 9000

CMD ["ota-update-server"]
