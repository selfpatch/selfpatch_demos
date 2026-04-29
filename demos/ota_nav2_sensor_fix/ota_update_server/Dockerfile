# Copyright 2026 bburda
# Apache 2.0
#
# Multi-stage build: stage 1 produces the SOVD catalog + tarballs from
# the demo's ros2_packages/ + the gateway repo (for ros2_medkit_msgs);
# stage 2 ships a slim FastAPI server that serves those artefacts. This
# keeps `docker compose build` self-contained and reproducible on CI -
# no separate "build artifacts on host" step required.
#
# Build context is the demo root (so we can pull in ros2_packages/ +
# scripts/ + ota_update_server/). docker-compose.yml wires that up.

# =============================================================================
# Stage 1: build artefacts + catalog inside ros:jazzy
# =============================================================================
FROM ros:jazzy AS artefact_builder

ARG GATEWAY_REPO=https://github.com/selfpatch/ros2_medkit.git
ARG GATEWAY_REF=fix/component-logs-aggregation

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

# ros2_medkit_msgs is the SOVD ReportFault service def - cloned at the
# same ref the gateway image pins so the message ABI stays in lockstep.
RUN git clone --depth=1 --branch ${GATEWAY_REF} ${GATEWAY_REPO} /tmp/ros2_medkit
RUN . /opt/ros/jazzy/setup.sh && \
    cd /tmp/ros2_medkit && \
    colcon build --packages-select ros2_medkit_msgs

# Now bring in the demo's source packages + pack_artifact.py.
WORKDIR /demo
COPY ros2_packages /demo/ros2_packages
COPY scripts /demo/scripts

# Build fixed_lidar + obstacle_classifier_v2 (the only artefacts that
# need a binary - broken_lidar_legacy is uninstall-only). Tarballs +
# catalog.json land in /demo/artifacts.
RUN mkdir -p /demo/ros2_ws/src && \
    for pkg in fixed_lidar obstacle_classifier_v2; do \
      ln -sfn /demo/ros2_packages/$pkg /demo/ros2_ws/src/$pkg; \
    done && \
    . /opt/ros/jazzy/setup.sh && \
    . /tmp/ros2_medkit/install/setup.sh && \
    cd /demo/ros2_ws && \
    colcon build --packages-select fixed_lidar obstacle_classifier_v2

# pack_artifact.py uses pure stdlib (json/tarfile/argparse) so we don't
# need the .venv/pytest dance build_artifacts.sh does locally.
RUN mkdir -p /demo/artifacts && \
    rm -f /demo/artifacts/catalog.json && \
    PACK="python3 /demo/scripts/pack_artifact.py" && \
    $PACK --package fixed_lidar --version 2.1.0 \
          --kind update --target-component scan_sensor_node \
          --executable fixed_lidar_node \
          --replaces-executable broken_lidar_node \
          --notes "Fix /scan noise filter" \
          --skip-build --workspace /demo/ros2_ws \
          --out-dir /demo/artifacts --catalog /demo/artifacts/catalog.json && \
    $PACK --package obstacle_classifier_v2 --version 1.0.0 \
          --kind install --target-component obstacle_classifier \
          --executable obstacle_classifier_node \
          --notes "Extra safety layer for nav2" \
          --skip-build --workspace /demo/ros2_ws \
          --out-dir /demo/artifacts --catalog /demo/artifacts/catalog.json && \
    $PACK --package broken_lidar_legacy --version "" \
          --kind uninstall --target-component broken_lidar_legacy \
          --notes "Clean up deprecated package" \
          --skip-build --workspace /demo/ros2_ws \
          --out-dir /demo/artifacts --catalog /demo/artifacts/catalog.json

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
