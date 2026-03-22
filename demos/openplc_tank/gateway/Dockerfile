# Multi-stage build: ros2_medkit gateway + OPC-UA plugin
# Stage 1: Build ros2_medkit
# Stage 2: Build OPC-UA plugin against installed medkit headers
# Stage 3: Runtime

# ---- Stage 1: Build ros2_medkit ----
FROM ros:jazzy AS medkit-builder

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    libsqlite3-dev \
    libssl-dev \
    nlohmann-json3-dev \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# Copy ros2_medkit source
COPY ros2_medkit/ /medkit_ws/src/ros2_medkit/

WORKDIR /medkit_ws

# Build ros2_medkit
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
      --packages-up-to ros2_medkit_gateway ros2_medkit_fault_manager ros2_medkit_msgs ros2_medkit_cmake

# ---- Stage 2: Build OPC-UA plugin ----
FROM medkit-builder AS plugin-builder

RUN apt-get update && apt-get install -y \
    git \
    && rm -rf /var/lib/apt/lists/*

# Copy plugin source
COPY opcua_gateway_plugin/ /plugin_ws/src/opcua_gateway_plugin/

WORKDIR /plugin_ws

# Build plugin against installed medkit
RUN . /opt/ros/jazzy/setup.sh && \
    . /medkit_ws/install/setup.sh && \
    colcon build \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

# ---- Stage 3: Runtime ----
FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    libsqlite3-0 \
    libssl3 \
    libyaml-cpp0.8 \
    curl \
    jq \
    && rm -rf /var/lib/apt/lists/*

# Copy medkit install
COPY --from=medkit-builder /medkit_ws/install/ /medkit_ws/install/

# Copy plugin install
COPY --from=plugin-builder /plugin_ws/install/ /plugin_ws/install/

# Copy configuration files
COPY config/ /config/

EXPOSE 8080

# Start gateway with plugin
CMD ["bash", "-c", \
     "mkdir -p /var/lib/ros2_medkit/rosbags && \
      source /opt/ros/jazzy/setup.bash && \
      source /medkit_ws/install/setup.bash && \
      source /plugin_ws/install/setup.bash && \
      ros2 run ros2_medkit_gateway gateway_node \
        --ros-args --params-file /config/gateway_params.yaml"]
