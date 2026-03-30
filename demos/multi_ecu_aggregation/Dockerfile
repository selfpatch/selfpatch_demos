# Multi-ECU Aggregation Demo
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV COLCON_WS=/root/demo_ws

# Install minimal dependencies (no GUI/simulation)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-domain-bridge \
    ros-${ROS_DISTRO}-ament-lint-auto \
    ros-${ROS_DISTRO}-ament-lint-common \
    ros-${ROS_DISTRO}-ament-cmake-gtest \
    ros-${ROS_DISTRO}-yaml-cpp-vendor \
    ros-${ROS_DISTRO}-example-interfaces \
    python3-colcon-common-extensions \
    python3-requests \
    nlohmann-json3-dev \
    libcpp-httplib-dev \
    libsystemd-dev \
    sqlite3 \
    libsqlite3-dev \
    git \
    curl \
    jq \
    && rm -rf /var/lib/apt/lists/*

# Clone ros2_medkit from GitHub (gateway + dependencies + plugins)
ARG ROS2_MEDKIT_REF=feat/entity-model-and-aggregation
WORKDIR ${COLCON_WS}/src
RUN git clone --depth 1 --branch ${ROS2_MEDKIT_REF} https://github.com/selfpatch/ros2_medkit.git && \
    mv ros2_medkit/src/ros2_medkit_cmake . && \
    mv ros2_medkit/src/ros2_medkit_gateway . && \
    mv ros2_medkit/src/ros2_medkit_serialization . && \
    mv ros2_medkit/src/ros2_medkit_msgs . && \
    mv ros2_medkit/src/ros2_medkit_fault_manager . && \
    mv ros2_medkit/src/ros2_medkit_fault_reporter . && \
    mv ros2_medkit/src/ros2_medkit_diagnostic_bridge . && \
    mv ros2_medkit/src/ros2_medkit_plugins/ros2_medkit_graph_provider . && \
    rm -rf ros2_medkit

# Copy demo package
COPY package.xml CMakeLists.txt ${COLCON_WS}/src/multi_ecu_demo/
COPY src/ ${COLCON_WS}/src/multi_ecu_demo/src/
COPY config/ ${COLCON_WS}/src/multi_ecu_demo/config/
COPY launch/ ${COLCON_WS}/src/multi_ecu_demo/launch/

# Copy container scripts
COPY container_scripts/ /var/lib/ros2_medkit/scripts/
RUN find /var/lib/ros2_medkit/scripts -name "*.bash" -exec chmod +x {} \;

# Build all packages (skip test dependencies that aren't in ros-base)
WORKDIR ${COLCON_WS}
RUN bash -c "source /opt/ros/jazzy/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y \
      --skip-keys='ament_cmake_clang_format ament_cmake_clang_tidy test_msgs example_interfaces sqlite3' && \
    colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF"

# Setup environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source ${COLCON_WS}/install/setup.bash" >> ~/.bashrc

# Expose gateway port
EXPOSE 8080

# ECU_LAUNCH env var selects which launch file to run
ENV ECU_LAUNCH=perception.launch.py
CMD ["bash", "-c", "mkdir -p /var/lib/ros2_medkit/rosbags && source /opt/ros/jazzy/setup.bash && source /root/demo_ws/install/setup.bash && ros2 launch multi_ecu_demo ${ECU_LAUNCH}"]
