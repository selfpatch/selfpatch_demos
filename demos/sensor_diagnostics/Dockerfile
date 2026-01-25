# Sensor Diagnostics Demo
# Lightweight ROS 2 image without Gazebo - fast build and startup
# Image size: ~500MB vs ~4GB for TurtleBot3 demo
# Startup time: ~5s vs ~60s

FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV COLCON_WS=/root/demo_ws

# Install minimal dependencies (no Gazebo, no simulation packages)
RUN apt-get update && apt-get install -y \
    ros-jazzy-ament-lint-auto \
    ros-jazzy-ament-lint-common \
    ros-jazzy-ament-cmake-gtest \
    ros-jazzy-yaml-cpp-vendor \
    ros-jazzy-example-interfaces \
    python3-colcon-common-extensions \
    python3-requests \
    nlohmann-json3-dev \
    libcpp-httplib-dev \
    libsqlite3-dev \
    git \
    curl \
    jq \
    && rm -rf /var/lib/apt/lists/*

# Clone ros2_medkit from GitHub (gateway + dependencies)
WORKDIR ${COLCON_WS}/src
RUN git clone --depth 1 --recurse-submodules https://github.com/selfpatch/ros2_medkit.git && \
    mv ros2_medkit/src/ros2_medkit_gateway . && \
    mv ros2_medkit/src/ros2_medkit_serialization . && \
    mv ros2_medkit/src/ros2_medkit_msgs . && \
    mv ros2_medkit/src/ros2_medkit_fault_manager . && \
    mv ros2_medkit/src/dynamic_message_introspection/dynmsg . && \
    rm -rf ros2_medkit

# Copy demo package
COPY package.xml CMakeLists.txt ${COLCON_WS}/src/sensor_diagnostics_demo/
COPY include/ ${COLCON_WS}/src/sensor_diagnostics_demo/include/
COPY src/ ${COLCON_WS}/src/sensor_diagnostics_demo/src/
COPY config/ ${COLCON_WS}/src/sensor_diagnostics_demo/config/
COPY launch/ ${COLCON_WS}/src/sensor_diagnostics_demo/launch/

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

# Default command: launch the demo
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && source /root/demo_ws/install/setup.bash && ros2 launch sensor_diagnostics_demo demo.launch.py"]
