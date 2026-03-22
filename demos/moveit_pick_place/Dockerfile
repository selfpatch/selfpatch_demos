# MoveIt 2 Panda + ros2_medkit Integration Demo
# Supports fake hardware (default) and Gazebo Harmonic simulation (--gazebo)

FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV COLCON_WS=/root/demo_ws

# Install MoveIt 2, Panda, ros2_control, Gazebo, and build dependencies
RUN apt-get update && apt-get install -y \
    ros-jazzy-moveit \
    ros-jazzy-moveit-resources-panda-moveit-config \
    ros-jazzy-moveit-resources-panda-description \
    ros-jazzy-moveit-planners-ompl \
    ros-jazzy-moveit-ros-planning-interface \
    ros-jazzy-moveit-ros-visualization \
    ros-jazzy-moveit-simple-controller-manager \
    ros-jazzy-moveit-servo \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros2-control \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ament-lint-auto \
    ros-jazzy-ament-lint-common \
    python3-colcon-common-extensions \
    nlohmann-json3-dev \
    libcpp-httplib-dev \
    ros-jazzy-rosbag2-storage-mcap \
    ros-jazzy-foxglove-bridge \
    libsystemd-dev \
    sqlite3 libsqlite3-dev git curl jq \
    && rm -rf /var/lib/apt/lists/*

# Create persistent directories for fault storage and rosbag recordings
RUN mkdir -p /var/lib/ros2_medkit/rosbags

# Clone ros2_medkit from GitHub (pinned to a specific ref for reproducibility)
ARG ROS2_MEDKIT_REF=main
WORKDIR ${COLCON_WS}/src
RUN git clone --depth 1 --branch ${ROS2_MEDKIT_REF} https://github.com/selfpatch/ros2_medkit.git && \
    mv ros2_medkit/src/ros2_medkit_cmake . && \
    mv ros2_medkit/src/ros2_medkit_gateway \
       ros2_medkit/src/ros2_medkit_msgs \
       ros2_medkit/src/ros2_medkit_serialization \
       ros2_medkit/src/ros2_medkit_fault_manager \
       ros2_medkit/src/ros2_medkit_fault_reporter \
       ros2_medkit/src/ros2_medkit_diagnostic_bridge . && \
    mv ros2_medkit/src/ros2_medkit_plugins/ros2_medkit_graph_provider . && \
    mv ros2_medkit/src/ros2_medkit_discovery_plugins/ros2_medkit_beacon_common . && \
    mv ros2_medkit/src/ros2_medkit_discovery_plugins/ros2_medkit_topic_beacon . && \
    mv ros2_medkit/src/ros2_medkit_discovery_plugins/ros2_medkit_param_beacon . && \
    mv ros2_medkit/src/ros2_medkit_discovery_plugins/ros2_medkit_linux_introspection . && \
    rm -rf ros2_medkit

# Copy demo package from local context
COPY package.xml CMakeLists.txt ${COLCON_WS}/src/moveit_medkit_demo/
COPY config/ ${COLCON_WS}/src/moveit_medkit_demo/config/
COPY launch/ ${COLCON_WS}/src/moveit_medkit_demo/launch/
COPY scripts/ ${COLCON_WS}/src/moveit_medkit_demo/scripts/
COPY worlds/ ${COLCON_WS}/src/moveit_medkit_demo/worlds/

# TODO(#49): Move to manifest-defined scripts once ros2_medkit#303 lands
COPY container_scripts/ /var/lib/ros2_medkit/scripts/
RUN find /var/lib/ros2_medkit/scripts -name "*.bash" -exec chmod +x {} \;

# Build ros2_medkit and demo package
# Note: rosdep install uses || true because ros2_medkit packages are not in
# rosdep indices (they're built from source). All system deps are already
# installed via apt above; rosdep handles any transitive deps it can resolve.
WORKDIR ${COLCON_WS}
RUN bash -c "source /opt/ros/jazzy/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y || true" && \
    bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF"

# Setup environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source ${COLCON_WS}/install/setup.bash" >> ~/.bashrc

EXPOSE 8080 8765

CMD ["bash"]
