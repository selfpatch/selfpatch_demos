# manymove_industrial demo image: manymove fork + ros2_medkit fault reporting.
# Builds the BT framework with medkit instrumentation enabled and ships
# fake-hardware launch files so the demo runs without xArm hardware.

FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV COLCON_WS=/root/demo_ws

# manymove + medkit runtime dependencies. ros2-medkit-* come from rosdistro
# debs (see ../README.md for the package list).
RUN apt-get update && apt-get install -y \
    ros-jazzy-moveit \
    ros-jazzy-moveit-ros-planning-interface \
    ros-jazzy-moveit-ros-visualization \
    ros-jazzy-moveit-servo \
    ros-jazzy-moveit-visual-tools \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros2-control \
    ros-jazzy-controller-manager \
    ros-jazzy-control-toolbox \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-tf-transformations \
    ros-jazzy-behaviortree-cpp-v3 \
    ros-jazzy-vision-msgs \
    ros-jazzy-simulation-interfaces \
    ros-jazzy-rosbag2-storage-mcap \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-ros2-medkit-fault-reporter \
    ros-jazzy-ros2-medkit-msgs \
    ros-jazzy-ros2-medkit-fault-manager \
    ros-jazzy-ros2-medkit-gateway \
    ros-jazzy-ros2-medkit-cmake \
    ros-jazzy-ros2-medkit-graph-provider \
    ros-jazzy-ros2-medkit-diagnostic-bridge \
    python3-colcon-common-extensions \
    python3-rosdep \
    nlohmann-json3-dev \
    libcpp-httplib-dev \
    libsystemd-dev \
    sqlite3 libsqlite3-dev git curl jq \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /var/lib/ros2_medkit/rosbags

# Pin to the selfpatch fork tip; bump as needed for releases.
ARG MANYMOVE_REF=feat/medkit-integration

WORKDIR ${COLCON_WS}/src
RUN git clone --depth 1 --branch ${MANYMOVE_REF} https://github.com/selfpatch/manymove.git && \
    git clone --depth 1 https://github.com/PickNikRobotics/topic_based_ros2_control.git || true

# Demo package (manifest + launch + container_scripts)
COPY package.xml CMakeLists.txt ${COLCON_WS}/src/manymove_industrial/
COPY config/   ${COLCON_WS}/src/manymove_industrial/config/
COPY launch/   ${COLCON_WS}/src/manymove_industrial/launch/

# medkit gateway scripts (mounted at the well-known path the gateway scans).
COPY container_scripts/ /var/lib/ros2_medkit/scripts/
RUN find /var/lib/ros2_medkit/scripts -name '*.bash' -exec chmod +x {} \;

WORKDIR ${COLCON_WS}
RUN bash -c "source /opt/ros/jazzy/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys topic_based_ros2_control || true" && \
    bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
      --packages-up-to manymove_industrial"

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source ${COLCON_WS}/install/setup.bash" >> ~/.bashrc

EXPOSE 8080 8765

CMD ["bash"]
