# manymove_industrial demo image: matches the manymove project's own docker
# style (manymove + Groot + xArm ROS 2 driver, all built from source) and
# layers on the ros2_medkit fault reporting stack.
#
# The base layers reproduce manymove_bringup/docker/Dockerfile.manymove and
# Dockerfile.manymove_xarm so the resulting image runs xarm7_movegroup_fake
# launches identically to the upstream demos. MANYMOVE_REPO defaults to the
# selfpatch fork on the feat/medkit-integration branch where the BT action
# nodes emit MANYMOVE_* fault codes via ros2_medkit_fault_reporter.

# syntax=docker/dockerfile:1
ARG ROS_DISTRO=jazzy
ARG BASE_IMAGE=ros:${ROS_DISTRO}
FROM ${BASE_IMAGE}

ARG ROS_DISTRO=jazzy
ARG USERNAME=manymove
ARG USER_UID=1000
ARG USER_GID=1000
ARG MANYMOVE_COLCON_WORKERS=
ARG MANYMOVE_REPO=https://github.com/selfpatch/manymove.git
ARG MANYMOVE_BRANCH=feat/medkit-integration
ARG MANYMOVE_COMMIT=
ARG GROOT_REPO=https://github.com/pastoriomarco/Groot.git
ARG GROOT_BRANCH=master
ARG GROOT_COMMIT=
ARG XARM_REPO=https://github.com/pastoriomarco/xarm_ros2.git
ARG XARM_BRANCH=${ROS_DISTRO}
ARG XARM_COMMIT=

ENV DEBIAN_FRONTEND=noninteractive \
    MANYMOVE_WS=/opt/manymove_ws \
    ROS_DISTRO=${ROS_DISTRO} \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    TZ=UTC \
    USERNAME=${USERNAME} \
    USER=${USERNAME} \
    QT_X11_NO_MITSHM=1 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    MANYMOVE_COLCON_WORKERS=${MANYMOVE_COLCON_WORKERS}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Isaac ROS apt repo for jazzy provides ros-jazzy-topic-based-ros2-control,
# matching what the upstream manymove docker image expects.
RUN set -euo pipefail; \
    if [[ "${ROS_DISTRO}" == "jazzy" ]]; then \
        apt-get update && \
        apt-get install -y curl gnupg && \
        mkdir -p /usr/share/keyrings && \
        curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key \
          | gpg --dearmor -o /usr/share/keyrings/isaac-ros-archive-keyring.gpg && \
        . /etc/os-release && \
        printf 'deb [signed-by=/usr/share/keyrings/isaac-ros-archive-keyring.gpg] https://isaac.download.nvidia.com/isaac-ros/release-4 %s main\n' "${VERSION_CODENAME}" \
          > /etc/apt/sources.list.d/isaac-ros.list && \
        apt-get update; \
    fi

# manymove + Groot build deps, MoveIt 2 stack, BT/ros2_control, plus the
# ros2_medkit deb packages this fork instruments against.
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        cmake build-essential curl git gosu locales sudo \
        python3-colcon-common-extensions python3-vcstool python3-rosdep python3-pip \
        nano jq sqlite3 libsqlite3-dev libsystemd-dev nlohmann-json3-dev libcpp-httplib-dev \
        mesa-utils libgl1-mesa-dri libglu1-mesa libglvnd0 libglx-mesa0 libegl1 \
        libxcb-xinerama0 libxkbcommon-x11-0 \
        libzmq3-dev qtbase5-dev qttools5-dev-tools libqt5svg5-dev libqt5opengl5-dev \
        ros-${ROS_DISTRO}-moveit \
        ros-${ROS_DISTRO}-moveit-msgs \
        ros-${ROS_DISTRO}-moveit-core \
        ros-${ROS_DISTRO}-moveit-ros-planning-interface \
        ros-${ROS_DISTRO}-moveit-ros-visualization \
        ros-${ROS_DISTRO}-moveit-servo \
        ros-${ROS_DISTRO}-moveit-visual-tools \
        ros-${ROS_DISTRO}-moveit-resources-panda-moveit-config \
        ros-${ROS_DISTRO}-controller-manager \
        ros-${ROS_DISTRO}-controller-manager-msgs \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-gripper-controllers \
        ros-${ROS_DISTRO}-control-toolbox \
        ros-${ROS_DISTRO}-control-msgs \
        ros-${ROS_DISTRO}-geometric-shapes \
        ros-${ROS_DISTRO}-behaviortree-cpp-v3 \
        ros-${ROS_DISTRO}-ur \
        ros-${ROS_DISTRO}-ur-moveit-config \
        ros-${ROS_DISTRO}-robotiq-description \
        ros-${ROS_DISTRO}-robotiq-controllers \
        ros-${ROS_DISTRO}-topic-based-ros2-control \
        ros-${ROS_DISTRO}-simulation-interfaces \
        ros-${ROS_DISTRO}-vision-msgs \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-joint-state-broadcaster \
        ros-${ROS_DISTRO}-joint-trajectory-controller \
        ros-${ROS_DISTRO}-diff-drive-controller \
        ros-${ROS_DISTRO}-tf-transformations \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-image-view \
        ros-${ROS_DISTRO}-find-object-2d \
        ros-${ROS_DISTRO}-joy \
        ros-${ROS_DISTRO}-camera-info-manager \
        ros-${ROS_DISTRO}-hardware-interface \
        ros-${ROS_DISTRO}-ros-gz \
        ros-${ROS_DISTRO}-ros-gz-bridge \
        ros-${ROS_DISTRO}-ros-gz-sim \
        ros-${ROS_DISTRO}-gz-ros2-control \
        ros-${ROS_DISTRO}-rosbag2-storage-mcap \
        ros-${ROS_DISTRO}-foxglove-bridge \
        ros-${ROS_DISTRO}-ros2-medkit-fault-reporter \
        ros-${ROS_DISTRO}-ros2-medkit-msgs \
        ros-${ROS_DISTRO}-ros2-medkit-fault-manager \
        ros-${ROS_DISTRO}-ros2-medkit-gateway \
        ros-${ROS_DISTRO}-ros2-medkit-cmake \
        ros-${ROS_DISTRO}-ros2-medkit-graph-provider \
        ros-${ROS_DISTRO}-ros2-medkit-diagnostic-bridge && \
    rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8

# Non-root user matching the manymove pattern so X11 forwarding lines up.
RUN set -eux; \
    existing_group="$(getent group ${USER_GID} | cut -d: -f1 || true)"; \
    if [[ -n "${existing_group}" ]]; then \
        if [[ "${existing_group}" != "${USERNAME}" ]]; then \
            groupmod --new-name "${USERNAME}" "${existing_group}"; \
        fi; \
    else \
        groupadd --gid ${USER_GID} "${USERNAME}"; \
    fi; \
    existing_uid_user="$(getent passwd ${USER_UID} | cut -d: -f1 || true)"; \
    if [[ -n "${existing_uid_user}" && "${existing_uid_user}" != "${USERNAME}" ]]; then \
        usermod --login "${USERNAME}" --home "/home/${USERNAME}" --move-home "${existing_uid_user}"; \
    elif getent passwd "${USERNAME}" >/dev/null 2>&1; then \
        usermod --gid ${USER_GID} --home "/home/${USERNAME}" --move-home "${USERNAME}"; \
    else \
        useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash "${USERNAME}"; \
    fi; \
    usermod --gid ${USER_GID} --shell /bin/bash "${USERNAME}"; \
    usermod -aG sudo "${USERNAME}"; \
    printf '%s ALL=(ALL) NOPASSWD:ALL\n' "${USERNAME}" > "/etc/sudoers.d/${USERNAME}"; \
    chmod 0440 "/etc/sudoers.d/${USERNAME}"

RUN mkdir -p /tmp/runtime-${USERNAME} && chown ${USERNAME}:${USER_GID} /tmp/runtime-${USERNAME}
ENV XDG_RUNTIME_DIR=/tmp/runtime-${USERNAME}
RUN mkdir -p /var/lib/ros2_medkit/rosbags && chown -R ${USERNAME}:${USER_GID} /var/lib/ros2_medkit
RUN mkdir -p ${MANYMOVE_WS}/src && chown -R ${USERNAME}:${USER_GID} ${MANYMOVE_WS}

# Clone the selfpatch manymove fork (override via --build-arg if needed).
RUN gosu ${USERNAME} bash -lc "set -euo pipefail; \
    rm -rf ${MANYMOVE_WS}/src/manymove && \
    git clone --branch ${MANYMOVE_BRANCH} --depth 1 ${MANYMOVE_REPO} ${MANYMOVE_WS}/src/manymove && \
    cd ${MANYMOVE_WS}/src/manymove && \
    if [ -n \"${MANYMOVE_COMMIT}\" ]; then \
        git fetch --depth 1 origin ${MANYMOVE_COMMIT} && \
        git checkout --detach ${MANYMOVE_COMMIT}; \
    fi && \
    git rev-parse HEAD > ${MANYMOVE_WS}/.manymove_commit"

# Groot - BT visualizer, lets viewers see the BT execution live during demos.
RUN gosu ${USERNAME} bash -lc "set -euo pipefail; \
    cd ${MANYMOVE_WS}/src && \
    rm -rf Groot && \
    git clone --recurse-submodules --branch ${GROOT_BRANCH} ${GROOT_REPO} Groot && \
    cd Groot && \
    cmake -S . -B build && \
    cmake --build build --parallel"

# xArm ROS 2 driver - source clone is what upstream manymove relies on.
RUN gosu ${USERNAME} bash -lc "set -euo pipefail; \
    cd ${MANYMOVE_WS}/src && \
    rm -rf xarm_ros2 && \
    git clone --recursive --branch ${XARM_BRANCH} ${XARM_REPO} xarm_ros2 && \
    cd xarm_ros2 && git submodule update --init --recursive"

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${MANYMOVE_WS} && \
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && \
    apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y -r && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

RUN gosu ${USERNAME} bash -lc "set -eo pipefail; \
    source /opt/ros/${ROS_DISTRO}/setup.bash; \
    cd ${MANYMOVE_WS}; \
    parallel_args=\"\"; \
    if [[ -n \"${MANYMOVE_COLCON_WORKERS}\" && \"${MANYMOVE_COLCON_WORKERS}\" =~ ^[0-9]+$ && \"${MANYMOVE_COLCON_WORKERS}\" -gt 0 ]]; then \
        parallel_args=\"--parallel-workers ${MANYMOVE_COLCON_WORKERS}\"; \
        export CMAKE_BUILD_PARALLEL_LEVEL=${MANYMOVE_COLCON_WORKERS}; \
    fi; \
    colcon build --symlink-install \${parallel_args}"
RUN chown -R ${USERNAME}:${USER_GID} ${MANYMOVE_WS}

# Demo overlay: manifest + container_scripts + launch files.
COPY package.xml CMakeLists.txt ${MANYMOVE_WS}/src/manymove_industrial/
COPY config/   ${MANYMOVE_WS}/src/manymove_industrial/config/
COPY launch/   ${MANYMOVE_WS}/src/manymove_industrial/launch/
COPY container_scripts/ /var/lib/ros2_medkit/scripts/
RUN find /var/lib/ros2_medkit/scripts -name '*.bash' -exec chmod +x {} \; && \
    chown -R ${USERNAME}:${USER_GID} ${MANYMOVE_WS}/src/manymove_industrial /var/lib/ros2_medkit
RUN gosu ${USERNAME} bash -lc "set -eo pipefail; \
    source /opt/ros/${ROS_DISTRO}/setup.bash; \
    source ${MANYMOVE_WS}/install/setup.bash; \
    cd ${MANYMOVE_WS}; \
    colcon build --symlink-install --packages-select manymove_industrial"

WORKDIR ${MANYMOVE_WS}
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc && \
    echo "source ${MANYMOVE_WS}/install/setup.bash" >> /etc/bash.bashrc

USER ${USERNAME}
EXPOSE 8080 8765

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
