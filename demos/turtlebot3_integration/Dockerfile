# TurtleBot3 + ros2_medkit Discovery Demo with Nav2 Navigation
# Full desktop image with Gazebo GUI support

FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV TURTLEBOT3_MODEL=burger
ENV GAZEBO_MODEL_PATH=/opt/ros/jazzy/share/turtlebot3_gazebo/models
ENV COLCON_WS=/root/demo_ws

# Install TurtleBot3, Nav2, and build dependencies
RUN apt-get update && apt-get install -y \
    ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-turtlebot3-msgs \
    ros-jazzy-turtlebot3-description \
    ros-jazzy-turtlebot3-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-bt-navigator \
    ros-jazzy-nav2-controller \
    ros-jazzy-nav2-planner \
    ros-jazzy-nav2-behaviors \
    ros-jazzy-nav2-costmap-2d \
    ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-nav2-map-server \
    ros-jazzy-nav2-amcl \
    ros-jazzy-ament-lint-auto \
    ros-jazzy-ament-lint-common \
    ros-jazzy-ament-clang-format \
    ros-jazzy-ament-cmake-clang-format \
    ros-jazzy-ament-cmake-clang-tidy \
    ros-jazzy-ament-cmake-gtest \
    ros-jazzy-ament-cmake-pytest \
    ros-jazzy-launch-testing-ament-cmake \
    python3-colcon-common-extensions \
    python3-requests \
    nlohmann-json3-dev \
    libcpp-httplib-dev \
    git \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Create workspace and clone ros2_medkit
# TODO: Replace with proper ROS 2 package dependency once ros2_medkit is released
WORKDIR ${COLCON_WS}/src
RUN git clone https://github.com/selfpatch/ros2_medkit.git

# Copy demo package
COPY package.xml CMakeLists.txt ${COLCON_WS}/src/turtlebot3_medkit_demo/
COPY config/ ${COLCON_WS}/src/turtlebot3_medkit_demo/config/
COPY launch/ ${COLCON_WS}/src/turtlebot3_medkit_demo/launch/

# Build ros2_medkit and demo package
WORKDIR ${COLCON_WS}
RUN bash -c "source /opt/ros/jazzy/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install"

# Setup environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source ${COLCON_WS}/install/setup.bash" >> ~/.bashrc && \
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=/opt/ros/jazzy/share/turtlebot3_gazebo/models" >> ~/.bashrc

EXPOSE 8080

CMD ["bash"]
