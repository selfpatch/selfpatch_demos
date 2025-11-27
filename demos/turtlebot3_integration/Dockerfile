# TurtleBot3 + ros2_medkit Discovery Demo
# Full desktop image with Gazebo GUI support

FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV TURTLEBOT3_MODEL=burger
ENV COLCON_WS=/root/demo_ws

# Install TurtleBot3 and build dependencies
RUN apt-get update && apt-get install -y \
    ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-turtlebot3-msgs \
    ros-jazzy-turtlebot3-description \
    ros-jazzy-ament-clang-format \
    python3-colcon-common-extensions \
    python3-requests \
    nlohmann-json3-dev \
    libcpp-httplib-dev \
    git \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Create workspace and clone ros2_medkit
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
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc

EXPOSE 8080

CMD ["bash"]
