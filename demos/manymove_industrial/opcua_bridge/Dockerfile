FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    OPCUA_ENDPOINT=opc.tcp://plc-sim:4840/manymove_plc/server \
    BRIDGE_SOURCE_ID=/plc/sensor_io \
    REPORT_FAULT_SERVICE=/fault_manager/report_fault

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        ros-jazzy-ros2-medkit-msgs=0.4.0-1* \
        ros-jazzy-ros2-medkit-fault-reporter=0.4.0-1* \
    && rm -rf /var/lib/apt/lists/* \
    && pip install --no-cache-dir --break-system-packages \
        "asyncua==1.1.5"

WORKDIR /opt/opcua_bridge
COPY bridge.py ./

CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && exec python3 -u bridge.py"]
