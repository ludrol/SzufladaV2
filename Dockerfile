# Dockerfile for ROS2 Jazzy rover workspace
# ==========================================
#
# Build:
#   docker build -t rover_ros2_jazzy .
#
# Run:
#   docker run -it --privileged --rm \
#     --net=host \
#     --env="DISPLAY=$DISPLAY" \
#     --env="QT_X11_NO_MITSHM=1" \
#     -v /dev/bus/usb:/dev/bus/usb \
#     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
#     rover_ros2_jazzy

# 1. Base image: ROS2 Jazzy
FROM ros:jazzy-ros-base

# 2. Install ROS2 desktop + Python dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-desktop \
    python3-pip \
    python3-paho-mqtt \
    mosquitto \
    mosquitto-clients \
    && rm -rf /var/lib/apt/lists/*

# 3. Install Python packages
RUN pip3 install --break-system-packages \
    pyserial \
    paho-mqtt

# 4. Create workspace and copy source
RUN mkdir -p /ros2_ws/src
COPY rover_autonomy /ros2_ws/src/rover_autonomy

# 5. Build the workspace
WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install" || true

# 6. Auto-source ROS2 and workspace on shell start
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
    && echo 'source /ros2_ws/install/setup.bash 2>/dev/null || true' >> /root/.bashrc

CMD ["bash"]
