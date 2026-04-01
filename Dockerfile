# Base OS: Ubuntu 24.04 (Noble) with ROS 2 Jazzy LTS
FROM osrf/ros:jazzy-desktop

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    v4l-utils \
    htop \
    iproute2 \
    ros-jazzy-rmw-fastrtps-cpp \
    ros-jazzy-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /swarm_ws
COPY ./src /swarm_ws/src

# C++20 compilation via Jazzy
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
