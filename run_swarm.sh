#!/bin/bash
echo "Initiating Zero-Copy Swarm Containers (ROS 2 Jazzy)..."

# 0. Preemptive strike: Kill and remove any hanging containers from previous runs
docker rm -f swarm_monitor camera_node 2>/dev/null || true

# 1. Start the Swarm Monitor (Watchdog) in the background
# The --rm flag ensures the container deletes itself when stopped.
docker run -d --rm \
    --name swarm_monitor \
    --network host \
    --ipc host \
    --pid host \
    swarm_image ros2 run swarm_core swarm_monitor_node

# 2. Start the Camera Node with Kernel Hardware Mapping capabilities
docker run -it --rm \
    --name camera_node \
    --network host \
    --ipc host \
    --pid host \
    --device /dev/video0:/dev/video0 \
    --group-add video \
    --cap-add=SYS_PTRACE \
    swarm_image ros2 run swarm_core camera_node
