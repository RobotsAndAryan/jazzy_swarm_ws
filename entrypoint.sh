#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /swarm_ws/install/setup.bash
exec "$@"
