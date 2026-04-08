# Swarm Core: Urban Autonomy & Visual SLAM

## Overview
This repository contains a ROS 2 Jazzy and Gazebo Harmonic simulation for a multi-agent drone swarm operating in GPS-denied urban environments (Stratford Olympic Park). 

## Architecture
The system utilizes a **Hybrid Architecture**:
* **High-Level Planner:** A tactical waypoint system utilizing "Manhattan Routing" and Breadcrumb logic to navigate sharp 90-degree urban canyons.
* **Low-Level Reflexes:** A **Vector Field Histogram (VFH)** local planner. The drone processes 360-degree LiDAR data into a 36-bin polar histogram, identifying physical obstacles and dynamically calculating the safest obstacle-free steering vector.
* **Visual SLAM:** Real-time 3D point-cloud mapping of the environment via `rviz2`, bridging Gazebo physics data to the ROS 2 computation graph.

## Configuration Parameters
* `safe_distance`: **1.2m** (Aggressively tuned for 5.0m wide urban corridors).
* `w_sep` (Separation Weight): **3.0**
* `w_mig` (Migration Weight): **2.5**

## Ignition Sequence
Ensure the workspace is built (`colcon build --packages-select swarm_core`) and sourced (`source install/setup.bash`).

1. **The Matrix:** `ros2 launch swarm_core sim_swarm.launch.py`
2. **The TF Bridge:** `ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom alpha/dummy_link/lidar_sensor`
3. **The Command Center:** `rviz2 -d src/swarm_core/rviz/mapper.rviz`
4. **The Hive-Mind:** `ros2 launch swarm_core swarm_brain.launch.py`
