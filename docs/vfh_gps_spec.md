# Module 3: VFH & GPS Integration

## Local Planner: Vector Field Histogram (VFH)
To solve the "Local Minima" trap, we transitioned from Boids to a VFH local planner.
- **Histogram Density:** 36 bins (10° resolution).
- **Logic:** Identifies "valleys" of low obstacle density and steers toward the gap closest to the target waypoint.

## Global Positioning: NavSat
- **Location:** Stratford Olympic Park, London.
- **Coordinates:** 51.5413 N, -0.0156 W.
- **Interface:** ROS 2 `sensor_msgs/NavSatFix`.
