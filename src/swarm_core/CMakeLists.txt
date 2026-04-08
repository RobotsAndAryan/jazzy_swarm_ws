cmake_minimum_required(VERSION 3.8)
project(swarm_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 1. The Boids Brain
add_executable(boid_controller src/boid_controller.cpp)
# THE FIX: Added sensor_msgs so the brain is authorized to use LiDAR
ament_target_dependencies(boid_controller rclcpp geometry_msgs nav_msgs sensor_msgs)

# 2. The Visual Cortex
add_executable(visual_cortex src/visual_cortex.cpp)
target_include_directories(visual_cortex PUBLIC ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(visual_cortex rclcpp sensor_msgs cv_bridge)
target_link_libraries(visual_cortex ${OpenCV_LIBRARIES}) 

# 3. Installation Pipeline
install(DIRECTORY launch urdf worlds rviz DESTINATION share/${PROJECT_NAME})
install(TARGETS boid_controller visual_cortex DESTINATION lib/${PROJECT_NAME})

ament_package()
