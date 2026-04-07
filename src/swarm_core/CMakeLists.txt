cmake_minimum_required(VERSION 3.8)
project(swarm_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(swarm_interfaces REQUIRED)

# Compile the Boids Brain
add_executable(boid_controller src/boid_controller.cpp)
ament_target_dependencies(boid_controller rclcpp geometry_msgs nav_msgs swarm_interfaces)

# Install Launch and URDF directories
install(DIRECTORY
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install the executable
install(TARGETS
  boid_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
