cmake_minimum_required(VERSION 3.12)
project(swarm_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -std=c++20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(swarm_interfaces REQUIRED)

# Tell the build system to find the Nvidia CUDA Toolkit
find_package(CUDAToolkit REQUIRED)

add_executable(core_bridge_node src/core_bridge_node.cpp)
ament_target_dependencies(core_bridge_node rclcpp std_msgs)

add_executable(swarm_monitor_node src/swarm_monitor_node.cpp)
ament_target_dependencies(swarm_monitor_node rclcpp std_msgs)

add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node rclcpp swarm_interfaces)
# Link the CUDA runtime library so we can use cudaMemcpy and cudaHostRegister
target_link_libraries(camera_node CUDA::cudart)

install(TARGETS
  core_bridge_node
  swarm_monitor_node
  camera_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
