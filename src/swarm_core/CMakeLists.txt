cmake_minimum_required(VERSION 3.12)
project(swarm_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -std=c++20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(swarm_interfaces REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(core_bridge_node src/core_bridge_node.cpp)
ament_target_dependencies(core_bridge_node rclcpp std_msgs)

add_executable(swarm_monitor_node src/swarm_monitor_node.cpp)
ament_target_dependencies(swarm_monitor_node rclcpp std_msgs)

add_executable(camera_node src/camera_node.cpp)
target_include_directories(camera_node PUBLIC ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(camera_node rclcpp swarm_interfaces)
target_link_libraries(camera_node CUDA::cudart ${OpenCV_LIBS})

add_executable(supervisor_node src/supervisor_node.cpp)
target_include_directories(supervisor_node PUBLIC ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(supervisor_node rclcpp swarm_interfaces geometry_msgs)
target_link_libraries(supervisor_node ${OpenCV_LIBS})

install(TARGETS
  core_bridge_node
  swarm_monitor_node
  camera_node
  supervisor_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
