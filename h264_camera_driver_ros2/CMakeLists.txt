cmake_minimum_required(VERSION 3.5)
project(fake_camera_driver)

# C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Build executable
add_executable(fake_camera_node src/fake_camera_node.cpp)

# Link libraries
ament_target_dependencies(fake_camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# Install
install(TARGETS
  fake_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()