cmake_minimum_required(VERSION 3.8)
project(h264_camera_driver)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========================================
# Find ROS2 Packages
# ========================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# NO cv_bridge needed! We manually construct messages

# ========================================
# Find OpenCV
# ========================================
find_package(OpenCV REQUIRED)

# ========================================
# NVIDIA Jetson Libraries
# ========================================
# These are specific to Jetson platforms
include_directories(
  /usr/include/aarch64-linux-gnu
  /usr/src/jetson_multimedia_api/include
)

link_directories(
  /usr/lib/aarch64-linux-gnu
  /usr/lib/aarch64-linux-gnu/tegra
)

# ========================================
# Build the Camera Feed Wrapper Library
# ========================================
add_library(camera_feed_wrapper
  src/core/CameraFeedWrapper_ros2.cpp
)

# FIX: Point to the include directory, not root!
target_include_directories(camera_feed_wrapper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(camera_feed_wrapper
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(camera_feed_wrapper
  ${OpenCV_LIBS}
  nvbufsurface
  nvbufsurftransform
)

# ========================================
# Build the Fake Camera Node (for testing)
# ========================================
add_executable(fake_camera_node
  src/fake_camera_node.cpp
)

# FIX: Point to include directory for fake_camera_node too!
target_include_directories(fake_camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(fake_camera_node
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(fake_camera_node
  camera_feed_wrapper
  ${OpenCV_LIBS}
)

# ========================================
# Install
# ========================================

# Install the library
install(TARGETS camera_feed_wrapper
  EXPORT camera_feed_wrapper
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS fake_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python test script
# install(PROGRAMS
#   test_camera_feed.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# FIX: Install entire include directory
install(
  DIRECTORY include/
  DESTINATION include
)

# ========================================
# Testing
# ========================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# FIX: Add this at the end!
ament_package()