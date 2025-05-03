cmake_minimum_required(VERSION 3.5)
project(ndt_localization)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${nav2_util_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

# Add executable
add_executable(ndt_localization_node
  src/ndt_localization_node.cpp
)

# Link libraries
# Link dependencies properly
ament_target_dependencies(ndt_localization_node
  rclcpp
  nav2_util
  tf2
  tf2_ros
  sensor_msgs
  nav_msgs
  geometry_msgs
)

# Install targets
install(TARGETS ndt_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  nav2_util
  tf2
  tf2_ros
  sensor_msgs
  nav_msgs
  geometry_msgs
)

# Generate the package
ament_package()
