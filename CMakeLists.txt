cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0074 NEW)
project(ndt_localization)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

# Include PCL directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Create NDT library
add_library(ndt_2d src/ndt_2d.cpp)
target_include_directories(ndt_2d PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ndt_2d ${PCL_LIBRARIES})

# Create NDT localization node
add_executable(ndt_localization_node src/ndt_localization_node.cpp)
target_link_libraries(ndt_localization_node ndt_2d ${PCL_LIBRARIES})
ament_target_dependencies(ndt_localization_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  nav2_util
  pcl_conversions
)

# Install files
install(TARGETS
  ndt_2d
  ndt_localization_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
DESTINATION include/
)

ament_package()
