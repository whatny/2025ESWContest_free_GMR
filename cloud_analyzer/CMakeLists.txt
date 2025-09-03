cmake_minimum_required(VERSION 3.5)
project(cloud_analyzer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

# TF2 dependencies for coordinate transformation
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Create executable
add_executable(cloud_subscriber src/cloud_subscriber.cpp)

# Target dependencies
ament_target_dependencies(cloud_subscriber 
  rclcpp 
  sensor_msgs 
  geometry_msgs
  pcl_ros 
  pcl_conversions
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Link PCL libraries
target_link_libraries(cloud_subscriber ${PCL_LIBRARIES})

# Install targets
install(TARGETS
  cloud_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if any
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
