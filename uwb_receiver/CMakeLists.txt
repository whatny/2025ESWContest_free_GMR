cmake_minimum_required(VERSION 3.5)
project(uwb_receiver)

# C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# node
add_executable(uwb_position_node src/uwb_position.cpp)

ament_target_dependencies(uwb_position_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Eigen (modern CMake target)
target_link_libraries(uwb_position_node Eigen3::Eigen)

# install
install(TARGETS uwb_position_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

