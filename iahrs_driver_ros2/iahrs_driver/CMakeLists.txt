cmake_minimum_required(VERSION 3.5)
project(iahrs_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interfaces REQUIRED) #interfaces package
find_package(rosidl_default_generators REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)
find_package(nav_msgs REQUIRED)


set(msg_files
  
  "msg/ImuArray.msg"

)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${dynamixel_sdk_INCLUDE_DIRS}
  ${dynamixel_sdk_custom_interfaces_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${dynamixel_workbench_toolbox_INCLUDE_DIRS}
  
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES sensor_msgs #std_msgs geometry_msgs
)


add_executable(iahrs_driver_node src/iahrs_driver.cpp)

add_executable(iahrs_driver_node_2 src/iahrs_driver_2.cpp)

# Set include directories for the target
#target_include_directories(iahrs_driver_2 PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>
#)

set(dependencies
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  interfaces
  rosidl_default_runtime

  dynamixel_sdk
  dynamixel_sdk_custom_interfaces
  dynamixel_workbench_toolbox
  nav_msgs
)

ament_target_dependencies(iahrs_driver_node
  ${dependencies}
)

ament_target_dependencies(iahrs_driver_node_2
  ${dependencies}
)

# 메시지를 실행 파일에 연결
#rosidl_target_interfaces(iahrs_driver ${PROJECT_NAME} rosidl_typesupport_cpp)
rosidl_target_interfaces(iahrs_driver_node_2 ${PROJECT_NAME} rosidl_typesupport_cpp)


install(TARGETS 
  iahrs_driver_node
  iahrs_driver_node_2
	DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

  	
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})



ament_package()
