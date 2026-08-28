cmake_minimum_required(VERSION 3.8)
project(xczs_inspection_robot_gazebo)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(xczs_inspection_robot_control REQUIRED)
find_package(xczs_inspection_robot_interfaces REQUIRED)

add_library(
  xczs_planar_stabilizer
  SHARED
  src/planar_stabilizer_plugin.cpp
)
add_library(
  xczs_cabinet_state
  SHARED
  src/cabinet_state_plugin.cpp
)
add_library(
  xczs_ros_global_args_guard
  SHARED
  src/xczs_ros_global_args_guard.cpp
)

target_compile_features(xczs_planar_stabilizer PUBLIC cxx_std_17)
ament_target_dependencies(
  xczs_planar_stabilizer
  gazebo_dev
  gazebo_ros
  geometry_msgs
  rclcpp
  std_msgs
)

target_compile_features(xczs_cabinet_state PUBLIC cxx_std_17)
ament_target_dependencies(
  xczs_cabinet_state
  gazebo_dev
  gazebo_ros
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  xczs_inspection_robot_control
  xczs_inspection_robot_interfaces
)
target_link_libraries(
  xczs_cabinet_state
  "${xczs_inspection_robot_interfaces_TARGETS__rosidl_typesupport_cpp}"
)

target_compile_features(xczs_ros_global_args_guard PUBLIC cxx_std_17)
ament_target_dependencies(
  xczs_ros_global_args_guard
  gazebo_dev
  rcl
  rclcpp
  std_srvs
)

install(
  TARGETS
    xczs_planar_stabilizer
    xczs_cabinet_state
    xczs_ros_global_args_guard
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(xczs_planar_stabilizer)
ament_export_libraries(xczs_cabinet_state)
ament_export_libraries(xczs_ros_global_args_guard)
ament_export_dependencies(gazebo_dev)

ament_package()
