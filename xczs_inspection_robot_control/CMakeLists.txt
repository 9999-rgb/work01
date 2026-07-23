cmake_minimum_required(VERSION 3.8)
project(xczs_inspection_robot_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(keyboard_teleop src/keyboard_teleop.cpp)
target_compile_features(keyboard_teleop PUBLIC cxx_std_17)
ament_target_dependencies(
  keyboard_teleop
  geometry_msgs
  rclcpp
  sensor_msgs
  trajectory_msgs
)

install(TARGETS keyboard_teleop DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

ament_package()
