cmake_minimum_required(VERSION 3.8)
project(xczs_inspection_robot_nav2)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
