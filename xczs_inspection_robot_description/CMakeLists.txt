cmake_minimum_required(VERSION 3.8)

project(xczs_inspection_robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes scripts urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
