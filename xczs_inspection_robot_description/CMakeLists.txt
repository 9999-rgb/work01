cmake_minimum_required(VERSION 3.8)

project(xczs_inspection_robot_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

xacro_add_files(
  urdf/control_cabinet.urdf.xacro
  OUTPUT control_cabinet.urdf
  TARGET generate_control_cabinet_urdf
  INSTALL
  DESTINATION urdf/generated
)

install(
  DIRECTORY config meshes urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
