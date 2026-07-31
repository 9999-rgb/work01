cmake_minimum_required(VERSION 3.8)
project(xczs_inspection_robot_control)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(keyboard_teleop src/keyboard_teleop.cpp)
add_executable(inspection_robot_gui src/inspection_robot_gui.cpp)
add_executable(
  legacy_trajectory_router
  src/legacy_trajectory_router.cpp
)
add_executable(
  cabinet_planning_scene
  src/cabinet_planning_scene.cpp
)
add_executable(moveit_planner src/moveit_planner.cpp)
add_library(
  xczs_planar_stabilizer
  SHARED
  src/gazebo/planar_stabilizer_plugin.cpp
)

foreach(target keyboard_teleop inspection_robot_gui)
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_include_directories(
    ${target}
    PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include>"
  )
  ament_target_dependencies(
    ${target}
    geometry_msgs
    rclcpp
    sensor_msgs
    trajectory_msgs
  )
endforeach()

target_link_libraries(inspection_robot_gui Qt5::Widgets)

target_compile_features(legacy_trajectory_router PUBLIC cxx_std_17)
ament_target_dependencies(
  legacy_trajectory_router
  action_msgs
  rclcpp
  trajectory_msgs
)

target_compile_features(cabinet_planning_scene PUBLIC cxx_std_17)
ament_target_dependencies(
  cabinet_planning_scene
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  shape_msgs
  tf2
)

target_compile_features(moveit_planner PUBLIC cxx_std_17)
ament_target_dependencies(
  moveit_planner
  geometry_msgs
  moveit_ros_planning_interface
  rclcpp
)

target_compile_features(xczs_planar_stabilizer PUBLIC cxx_std_17)
ament_target_dependencies(xczs_planar_stabilizer gazebo_dev)

install(
  TARGETS
    keyboard_teleop
    inspection_robot_gui
    legacy_trajectory_router
    cabinet_planning_scene
    moveit_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS xczs_planar_stabilizer
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(xczs_planar_stabilizer)
ament_export_dependencies(gazebo_dev)
ament_package()
