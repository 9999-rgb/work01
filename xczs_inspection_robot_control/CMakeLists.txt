cmake_minimum_required(VERSION 3.8)
project(xczs_inspection_robot_control)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/OperateCabinetControl.action"
  "action/PressCabinetButton.action"
  "msg/CabinetControl.msg"
  "msg/CabinetControlCatalog.msg"
  "msg/CabinetControlState.msg"
  "srv/SetCabinetGrasp.srv"
  DEPENDENCIES std_msgs
)

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
add_executable(cabinet_pose_authority src/cabinet_pose_authority.cpp)
add_executable(moveit_planner src/moveit_planner.cpp)
add_executable(cabinet_button_operator src/cabinet_button_operator.cpp)
add_executable(base_command_router src/base_command_router.cpp)
add_library(
  xczs_planar_stabilizer
  SHARED
  src/gazebo/planar_stabilizer_plugin.cpp
)
add_library(
  xczs_cabinet_state
  SHARED
  src/gazebo/cabinet_state_plugin.cpp
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
  rclcpp
  shape_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
rosidl_get_typesupport_target(
  cabinet_planning_scene_typesupport
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
target_link_libraries(
  cabinet_planning_scene
  "${cabinet_planning_scene_typesupport}"
)

target_compile_features(cabinet_pose_authority PUBLIC cxx_std_17)
ament_target_dependencies(
  cabinet_pose_authority
  geometry_msgs
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

target_compile_features(moveit_planner PUBLIC cxx_std_17)
ament_target_dependencies(
  moveit_planner
  geometry_msgs
  moveit_ros_planning_interface
  rclcpp
)

target_compile_features(cabinet_button_operator PUBLIC cxx_std_17)
ament_target_dependencies(
  cabinet_button_operator
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  nav2_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
rosidl_get_typesupport_target(
  cabinet_button_operator_typesupport
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
target_link_libraries(
  cabinet_button_operator
  "${cabinet_button_operator_typesupport}"
)

target_compile_features(base_command_router PUBLIC cxx_std_17)
ament_target_dependencies(
  base_command_router
  geometry_msgs
  rclcpp
  std_msgs
  std_srvs
)

target_compile_features(xczs_planar_stabilizer PUBLIC cxx_std_17)
ament_target_dependencies(
  xczs_planar_stabilizer
  gazebo_dev
  gazebo_ros
  rclcpp
  std_msgs
)

target_compile_features(xczs_cabinet_state PUBLIC cxx_std_17)
ament_target_dependencies(
  xczs_cabinet_state
  gazebo_dev
  gazebo_ros
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)
rosidl_get_typesupport_target(
  cabinet_state_plugin_typesupport
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
target_link_libraries(
  xczs_cabinet_state
  "${cabinet_state_plugin_typesupport}"
)

install(
  TARGETS
    keyboard_teleop
    inspection_robot_gui
    legacy_trajectory_router
    cabinet_planning_scene
    cabinet_pose_authority
    cabinet_button_operator
    moveit_planner
    base_command_router
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    xczs_planar_stabilizer
    xczs_cabinet_state
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(xczs_planar_stabilizer)
ament_export_libraries(xczs_cabinet_state)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
