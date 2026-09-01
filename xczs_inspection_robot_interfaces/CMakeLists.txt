cmake_minimum_required(VERSION 3.8)
project(xczs_inspection_robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/OperateCabinetControl.action"
  "action/PressCabinetButton.action"
  "msg/CabinetControl.msg"
  "msg/CabinetControlCatalog.msg"
  "msg/CabinetControlState.msg"
  "srv/ManageOperationLease.srv"
  "srv/SetCabinetBimanualGrasp.srv"
  "srv/SetCabinetGrasp.srv"
  "srv/SetCabinetUnlock.srv"
  "srv/SwitchToolset.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
