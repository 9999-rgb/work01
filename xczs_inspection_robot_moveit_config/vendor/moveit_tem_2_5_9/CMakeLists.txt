# Keep the ABI of the installed Humble library; never replace the ROS install.
find_package(moveit_ros_planning REQUIRED)
if(NOT moveit_ros_planning_VERSION VERSION_EQUAL "2.5.9")
  message(FATAL_ERROR "XCZS MoveIt shutdown fix requires audited MoveIt 2.5.9; update vendor source and ABI audit for this version")
endif()
get_filename_component(_tem_ros_prefix "${moveit_ros_planning_DIR}/../../.." ABSOLUTE)
set(_tem_header "${_tem_ros_prefix}/include/moveit/trajectory_execution_manager/trajectory_execution_manager.h")
file(SHA256 "${_tem_header}" _tem_header_sha)
if(NOT _tem_header_sha STREQUAL "4c6216ad81112855eb467ef5d83e3210f43c8317d382e922f9045dabd19d7d1c")
  message(FATAL_ERROR "Installed MoveIt TEM header differs from audited 2.5.9 ABI")
endif()
find_package(moveit_core REQUIRED)
find_package(moveit_ros_occupancy_map_monitor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
add_library(xczs_moveit_tem_fix SHARED trajectory_execution_manager.cpp)
target_compile_features(xczs_moveit_tem_fix PRIVATE cxx_std_17)
set_target_properties(xczs_moveit_tem_fix PROPERTIES
  OUTPUT_NAME moveit_trajectory_execution_manager VERSION 2.5.9)
ament_target_dependencies(xczs_moveit_tem_fix
  moveit_core moveit_ros_occupancy_map_monitor rclcpp pluginlib
  std_msgs sensor_msgs moveit_msgs tf2_eigen)
target_link_libraries(xczs_moveit_tem_fix moveit_ros_planning::moveit_planning_scene_monitor)
install(TARGETS xczs_moveit_tem_fix LIBRARY DESTINATION lib/xczs_moveit_tem_fix)
find_package(moveit_ros_move_group REQUIRED)
if(NOT moveit_ros_move_group_VERSION VERSION_EQUAL "2.5.9")
  message(FATAL_ERROR "XCZS capability shutdown fix requires MoveIt 2.5.9")
endif()
add_executable(xczs_move_group move_group.cpp)
target_compile_features(xczs_move_group PRIVATE cxx_std_17)
set_target_properties(xczs_move_group PROPERTIES OUTPUT_NAME move_group)
ament_target_dependencies(xczs_move_group rclcpp pluginlib moveit_ros_planning moveit_ros_move_group)
target_link_libraries(xczs_move_group
  moveit_ros_planning::moveit_cpp
  moveit_ros_move_group::moveit_move_group_capabilities_base)
install(TARGETS xczs_move_group DESTINATION lib/${PROJECT_NAME})
install(FILES README.md provenance.json shutdown.patch move_group_shutdown.patch
  DESTINATION share/${PROJECT_NAME}/moveit_tem_fix)
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  foreach(_toolset A B)
    add_test(NAME move_group_capability_shutdown_${_toolset}
      COMMAND ${CMAKE_COMMAND} -E env
      "LD_LIBRARY_PATH=$<TARGET_FILE_DIR:xczs_moveit_tem_fix>:$ENV{LD_LIBRARY_PATH}"
      ROS_DOMAIN_ID=142 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../../test/check_move_group_shutdown.py
      $<TARGET_FILE:xczs_move_group> ${_toolset})
    set_tests_properties(move_group_capability_shutdown_${_toolset} PROPERTIES
      TIMEOUT 100 RUN_SERIAL TRUE PASS_REGULAR_EXPRESSION "Capability shutdown ${_toolset} passed")
  endforeach()
  add_test(NAME move_group_startup_shutdown
    COMMAND ${CMAKE_COMMAND} -E env
    "LD_LIBRARY_PATH=$<TARGET_FILE_DIR:xczs_moveit_tem_fix>:$ENV{LD_LIBRARY_PATH}"
    ROS_DOMAIN_ID=142 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../../test/check_move_group_shutdown.py
    $<TARGET_FILE:xczs_move_group> A startup)
  set_tests_properties(move_group_startup_shutdown PROPERTIES
    TIMEOUT 100 RUN_SERIAL TRUE PASS_REGULAR_EXPRESSION "Capability shutdown A passed")
  find_package(rclcpp_action REQUIRED)
  find_package(control_msgs REQUIRED)
  add_executable(test_moveit_tem_shutdown test_shutdown.cpp)
  target_compile_features(test_moveit_tem_shutdown PRIVATE cxx_std_17)
  ament_target_dependencies(test_moveit_tem_shutdown rclcpp rclcpp_action control_msgs)
  target_link_libraries(test_moveit_tem_shutdown
    moveit_ros_planning::moveit_trajectory_execution_manager)
  foreach(_mode shutdown live)
    add_test(NAME moveit_tem_shutdown_${_mode}
      COMMAND ${CMAKE_COMMAND} -E env
      "LD_LIBRARY_PATH=$<TARGET_FILE_DIR:xczs_moveit_tem_fix>:$ENV{LD_LIBRARY_PATH}"
      ROS_DOMAIN_ID=142 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      $<TARGET_FILE:test_moveit_tem_shutdown> ${_mode})
    set_tests_properties(moveit_tem_shutdown_${_mode} PROPERTIES
      TIMEOUT 30 RUN_SERIAL TRUE
      PASS_REGULAR_EXPRESSION "Controller manager destroyed cleanly")
  endforeach()
endif()
