cmake_minimum_required(VERSION 3.8)
project(xczs_inspection_robot_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    move_group_launch_policy_test
    test/test_move_group_launch_policy.py
  )
endif()

ament_package()
