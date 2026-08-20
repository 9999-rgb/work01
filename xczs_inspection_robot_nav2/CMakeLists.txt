cmake_minimum_required(VERSION 3.8)
project(xczs_inspection_robot_nav2)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    navigation_launch_policy_test
    test/test_navigation_launch_policy.py
  )
endif()

ament_package()
