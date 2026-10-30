cmake_minimum_required(VERSION 3.8)
project(mecanum_controllers)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(launch_testing_ament_cmake REQUIRED)
  # Own DDS domain, so a controller manager running elsewhere on the machine cannot answer.
  add_launch_test(test/test_mock_kinematics.py TIMEOUT 120 ENV ROS_DOMAIN_ID=91)
endif()

ament_package()
