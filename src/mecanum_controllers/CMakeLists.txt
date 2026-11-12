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
  # The real-drive stack on mecanum_hardware's fake ESP32 (docs/hardware_drive_test.md, T0).
  add_launch_test(test/test_esp32_fake.py TIMEOUT 180 ENV ROS_DOMAIN_ID=92)
endif()

ament_package()
