cmake_minimum_required(VERSION 3.8)
project(mecanum_lidar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Vendored Unitree SDK: headers plus a prebuilt static library per architecture.
# See third_party/unilidar_sdk2/README.md.
set(UNILIDAR_SDK2_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/unilidar_sdk2)
set(UNILIDAR_SDK2_LIB ${UNILIDAR_SDK2_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}/libunilidar_sdk2.a)
if(NOT EXISTS ${UNILIDAR_SDK2_LIB})
  message(FATAL_ERROR
    "unilidar_sdk2 has no prebuilt library for ${CMAKE_SYSTEM_PROCESSOR} "
    "(upstream ships x86_64 and aarch64 only).")
endif()
find_package(Threads REQUIRED)
add_library(unilidar_sdk2 STATIC IMPORTED)
set_target_properties(unilidar_sdk2 PROPERTIES IMPORTED_LOCATION ${UNILIDAR_SDK2_LIB})
# SYSTEM: the vendor headers are not held to this package's warning flags.
target_include_directories(unilidar_sdk2 SYSTEM INTERFACE ${UNILIDAR_SDK2_DIR}/include)
target_link_libraries(unilidar_sdk2 INTERFACE Threads::Threads)

# SDK message -> ROS message conversions, separate so they can be unit tested.
add_library(l2_conversions STATIC src/l2_conversions.cpp)
target_include_directories(l2_conversions PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(l2_conversions PUBLIC
  unilidar_sdk2
  ${builtin_interfaces_TARGETS}
  ${sensor_msgs_TARGETS})

add_executable(l2_driver src/l2_driver_node.cpp)
target_link_libraries(l2_driver
  l2_conversions
  unilidar_sdk2
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS})

install(TARGETS l2_driver
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # The vendored SDK keeps its upstream style and license header.
  file(GLOB_RECURSE _vendored_files ${UNILIDAR_SDK2_DIR}/*)
  set(AMENT_LINT_AUTO_FILE_EXCLUDE ${_vendored_files})
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_l2_conversions test/test_l2_conversions.cpp)
  target_link_libraries(test_l2_conversions l2_conversions ${sensor_msgs_TARGETS})
endif()

ament_package()
