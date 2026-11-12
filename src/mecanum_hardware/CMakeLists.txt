cmake_minimum_required(VERSION 3.8)
project(mecanum_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# The encoder model shared by the simulation plugin and the future ESP32 plugin (decision 0006).
# No ROS dependencies.
add_library(wheel_encoder STATIC src/wheel_encoder.cpp)
set_target_properties(wheel_encoder PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(wheel_encoder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# The simulation plugin needs Gazebo; it is skipped where gz_ros2_control is not installed (the
# robot's computer).
# gz_ros2_control's exported targets name gz-sim8, which must be found first.
find_package(gz_sim_vendor QUIET)
if(gz_sim_vendor_FOUND)
  find_package(gz-sim QUIET)
  find_package(gz_ros2_control QUIET)
endif()
if(gz_ros2_control_FOUND)
  find_package(hardware_interface REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)

  add_library(mecanum_hardware_sim SHARED src/sim_encoder_system.cpp)
  # gz_hardware_plugins exports no include directory; the system plugin's target has it.
  target_include_directories(mecanum_hardware_sim PRIVATE
    $<TARGET_PROPERTY:gz_ros2_control::gz_ros2_control-system,INTERFACE_INCLUDE_DIRECTORIES>)
  target_link_libraries(mecanum_hardware_sim
    wheel_encoder
    gz_ros2_control::gz_hardware_plugins
    hardware_interface::hardware_interface
    pluginlib::pluginlib
    rclcpp::rclcpp
    ${std_msgs_TARGETS})
  install(TARGETS mecanum_hardware_sim
    LIBRARY DESTINATION lib)
  pluginlib_export_plugin_description_file(gz_ros2_control mecanum_hardware_sim_plugins.xml)
else()
  message(STATUS "gz_ros2_control not found: the simulation plugin is not built")
endif()

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS wheel_encoder EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib)
ament_export_targets(export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_wheel_encoder test/test_wheel_encoder.cpp)
  target_link_libraries(test_wheel_encoder wheel_encoder)
endif()

ament_package()
