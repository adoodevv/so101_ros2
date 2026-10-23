cmake_minimum_required(VERSION 3.8)
project(so101_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Protocol, serial port, calibration and the teleop filter: no ROS
# dependencies, shared by the plugin, the tools and the unit tests.
add_library(so101_feetech STATIC
  src/calibration.cpp
  src/feetech_bus.cpp
  src/feetech_protocol.cpp
  src/serial_port.cpp
  src/teleop_filter.cpp
)
set_target_properties(so101_feetech PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(so101_feetech PUBLIC cxx_std_17)
target_include_directories(so101_feetech PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(so101_feetech PUBLIC yaml-cpp)

add_library(so101_hardware SHARED src/so101_system.cpp)
target_link_libraries(so101_hardware PUBLIC
  so101_feetech
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${lifecycle_msgs_TARGETS}
)
pluginlib_export_plugin_description_file(hardware_interface so101_hardware.xml)

add_executable(so101_hardware_check src/so101_hardware_check.cpp)
target_link_libraries(so101_hardware_check so101_feetech)

add_executable(so101_leader src/so101_leader_node.cpp)
target_link_libraries(so101_leader
  so101_feetech
  rclcpp::rclcpp
  ${controller_manager_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS so101_hardware so101_feetech
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS so101_hardware_check so101_leader DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/so101_servo_sim DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_feetech_protocol test/test_feetech_protocol.cpp)
  target_link_libraries(test_feetech_protocol so101_feetech)
  ament_add_gtest(test_feetech_bus test/test_feetech_bus.cpp)
  target_link_libraries(test_feetech_bus so101_feetech)
  ament_add_gtest(test_calibration test/test_calibration.cpp
    ENV TEST_DATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}/config)
  target_link_libraries(test_calibration so101_feetech)
  ament_add_gtest(test_teleop_filter test/test_teleop_filter.cpp)
  target_link_libraries(test_teleop_filter so101_feetech)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle yaml-cpp)
ament_package()
