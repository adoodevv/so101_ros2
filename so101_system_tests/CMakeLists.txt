cmake_minimum_required(VERSION 3.8)
project(so101_system_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # End-to-end tests. The same suite runs against fake hardware and against
  # the real so101_hardware plugin driving simulated servos. Each run gets its
  # own DDS domain, localhost only, so it neither sees nor disturbs other
  # ROS 2 graphs on the machine.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(
    test/test_moveit_system.launch.py
    TARGET test_moveit_system_fake
    TIMEOUT 300
    ENV SO101_TEST_BACKEND=fake ROS_DOMAIN_ID=73 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  )
  add_launch_test(
    test/test_moveit_system.launch.py
    TARGET test_moveit_system_servo_sim
    TIMEOUT 300
    ENV SO101_TEST_BACKEND=servo_sim ROS_DOMAIN_ID=74 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  )
  # Leader-arm teleoperation, both arms on simulated servos.
  add_launch_test(
    test/test_teleop.launch.py
    TARGET test_teleop
    TIMEOUT 180
    ENV ROS_DOMAIN_ID=75 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  )
endif()

ament_package()
