cmake_minimum_required(VERSION 3.8)
project(pluto)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# Install Python scripts
install(
  PROGRAMS
    scripts/comm_test.py
    scripts/drive_client_node.py
    scripts/test_sequence_node.py
    scripts/navigation_node.py
    scripts/obstacle_detection_test.py
    scripts/mission_node.py
    scripts/test_camera.py
    scripts/test_pid.py
  DESTINATION lib/${PROJECT_NAME}
)
# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/vision_node.py
  DESTINATION lib/${PROJECT_NAME}
)

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
endif()

ament_package()
