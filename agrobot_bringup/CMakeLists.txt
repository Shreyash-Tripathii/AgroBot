cmake_minimum_required(VERSION 3.19)
project(agrobot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ros2_control REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Automatically make scripts executable in source dir (vital for colcon --symlink-install)
file(CHMOD
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/yaw_heading_controller.py
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/gain_scheduler_node.py
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/bno085_driver.py
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/click_goal.py
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

# Install Python nodes
install(PROGRAMS
  scripts/yaw_heading_controller.py
  scripts/gain_scheduler_node.py
  scripts/bno085_driver.py
  scripts/click_goal.py
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
