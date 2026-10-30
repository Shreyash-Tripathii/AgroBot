cmake_minimum_required(VERSION 3.19)
project(agrobot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

install(DIRECTORY
  config
  maps
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Automatically make scripts executable in source dir (vital for colcon --symlink-install)
file(CHMOD
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/global_localizer.py
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/map_autosaver.py
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/frontier_explorer.py
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

install(PROGRAMS
  scripts/global_localizer.py
  scripts/map_autosaver.py
  scripts/frontier_explorer.py
  DESTINATION lib/${PROJECT_NAME}
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(slam_toolbox REQUIRED)

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
