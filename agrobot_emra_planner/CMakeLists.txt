cmake_minimum_required(VERSION 3.8)
project(agrobot_emra_planner)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Threads REQUIRED)

# ---- ROS-free algorithm core (unit-testable, no rclcpp dependency) ----
add_library(emra_core STATIC
  src/grid_map_view.cpp
  src/path_validator.cpp
  src/seed_planner.cpp
  src/bspline.cpp
  src/emra_optimizer.cpp
  src/path_problem.cpp
  src/planning_core.cpp
)
target_include_directories(emra_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(emra_core PUBLIC Threads::Threads)
set_target_properties(emra_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---- Nav2 global planner plugin ----
add_library(${PROJECT_NAME}_plugin SHARED src/emra_planner.cpp)
target_include_directories(${PROJECT_NAME}_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME}_plugin PUBLIC emra_core)
ament_target_dependencies(${PROJECT_NAME}_plugin PUBLIC
  rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d nav2_util pluginlib
  geometry_msgs nav_msgs std_msgs tf2_ros)
pluginlib_export_plugin_description_file(nav2_core emra_planner_plugin.xml)

# ---- BT condition plugin ----
add_library(agrobot_is_path_deviated_condition_bt_node SHARED src/is_path_deviated_condition.cpp)
target_include_directories(agrobot_is_path_deviated_condition_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(agrobot_is_path_deviated_condition_bt_node PUBLIC
  rclcpp nav2_util nav2_behavior_tree behaviortree_cpp geometry_msgs nav_msgs tf2_ros)
target_compile_definitions(agrobot_is_path_deviated_condition_bt_node PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS emra_core ${PROJECT_NAME}_plugin agrobot_is_path_deviated_condition_bt_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY behavior_trees config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/benchmark_planners.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_emra_core test/test_emra_core.cpp)
  target_link_libraries(test_emra_core emra_core)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME}_plugin)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d nav2_util pluginlib
  geometry_msgs nav_msgs std_msgs tf2_ros)
ament_package()
