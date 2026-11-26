cmake_minimum_required(VERSION 3.8)
project(agrobot_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config worlds models
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/generate_field_world.py
  DESTINATION lib/${PROJECT_NAME})

# GZ_SIM_RESOURCE_PATH so `gz sim` finds model://cabbage etc. after sourcing the workspace
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/agrobot_gazebo.dsv")

ament_package()
