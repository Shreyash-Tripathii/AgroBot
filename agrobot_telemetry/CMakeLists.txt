cmake_minimum_required(VERSION 3.8)
project(agrobot_telemetry)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
# layouts + extension sources (not the npm build products)
install(DIRECTORY foxglove/layouts DESTINATION share/${PROJECT_NAME}/foxglove)
install(DIRECTORY foxglove/extension/src DESTINATION share/${PROJECT_NAME}/foxglove/extension)
install(FILES foxglove/extension/package.json foxglove/extension/tsconfig.json
  foxglove/extension/README.md foxglove/extension/CHANGELOG.md foxglove/extension/.gitignore
  DESTINATION share/${PROJECT_NAME}/foxglove/extension)

install(PROGRAMS scripts/telemetry_monitor.py scripts/generate_layouts.py
  scripts/arm_commander.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
