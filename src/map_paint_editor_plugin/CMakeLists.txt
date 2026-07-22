cmake_minimum_required(VERSION 3.5)
project(map_paint_editor_plugin)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/map_paint_editor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY scripts/map_editor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
