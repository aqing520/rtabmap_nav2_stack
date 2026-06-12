cmake_minimum_required(VERSION 3.8)
project(forbidden_area_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成消息和服务
# rosidl_generate_interfaces 会占用 ${PROJECT_NAME} 这个 target 名，
# 因此插件库 target 必须用不同名称（forbidden_area_layer_plugin）
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ForbiddenArea.msg"
  "srv/GetForbiddenAreas.srv"
  "srv/SetForbiddenAreas.srv"
  DEPENDENCIES geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# ── Costmap 插件库 ────────────────────────────────────────────────
set(PLUGIN_LIB ${PROJECT_NAME}_plugin)

add_library(${PLUGIN_LIB} SHARED
  src/forbidden_area_layer.cpp
)

target_include_directories(${PLUGIN_LIB} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PLUGIN_LIB}
  nav2_costmap_2d
  rclcpp
  geometry_msgs
  nav_msgs
  pluginlib
  rosidl_default_runtime
)

target_link_libraries(${PLUGIN_LIB}
  ${cpp_typesupport_target}
)

pluginlib_export_plugin_description_file(nav2_costmap_2d forbidden_area_layer.xml)

# ── 独立调试工具（命令行 get/set 禁行区）─────────────────────────
add_executable(forbidden_area_server
  src/forbidden_area_server.cpp
)

ament_target_dependencies(forbidden_area_server
  rclcpp
  geometry_msgs
  rosidl_default_runtime
)

target_link_libraries(forbidden_area_server
  ${cpp_typesupport_target}
)

# ── 安装 ─────────────────────────────────────────────────────────
install(
  TARGETS ${PLUGIN_LIB} forbidden_area_server
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES forbidden_area_layer.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY scripts launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".claude" EXCLUDE
  PATTERN "*.code-workspace" EXCLUDE
)

install(
  PROGRAMS
    scripts/forbidden_area_visualizer.py
    scripts/forbidden_area_demo_server.py
    scripts/forbidden_area_client.py
    scripts/manage_forbidden_areas.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
