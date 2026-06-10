cmake_minimum_required(VERSION 3.8)
project(scan_context_relocalization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

add_executable(
  scan_context_relocalization_node
  src/scan_context_relocalization_node.cpp
  src/scan_context/Scancontext.cpp
)

target_include_directories(
  scan_context_relocalization_node
  PUBLIC
  include
  include/scan_context
  /usr/include/eigen3
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(
  scan_context_relocalization_node
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(
  scan_context_relocalization_node
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
  cv_bridge
)

install(
  TARGETS scan_context_relocalization_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
