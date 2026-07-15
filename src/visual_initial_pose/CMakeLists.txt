cmake_minimum_required(VERSION 3.8)
project(visual_initial_pose)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rtabmap_conversions REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(RTABMap 0.23.4 COMPONENTS core REQUIRED)

add_executable(visual_initial_pose_node src/visual_initial_pose_node.cpp)
target_compile_features(visual_initial_pose_node PUBLIC cxx_std_17)
ament_target_dependencies(
  visual_initial_pose_node
  geometry_msgs
  nav2_msgs
  rclcpp
  rtabmap_conversions
  rtabmap_msgs
  std_msgs
  tf2_ros
)
target_link_libraries(visual_initial_pose_node rtabmap::core)

install(TARGETS visual_initial_pose_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
