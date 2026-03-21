#!/usr/bin/env bash
export LIDAR_CMD_POINTCLOUD2="ros2 launch livox_ros_driver2 rviz_MID360_launch.py"
source /opt/ros/humble/setup.bash
source /home/wheeltec/wzy/install/setup.bash
exec bash /home/wheeltec/wzy/scripts/check_mapping.sh real-up "$@"
