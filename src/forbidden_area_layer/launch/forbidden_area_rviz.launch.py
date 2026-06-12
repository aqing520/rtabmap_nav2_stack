#!/usr/bin/env python3
"""
禁行区 RViz 可视化 launch 文件
启动：
  1. forbidden_area_demo_server  — 独立服务端，预加载演示禁行区
  2. forbidden_area_visualizer   — 可视化节点，发布 MarkerArray
  3. rviz2                       — 打开 RViz

用法：
  source install/setup.bash
  ros2 launch forbidden_area_layer forbidden_area_rviz.launch.py

运行时修改禁行区（另开终端）：
  source install/setup.bash
  python3 forbidden_area_layer/scripts/manage_forbidden_areas.py        # 设置默认区域
  python3 forbidden_area_layer/scripts/manage_forbidden_areas.py clear  # 清空
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("forbidden_area_layer")
    rviz_config = os.path.join(pkg_dir, "config", "forbidden_area.rviz")
    rviz_args = ["-d", rviz_config] if os.path.exists(rviz_config) else []

    return LaunchDescription([
        # 1. 独立演示服务端
        Node(
            package="forbidden_area_layer",
            executable="forbidden_area_demo_server.py",
            name="forbidden_area_demo_server",
            output="screen",
        ),

        # 2. 可视化节点
        Node(
            package="forbidden_area_layer",
            executable="forbidden_area_visualizer.py",
            name="forbidden_area_visualizer",
            output="screen",
        ),

        # 3. RViz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=rviz_args,
            output="screen",
        ),
    ])
