#!/usr/bin/env python3
"""
全局重定位导航启动入口。

在 bringup.launch.py 基础上增加 hdl_global_localization_node，
供项目根目录 robot.sh rel 使用。

Nav2 固定 autostart=false。robot.sh rel 单次发布 /initialpose，发布完成后
直接手动激活 Nav2，不再等待 localization_pose 或 TF 二次确认。

Usage（通常由 robot.sh rel 调用）:
  ros2 launch robot_bringup global_localization_bringup.launch.py
  ros2 launch robot_bringup global_localization_bringup.launch.py enable_rviz:=true
"""
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _workspace_root(package_name: str) -> Path:
    prefix = Path(get_package_prefix(package_name))
    if prefix.name == 'install':
        return prefix.parent
    if prefix.parent.name == 'install':
        return prefix.parent.parent
    return Path.cwd()


def generate_launch_description() -> LaunchDescription:
    workspace_root = _workspace_root('robot_bringup')

    database_path = LaunchConfiguration('database_path')
    effective_database_path = PathJoinSubstitution([
        str(workspace_root),
        database_path,
    ])
    enable_rviz   = LaunchConfiguration('enable_rviz')
    mode          = LaunchConfiguration('mode')
    nav2_controller = LaunchConfiguration('nav2_controller')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    enable_collision_monitor = LaunchConfiguration('enable_collision_monitor')
    use_edited_map = LaunchConfiguration('use_edited_map')

    robot_bringup_share = FindPackageShare('robot_bringup')

    declare_args = [
        DeclareLaunchArgument(
            'database_path',
            default_value='db/rtabmap.db',
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='navigation',
            description='navigation | localization',
        ),
        DeclareLaunchArgument(
            'enable_rviz',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'nav2_controller',
            default_value='cuda_mppi',
            description='Nav2 local controller: mppi | cuda_mppi',
            choices=['mppi', 'cuda_mppi'],
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value='',
            description='Optional Nav2 params file. Overrides nav2_controller when set.',
        ),
        DeclareLaunchArgument(
            'enable_collision_monitor',
            default_value='true',
            description='Filter Nav2 /cmd_vel through collision_monitor',
        ),
        DeclareLaunchArgument(
            'use_edited_map',
            default_value='false',
            description='Use an offline edited Nav2 map instead of RTAB-Map /map.',
        ),
    ]

    # ── 完整导航栈（Nav2 暂不自动激活）──
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([robot_bringup_share, 'launch', 'bringup.launch.py'])
        ),
        launch_arguments={
            'mode':          mode,
            'database_path': effective_database_path,
            'enable_rviz':   enable_rviz,
            'autostart':     'false',
            'nav2_controller': nav2_controller,
            'nav2_params_file': nav2_params_file,
            'enable_collision_monitor': enable_collision_monitor,
            'use_edited_map': use_edited_map,
            'sensor_profile': 'lidar_only',
            'start_livox': 'true',
            'start_camera': 'false',
            'enable_gps': 'false',
            'start_multi_waypoint_routes': 'false',
        }.items(),
    )

    # ── hdl 全局定位服务节点 ──
    hdl_node = Node(
        package='hdl_global_localization',
        executable='hdl_global_localization_node',
        name='hdl_global_localization_node',
        output='screen',
    )

    ld = LaunchDescription()
    for arg in declare_args:
        ld.add_action(arg)
    ld.add_action(bringup)
    ld.add_action(hdl_node)
    return ld
