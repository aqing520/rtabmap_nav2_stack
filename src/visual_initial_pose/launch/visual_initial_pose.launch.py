#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _workspace_root(package_name: str) -> Path:
    prefix = Path(get_package_prefix(package_name))
    if prefix.name == 'install':
        return prefix.parent
    if prefix.parent.name == 'install':
        return prefix.parent.parent
    return Path.cwd()


def generate_launch_description():
    workspace_root = _workspace_root('visual_initial_pose')
    database_path = LaunchConfiguration('database_path')
    effective_database_path = PathJoinSubstitution([
        str(workspace_root),
        database_path,
    ])
    package_share = FindPackageShare('visual_initial_pose')
    default_config = PathJoinSubstitution(
        [package_share, 'config', 'visual_initial_pose.yaml'])

    args = [
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument(
            'database_path',
            default_value='rtabmap_orbbec.db',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('allow_last_pose_fallback', default_value='false'),
        DeclareLaunchArgument('activate_nav2_on_success', default_value='false'),
        DeclareLaunchArgument('min_hypothesis', default_value='0.11'),
        DeclareLaunchArgument('min_visual_inliers', default_value='15'),
        DeclareLaunchArgument('min_best_second_ratio', default_value='1.0'),
        DeclareLaunchArgument('localization_timeout_sec', default_value='30.0'),
    ]

    node = Node(
        package='visual_initial_pose',
        executable='visual_initial_pose_node',
        name='visual_initial_pose',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'database_path': effective_database_path,
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
                'allow_last_pose_fallback': ParameterValue(
                    LaunchConfiguration('allow_last_pose_fallback'), value_type=bool),
                'activate_nav2_on_success': ParameterValue(
                    LaunchConfiguration('activate_nav2_on_success'), value_type=bool),
                'min_hypothesis': ParameterValue(
                    LaunchConfiguration('min_hypothesis'), value_type=float),
                'min_visual_inliers': ParameterValue(
                    LaunchConfiguration('min_visual_inliers'), value_type=int),
                'min_best_second_ratio': ParameterValue(
                    LaunchConfiguration('min_best_second_ratio'), value_type=float),
                'localization_timeout_sec': ParameterValue(
                    LaunchConfiguration('localization_timeout_sec'), value_type=float),
            },
        ],
    )

    return LaunchDescription(args + [node])
