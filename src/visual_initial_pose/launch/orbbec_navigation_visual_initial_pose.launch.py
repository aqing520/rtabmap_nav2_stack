#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_share = FindPackageShare('robot_bringup')
    visual_share = FindPackageShare('visual_initial_pose')

    database_path = LaunchConfiguration('database_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    allow_fallback = LaunchConfiguration('allow_last_pose_fallback')
    use_edited_map = LaunchConfiguration('use_edited_map')
    edited_map_yaml = LaunchConfiguration('edited_map_yaml')

    args = [
        DeclareLaunchArgument('database_path', default_value='./rtabmap_orbbec.db'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('sensor_profile', default_value='lidar_rgbd'),
        DeclareLaunchArgument('start_livox', default_value='true'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('enable_rtabmap_viz', default_value='false'),
        DeclareLaunchArgument(
            'use_edited_map',
            default_value='false',
            description='Load an offline-edited static map for Nav2.',
        ),
        DeclareLaunchArgument(
            'edited_map_yaml',
            default_value='/data/maps/site_a/map.yaml',
            description='Offline-edited map YAML loaded when use_edited_map is true.',
        ),
        DeclareLaunchArgument('allow_last_pose_fallback', default_value='false'),
        DeclareLaunchArgument('min_hypothesis', default_value='0.08'),
        DeclareLaunchArgument('min_visual_inliers', default_value='10'),
        DeclareLaunchArgument('min_best_second_ratio', default_value='0.8'),
        DeclareLaunchArgument('localization_timeout_sec', default_value='30.0'),
    ]

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [robot_share, 'launch', 'bringup_orbbec.launch.py'])),
        launch_arguments={
            'mode': 'navigation',
            'sensor_profile': LaunchConfiguration('sensor_profile'),
            'start_livox': LaunchConfiguration('start_livox'),
            'start_camera': LaunchConfiguration('start_camera'),
            'use_sim_time': use_sim_time,
            'database_path': database_path,
            'enable_rviz': LaunchConfiguration('enable_rviz'),
            'enable_rtabmap_viz': LaunchConfiguration('enable_rtabmap_viz'),
            'use_edited_map': use_edited_map,
            'edited_map_yaml': edited_map_yaml,
            'autostart': 'false',
            'enable_startup_localization_guard': 'false',
        }.items(),
    )

    visual_node = Node(
        package='visual_initial_pose',
        executable='visual_initial_pose_node',
        name='visual_initial_pose',
        output='screen',
        parameters=[
            PathJoinSubstitution(
                [visual_share, 'config', 'visual_initial_pose.yaml']),
            {
                'database_path': database_path,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'allow_last_pose_fallback': ParameterValue(
                    allow_fallback, value_type=bool),
                'activate_nav2_on_success': True,
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

    return LaunchDescription(args + [bringup, visual_node])
