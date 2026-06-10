#!/usr/bin/env python3
"""Orbbec Gemini/Astra RGB-D camera starter for RTAB-Map."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    astra_share = FindPackageShare('astra_camera')

    declare_args = [
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('camera_base_frame', default_value='base_link'),
        DeclareLaunchArgument('camera_link_frame', default_value='camera_link'),
        DeclareLaunchArgument('camera_x', default_value='0.10'),
        DeclareLaunchArgument('camera_y', default_value='0.0'),
        DeclareLaunchArgument('camera_z', default_value='0.616'),
        DeclareLaunchArgument('camera_roll', default_value='-1.5707'),
        DeclareLaunchArgument('camera_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_yaw', default_value='-1.5707'),
        DeclareLaunchArgument('publish_camera_tf', default_value='true'),
    ]

    camera_driver = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([astra_share, 'launch', 'gemini.launch.xml'])),
        condition=IfCondition(LaunchConfiguration('start_camera')),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
            'depth_registration': 'true',
            'color_depth_synchronization': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_ir': 'false',
            'enable_point_cloud': 'false',
            'enable_colored_point_cloud': 'false',
            'publish_tf': 'true',
        }.items(),
    )

    camera_mount_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera_link',
        condition=IfCondition(LaunchConfiguration('publish_camera_tf')),
        arguments=[
            '--x', LaunchConfiguration('camera_x'),
            '--y', LaunchConfiguration('camera_y'),
            '--z', LaunchConfiguration('camera_z'),
            '--roll', LaunchConfiguration('camera_roll'),
            '--pitch', LaunchConfiguration('camera_pitch'),
            '--yaw', LaunchConfiguration('camera_yaw'),
            '--frame-id', LaunchConfiguration('camera_base_frame'),
            '--child-frame-id', LaunchConfiguration('camera_link_frame'),
        ],
    )

    ld = LaunchDescription()
    for action in declare_args:
        ld.add_action(action)
    ld.add_action(camera_mount_tf)
    ld.add_action(camera_driver)
    return ld
