#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_id = LaunchConfiguration('map_id')
    map_frame_id = LaunchConfiguration('map_frame_id')
    clicked_point_topic = LaunchConfiguration('clicked_point_topic')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map_id', default_value='site_a'),
        DeclareLaunchArgument('map_frame_id', default_value='map'),
        DeclareLaunchArgument('clicked_point_topic', default_value='/clicked_point'),
        Node(
            package='robot_bringup',
            executable='multi_waypoint_route_node.py',
            name='multi_waypoint_route',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'map_id': map_id,
                'map_frame_id': map_frame_id,
                'clicked_point_topic': clicked_point_topic,
            }],
        ),
    ])
