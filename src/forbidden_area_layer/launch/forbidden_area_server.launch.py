#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='forbidden_area_layer',
            executable='forbidden_area_server',
            name='forbidden_area_server',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        )
    ])
