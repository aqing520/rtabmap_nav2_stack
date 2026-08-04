#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_map_yaml = '/data/maps/site_a/map.yaml'
    load_yaml_path = LaunchConfiguration('load_yaml_path')
    save_yaml_path = LaunchConfiguration('save_yaml_path')
    brush_radius_cells = LaunchConfiguration('brush_radius_cells')
    max_window_size = LaunchConfiguration('max_window_size')
    initial_zoom = LaunchConfiguration('initial_zoom')
    max_zoom = LaunchConfiguration('max_zoom')

    return LaunchDescription([
        DeclareLaunchArgument('load_yaml_path', default_value=default_map_yaml),
        DeclareLaunchArgument(
            'save_yaml_path',
            default_value=default_map_yaml,
            description='SAVE OVERWRITE replaces this YAML and its PGM image.',
        ),
        DeclareLaunchArgument('brush_radius_cells', default_value='5'),
        DeclareLaunchArgument('max_window_size', default_value='1000'),
        DeclareLaunchArgument('initial_zoom', default_value='1.0'),
        DeclareLaunchArgument('max_zoom', default_value='8.0'),
        Node(
            package='map_paint_editor_plugin',
            executable='map_paint_editor.py',
            name='map_paint_editor',
            output='screen',
            parameters=[{
                'load_yaml_path': load_yaml_path,
                'save_yaml_path': save_yaml_path,
                'brush_radius_cells': brush_radius_cells,
                'max_window_size': max_window_size,
                'initial_zoom': initial_zoom,
                'max_zoom': max_zoom,
            }],
        ),
    ])
