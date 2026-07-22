#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    load_yaml_path = LaunchConfiguration('load_yaml_path')
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    save_yaml_path = LaunchConfiguration('save_yaml_path')
    brush_radius_cells = LaunchConfiguration('brush_radius_cells')
    max_window_size = LaunchConfiguration('max_window_size')
    initial_zoom = LaunchConfiguration('initial_zoom')
    max_zoom = LaunchConfiguration('max_zoom')
    reissue_goal_on_publish = LaunchConfiguration('reissue_goal_on_publish')
    goal_topic = LaunchConfiguration('goal_topic')

    return LaunchDescription([
        DeclareLaunchArgument('load_yaml_path', default_value='/data/maps/site_a/map.yaml'),
        DeclareLaunchArgument('input_topic', default_value=''),
        DeclareLaunchArgument('output_topic', default_value='/map_edited'),
        DeclareLaunchArgument(
            'save_yaml_path',
            default_value='/data/maps/site_a/map.yaml',
        ),
        DeclareLaunchArgument('brush_radius_cells', default_value='5'),
        DeclareLaunchArgument('max_window_size', default_value='1000'),
        DeclareLaunchArgument('initial_zoom', default_value='1.0'),
        DeclareLaunchArgument('max_zoom', default_value='8.0'),
        DeclareLaunchArgument('reissue_goal_on_publish', default_value='false'),
        DeclareLaunchArgument('goal_topic', default_value='/goal_pose'),
        Node(
            package='map_paint_editor_plugin',
            executable='map_paint_editor.py',
            name='map_paint_editor',
            output='screen',
            parameters=[{
                'load_yaml_path': load_yaml_path,
                'input_topic': input_topic,
                'output_topic': output_topic,
                'save_yaml_path': save_yaml_path,
                'brush_radius_cells': brush_radius_cells,
                'max_window_size': max_window_size,
                'initial_zoom': initial_zoom,
                'max_zoom': max_zoom,
                'reissue_goal_on_publish': reissue_goal_on_publish,
                'goal_topic': goal_topic,
            }],
        ),
    ])
