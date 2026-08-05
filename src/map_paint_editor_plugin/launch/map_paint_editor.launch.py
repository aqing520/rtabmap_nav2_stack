#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _workspace_root(package_name: str) -> Path:
    prefix = Path(get_package_prefix(package_name))
    if prefix.name == 'install':
        return prefix.parent
    if prefix.parent.name == 'install':
        return prefix.parent.parent
    return Path.cwd()


def generate_launch_description():
    workspace_root = _workspace_root('map_paint_editor_plugin')
    load_yaml_path = LaunchConfiguration('load_yaml_path')
    save_yaml_path = LaunchConfiguration('save_yaml_path')
    effective_load_yaml_path = PathJoinSubstitution([
        str(workspace_root),
        load_yaml_path,
    ])
    effective_save_yaml_path = PathJoinSubstitution([
        str(workspace_root),
        save_yaml_path,
    ])
    brush_radius_cells = LaunchConfiguration('brush_radius_cells')
    max_window_size = LaunchConfiguration('max_window_size')
    initial_zoom = LaunchConfiguration('initial_zoom')
    max_zoom = LaunchConfiguration('max_zoom')

    return LaunchDescription([
        DeclareLaunchArgument(
            'load_yaml_path',
            default_value='pgm_map/map.yaml',
        ),
        DeclareLaunchArgument(
            'save_yaml_path',
            default_value='pgm_map/map.yaml',
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
                'load_yaml_path': effective_load_yaml_path,
                'save_yaml_path': effective_save_yaml_path,
                'brush_radius_cells': brush_radius_cells,
                'max_window_size': max_window_size,
                'initial_zoom': initial_zoom,
                'max_zoom': max_zoom,
            }],
        ),
    ])
