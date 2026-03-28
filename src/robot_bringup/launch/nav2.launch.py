#!/usr/bin/env python3
"""
Unified navigation bringup (replaces bringup.launch.py + rtabmap_bridge.launch.py).

Starts:
  1. Livox MID360 lidar driver
  2. FAST-LIO odometry -> /Odometry (and odom->base_footprint TF)
  3. BT-468 RTK GNSS driver       (optional, enable_gps)
  4. navsat_transform_node          (optional, enable_gps)
  5. RTAB-Map (localization mode, reads existing database)
  6. Nav2 navigation stack
  7. Collision monitor
  8. Static TFs

Usage:
  # Normal navigation (no GPS):
  ros2 launch robot_bringup nav2.launch.py

  # Navigation with RTK:
  ros2 launch robot_bringup nav2.launch.py enable_gps:=true

  # With custom map database:
  ros2 launch robot_bringup nav2.launch.py database_path:=/data/maps/my_site/rtabmap.db
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_livox = LaunchConfiguration('start_livox')
    publish_base_link_tf = LaunchConfiguration('publish_base_link_tf')
    enable_gps = LaunchConfiguration('enable_gps')
    enable_rviz = LaunchConfiguration('enable_rviz')
    autostart = LaunchConfiguration('autostart')
    database_path = LaunchConfiguration('database_path')
    sensor_profile = LaunchConfiguration('sensor_profile')
    gps_fix_topic = LaunchConfiguration('gps_fix_topic')

    robot_bringup_share = FindPackageShare('robot_bringup')
    nav2_bringup_share = FindPackageShare('nav2_bringup')
    livox_share = FindPackageShare('livox_ros_driver2')
    fast_lio_share = FindPackageShare('fast_lio')
    rtabmap_launch_share = FindPackageShare('rtabmap_launch')

    # ── Declare arguments ──
    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_livox', default_value='true',
                              description='Start Livox MID360 driver'),
        DeclareLaunchArgument('publish_base_link_tf', default_value='true',
                              description='Publish static TF base_footprint -> base_link'),
        DeclareLaunchArgument('enable_gps', default_value='false',
                              description='Enable RTK GPS (navsat_transform + RTAB-Map GPS constraint)'),
        DeclareLaunchArgument('enable_rviz', default_value='false',
                              description='Launch RViz'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Auto-start Nav2 lifecycle nodes'),
        DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap.db',
                              description='Path to RTAB-Map database for localization'),
        DeclareLaunchArgument('sensor_profile', default_value='lidar_only',
                              description='lidar_only | lidar_rgbd | lidar_stereo | lidar_mono'),
        DeclareLaunchArgument('nav2_params_file',
                              default_value=PathJoinSubstitution([robot_bringup_share, 'config', 'nav2_common.yaml']),
                              description='Nav2 parameters file'),
        DeclareLaunchArgument('gps_fix_topic', default_value='/fix',
                              description='NavSatFix topic from RTK driver'),
        # RTAB-Map frame configuration
        DeclareLaunchArgument('frame_id', default_value='base_footprint',
                              description='Robot base frame'),
        DeclareLaunchArgument('map_frame_id', default_value='map',
                              description='Map frame'),
        DeclareLaunchArgument('odom_topic', default_value='/Odometry',
                              description='Odometry topic from FAST-LIO'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu',
                              description='IMU topic'),
        DeclareLaunchArgument('scan_cloud_topic', default_value='/cloud_registered_body',
                              description='Point cloud topic for RTAB-Map and costmaps'),
        # Camera topics (for lidar_rgbd / lidar_stereo / lidar_mono profiles)
        DeclareLaunchArgument('rgb_topic', default_value='/sensors/camera/rgb/image_rect'),
        DeclareLaunchArgument('depth_topic', default_value='/sensors/camera/depth/image_rect'),
        DeclareLaunchArgument('camera_info_topic', default_value='/sensors/camera/rgb/camera_info'),
        DeclareLaunchArgument('left_image_topic', default_value='/sensors/camera/left/image_rect'),
        DeclareLaunchArgument('right_image_topic', default_value='/sensors/camera/right/image_rect'),
        DeclareLaunchArgument('left_camera_info_topic', default_value='/sensors/camera/left/camera_info'),
        DeclareLaunchArgument('right_camera_info_topic', default_value='/sensors/camera/right/camera_info'),
    ]

    # ── 1. Livox MID360 driver ──
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([livox_share, 'launch', 'msg_MID360_launch.py'])),
        condition=IfCondition(start_livox),
    )

    # ── 2. Static TFs ──
    base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['-0.119', '0', '-0.01', '0', '0', '0', 'base_footprint', 'base_link'],
        condition=IfCondition(publish_base_link_tf),
    )

    livox_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_livox_frame',
        arguments=['0.119', '0', '0.01', '0', '0', '0', 'base_link', 'livox_frame'],
    )

    gnss_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_gnss_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gnss_link'],
        condition=IfCondition(enable_gps),
    )

    # ── 3. FAST-LIO ──
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([fast_lio_share, 'launch', 'mapping.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_file': 'mid360.yaml',
            'rviz': 'false',
        }.items(),
    )

    # ── 4. RTK GNSS driver (optional) ──
    rtk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('bt468_rtk_driver'), 'launch', 'bt468_rtk.launch.py'])),
        condition=IfCondition(enable_gps),
    )

    # ── 5. navsat_transform_node (optional) ──
    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        condition=IfCondition(enable_gps),
        parameters=[{
            'use_sim_time': use_sim_time,
            'frequency': 20.0,
            'delay': 1.0,
            'magnetic_declination_radians': 0.0,
            'yaw_offset': 0.0,
            'zero_altitude': True,
            'broadcast_utm_transform': False,
            'publish_filtered_gps': False,
            'use_odometry_yaw': False,
            'wait_for_datum': False,
        }],
        remappings=[
            ('imu/data', LaunchConfiguration('imu_topic')),
            ('gps/fix', gps_fix_topic),
            ('odometry/filtered', LaunchConfiguration('odom_topic')),
            ('odometry/gps', '/odometry/gps'),
        ],
    )

    # ── 6. RTAB-Map (localization mode) ──
    rtabmap_la = {
        'use_sim_time': use_sim_time,
        'localization': 'true',
        'database_path': database_path,
        'frame_id': LaunchConfiguration('frame_id'),
        'map_frame_id': LaunchConfiguration('map_frame_id'),
        'publish_tf_map': 'true',
        'publish_tf_odom': 'false',
        'odom_topic': LaunchConfiguration('odom_topic'),
        'imu_topic': LaunchConfiguration('imu_topic'),
        'scan_cloud_topic': LaunchConfiguration('scan_cloud_topic'),
        'subscribe_scan_cloud': 'true',
        'subscribe_scan': 'false',
        'visual_odometry': 'false',
        'icp_odometry': 'false',
        'rviz': enable_rviz,
        'rtabmap_viz': 'false',
        'gps_topic': gps_fix_topic,
        # Camera configuration driven by sensor_profile
        'rgb_topic': LaunchConfiguration('rgb_topic'),
        'depth_topic': LaunchConfiguration('depth_topic'),
        'camera_info_topic': LaunchConfiguration('camera_info_topic'),
        'left_image_topic': LaunchConfiguration('left_image_topic'),
        'right_image_topic': LaunchConfiguration('right_image_topic'),
        'left_camera_info_topic': LaunchConfiguration('left_camera_info_topic'),
        'right_camera_info_topic': LaunchConfiguration('right_camera_info_topic'),
        'rgbd_sync': PythonExpression(["'", sensor_profile, "' == 'lidar_rgbd'"]),
        'subscribe_rgbd': PythonExpression(["'", sensor_profile, "' == 'lidar_rgbd'"]),
        'stereo': PythonExpression(["'", sensor_profile, "' == 'lidar_stereo'"]),
        'depth': PythonExpression(["'", sensor_profile, "' == 'lidar_rgbd'"]),
        'subscribe_rgb': PythonExpression(["'", sensor_profile, "' == 'lidar_rgbd' or '", sensor_profile, "' == 'lidar_mono'"]),
    }

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rtabmap_launch_share, 'launch', 'rtabmap.launch.py'])),
        launch_arguments=rtabmap_la.items(),
    )

    # ── 7. Nav2 ──
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_share, 'launch', 'navigation_launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': LaunchConfiguration('nav2_params_file'),
            'use_composition': 'False',
            'use_respawn': 'False',
            'log_level': 'info',
        }.items(),
    )

    # ── 8. Collision Monitor ──
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'cmd_vel_in_topic': '/cmd_vel',
            'cmd_vel_out_topic': '/cmd_vel_safe',
            'transform_tolerance': 0.3,
            'source_timeout': 1.0,
            'base_shift_correction': True,
            'stop_pub_timeout': 1.0,
            'polygons': ['StopZone', 'SlowZone'],
            'observation_sources': ['pointcloud'],
            'StopZone.type': 'polygon',
            'StopZone.points': [0.35, 0.30, 0.35, -0.30, -0.10, -0.30, -0.10, 0.30],
            'StopZone.action_type': 'stop',
            'StopZone.max_points': 3,
            'StopZone.visualize': True,
            'StopZone.polygon_pub_topic': 'collision_monitor/stop_zone',
            'StopZone.enabled': True,
            'SlowZone.type': 'polygon',
            'SlowZone.points': [0.55, 0.40, 0.55, -0.40, -0.25, -0.40, -0.25, 0.40],
            'SlowZone.action_type': 'slowdown',
            'SlowZone.max_points': 3,
            'SlowZone.slowdown_ratio': 0.35,
            'SlowZone.visualize': True,
            'SlowZone.polygon_pub_topic': 'collision_monitor/slow_zone',
            'SlowZone.enabled': True,
            'pointcloud.type': 'pointcloud',
            'pointcloud.topic': '/cloud_registered_body',
            'pointcloud.min_height': 0.05,
            'pointcloud.max_height': 1.80,
            'pointcloud.enabled': True,
        }],
    )

    # ── Assemble ──
    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

    # RTAB-Map library workaround
    _colcon_prefix = os.environ.get('COLCON_PREFIX_PATH', '')
    if _colcon_prefix:
        _ws_root = os.path.dirname(_colcon_prefix.split(':')[0])
        _rtabmap_lib = os.path.join(_ws_root, 'third_party', 'rtabmap-0.23.4', 'install', 'lib')
        _existing_ldpath = os.environ.get('LD_LIBRARY_PATH', '')
        ld.add_action(SetEnvironmentVariable('LD_LIBRARY_PATH',
            _rtabmap_lib + ':' + _existing_ldpath if _existing_ldpath else _rtabmap_lib
        ))

    for arg in declare_args:
        ld.add_action(arg)

    ld.add_action(base_tf)
    ld.add_action(livox_tf)
    ld.add_action(gnss_tf)
    ld.add_action(livox_launch)
    ld.add_action(fast_lio_launch)
    ld.add_action(rtk_launch)
    ld.add_action(navsat_transform)
    ld.add_action(rtabmap_launch)
    ld.add_action(nav2_launch)
    ld.add_action(collision_monitor)

    return ld
