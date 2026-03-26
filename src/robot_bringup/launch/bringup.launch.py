#!/usr/bin/env python3
"""RTAB-Map-led AMR bringup starter.

Suggested location:
  robot_bringup/launch/bringup.launch.py

Assumptions:
- RTAB-Map is the only publisher of map -> odom.
- FAST-LIO publishes odom -> base_footprint and /Odometry.
- Nav2 consumes /map and /Odometry.
- The base controller should subscribe to /cmd_vel_safe.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    namespace = LaunchConfiguration('namespace')
    mode = LaunchConfiguration('mode')
    sensor_profile = LaunchConfiguration('sensor_profile')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    start_livox = LaunchConfiguration('start_livox')
    enable_gps = LaunchConfiguration('enable_gps')
    enable_rviz = LaunchConfiguration('enable_rviz')
    publish_base_link_tf = LaunchConfiguration('publish_base_link_tf')
    database_path = LaunchConfiguration('database_path')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    robot_bringup_share = FindPackageShare('robot_bringup')
    nav2_bringup_share = FindPackageShare('nav2_bringup')
    livox_share = FindPackageShare('livox_ros_driver2')
    fast_lio_share = FindPackageShare('fast_lio')

    declare_args = [
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('mode', default_value='navigation', description='mapping | localization | navigation'),
        DeclareLaunchArgument('sensor_profile', default_value='lidar_rgbd', description='lidar_only | lidar_rgbd | lidar_stereo | lidar_mono'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('start_livox', default_value='true', description='Start Livox MID360 launch'),
        DeclareLaunchArgument('enable_gps', default_value='false', description='Enable navsat_transform and pass GPS fix to RTAB-Map'),
        DeclareLaunchArgument('enable_rviz', default_value='false', description='Launch RViz through the RTAB-Map bridge'),
        DeclareLaunchArgument('publish_base_link_tf', default_value='true', description='Publish a zero static TF from base_footprint to base_link if URDF is not ready'),
        DeclareLaunchArgument('database_path', default_value='/data/maps/site_a/rtabmap.db'),
        DeclareLaunchArgument('nav2_params_file', default_value=PathJoinSubstitution([robot_bringup_share, 'config', 'nav2_common.yaml'])),
        DeclareLaunchArgument('rtabmap_frame_id', default_value='base_footprint'),
        DeclareLaunchArgument('rtabmap_map_frame', default_value='map'),
        DeclareLaunchArgument('rtabmap_odom_topic', default_value='/Odometry'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('gps_fix_topic', default_value='/sensors/gps/fix'),
        DeclareLaunchArgument('scan_cloud_topic', default_value='/cloud_registered_body'),
    ]

    # ── 1. Livox MID360 driver ──
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([livox_share, 'launch', 'msg_MID360_launch.py'])),
        condition=IfCondition(start_livox),
    )

    # ── 2. Static TFs ──
    base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        condition=IfCondition(publish_base_link_tf),
    )

    livox_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_livox_frame',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'livox_frame'],
    )

    # ── 3. FAST-LIO (replaces icp_odometry + EKF) ──
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([fast_lio_share, 'launch', 'mapping.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_file': 'mid360.yaml',
            'rviz': 'false',
        }.items(),
    )

    # ── 4. GPS (optional, requires robot_localization installed separately) ──
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
            ('gps/fix', LaunchConfiguration('gps_fix_topic')),
            ('odometry/filtered', '/Odometry'),
            ('odometry/gps', '/odometry/gps'),
        ],
    )

    # ── 5. RTAB-Map (SLAM / Localization) ──
    rtabmap_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([robot_bringup_share, 'launch', 'rtabmap_bridge.launch.py'])),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'sensor_profile': sensor_profile,
            'enable_gps': enable_gps,
            'localization': PythonExpression(["'", mode, "' != 'mapping'"]),
            'database_path': database_path,
            'frame_id': LaunchConfiguration('rtabmap_frame_id'),
            'map_frame_id': LaunchConfiguration('rtabmap_map_frame'),
            'odom_topic': LaunchConfiguration('rtabmap_odom_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'gps_topic': LaunchConfiguration('gps_fix_topic'),
            'scan_cloud_topic': LaunchConfiguration('scan_cloud_topic'),
            'rviz': enable_rviz,
        }.items(),
    )

    # ── 6. Nav2 ──
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([nav2_bringup_share, 'launch', 'navigation_launch.py'])),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'navigation'"])),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': nav2_params_file,
            'use_composition': 'False',
            'use_respawn': 'False',
            'log_level': 'info',
        }.items(),
    )

    # ── 7. Collision Monitor ──
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'navigation'"])),
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

    # RTAB-Map library workaround: dynamically locate from workspace root
    _colcon_prefix = os.environ.get('COLCON_PREFIX_PATH', '')
    if _colcon_prefix:
        _ws_root = os.path.dirname(_colcon_prefix.split(':')[0])
        _rtabmap_lib = os.path.join(_ws_root, 'third_party', 'rtabmap-0.23.4', 'install', 'lib')
        _existing_ldpath = os.environ.get('LD_LIBRARY_PATH', '')
        ld.add_action(SetEnvironmentVariable('LD_LIBRARY_PATH',
            _rtabmap_lib + ':' + _existing_ldpath if _existing_ldpath else _rtabmap_lib
        ))

    for action in declare_args:
        ld.add_action(action)
    ld.add_action(base_tf)
    ld.add_action(livox_tf)
    ld.add_action(livox_launch)
    ld.add_action(fast_lio_launch)
    ld.add_action(navsat_transform)
    ld.add_action(rtabmap_bridge)
    ld.add_action(nav2_launch)
    ld.add_action(collision_monitor)
    return ld
