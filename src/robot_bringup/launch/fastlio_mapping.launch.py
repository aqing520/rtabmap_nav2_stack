#!/usr/bin/env python3
"""
Main mapping bringup launch file using FAST-LIO, with optional RTK GPS.

Starts:
  1. Livox MID360 lidar driver
  2. FAST-LIO odometry -> /Odometry (and odom->base_footprint TF)
  3. BT-468 RTK GNSS driver          (optional, enable_gps)
  4. navsat_transform_node             (optional, enable_gps)
  5. RTAB-Map SLAM (consumes /Odometry + optional GPS)
  6. Static TFs

Usage:
  ros2 launch robot_bringup fastlio_mapping.launch.py
  ros2 launch robot_bringup fastlio_mapping.launch.py enable_gps:=true
  ros2 launch robot_bringup fastlio_mapping.launch.py start_livox:=false rviz:=true
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
    rviz = LaunchConfiguration('rviz')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    delete_db_on_start = LaunchConfiguration('delete_db_on_start')
    enable_gps = LaunchConfiguration('enable_gps')
    gps_fix_topic = LaunchConfiguration('gps_fix_topic')

    livox_share = FindPackageShare('livox_ros_driver2')
    rtabmap_launch_share = FindPackageShare('rtabmap_launch')
    fast_lio_share = FindPackageShare('fast_lio')

    # ── Declare arguments ──
    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_livox', default_value='true',
                              description='Start Livox MID360 driver'),
        DeclareLaunchArgument('publish_base_link_tf', default_value='true',
                              description='Publish static TF base_footprint -> base_link'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rtabmap_viz', default_value='true'),
        DeclareLaunchArgument('delete_db_on_start', default_value='true',
                              description='Delete old RTAB-Map database on startup for a clean mapping session'),
        DeclareLaunchArgument('scan_cloud_topic', default_value='/cloud_registered_body',
                              description='Input point cloud topic for RTAB-Map'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu',
                              description='IMU topic (Livox MID360 onboard IMU)'),
        DeclareLaunchArgument('frame_id', default_value='base_footprint',
                              description='Robot base frame'),
        DeclareLaunchArgument('odom_frame_id', default_value='',
                              description='RTAB-Map odometry TF frame. Keep empty to use odom_topic'),
        # GPS / RTK arguments
        DeclareLaunchArgument('enable_gps', default_value='false',
                              description='Enable RTK GPS (navsat_transform + RTAB-Map GPS constraint)'),
        DeclareLaunchArgument('gps_fix_topic', default_value='/fix',
                              description='NavSatFix topic from RTK driver'),
    ]

    # ── 1. Livox MID360 driver ──
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([livox_share, 'launch', 'rviz_MID360_launch.py'])),
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
            ('odometry/filtered', '/Odometry'),
            ('odometry/gps', '/odometry/gps'),
        ],
    )

    # ── 6. RTAB-Map SLAM ──
    rtabmap_args_expr = PythonExpression([
        "('--delete_db_on_start ' if '", delete_db_on_start, "' == 'true' else '') + "
        "'--Reg/Strategy 1 --RGBD/ProximityBySpace true --Mem/NotLinkedNodesKept false "
        "--Icp/VoxelSize 0.05 --Icp/DownsamplingStep 1 --Icp/MaxTranslation 1.5 "
        "--Icp/MaxRotation 0.7 --Icp/MaxCorrespondenceDistance 0.5 "
        "--Icp/CorrespondenceRatio 0.05 --Icp/PointToPlane true "
        "--Icp/PointToPlaneK 15 --Icp/PointToPlaneMinComplexity 0.04'"
    ])

    rtabmap_la = {
        'use_sim_time': use_sim_time,
        'localization': 'false',
        'frame_id': LaunchConfiguration('frame_id'),
        'odom_frame_id': LaunchConfiguration('odom_frame_id'),
        'publish_tf_map': 'true',
        'publish_tf_odom': 'false',
        'odom_topic': '/Odometry',
        'imu_topic': LaunchConfiguration('imu_topic'),
        'scan_cloud_topic': LaunchConfiguration('scan_cloud_topic'),
        'subscribe_scan_cloud': 'true',
        'subscribe_scan': 'false',
        'visual_odometry': 'false',
        'icp_odometry': 'false',
        'rviz': rviz,
        'rtabmap_viz': rtabmap_viz,
        'rgbd_sync': 'false',
        'subscribe_rgbd': 'false',
        'subscribe_rgb': 'false',
        'depth': 'false',
        'stereo': 'false',
        'approx_sync': 'true',
        'qos': '2',
        'namespace': 'rtabmap',
        'args': rtabmap_args_expr,
        'gps_topic': gps_fix_topic,
    }

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rtabmap_launch_share, 'launch', 'rtabmap.launch.py'])),
        launch_arguments=rtabmap_la.items(),
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

    return ld
