#!/usr/bin/env python3
"""
Main mapping bringup launch file.

Starts:
  1. Livox MID360 lidar driver
  2. ICP odometry (rtabmap_odom) → /odometry/lio
  3. EKF fusion (robot_localization) → /odometry/local
  4. RTAB-Map SLAM (consumes /odometry/local)
  5. Static TFs

Usage:
  ros2 launch robot_bringup mapping.launch.py
  ros2 launch robot_bringup mapping.launch.py start_livox:=false rviz:=true
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_livox = LaunchConfiguration('start_livox')
    publish_base_link_tf = LaunchConfiguration('publish_base_link_tf')
    rviz = LaunchConfiguration('rviz')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    ekf_params_file = LaunchConfiguration('ekf_params_file')

    robot_bringup_share = FindPackageShare('robot_bringup')
    livox_share = FindPackageShare('livox_ros_driver2')
    rtabmap_launch_share = FindPackageShare('rtabmap_launch')

    # ── Declare arguments ──
    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_livox', default_value='true',
                              description='Start Livox MID360 driver'),
        DeclareLaunchArgument('publish_base_link_tf', default_value='true',
                              description='Publish static TF base_footprint → base_link'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rtabmap_viz', default_value='true'),
        DeclareLaunchArgument('ekf_params_file',
                              default_value=PathJoinSubstitution([robot_bringup_share, 'config', 'ekf_local.yaml'])),
        DeclareLaunchArgument('scan_cloud_topic', default_value='/livox/lidar',
                              description='Input point cloud topic for ICP and RTAB-Map'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu',
                              description='IMU topic (Livox MID360 onboard IMU)'),
        DeclareLaunchArgument('frame_id', default_value='base_footprint',
                              description='Robot base frame'),
        DeclareLaunchArgument('odom_frame_id', default_value='',
                              description='RTAB-Map odometry TF frame. Keep empty to use odom_topic'),
    ]

    # ── 1. Livox MID360 driver (rviz launch outputs PointCloud2) ──
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
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        condition=IfCondition(publish_base_link_tf),
    )

    livox_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_livox_frame',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'livox_frame'],
    )

    # ── 3. ICP odometry → /odometry/lio ──
    icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': LaunchConfiguration('frame_id'),
            'odom_frame_id': 'icp_odom',
            'publish_tf': False,
            'wait_for_transform': 0.2,
            'Icp/VoxelSize': '0.15',
            'Icp/DownsamplingStep': '1',
            'Icp/MaxTranslation': '1.5',
            'Icp/MaxRotation': '0.7',
            'Icp/MaxCorrespondenceDistance': '0.8',
            'Icp/CorrespondenceRatio': '0.05',
            'Icp/PointToPlane': 'true',
            'Icp/PointToPlaneK': '20',
            'Icp/PointToPlaneMinComplexity': '0.04',
            'Odom/ResetCountdown': '1',
            'OdomF2M/ScanSubtractRadius': '0.15',
        }],
        remappings=[
            ('scan_cloud', LaunchConfiguration('scan_cloud_topic')),
            ('imu', LaunchConfiguration('imu_topic')),
            ('odom', '/odometry/lio'),
        ],
    )

    # ── 4. EKF fusion → /odometry/local ──
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local_filter',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', '/odometry/local'),
        ],
    )

    # ── 4b. Lifecycle manager to auto-activate EKF ──
    ekf_lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='ekf_lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['ekf_local_filter'],
        }],
    )

    # ── 5. RTAB-Map SLAM ──
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rtabmap_launch_share, 'launch', 'rtabmap.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'localization': 'false',
            'frame_id': LaunchConfiguration('frame_id'),
            'odom_frame_id': LaunchConfiguration('odom_frame_id'),
            'publish_tf_map': 'true',
            'publish_tf_odom': 'false',
            'odom_topic': '/odometry/local',
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
            'rtabmap_args': '--Reg/Strategy 1 '
                            '--RGBD/ProximityBySpace true '
                            '--Mem/NotLinkedNodesKept false '
                            '--Icp/VoxelSize 0.12 '
                            '--Icp/DownsamplingStep 1 '
                            '--Icp/MaxTranslation 1.5 '
                            '--Icp/MaxRotation 0.7 '
                            '--Icp/MaxCorrespondenceDistance 0.8 '
                            '--Icp/CorrespondenceRatio 0.05 '
                            '--Icp/PointToPlane true '
                            '--Icp/PointToPlaneK 15 '
                            '--Icp/PointToPlaneMinComplexity 0.04',
        }.items(),
    )

    # ── Assemble ──
    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))
    _rtabmap_lib = '/home/wheeltec/wzy/third_party/rtabmap-0.23.4/install/lib'
    _existing_ldpath = os.environ.get('LD_LIBRARY_PATH', '')
    ld.add_action(SetEnvironmentVariable('LD_LIBRARY_PATH',
        _rtabmap_lib + ':' + _existing_ldpath if _existing_ldpath else _rtabmap_lib
    ))
    for arg in declare_args:
        ld.add_action(arg)
    ld.add_action(base_tf)
    ld.add_action(livox_tf)
    ld.add_action(livox_launch)
    ld.add_action(icp_odometry)
    ld.add_action(ekf_local)
    ld.add_action(ekf_lifecycle_mgr)
    ld.add_action(rtabmap_launch)
    return ld
