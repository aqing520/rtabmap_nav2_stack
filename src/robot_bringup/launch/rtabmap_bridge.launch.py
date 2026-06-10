#!/usr/bin/env python3
"""RTAB-Map bridge starter.

Suggested location:
  robot_bringup/launch/rtabmap_bridge.launch.py

This wrapper keeps RTAB-Map focused on global SLAM, loop closure, mapping, and
localization, while local continuous odometry comes from FAST-LIO.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    rtabmap_launch_share = FindPackageShare('rtabmap_launch')

    declare_args = [
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('sensor_profile', default_value='lidar_only', description='lidar_only | lidar_rgbd | lidar_stereo | lidar_mono'),
        DeclareLaunchArgument('enable_gps', default_value='false'),
        DeclareLaunchArgument('localization', default_value='true', description='false=mapping, true=localization/navigation'),
        DeclareLaunchArgument('database_path', default_value='/data/maps/site_a/rtabmap.db'),
        DeclareLaunchArgument(
            'rtabmap_args',
            default_value=(
                '--Grid/RayTracing true'
                ' --Grid/CellSize 0.05'
                ' --Grid/FromDepth false'
                ' --Grid/PreVoxelFiltering true'
                ' --Grid/RangeMax 20.0'
                ' --Grid/RangeMin 0.1'
                ' --Grid/MaxGroundHeight 0.08'
                ' --Grid/MinGroundHeight -0.6'
                ' --Grid/MaxObstacleHeight 1.6'
                ' --Grid/NormalK 20'
                ' --Grid/MaxGroundAngle 22'
                ' --Grid/ClusterRadius 0.18'
                ' --Grid/FootprintRadius 0.3'
                ' --Grid/MinClusterSize 5'
                ' --Grid/NoiseFilteringRadius 0.12'
                ' --Grid/NoiseFilteringMinNeighbors 2'
                ' --Grid/Scan2dUnknownSpaceFilled true'
            ),
        ),
        DeclareLaunchArgument('frame_id', default_value='base_footprint'),
        DeclareLaunchArgument('map_frame_id', default_value='map'),
        DeclareLaunchArgument('odom_topic', default_value='/Odometry'),
        DeclareLaunchArgument('imu_topic', default_value='/unused_imu'),
        DeclareLaunchArgument('gps_topic', default_value='/sensors/gps/fix'),
        DeclareLaunchArgument('scan_cloud_topic', default_value='/cloud_registered_body'),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('left_image_topic', default_value='/sensors/camera/left/image_rect'),
        DeclareLaunchArgument('right_image_topic', default_value='/sensors/camera/right/image_rect'),
        DeclareLaunchArgument('left_camera_info_topic', default_value='/sensors/camera/left/camera_info'),
        DeclareLaunchArgument('right_camera_info_topic', default_value='/sensors/camera/right/camera_info'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false'),
        DeclareLaunchArgument('wait_imu_to_init', default_value='false'),
        DeclareLaunchArgument('delete_db_on_start', default_value='false'),
    ]

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([rtabmap_launch_share, 'launch', 'rtabmap.launch.py'])),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'localization': LaunchConfiguration('localization'),
            'database_path': LaunchConfiguration('database_path'),
            'frame_id': LaunchConfiguration('frame_id'),
            'map_frame_id': LaunchConfiguration('map_frame_id'),
            'publish_tf_map': 'true',
            'odom_topic': LaunchConfiguration('odom_topic'),
            'publish_tf_odom': 'false',
            'imu_topic': LaunchConfiguration('imu_topic'),
            'wait_imu_to_init': LaunchConfiguration('wait_imu_to_init'),
            'gps_topic': LaunchConfiguration('gps_topic'),
            'scan_cloud_topic': LaunchConfiguration('scan_cloud_topic'),
            'subscribe_scan_cloud': 'true',
            'subscribe_scan': 'false',
            'visual_odometry': 'false',
            'icp_odometry': 'false',
            'approx_sync': 'true',
            'qos': '2',
            'latch': 'true',
            'rviz': LaunchConfiguration('rviz'),
            'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'left_image_topic': LaunchConfiguration('left_image_topic'),
            'right_image_topic': LaunchConfiguration('right_image_topic'),
            'left_camera_info_topic': LaunchConfiguration('left_camera_info_topic'),
            'right_camera_info_topic': LaunchConfiguration('right_camera_info_topic'),
            'rgbd_sync': PythonExpression(["'", LaunchConfiguration('sensor_profile'), "' == 'lidar_rgbd'"]),
            'subscribe_rgbd': PythonExpression(["'", LaunchConfiguration('sensor_profile'), "' == 'lidar_rgbd'"]),
            'stereo': PythonExpression(["'", LaunchConfiguration('sensor_profile'), "' == 'lidar_stereo'"]),
            'depth': PythonExpression(["'", LaunchConfiguration('sensor_profile'), "' == 'lidar_rgbd'"]),
            'subscribe_rgb': PythonExpression(["'", LaunchConfiguration('sensor_profile'), "' == 'lidar_rgbd' or '", LaunchConfiguration('sensor_profile'), "' == 'lidar_mono'"]),
            'args': [
                PythonExpression(["'-d ' if '", LaunchConfiguration('delete_db_on_start'), "' == 'true' else ''"]),
                LaunchConfiguration('rtabmap_args'),
            ],
        }.items(),
    )

    fixed_map = Node(
        package='robot_bringup',
        executable='fill_unknown_map_node.py',
        name='fill_unknown_map',
        output='screen',
        parameters=[{
            'input_topic': '/map',
            'output_topic': '/map_fixed',
            'max_fill_area_m2': 4.0,
            'obstacle_keepout_m': 0.7,
        }],
    )

    ld = LaunchDescription()
    for action in declare_args:
        ld.add_action(action)
    ld.add_action(rtabmap_launch)
    ld.add_action(fixed_map)
    return ld
