#!/usr/bin/env python3
"""
Main mapping bringup launch file using FAST-LIO.

Starts:
  1. Livox MID360 lidar driver
  2. FAST-LIO odometry -> /Odometry (and odom->base_footprint TF)
  3. RTAB-Map SLAM (consumes /Odometry)
  4. Static TFs

Usage:
  ros2 launch robot_bringup fastlio_mapping.launch.py
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
    start_camera = LaunchConfiguration('start_camera')
    publish_base_link_tf = LaunchConfiguration('publish_base_link_tf')
    rviz = LaunchConfiguration('rviz')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    delete_db_on_start = LaunchConfiguration('delete_db_on_start')
    fast_lio_config_file = LaunchConfiguration('fast_lio_config_file')

    livox_share = FindPackageShare('livox_ros_driver2')
    robot_bringup_share = FindPackageShare('robot_bringup')
    rtabmap_launch_share = FindPackageShare('rtabmap_launch')
    fast_lio_share = FindPackageShare('fast_lio')

    # ── Declare arguments ──
    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_livox', default_value='true',
                              description='Start Livox MID360 driver'),
        DeclareLaunchArgument('start_camera', default_value='false',
                              description='Start Orbbec Gemini/Astra RGB-D camera'),
        DeclareLaunchArgument('publish_base_link_tf', default_value='true',
                              description='Publish static TF base_footprint -> base_link'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rtabmap_viz', default_value='true'),
        DeclareLaunchArgument('delete_db_on_start', default_value='true',
                              description='Delete old RTAB-Map database on startup for a clean mapping session'),
        
        # We change the default scan_cloud_topic from /livox/lidar to /cloud_registered_body.
        # This is the motion-deskewed pointcloud exported by FAST-LIO, meaning 
        # RTAB-Map's loop closure and map generation will be much cleaner/sharper.
        DeclareLaunchArgument('scan_cloud_topic', default_value='/cloud_registered_body',
                              description='Input point cloud topic for RTAB-Map'),
        DeclareLaunchArgument('sensor_profile', default_value='lidar_only',
                              description='lidar_only | lidar_rgbd'),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('camera_base_frame', default_value='base_link'),
        DeclareLaunchArgument('camera_link_frame', default_value='camera_link'),
        DeclareLaunchArgument('camera_x', default_value='0.10'),
        DeclareLaunchArgument('camera_y', default_value='0.0'),
        DeclareLaunchArgument('camera_z', default_value='0.616'),
        DeclareLaunchArgument('camera_roll', default_value='-1.5707'),
        DeclareLaunchArgument('camera_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_yaw', default_value='-1.5707'),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/unused_imu',
            description='Optional RTAB-Map IMU topic. FAST-LIO still uses /livox/imu from its own config.',
        ),
        DeclareLaunchArgument('database_path', default_value='/data/maps/site_a/rtabmap.db',
                              description='Path to save/load the RTAB-Map database'),
        DeclareLaunchArgument(
            'fast_lio_config_file',
            default_value='mid360_save.yaml',
            description='FAST-LIO config file. The default enables raw accumulated PCD saving.',
        ),
        DeclareLaunchArgument('frame_id', default_value='base_footprint',
                              description='Robot base frame'),
        DeclareLaunchArgument('odom_frame_id', default_value='',
                              description='RTAB-Map odometry TF frame. Keep empty to use odom_topic'),
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
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        condition=IfCondition(publish_base_link_tf),
    )

    livox_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_livox_frame',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'livox_frame'],
    )

    # ── 3. FAST-LIO (Replaces icp_odometry & ekf_node) ──
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([fast_lio_share, 'launch', 'mapping.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_file': fast_lio_config_file,
            'rviz': 'false', # Disable FAST-LIO's separate RViz to prevent conflict
        }.items(),
    )

    # ── 3b. Orbbec Gemini/Astra RGB-D camera ──
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([robot_bringup_share, 'launch', 'orbbec_camera.launch.py'])),
        condition=IfCondition(start_camera),
        launch_arguments={
            'start_camera': start_camera,
            'camera_name': LaunchConfiguration('camera_name'),
            'camera_base_frame': LaunchConfiguration('camera_base_frame'),
            'camera_link_frame': LaunchConfiguration('camera_link_frame'),
            'camera_x': LaunchConfiguration('camera_x'),
            'camera_y': LaunchConfiguration('camera_y'),
            'camera_z': LaunchConfiguration('camera_z'),
            'camera_roll': LaunchConfiguration('camera_roll'),
            'camera_pitch': LaunchConfiguration('camera_pitch'),
            'camera_yaw': LaunchConfiguration('camera_yaw'),
        }.items(),
    )

    # ── 4. RTAB-Map SLAM ──
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rtabmap_launch_share, 'launch', 'rtabmap.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'localization': 'false',
            'database_path': LaunchConfiguration('database_path'),
            'frame_id': LaunchConfiguration('frame_id'),
            'odom_frame_id': LaunchConfiguration('odom_frame_id'),
            'publish_tf_map': 'true',
            'publish_tf_odom': 'false',
            'odom_topic': '/Odometry',   # <--- Link to FAST-LIO's odometry topic
            'imu_topic': LaunchConfiguration('imu_topic'),
            'scan_cloud_topic': LaunchConfiguration('scan_cloud_topic'),
            'subscribe_scan_cloud': 'true',
            'subscribe_scan': 'false',
            'visual_odometry': 'false',
            'icp_odometry': 'false',
            'rviz': rviz,
            'rtabmap_viz': rtabmap_viz,
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'rgbd_sync': PythonExpression(["'", LaunchConfiguration('sensor_profile'), "' == 'lidar_rgbd'"]),
            'subscribe_rgbd': PythonExpression(["'", LaunchConfiguration('sensor_profile'), "' == 'lidar_rgbd'"]),
            'subscribe_rgb': PythonExpression(["'", LaunchConfiguration('sensor_profile'), "' == 'lidar_rgbd'"]),
            'depth': PythonExpression(["'", LaunchConfiguration('sensor_profile'), "' == 'lidar_rgbd'"]),
            'stereo': 'false',
            'approx_sync': 'true',
            'qos': '2',
            'namespace': 'rtabmap',
            'args': PythonExpression([
                "('--delete_db_on_start ' if '", delete_db_on_start, "' == 'true' else '') + "
                "'--Reg/Strategy 1 --RGBD/ProximityBySpace true --Mem/NotLinkedNodesKept false --Icp/VoxelSize 0.05 --Icp/DownsamplingStep 1 --Icp/MaxTranslation 1.5 --Icp/MaxRotation 0.7 --Icp/MaxCorrespondenceDistance 0.5 --Icp/CorrespondenceRatio 0.05 --Icp/PointToPlane true --Icp/PointToPlaneK 15 --Icp/PointToPlaneMinComplexity 0.04 --Grid/CellSize 0.05 --Grid/FromDepth false --Grid/PreVoxelFiltering true --Grid/3D false --Grid/NormalsSegmentation true --Grid/NormalK 20 --Grid/MaxGroundAngle 22 --Grid/ClusterRadius 0.18 --Grid/RangeMin 0.1 --Grid/RangeMax 20.0 --Grid/FootprintLength 0.55 --Grid/FootprintWidth 0.45 --Grid/MinClusterSize 5 --Grid/NoiseFilteringRadius 0.12 --Grid/NoiseFilteringMinNeighbors 2 --Grid/MaxObstacleHeight 1.6 --Grid/MinGroundHeight -0.6 --Grid/MaxGroundHeight 0.08 --Grid/Scan2dUnknownSpaceFilled true --Grid/RayTracing true'"
            ]),
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
    
    for arg in declare_args:
        ld.add_action(arg)
        
    ld.add_action(base_tf)
    ld.add_action(livox_tf)
    ld.add_action(livox_launch)
    ld.add_action(fast_lio_launch)
    ld.add_action(camera_launch)
    ld.add_action(rtabmap_launch)
    ld.add_action(fixed_map)
    
    return ld
