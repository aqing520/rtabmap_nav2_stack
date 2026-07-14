#!/usr/bin/env python3
"""
Main mapping bringup launch file using FAST-LIO.

Starts:
  1.1 Livox MID360 lidar driver
  1.2 RealSense D435i driver (optional, depending on sensor_profile)
  2. FAST-LIO odometry -> /Odometry (and odom->base_footprint TF)
  3. RTAB-Map SLAM (consumes /Odometry)
  4. Static TFs

Usage:
  ros2 launch robot_bringup fastlio_mapping_realsense_d435i.launch.py
"""

import math
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
    sensor_profile = LaunchConfiguration('sensor_profile')
    start_realsense = LaunchConfiguration('start_realsense')
    publish_base_link_tf = LaunchConfiguration('publish_base_link_tf')
    bag_tf_override = LaunchConfiguration('bag_tf_override')
    rviz = LaunchConfiguration('rviz')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    delete_db_on_start = LaunchConfiguration('delete_db_on_start')

    livox_share = FindPackageShare('livox_ros_driver2')
    rtabmap_launch_share = FindPackageShare('rtabmap_launch')
    fast_lio_share = FindPackageShare('fast_lio')

    # ── Declare arguments ──
    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('sensor_profile', default_value='lidar_stereo', description='lidar_only | lidar_rgbd | lidar_infra | lidar_stereo | lidar_mono'),
        DeclareLaunchArgument('start_livox', default_value='true',
                              description='Start Livox MID360 driver'),
        DeclareLaunchArgument('start_realsense', default_value='true',
                              description='Start RealSense D435i driver'),
        DeclareLaunchArgument('publish_base_link_tf', default_value='true',
                              description='Publish static TF base_footprint -> base_link'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rtabmap_viz', default_value='true'),
        DeclareLaunchArgument('delete_db_on_start', default_value='true',
                              description='Delete old RTAB-Map database on startup for a clean mapping session'),
        
        # We change the default scan_cloud_topic from /livox/lidar to /cloud_registered_body.
        # This is the motion-deskewed pointcloud exported by FAST-LIO, meaning 
        # RTAB-Map's loop closure and map generation will be much cleaner/sharper.
        DeclareLaunchArgument('scan_cloud_topic', default_value='/cloud_registered_body',
                              description='Input point cloud topic for RTAB-Map'),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/livox/imu',
            description='Optional RTAB-Map IMU topic. FAST-LIO still uses /livox/imu from its own config.',
        ),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('left_image_topic', default_value='/camera/infra1/image_rect_raw'),
        DeclareLaunchArgument('right_image_topic', default_value='/camera/infra2/image_rect_raw'),
        DeclareLaunchArgument('left_camera_info_topic', default_value='/camera/infra1/camera_info'),
        DeclareLaunchArgument('right_camera_info_topic', default_value='/camera/infra2/camera_info'),
        DeclareLaunchArgument('database_path', default_value='rtabmap_realsense.db',
                              description='Path to save/load the RTAB-Map database'),
        DeclareLaunchArgument('frame_id', default_value='base_footprint',
                              description='Robot base frame'),
        DeclareLaunchArgument('odom_frame_id', default_value='',
                              description='RTAB-Map odometry TF frame. Keep empty to use odom_topic'),
        # Kp/DetectorStrategy and Vis/FeatureType
        # 0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint 12=SURF/FREAK 13=GFTT/DAISY 14=SURF/DAISY 15=PyDetector 16=SuperPoint-Rpautrat")
        DeclareLaunchArgument('rtabmap_args', default_value=
            "--Reg/Strategy 2 \
            --Stereo/Gpu true \
            --GFTT/Gpu true \
            --Vis/MaxFeatures 800 \
            --Vis/MinInliers 15 \
            --Grid/3D false \
            --Grid/Sensor 0 \
            --Grid/RayTracing true \
            --Grid/NormalsSegmentation false \
            --Grid/MaxGroundHeight 0.3 \
            --Grid/MaxObstacleHeight 1.0 \
            --Grid/CellSize 0.10 \
            --Grid/RangeMin 0.0 \
            --Grid/RangeMax 20.0 "
        ),
        DeclareLaunchArgument(
            'bag_tf_override',
            default_value='true',
            description='When playing bag with /tf_static remapped, publish all required static TFs from launch'
        ),
    ]

    # ── 1. Livox MID360 driver ──
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([livox_share, 'launch', 'msg_MID360_launch.py'])),
        condition=IfCondition(PythonExpression([
            "'", start_livox, "' == 'true'"
        ])),
    )
    
    # ── 1.2. RealSense D435i driver ──
    realsense_rgbd_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_accel': False,
            'enable_gyro': False,
            'depth_module.emitter_enabled': 1,
            # 分辨率
            'depth_module.depth_profile': '424x240x30',
            'depth_module.infra_profile': '424x240x30',
            'rgb_camera.color_profile': '424x240x30',
            # === 功能配置 ===
            'enable_sync': True, # 内部硬件同步
            'align_depth.enable': True, # 深度对齐到彩色
            'initial_reset': False, # 启动时不重置设备，避免丢帧

        }],
        condition=IfCondition(PythonExpression([
            "'", start_realsense, "' == 'true' and '",
            sensor_profile, "' == 'lidar_rgbd'"
        ])),
    )

    realsense_stereo_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='',
        parameters=[{
            'enable_color': False,
            'enable_depth': False,
            'enable_infra1': True,
            'enable_infra2': True,
            'enable_accel': False,
            'enable_gyro': False,
            'depth_module.emitter_enabled': 0,
            'depth_module.depth_profile': '640x480x30',
            'depth_module.infra_profile': '640x480x30',
            'rgb_camera.color_profile': '640x480x30',
            'enable_sync': True, # 内部硬件同步
            'align_depth.enable': False, # 深度对齐到彩色
            'initial_reset': False, # 启动时不重置设备，避免丢帧
        }],
        condition=IfCondition(PythonExpression([
            "'", start_realsense, "' == 'true' and '",
            sensor_profile, "' == 'lidar_stereo'"
        ])),
    )
    realsense_mono_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='',
        parameters=[{
            'enable_color': True,
            'enable_depth': False,
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_accel': False,
            'enable_gyro': False,
            'depth_module.emitter_enabled': 0,
            'depth_module.depth_profile': '640x480x30',
            'depth_module.infra_profile': '640x480x30',
            'rgb_camera.color_profile': '640x480x30',
            'enable_sync': True, # 内部硬件同步
            'align_depth.enable': False, # 深度对齐到彩色
            'initial_reset': False, # 启动时不重置设备，避免丢帧
        }],
        condition=IfCondition(PythonExpression([
            "'", start_realsense, "' == 'true' and '",
            sensor_profile, "' == 'lidar_mono'"
        ])),
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

    # 相机 TF (根据实际安装位置修改平移和旋转参数，单位为米和弧度)
    # 可以找一个白墙，在rtabmap rviz中观察相机点云和雷达点云的对齐情况，调整参数直到对齐
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera_link',
        # 参数顺序：   x,     y,       z, roll, pitch, yaw,   parent_frame, child_frame
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'camera_link'],
        condition=IfCondition(PythonExpression([
            "'", start_realsense, "' == 'true' or '",
            bag_tf_override, "' == 'true'"
        ])),
    )

    # ── 3. FAST-LIO (Replaces icp_odometry & ekf_node) ──
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([fast_lio_share, 'launch', 'mapping.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_file': 'mid360.yaml',
            'rviz': 'false', # Disable FAST-LIO's separate RViz to prevent conflict
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
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'left_image_topic': LaunchConfiguration('left_image_topic'),
            'right_image_topic': LaunchConfiguration('right_image_topic'),
            'left_camera_info_topic': LaunchConfiguration('left_camera_info_topic'),
            'right_camera_info_topic': LaunchConfiguration('right_camera_info_topic'),
            'subscribe_scan_cloud': 'true',
            'subscribe_scan': 'false',
            'visual_odometry': 'false',
            'icp_odometry': 'false',
            'rviz': 'true',
            'rtabmap_viz': rtabmap_viz,
            'rgbd_sync': PythonExpression(["'true' if '", LaunchConfiguration('sensor_profile'), "' == 'lidar_rgbd' or '", LaunchConfiguration('sensor_profile'), "' == 'lidar_stereo' else 'false'"]),
            'subscribe_rgbd': PythonExpression(["'true' if '", LaunchConfiguration('sensor_profile'), "' == 'lidar_rgbd' or '", LaunchConfiguration('sensor_profile'), "' == 'lidar_stereo' else 'false'"]),
            'subscribe_rgb': PythonExpression(["'true' if '", LaunchConfiguration('sensor_profile'), "' == 'lidar_rgbd' or '", LaunchConfiguration('sensor_profile'), "' == 'lidar_mono' else 'false'"]),
            'depth': PythonExpression(["'true' if '", LaunchConfiguration('sensor_profile'), "' == 'lidar_rgbd' else 'false'"]),
            'stereo': PythonExpression(["'true' if '", LaunchConfiguration('sensor_profile'), "' == 'lidar_stereo' else 'false'"]),
            'approx_sync': 'true',
            'qos': '2',
            'namespace': 'rtabmap',
            'args':[
                PythonExpression(["'-d ' if '", delete_db_on_start, "' == 'true' else ''"]),
                LaunchConfiguration('rtabmap_args'),
            ],
        }.items(),
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
    ld.add_action(camera_tf)

    ld.add_action(livox_launch)
    ld.add_action(realsense_rgbd_node)
    ld.add_action(realsense_stereo_node)
    ld.add_action(realsense_mono_node)
    ld.add_action(fast_lio_launch)
    ld.add_action(rtabmap_launch)
    
    return ld
