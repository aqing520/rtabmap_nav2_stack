#!/usr/bin/env python3
"""
Main mapping bringup launch file using FAST-LIO with Orbbec depth camera.

Starts:
  1.1 Livox MID360 lidar driver
  1.2 Orbbec depth camera driver (optional, depending on sensor_profile)
  2. FAST-LIO odometry -> /Odometry (and odom->base_footprint TF)
  3. RTAB-Map SLAM (consumes /Odometry)
  4. Static TFs

Usage:
  ros2 launch robot_bringup fastlio_mapping_orbbec.launch.py sensor_profile:=lidar_rgbd
"""

import math
import os
from datetime import datetime
from pathlib import Path
import time

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    LogInfo, OpaqueFunction, SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def prepare_mapping_session(context):
    database_path = Path(
        LaunchConfiguration('database_path').perform(context)
    ).expanduser()
    if not database_path.is_absolute():
        database_path = (Path.cwd() / database_path).resolve()
    else:
        database_path = database_path.resolve()

    delete_old_database = (
        LaunchConfiguration('delete_db_on_start').perform(context).lower()
        == 'true'
    )
    session_started = int(time.time())

    marker_path = Path.home() / '.ros' / 'last_rtabmap_mapping_session'
    marker_path.parent.mkdir(parents=True, exist_ok=True)
    marker_path.write_text(
        'database_path=%s\n'
        'session_started_epoch=%d\n'
        'launch_file=fastlio_mapping_orbbec.launch.py\n'
        % (database_path, session_started),
        encoding='utf-8',
    )

    backup_messages = []
    if delete_old_database:
        exported_map_dir = database_path.parent / 'pgm_map'
        backup_stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        for suffix in ('.yaml', '.pgm'):
            old_map = exported_map_dir / ('map' + suffix)
            if old_map.exists():
                backup = exported_map_dir / (
                    'map%s.before_mapping_%s.bak' % (suffix, backup_stamp)
                )
                old_map.replace(backup)
                backup_messages.append('%s -> %s' % (old_map, backup))

    messages = [
        LogInfo(msg='[mapping-session] RTAB-Map database: %s' % database_path),
        LogInfo(msg='[mapping-session] Session marker: %s' % marker_path),
    ]
    if backup_messages:
        messages.append(
            LogInfo(
                msg=(
                    '[mapping-session] Previous exported map moved aside: '
                    + '; '.join(backup_messages)
                )
            )
        )
    return messages


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_livox = LaunchConfiguration('start_livox')
    sensor_profile = LaunchConfiguration('sensor_profile')
    start_camera = LaunchConfiguration('start_camera')
    publish_base_link_tf = LaunchConfiguration('publish_base_link_tf')
    rviz = LaunchConfiguration('rviz')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    delete_db_on_start = LaunchConfiguration('delete_db_on_start')

    livox_share = FindPackageShare('livox_ros_driver2')
    rtabmap_launch_share = FindPackageShare('rtabmap_launch')
    fast_lio_share = FindPackageShare('fast_lio')
    astra_share = FindPackageShare('astra_camera')
    rtabmap_rviz_config = PathJoinSubstitution([
        rtabmap_launch_share, 'launch', 'config', 'rgbd.rviz'])

    # ── Declare arguments ──
    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('sensor_profile', default_value='lidar_rgbd',
                              description='lidar_only | lidar_rgbd | lidar_stereo | lidar_mono'),
        DeclareLaunchArgument('start_livox', default_value='true',
                              description='Start Livox MID360 driver'),
        DeclareLaunchArgument('start_camera', default_value='true',
                              description='Start Orbbec depth camera driver'),
        DeclareLaunchArgument('publish_base_link_tf', default_value='true',
                              description='Publish static TF base_footprint -> base_link'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rtabmap_viz', default_value='true'),
        DeclareLaunchArgument('delete_db_on_start', default_value='true',
                              description='Delete old RTAB-Map database on startup for a clean mapping session'),
        DeclareLaunchArgument('scan_cloud_topic', default_value='/cloud_registered_body',
                              description='Input point cloud topic for RTAB-Map'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu',
                              description='Optional RTAB-Map IMU topic'),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('left_image_topic', default_value='/camera/ir/image_raw'),
        DeclareLaunchArgument('right_image_topic', default_value='/camera/ir2/image_raw'),
        DeclareLaunchArgument('left_camera_info_topic', default_value='/camera/ir/camera_info'),
        DeclareLaunchArgument('right_camera_info_topic', default_value='/camera/ir2/camera_info'),
        DeclareLaunchArgument('database_path', default_value='rtabmap_orbbec.db',
                              description='Path to save/load the RTAB-Map database'),
        DeclareLaunchArgument('frame_id', default_value='base_footprint',
                              description='Robot base frame'),
        DeclareLaunchArgument('odom_frame_id', default_value='',
                              description='RTAB-Map odometry TF frame. Keep empty to use odom_topic'),
        DeclareLaunchArgument('rtabmap_args', default_value=
            "--Reg/Strategy 2 \
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
    ]

    # ── 1. Livox MID360 driver ──
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([livox_share, 'launch', 'msg_MID360_launch.py'])),
        condition=IfCondition(PythonExpression([
            "'", start_livox, "' == 'true'"
        ])),
    )

    # ── 1.2. Orbbec depth camera driver ──
    # RGBD mode: color + depth (with depth registration)
    orbbec_rgbd_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([astra_share, 'launch', 'gemini.launch.xml'])),
        condition=IfCondition(PythonExpression([
            "'", start_camera, "' == 'true' and '",
            sensor_profile, "' == 'lidar_rgbd'"
        ])),
        launch_arguments={
            'camera_name': 'camera',
            'depth_registration': 'true',
            'color_depth_synchronization': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_ir': 'false',
            'enable_point_cloud': 'false',
            'enable_colored_point_cloud': 'false',
            'publish_tf': 'false',  # We publish our own TF
            'color_width': '640',
            'color_height': '480',
            'color_fps': '30',
            'depth_width': '640',
            'depth_height': '400',
            'depth_fps': '30',
            'depth_scale': '1',
        }.items(),
    )

    # Stereo mode: IR left + IR right (using stereo_s_u3 launch for Orbbec Stereo S U3)
    orbbec_stereo_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([astra_share, 'launch', 'stereo_s_u3.launch.xml'])),
        condition=IfCondition(PythonExpression([
            "'", start_camera, "' == 'true' and '",
            sensor_profile, "' == 'lidar_stereo'"
        ])),
        launch_arguments={
            'camera_name': 'camera',
            'depth_registration': 'false',
            'enable_color': 'false',
            'enable_depth': 'false',
            'enable_ir': 'true',  # Enable IR for stereo
            'enable_point_cloud': 'false',
            'enable_colored_point_cloud': 'false',
            'publish_tf': 'false',
            'ir_width': '640',
            'ir_height': '480',
            'ir_fps': '30',
        }.items(),
    )

    # Mono mode: color only (no depth)
    orbbec_mono_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([astra_share, 'launch', 'gemini.launch.xml'])),
        condition=IfCondition(PythonExpression([
            "'", start_camera, "' == 'true' and '",
            sensor_profile, "' == 'lidar_mono'"
        ])),
        launch_arguments={
            'camera_name': 'camera',
            'depth_registration': 'false',
            'enable_color': 'true',
            'enable_depth': 'false',
            'enable_ir': 'false',
            'enable_point_cloud': 'false',
            'enable_colored_point_cloud': 'false',
            'publish_tf': 'false',
            'color_width': '640',
            'color_height': '480',
            'color_fps': '30',
        }.items(),
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
        arguments=['0.07', '0.0', '-0.04', '0.0', '-0.63', '0.0', 'base_link', 'camera_link'],
        condition=IfCondition(PythonExpression([
            "'", start_camera, "' == 'true'"
        ])),
    )

    camera_color_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_to_camera_color_frame',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'camera_link', 'camera_color_frame'],
        condition=IfCondition(PythonExpression([
            "'", start_camera, "' == 'true'"
        ])),
    )

    camera_color_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_color_frame_to_camera_color_optical_frame',
        arguments=['0.0', '0.0', '0.0', str(-math.pi / 2.0), '0.0', str(-math.pi / 2.0),
                   'camera_color_frame', 'camera_color_optical_frame'],
        condition=IfCondition(PythonExpression([
            "'", start_camera, "' == 'true'"
        ])),
    )

    camera_depth_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_to_camera_depth_frame',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'camera_link', 'camera_depth_frame'],
        condition=IfCondition(PythonExpression([
            "'", start_camera, "' == 'true'"
        ])),
    )

    camera_depth_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_depth_frame_to_camera_depth_optical_frame',
        arguments=['0.0', '0.0', '0.0', str(-math.pi / 2.0), '0.0', str(-math.pi / 2.0),
                   'camera_depth_frame', 'camera_depth_optical_frame'],
        condition=IfCondition(PythonExpression([
            "'", start_camera, "' == 'true'"
        ])),
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
            'odom_topic': '/Odometry',
            'imu_topic': LaunchConfiguration('imu_topic'),
            'scan_cloud_topic': LaunchConfiguration('scan_cloud_topic'),
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'depth_scale': '1.0',
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'left_image_topic': LaunchConfiguration('left_image_topic'),
            'right_image_topic': LaunchConfiguration('right_image_topic'),
            'left_camera_info_topic': LaunchConfiguration('left_camera_info_topic'),
            'right_camera_info_topic': LaunchConfiguration('right_camera_info_topic'),
            'subscribe_scan_cloud': 'true',
            'subscribe_scan': 'false',
            'visual_odometry': 'false',
            'icp_odometry': 'false',
            'rviz': 'false',
            'rtabmap_viz': rtabmap_viz,
            'rgbd_sync': PythonExpression(["'true' if '", LaunchConfiguration('sensor_profile'), "' == 'lidar_rgbd' else 'false'"]),
            'subscribe_rgbd': PythonExpression(["'true' if '", LaunchConfiguration('sensor_profile'), "' == 'lidar_rgbd' else 'false'"]),
            'subscribe_rgb': PythonExpression(["'true' if '", LaunchConfiguration('sensor_profile'), "' == 'lidar_mono' else 'false'"]),
            'depth': PythonExpression(["'true' if '", LaunchConfiguration('sensor_profile'), "' == 'lidar_rgbd' else 'false'"]),
            'stereo': PythonExpression(["'true' if '", LaunchConfiguration('sensor_profile'), "' == 'lidar_stereo' else 'false'"]),
            'approx_sync': 'true',
            'qos': '2',
            'namespace': 'rtabmap',
            'args': [
                PythonExpression(["'-d ' if '", delete_db_on_start, "' == 'true' else ''"]),
                LaunchConfiguration('rtabmap_args'),
            ],
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rtabmap_rviz_config],
        condition=IfCondition(rviz),
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

    ld.add_action(OpaqueFunction(function=prepare_mapping_session))
    ld.add_action(base_tf)
    ld.add_action(livox_tf)
    ld.add_action(camera_tf)
    ld.add_action(camera_color_tf)
    ld.add_action(camera_color_optical_tf)
    ld.add_action(camera_depth_tf)
    ld.add_action(camera_depth_optical_tf)

    ld.add_action(livox_launch)
    ld.add_action(orbbec_rgbd_node)
    ld.add_action(orbbec_stereo_node)
    ld.add_action(orbbec_mono_node)
    ld.add_action(fast_lio_launch)
    ld.add_action(rviz_node)
    ld.add_action(rtabmap_launch)

    return ld
