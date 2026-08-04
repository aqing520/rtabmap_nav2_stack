#!/usr/bin/env python3
"""RTAB-Map-led AMR bringup starter.

Suggested location:
  robot_bringup/launch/bringup.launch.py

Assumptions:
- RTAB-Map is the only publisher of map -> odom.
- FAST-LIO publishes odom -> base_footprint and /Odometry.
- Nav2 consumes /map and /Odometry.
- Nav2 outputs /cmd_vel_nav → collision_monitor filters → /cmd_vel → wheeltec hardware.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare

DEFAULT_RTABMAP_ARGS = (
    "--Reg/Strategy 1 "

    "--RGBD/ProximityBySpace true "

    "--Mem/NotLinkedNodesKept false "

    "--RGBD/StartAtOrigin true "

    "--Icp/VoxelSize 0.05 "
    "--Icp/DownsamplingStep 1 "
    
    "--Icp/MaxTranslation 1.5 "
    
    "--Icp/MaxRotation 0.7 "
    "--Icp/MaxCorrespondenceDistance 0.5 "
    "--Icp/CorrespondenceRatio 0.05 "
    "--Icp/PointToPlane true "
    "--Icp/PointToPlaneK 15 "
    "--Icp/PointToPlaneMinComplexity 0.04 "
    "--Grid/CellSize 0.05 "
    "--Grid/FromDepth false "
    "--Grid/PreVoxelFiltering true "
    "--Grid/3D false "
    "--Grid/NormalsSegmentation true "
    "--Grid/NormalK 20 "
    "--Grid/MaxGroundAngle 22 "
    "--Grid/ClusterRadius 0.18 "
    "--Grid/RangeMin 0.1 "
    "--Grid/RangeMax 20.0 "
    "--Grid/FootprintLength 0.55 "
    "--Grid/FootprintWidth 0.45 "
    "--Grid/MinClusterSize 5 "
    "--Grid/NoiseFilteringRadius 0.12 "
    "--Grid/NoiseFilteringMinNeighbors 2 "
    "--Grid/MaxObstacleHeight 1.6 "
    "--Grid/MinGroundHeight -0.6 "
    "--Grid/MaxGroundHeight 0.08 "
    "--Grid/Scan2dUnknownSpaceFilled true "
    "--Grid/RayTracing true"
)


def generate_launch_description() -> LaunchDescription:
    namespace = LaunchConfiguration('namespace')
    mode = LaunchConfiguration('mode')
    sensor_profile = LaunchConfiguration('sensor_profile')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    start_livox = LaunchConfiguration('start_livox')
    start_camera = LaunchConfiguration('start_camera')
    enable_gps = LaunchConfiguration('enable_gps')
    enable_rviz = LaunchConfiguration('enable_rviz')
    enable_collision_monitor = LaunchConfiguration('enable_collision_monitor')
    publish_base_link_tf = LaunchConfiguration('publish_base_link_tf')
    delete_db_on_mapping_start = LaunchConfiguration('delete_db_on_mapping_start')
    database_path = LaunchConfiguration('database_path')
    nav2_controller = LaunchConfiguration('nav2_controller')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    use_edited_map = LaunchConfiguration('use_edited_map')
    edited_map_yaml = LaunchConfiguration('edited_map_yaml')
    start_multi_waypoint_routes = LaunchConfiguration('start_multi_waypoint_routes')
    waypoint_map_id = LaunchConfiguration('waypoint_map_id')
    waypoint_map_frame_id = LaunchConfiguration('waypoint_map_frame_id')

    robot_bringup_share = FindPackageShare('robot_bringup')
    nav2_bringup_share = FindPackageShare('nav2_bringup')
    livox_share = FindPackageShare('livox_ros_driver2')
    fast_lio_share = FindPackageShare('fast_lio')
    selected_nav2_params_file = PythonExpression([
        "'", nav2_params_file, "' if '", nav2_params_file, "' else '",
        PathJoinSubstitution([
            robot_bringup_share,
            'config',
            PythonExpression([
                "'nav2_cuda_mppi.yaml' if '", nav2_controller, "' == 'cuda_mppi' "
                "else 'nav2_mppi.yaml' if '", nav2_controller, "' == 'mppi' "
                "else 'nav2_dwb.yaml'"
            ]),
        ]),
        "'"
    ])
    effective_edited_map_yaml = PythonExpression([
        "'", edited_map_yaml, "' if '", edited_map_yaml,
        "' else __import__('os').path.join(__import__('os').path.dirname('",
        database_path, "'), 'map.yaml')"
    ])

    declare_args = [
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('mode', default_value='navigation', description='mapping | localization | navigation'),
        DeclareLaunchArgument('sensor_profile', default_value='lidar_only', description='lidar_only | lidar_rgbd | lidar_stereo | lidar_mono'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('start_livox', default_value='true', description='Start Livox MID360 launch'),
        DeclareLaunchArgument('start_camera', default_value='false', description='Start Orbbec Gemini/Astra RGB-D camera'),
        DeclareLaunchArgument('enable_gps', default_value='false', description='Enable navsat_transform and pass GPS fix to RTAB-Map'),
        DeclareLaunchArgument('enable_rviz', default_value='false', description='Launch RViz with Nav2 navigation config'),
        DeclareLaunchArgument('enable_collision_monitor', default_value='true', description='Filter Nav2 /cmd_vel through collision_monitor'),
        DeclareLaunchArgument('publish_base_link_tf', default_value='true', description='Publish a zero static TF from base_footprint to base_link if URDF is not ready'),
        DeclareLaunchArgument(
            'delete_db_on_mapping_start',
            default_value='true',
            description='Delete the RTAB-Map database automatically when mode:=mapping. Ignored for localization/navigation.',
        ),
        DeclareLaunchArgument('database_path', default_value='/data/maps/site_a/rtabmap.db'),
        DeclareLaunchArgument(
            'use_edited_map',
            default_value='true',
            description=(
                'Publish /map from the exported offline map instead of '
                'RTAB-Map during navigation.'
            ),
        ),
        DeclareLaunchArgument(
            'edited_map_yaml',
            default_value='',
            description=(
                'Static map YAML used when use_edited_map is true. An empty '
                'value selects map.yaml next to database_path.'
            ),
        ),
        DeclareLaunchArgument('rtabmap_args', default_value=DEFAULT_RTABMAP_ARGS),
        DeclareLaunchArgument('nav2_controller', default_value='cuda_mppi', description='Nav2 local controller: dwb | mppi | cuda_mppi'),
        DeclareLaunchArgument('nav2_params_file', default_value='', description='Optional Nav2 params file. Overrides nav2_controller when set.'),
        DeclareLaunchArgument('start_multi_waypoint_routes', default_value='false', description='Start RViz clicked-point waypoint manager'),
        DeclareLaunchArgument('waypoint_map_id', default_value='site_a', description='Map name/id attached to RViz waypoint collection'),
        DeclareLaunchArgument('waypoint_map_frame_id', default_value='map', description='Required frame_id for RViz clicked points'),
        DeclareLaunchArgument('rtabmap_frame_id', default_value='base_footprint'),
        DeclareLaunchArgument('rtabmap_map_frame', default_value='map'),
        DeclareLaunchArgument('rtabmap_odom_topic', default_value='/Odometry'),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/unused_imu',
            description='Optional IMU topic for RTAB-Map and navsat_transform when GPS is enabled.',
        ),
        DeclareLaunchArgument('gps_fix_topic', default_value='/sensors/gps/fix'),
        DeclareLaunchArgument('scan_cloud_topic', default_value='/cloud_registered_body'),
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
        DeclareLaunchArgument('rtk_port', default_value='auto', description='BT-468 serial port, e.g. /dev/ttyUSB0 or "auto"'),
        DeclareLaunchArgument('rtk_baud', default_value='38400', description='BT-468 baud rate'),
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

    # ── 4. BT-468 RTK driver (started when enable_gps=true) ──
    btk_rtk_node = Node(
        package='bt468_rtk_driver',
        executable='bt468_rtk_node',
        name='bt468_rtk_node',
        output='screen',
        condition=IfCondition(enable_gps),
        parameters=[{
            'port': LaunchConfiguration('rtk_port'),
            'baud': LaunchConfiguration('rtk_baud'),
            'timeout_sec': 0.2,
            'reconnect_delay_sec': 1.0,
            'frame_id': 'gnss_link',
            'log_summary': True,
        }],
        remappings=[
            ('fix', LaunchConfiguration('gps_fix_topic')),
        ],
    )

    # ── 4b. GPS fusion (optional, requires robot_localization installed separately) ──
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
    rtabmap_map_topic = PythonExpression([
        "'/rtabmap/map_raw' if '", mode, "' == 'navigation' and '",
        use_edited_map, "' == 'true' else 'map'"
    ])
    rtabmap_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([robot_bringup_share, 'launch', 'rtabmap_bridge.launch.py'])),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'sensor_profile': sensor_profile,
            'enable_gps': enable_gps,
            'localization': PythonExpression(["'true' if '", mode, "' != 'mapping' else 'false'"]),
            'database_path': database_path,
            'rtabmap_args': LaunchConfiguration('rtabmap_args'),
            'frame_id': LaunchConfiguration('rtabmap_frame_id'),
            'map_frame_id': LaunchConfiguration('rtabmap_map_frame'),
            'map_topic': rtabmap_map_topic,
            'odom_topic': LaunchConfiguration('rtabmap_odom_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'gps_topic': LaunchConfiguration('gps_fix_topic'),
            'scan_cloud_topic': LaunchConfiguration('scan_cloud_topic'),
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'delete_db_on_start': PythonExpression([
                "'true' if '", mode, "' == 'mapping' and '",
                delete_db_on_mapping_start, "' == 'true' else 'false'"
            ]),
            'rviz': 'false',
        }.items(),
    )

    edited_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'navigation' and '", use_edited_map, "' == 'true'"
        ])),
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': effective_edited_map_yaml,
        }],
    )

    edited_map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_edited_map',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'navigation' and '", use_edited_map, "' == 'true'"
        ])),
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server'],
        }],
    )

    # ── 5b. RViz with Nav2 navigation config ──
    nav2_rviz_config = PathJoinSubstitution([robot_bringup_share, 'config', 'nav2_navigation.rviz'])
    show_rviz = PythonExpression([
        "'true' if '", enable_rviz, "' == 'true' or '",
        start_multi_waypoint_routes, "' == 'true' else 'false'"
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(show_rviz),
        arguments=['-d', nav2_rviz_config],
    )
    # ── 6. Nav2 ──
    # Remap /cmd_vel → /cmd_vel_nav so collision_monitor can intercept before the hardware.
    nav2_launch = GroupAction(
        condition=IfCondition(PythonExpression(["'", mode, "' == 'navigation'"])),
        actions=[
            SetRemap('/cmd_vel', '/cmd_vel_nav', condition=IfCondition(enable_collision_monitor)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([nav2_bringup_share, 'launch', 'navigation_launch.py'])),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': selected_nav2_params_file,
                    'use_composition': 'False',
                    'use_respawn': 'False',
                    'log_level': 'info',
                }.items(),
            ),
        ],
    )

    # ── 7. Collision Monitor ──
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'navigation' and '", enable_collision_monitor, "' == 'true'"
        ])),
        parameters=[{
            'use_sim_time': use_sim_time,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'cmd_vel_in_topic': '/cmd_vel_nav',
            'cmd_vel_out_topic': '/cmd_vel',
            'transform_tolerance': 1.0,
            'source_timeout': 1.0,
            'base_shift_correction': True,
            'stop_pub_timeout': 1.0,
            'polygons': ['StopZone', 'SlowZone'],
            'observation_sources': ['pointcloud'],
            'StopZone.type': 'polygon',
            'StopZone.points': [0.30, 0.24, 0.30, -0.24, -0.10, -0.24, -0.10, 0.24],
            'StopZone.action_type': 'stop',
            'StopZone.max_points': 3,
            'StopZone.visualize': True,
            'StopZone.polygon_pub_topic': 'collision_monitor/stop_zone',
            'StopZone.enabled': True,
            'SlowZone.type': 'polygon',
            'SlowZone.points': [0.80, 0.38, 0.80, -0.38, -0.20, -0.38, -0.20, 0.38],
            'SlowZone.action_type': 'slowdown',
            'SlowZone.max_points': 3,
            'SlowZone.slowdown_ratio': 0.45,
            'SlowZone.visualize': True,
            'SlowZone.polygon_pub_topic': 'collision_monitor/slow_zone',
            'SlowZone.enabled': True,
            'pointcloud.type': 'pointcloud',
            'pointcloud.topic': '/cloud_registered_body',
            'pointcloud.min_height': 0.12,
            'pointcloud.max_height': 1.00,
            'pointcloud.enabled': True,
        }],
    )

    collision_monitor_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision_monitor',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'navigation' and '", enable_collision_monitor, "' == 'true'"
        ])),
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['collision_monitor'],
        }],
    )

    multi_waypoint_route_node = Node(
        package='robot_bringup',
        executable='multi_waypoint_route_node.py',
        name='multi_waypoint_route',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'navigation' and '", start_multi_waypoint_routes, "' == 'true'"
        ])),
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_id': waypoint_map_id,
            'map_frame_id': waypoint_map_frame_id,
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
    ld.add_action(camera_launch)
    ld.add_action(btk_rtk_node)
    ld.add_action(navsat_transform)
    ld.add_action(rtabmap_bridge)
    ld.add_action(edited_map_server)
    ld.add_action(edited_map_lifecycle)
    ld.add_action(rviz_node)
    ld.add_action(nav2_launch)
    ld.add_action(collision_monitor)
    ld.add_action(collision_monitor_lifecycle)
    ld.add_action(multi_waypoint_route_node)
    return ld
