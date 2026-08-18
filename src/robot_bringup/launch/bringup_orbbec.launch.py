#!/usr/bin/env python3
"""RTAB-Map-led AMR bringup starter with Orbbec depth camera.

Assumptions:
- RTAB-Map is the only publisher of map -> odom.
- FAST-LIO publishes odom -> base_footprint and /Odometry.
- Nav2 consumes /map and /Odometry.
- Nav2 outputs /cmd_vel directly to the wheeltec hardware subscriber.
"""

import math
import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _workspace_root(package_name: str) -> Path:
    """Return the colcon workspace containing an installed package."""
    prefix = Path(get_package_prefix(package_name))
    if prefix.name == 'install':
        return prefix.parent
    if prefix.parent.name == 'install':
        return prefix.parent.parent
    return Path.cwd()


def generate_launch_description() -> LaunchDescription:
    workspace_root = _workspace_root('robot_bringup')

    namespace = LaunchConfiguration('namespace')
    mode = LaunchConfiguration('mode')
    sensor_profile = LaunchConfiguration('sensor_profile')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    enable_startup_localization_guard = LaunchConfiguration('enable_startup_localization_guard')
    start_livox = LaunchConfiguration('start_livox')
    start_camera = LaunchConfiguration('start_camera')
    enable_gps = LaunchConfiguration('enable_gps')
    enable_rviz = LaunchConfiguration('enable_rviz')
    enable_rtabmap_viz = LaunchConfiguration('enable_rtabmap_viz')
    publish_base_link_tf = LaunchConfiguration('publish_base_link_tf')
    database_path = LaunchConfiguration('database_path')
    effective_database_path = PathJoinSubstitution([
        str(workspace_root),
        database_path,
    ])
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    use_edited_map = LaunchConfiguration('use_edited_map')
    edited_map_yaml = LaunchConfiguration('edited_map_yaml')

    robot_bringup_share = FindPackageShare('robot_bringup')
    nav2_bringup_share = FindPackageShare('nav2_bringup')
    livox_share = FindPackageShare('livox_ros_driver2')
    fast_lio_share = FindPackageShare('fast_lio')
    astra_share = FindPackageShare('astra_camera')
    effective_autostart = PythonExpression([
        "'false' if '", enable_startup_localization_guard,
        "' == 'true' else '", autostart, "'"
    ])
    effective_rtabmap_args = [
        LaunchConfiguration('rtabmap_args'),
        PythonExpression([
            "' --Rtabmap/LoopThr ", LaunchConfiguration('startup_min_hypothesis'),
            "' if '", enable_startup_localization_guard, "' == 'true' else ''"
        ]),
    ]
    effective_edited_map_yaml = PythonExpression([
        "'", edited_map_yaml, "' if '", edited_map_yaml,
        "' else __import__('os').path.join(__import__('os').path.dirname('",
        effective_database_path, "'), 'pgm_map', 'map.yaml')"
    ])
    bt_xml_path = PathJoinSubstitution([
        robot_bringup_share,
        'config',
        'navigate_to_pose_clear_costmaps_on_goal_start.xml',
    ])
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={
            'default_nav_to_pose_bt_xml': bt_xml_path,
        },
        convert_types=True,
    )
    declare_args = [
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('mode', default_value='navigation', description='mapping | localization | navigation'),
        DeclareLaunchArgument('sensor_profile', default_value='lidar_rgbd', description='lidar_only | lidar_rgbd | lidar_stereo | lidar_mono'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'enable_startup_localization_guard',
            default_value='false',
            description='Keep Nav2 inactive until RTAB-Map accepts a startup visual localization.',
        ),
        DeclareLaunchArgument('startup_sensor_wait_timeout', default_value='30.0'),
        DeclareLaunchArgument('startup_localization_timeout', default_value='20.0'),
        DeclareLaunchArgument('startup_service_wait_timeout', default_value='30.0'),
        DeclareLaunchArgument(
            'startup_min_hypothesis',
            default_value='0.11',
            description=(
                'Minimum RTAB-Map visual hypothesis. When the startup guard is '
                'enabled, this is applied to both Rtabmap/LoopThr and the guard.'
            ),
        ),
        DeclareLaunchArgument('startup_min_visual_inliers', default_value='15'),
        DeclareLaunchArgument('startup_min_visual_inliers_ratio', default_value='0.0'),
        DeclareLaunchArgument(
            'startup_min_best_second_ratio',
            default_value='1.0',
            description='Minimum best/second posterior ratio. 1.0 disables this extra check.',
        ),
        DeclareLaunchArgument('startup_max_optimization_error_ratio', default_value='3.0'),
        DeclareLaunchArgument('startup_max_linear_variance', default_value='1.0'),
        DeclareLaunchArgument('startup_max_yaw_variance', default_value='1.0'),
        DeclareLaunchArgument('startup_required_confirmations', default_value='1'),
        DeclareLaunchArgument(
            'allow_last_pose_fallback',
            default_value='false',
            description='Activate Nav2 with the last database pose when startup visual localization times out.',
        ),
        DeclareLaunchArgument(
            'startup_info_topic',
            default_value='info',
            description='RTAB-Map Info topic, relative to namespace unless an absolute name is supplied.',
        ),
        DeclareLaunchArgument(
            'startup_localization_pose_topic',
            default_value='localization_pose',
            description='RTAB-Map localization pose topic.',
        ),
        DeclareLaunchArgument(
            'navigation_manager_service',
            default_value='/lifecycle_manager_navigation/manage_nodes',
        ),
        DeclareLaunchArgument('start_livox', default_value='false', description='Start Livox MID360 launch'),
        DeclareLaunchArgument('start_camera', default_value='false', description='Start Orbbec depth camera launch'),
        DeclareLaunchArgument('enable_gps', default_value='false', description='Enable navsat_transform and pass GPS fix to RTAB-Map'),
        DeclareLaunchArgument('enable_rviz', default_value='true', description='Launch RViz with Nav2 navigation config'),
        DeclareLaunchArgument('enable_rtabmap_viz', default_value='false', description='Launch the RTAB-Map visualization window'),
        DeclareLaunchArgument('publish_base_link_tf', default_value='true', description='Publish a zero static TF from base_footprint to base_link if URDF is not ready'),
        DeclareLaunchArgument(
            'database_path',
            default_value='rtabmap_orbbec.db',
            description='RTAB-Map database under the current workspace by default.',
        ),
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
        # CUDA MPPI requires cuda_mppi_controller to be present in the sourced
        # overlay.  If it is missing, controller_server fails to configure and
        # the Nav2 lifecycle manager aborts before the global costmap is
        # activated.  Make sure to source cuda_robotics_ws before launching.
        DeclareLaunchArgument('nav2_params_file', default_value=PathJoinSubstitution([robot_bringup_share, 'config', 'nav2_cuda_mppi.yaml'])),
        DeclareLaunchArgument(
            'use_edited_map',
            default_value='true',
            description=(
                'During navigation, publish /map from the exported offline map '
                'and move the RTAB-Map occupancy-grid output to /rtabmap/map_raw.'
            ),
        ),
        DeclareLaunchArgument(
            'edited_map_yaml',
            default_value='',
            description=(
                'Static map YAML used when use_edited_map is true. An empty '
                'value selects pgm_map/map.yaml next to database_path.'
            ),
        ),
        DeclareLaunchArgument('rtabmap_frame_id', default_value='base_footprint'),
        DeclareLaunchArgument('rtabmap_map_frame', default_value='map'),
        DeclareLaunchArgument('rtabmap_odom_topic', default_value='/Odometry'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu', description='Optional IMU topic for RTAB-Map and navsat_transform when GPS is enabled.'),
        DeclareLaunchArgument('gps_fix_topic', default_value='/sensors/gps/fix'),
        DeclareLaunchArgument('scan_cloud_topic', default_value='/cloud_registered_body'),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('left_image_topic', default_value='/camera/ir/image_raw'),
        DeclareLaunchArgument('right_image_topic', default_value='/camera/ir2/image_raw'),
        DeclareLaunchArgument('left_camera_info_topic', default_value='/camera/ir/camera_info'),
        DeclareLaunchArgument('right_camera_info_topic', default_value='/camera/ir2/camera_info'),
    ]

    # ── 1. Livox MID360 driver ──
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([livox_share, 'launch', 'msg_MID360_launch.py'])),
        condition=IfCondition(
            PythonExpression(["'", start_livox, "' == 'true'"])
        ),
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
    #   相机 TF (根据实际安装位置修改平移和旋转参数，单位为米和弧度)
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera_link',
        arguments=['0.07', '0.0', '-0.04', '0.0', '-0.63', '0.0', 'base_link', 'camera_link'],
        condition=IfCondition(PythonExpression([
            "'", sensor_profile, "' != 'lidar_only' "
        ])),
    )

    camera_color_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_to_camera_color_frame',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'camera_link', 'camera_color_frame'],
        condition=IfCondition(PythonExpression([
            "'", sensor_profile, "' != 'lidar_only' "
        ])),
    )

    camera_color_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_color_frame_to_camera_color_optical_frame',
        arguments=['0.0', '0.0', '0.0', str(-math.pi / 2.0), '0.0', str(-math.pi / 2.0),
                   'camera_color_frame', 'camera_color_optical_frame'],
        condition=IfCondition(PythonExpression([
            "'", sensor_profile, "' != 'lidar_only' "
        ])),
    )

    camera_depth_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_to_camera_depth_frame',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'camera_link', 'camera_depth_frame'],
        condition=IfCondition(PythonExpression([
            "'", sensor_profile, "' != 'lidar_only' "
        ])),
    )

    camera_depth_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_depth_frame_to_camera_depth_optical_frame',
        arguments=['0.0', '0.0', '0.0', str(-math.pi / 2.0), '0.0', str(-math.pi / 2.0),
                   'camera_depth_frame', 'camera_depth_optical_frame'],
        condition=IfCondition(PythonExpression([
            "'", sensor_profile, "' != 'lidar_only' "
        ])),
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
            'database_path': effective_database_path,
            'rtabmap_args': effective_rtabmap_args,
            'frame_id': LaunchConfiguration('rtabmap_frame_id'),
            'map_frame_id': LaunchConfiguration('rtabmap_map_frame'),
            'map_topic': rtabmap_map_topic,
            'odom_topic': LaunchConfiguration('rtabmap_odom_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'gps_topic': LaunchConfiguration('gps_fix_topic'),
            'scan_cloud_topic': LaunchConfiguration('scan_cloud_topic'),
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'depth_scale': '1.0',
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'left_image_topic': LaunchConfiguration('left_image_topic'),
            'right_image_topic': LaunchConfiguration('right_image_topic'),
            'left_camera_info_topic': LaunchConfiguration('left_camera_info_topic'),
            'right_camera_info_topic': LaunchConfiguration('right_camera_info_topic'),
            'rviz': 'false',
            'rtabmap_viz': enable_rtabmap_viz,
        }.items(),
    )

    # Keep the edited map server independent from the Nav2 activation gate.
    # The visual-initial-pose workflow intentionally starts Nav2 with
    # autostart=false, but /map must already be available when Nav2 is
    # activated after visual localization succeeds.
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
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    # ── 5b. RViz with Nav2 navigation config ──
    nav2_rviz_config = PathJoinSubstitution([robot_bringup_share, 'config', 'nav2_navigation.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(enable_rviz),
        arguments=['-d', nav2_rviz_config],
    )
    # ── 5c. Startup localization gate ──
    # RTAB-Map keeps running as the localization backend. This one-shot
    # supervisor only delays Nav2 activation until RTAB-Map accepts a global
    # visual loop closure and publishes a localization pose with sane
    # covariance.
    startup_localization_guard = Node(
        package='robot_bringup',
        executable='rtabmap_startup_localization_guard.py',
        name='rtabmap_startup_localization_guard',
        namespace=namespace,
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'navigation' and '",
            enable_startup_localization_guard, "' == 'true'"
        ])),
        parameters=[{
            'use_sim_time': use_sim_time,
            'info_topic': LaunchConfiguration('startup_info_topic'),
            'localization_pose_topic': LaunchConfiguration('startup_localization_pose_topic'),
            'navigation_manager_service': LaunchConfiguration('navigation_manager_service'),
            'sensor_wait_timeout_sec': ParameterValue(
                LaunchConfiguration('startup_sensor_wait_timeout'), value_type=float),
            'localization_timeout_sec': ParameterValue(
                LaunchConfiguration('startup_localization_timeout'), value_type=float),
            'service_wait_timeout_sec': ParameterValue(
                LaunchConfiguration('startup_service_wait_timeout'), value_type=float),
            'min_hypothesis': ParameterValue(
                LaunchConfiguration('startup_min_hypothesis'), value_type=float),
            'min_visual_inliers': ParameterValue(
                LaunchConfiguration('startup_min_visual_inliers'), value_type=int),
            'min_visual_inliers_ratio': ParameterValue(
                LaunchConfiguration('startup_min_visual_inliers_ratio'), value_type=float),
            'min_best_second_ratio': ParameterValue(
                LaunchConfiguration('startup_min_best_second_ratio'), value_type=float),
            'max_optimization_error_ratio': ParameterValue(
                LaunchConfiguration('startup_max_optimization_error_ratio'), value_type=float),
            'max_linear_variance': ParameterValue(
                LaunchConfiguration('startup_max_linear_variance'), value_type=float),
            'max_yaw_variance': ParameterValue(
                LaunchConfiguration('startup_max_yaw_variance'), value_type=float),
            'required_confirmations': ParameterValue(
                LaunchConfiguration('startup_required_confirmations'), value_type=int),
            'allow_last_pose_fallback': ParameterValue(
                LaunchConfiguration('allow_last_pose_fallback'), value_type=bool),
        }],
    )

    # ── 6. Nav2 ──
    nav2_launch = GroupAction(
        condition=IfCondition(PythonExpression(["'", mode, "' == 'navigation'"])),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([nav2_bringup_share, 'launch', 'navigation_launch.py'])),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'autostart': effective_autostart,
                    'params_file': configured_nav2_params,
                    'use_composition': 'False',
                    'use_respawn': 'False',
                    'log_level': 'info',
                }.items(),
            ),
        ],
    )

    # ── Assemble ──
    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

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
    ld.add_action(navsat_transform)
    ld.add_action(rtabmap_bridge)
    ld.add_action(edited_map_server)
    ld.add_action(edited_map_lifecycle)
    ld.add_action(rviz_node)
    ld.add_action(startup_localization_guard)
    ld.add_action(nav2_launch)
    return ld
