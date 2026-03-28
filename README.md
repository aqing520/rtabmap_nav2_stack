# RTAB-Map 导航项目（从零到一）
官方链接：https://github.com/introlab/rtabmap_ros

本仓库用于落地一套分层清晰的 ROS2 导航方案：

- `FAST-LIO` 负责雷达惯性里程计（发布 `odom -> base_footprint`）
- `RTAB-Map` 负责全局建图/回环/重定位（发布 `map -> odom`）
- `Nav2` 负责规划、控制与避障执行
## 0.编译
```bash
do_build_all.sh  #直接编译就行
```

## 1. 仓库结构

```text
rtabmap_nav2_stack/                 # 工作空间
├── src/                            # ROS 2 工作空间源码目录
│   ├── robot_bringup/              # 高级导航启动包组，包含 Nav2 的精细配置
│   │   ├── launch/                 # 启动脚本存放目录
│   │   │   ├── fastlio_mapping.launch.py  # 建图入口（FAST-LIO + RTAB-Map，可选 RTK）
│   │   │   └── nav2.launch.py             # 导航入口（定位 + Nav2，可选 RTK）
│   │   └── config/                 # 核心参数配置目录
│   │       └── nav2_common.yaml    # Navigation 2 配置
│   ├── rtk-driver/                 # BT-468 RTK GNSS 驱动（串口读取 NMEA，发布 /fix）
│   ├── FAST_LIO_ROS2/              # FAST-LIO 激光惯性紧耦合里程计（提供 odom 和去畸变点云）
│   ├── livox_ros_driver2/          # Livox MID360 雷达驱动
│   └── rtabmap_ros/                # 官方 RTAB-Map ROS 2 包装层
│       ├── rtabmap_launch/         # （done）通用 Launch 入口，  目前的脚本走的就是这个
│       ├── rtabmap_slam/           # （done）SLAM 核心节点，负责建图、回环检测、图优化与重定位
│       ├── rtabmap_odom/           # （done）视觉/深度/激光前端里程计节点，提供局部连续位姿估计
│       ├── rtabmap_sync/           # （done）多传感器时间同步节点，将 RGB、Depth、Scan、IMU 整理成统一输入
│       ├── rtabmap_util/           # （done）点云、栅格、图像和 TF 处理工具节点集合
│       ├── rtabmap_msgs/           # （done）RTAB-Map  ROS 2 消息与服务类型定义
│       ├── rtabmap_conversions/    # （done）RTAB-Map C++ 核心数据结构与 ROS 消息/TF/OpenCV 之间的转换库
│       ├── rtabmap_rviz_plugins/   # （done）RViz 中显示地图图结构、点云、回环和调试信息的插件
│       ├── rtabmap_viz/            # （done）RTAB-Map 自带可视化界面节点，便于查看节点图、回环和局部地图
│       ├── rtabmap_costmap_plugins/# （done）给 Nav2提供3D体素地图能力，
│       ├── rtabmap_python/         # （done）Python 绑定与脚本接口，便于离线分析和轻量二次开发
│       ├── rtabmap_ros/            # （done）元包/聚合包，用于统一依赖导出和整体发布
│       ├── rtsp_camera_bridge/     # （不用理，不是原生包）RTSP 相机桥接节点，把网络视频流接入 ROS 图像话题
│       ├── rtabmap_examples/       # （done）单体传感器或典型设备（如 Realsense）的使用示例
│       └── rtabmap_demos/          # （done）完整机器人的离线建图仿真与演示程序
├── third_party/                    #
│   └── rtabmap-0.23.4/             # RTAB-Map C++ 核心算法源码，保证版本一致性
├── scripts/                        # 编译和配置脚本
│   ├── build_rtabmap_0234.sh       # （done）隔离编译 RTAB-Map 核心层脚本
│   └── use_rtabmap_0234_env.sh     # （done）供 colcon 编译时挂载核心库路径的环境脚本
```

## 2.建图过程  

建图统一入口为 `fastlio_mapping.launch.py`，RTK GPS 通过 `enable_gps` 参数可选开启（默认关闭）。

```bash
# 纯 LiDAR 建图（默认）
ros2 launch robot_bringup fastlio_mapping.launch.py

# 开启 RTK GPS 建图
ros2 launch robot_bringup fastlio_mapping.launch.py enable_gps:=true
```

### 2.1  关键数据流向

```mermaid
flowchart LR
    CLOUD["/livox/lidar"] --> FASTLIO["FAST-LIO"]
    IMU1["/livox/imu"] --> FASTLIO
    FASTLIO --> ODOM["/Odometry"]
    FASTLIO --> CLOUD_REG["/cloud_registered_body"]

    ODOM --> SLAM["rtabmap_slam"]
    CLOUD_REG --> SLAM
    SLAM --> MAP["/map"]
    SLAM --> TF1["TF: map → odom"]
    SLAM --> DB1["rtabmap.db"]

    subgraph GPS_OPT["可选: RTK GPS"]
        RTK["bt468_rtk_node\n串口 NMEA"] --> FIX["/fix\nNavSatFix"]
        FIX --> NAVSAT["navsat_transform_node"]
        NAVSAT --> ODOM_GPS["/odometry/gps"]
    end

    FIX -.-> SLAM
```

- FAST-LIO 直接消费 `/livox/lidar` 和 `/livox/imu`，输出激光惯性紧耦合里程计 `/Odometry`，同时发布 `odom -> base_footprint` TF
- FAST-LIO 还输出去运动畸变后的点云 `/cloud_registered_body`，供 RTAB-Map 做回环检测和建图
- `rtabmap_slam` 消费 `/Odometry` 和 `/cloud_registered_body`，执行图优化并发布 `map -> odom` TF
- **可选**：启用 RTK 后，`/fix` 作为 GPS 弱约束传入 RTAB-Map，为大范围室外建图提供全局锚点，防止累积漂移

### 2.2  TF 主链

```text
map → odom → base_footprint → base_link → livox_frame
 ↑      ↑        (static)       (static)
rtabmap FAST-LIO

                 base_link → gnss_link    (仅 enable_gps=true 时发布)
```

## 3.导航过程

导航统一入口为 `nav2.launch.py`，RTK GPS 为可选模式。

```bash
# 不带 GPS 导航
ros2 launch robot_bringup nav2.launch.py

# 带 RTK GPS 导航
ros2 launch robot_bringup nav2.launch.py enable_gps:=true

# 指定地图数据库
ros2 launch robot_bringup nav2.launch.py database_path:=/data/maps/my_site/rtabmap.db
```

### 3.1  关键数据流向

```mermaid
flowchart LR
    CLOUD["/livox/lidar"] --> FASTLIO["FAST-LIO"]
    IMU["/livox/imu"] --> FASTLIO
    FASTLIO --> ODOM["/Odometry"]
    FASTLIO --> CLOUD_REG["/cloud_registered_body"]

    ODOM --> SLAM["rtabmap_slam\n定位模式"]
    CLOUD_REG --> SLAM
    SLAM --> MAP["/map"]
    SLAM --> TF_MAP["TF: map → odom"]

    MAP --> NAV2["Nav2\n规划 + 控制"]
    ODOM --> NAV2
    NAV2 --> CMD["/cmd_vel"]

    CMD --> CM["collision_monitor"]
    CLOUD_REG --> CM
    CM --> CMD_SAFE["/cmd_vel_safe\n→ 底盘"]

    subgraph GPS_OPT["可选: RTK GPS"]
        RTK["bt468_rtk_node"] --> FIX["/fix"]
        FIX --> NAVSAT["navsat_transform_node"]
    end

    FIX -.-> SLAM
```

### 3.2  主要组件职责

| 组件 | 职责 |
|------|------|
| FAST-LIO | 激光惯性里程计，发布 `/Odometry` 和去畸变点云 |
| RTAB-Map（定位模式） | 基于已有 `rtabmap.db` 重定位，发布 `map -> odom` TF 和 `/map` |
| Nav2 | 全局/局部规划 + 路径跟踪（MPPI 控制器）|
| Collision Monitor | 碰撞安全层，将 `/cmd_vel` 过滤为 `/cmd_vel_safe` |
| RTK GPS（可选） | 提供全局 GPS 约束，辅助重定位和防漂移 |
