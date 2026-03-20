# RTAB-Map 导航项目（从零到一）
官方链接：https://github.com/introlab/rtabmap_ros

本仓库用于落地一套分层清晰的 ROS2 导航方案：

- `RTAB-Map` 负责全局建图/回环/重定位（发布 `map -> odom`）
- `robot_localization (EKF)` 负责本地连续里程计（发布 `odom -> base_footprint`）
- `Nav2` 负责规划、控制与避障执行

## 1. 仓库结构

```text
rtabmap_nav2_stack/                 # 项目根目录
├── src/                            # ROS 2 工作空间源码目录
│   ├── robot_bringup/              # (未启用) 高级导航启动包组，包含 EKF 和 Nav2 的精细配置
│   │   ├── launch/                 # 启动脚本存放目录
│   │   │   ├── bringup.launch.py   # 总启动入口，控制全自动导航的各个模块
│   │   │   └── rtabmap_bridge.launch.py # 负责将 RTAB-Map 的输出桥接到 Nav2 栈
│   │   └── config/                 # 核心参数配置目录
│   │       ├── ekf_local.yaml      # 扩展卡尔曼滤波(EKF)配置，用于融合 Lidar, IMU, 轮速里程计
│   │       └── nav2_common.yaml    # Navigation 2 配置，包含 MPPI 局部规划器与成本地图参数
│   └── rtabmap_ros/                # 官方 RTAB-Map ROS 2 包装层代码副本
│       ├── rtabmap_launch/         # RTAB-Map 核心 Launch 文件
│       ├── rtabmap_slam/           # SLAM 核心节点，负责构图与回环检测
│       ├── rtabmap_odom/           # 视觉/激光里程计计算节点
│       ├── rtabmap_sync/           # 多传感器数据（雷达、RGBD、IMU）时间同步节点
│       ├── rtabmap_util/           # 点云和图像处理工具集节点
│       ├── rtabmap_msgs/           # RTAB-Map 自定义 ROS 2 消息类型定义
│       ├── rtabmap_rviz_plugins/   # RViz 界面中用于显示 RTAB-Map 数据的图形插件
│       ├── rtabmap_examples/       # 单体传感器（如 Realsense, ZXing）的使用演示
│       └── rtabmap_demos/          # 完整机器人的离线建图仿真与演示程序
├── third_party/                    # 第三方依赖库目录
│   └── rtabmap-0.23.4/             # RTAB-Map C++ 核心算法源码，保证版本一致性
├── scripts/                        # 工业级自动化运维与建图脚本集合（目前主用的建图入口）
│   ├── build_rtabmap_0234.sh       # 隔离编译 RTAB-Map 核心层脚本
│   └── use_rtabmap_0234_env.sh     # 供 colcon 编译时挂载核心库路径的环境脚本
└── rtabmap_solution_package.pdf    # 项目方案说明文档
```

## 2. 编译顺序（推荐）

'''
do_build_all.sh  直接一键编译环境
'''
### 2.1 先编译本地 RTABMap 0.23.4

> 你的 `rtabmap_ros` 已要求 `find_package(RTABMap 0.23.4 REQUIRED)`，
> 因此建议先编译 `third_party/rtabmap-0.23.4`。

```bash
cd ~/rtabmap_nav2_stack
JOBS=4 ./scripts/build_rtabmap_0234.sh
source ./scripts/use_rtabmap_0234_env.sh
```

### 2.2 再编译工作区

```bash
cd ~/rtabmap_nav2_stack
source /opt/ros/humble/setup.bash
source ./scripts/use_rtabmap_0234_env.sh

export MAKEFLAGS="-j4 -l4"
export CMAKE_BUILD_PARALLEL_LEVEL=4

colcon build \
  --symlink-install \
  --executor parallel \
  --parallel-workers 4 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

## 3. 三种运行模式

### 3.1 建图模式

```bash
ros2 launch robot_bringup bringup.launch.py \
  mode:=mapping sensor_profile:=lidar_rgbd \
  database_path:=/data/maps/site_a/rtabmap.db
```

### 3.2 定位模式

```bash
ros2 launch robot_bringup bringup.launch.py \
  mode:=localization sensor_profile:=lidar_rgbd \
  database_path:=/data/maps/site_a/rtabmap.db
```

### 3.3 导航模式

```bash
ros2 launch robot_bringup bringup.launch.py \
  mode:=navigation sensor_profile:=lidar_rgbd enable_gps:=true \
  database_path:=/data/maps/site_a/rtabmap.db
```

## 4. 最小验收命令

```bash
ros2 node list
ros2 topic hz /odometry/local
ros2 topic hz /map
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

导航模式再检查：

```bash
ros2 topic hz /cmd_vel_safe
```

## 5. 两条核心工程纪律

1. 只允许 RTAB-Map 发布 `map -> odom`
2. `local_costmap` 只能在 `odom`，`global_costmap` 只能在 `map`

## 6. 关键文件入口

- `src/robot_bringup/launch/bringup.launch.py`：总入口（按 `mode` 切换）
- `src/robot_bringup/launch/rtabmap_bridge.launch.py`：RTAB-Map 包装层
- `src/robot_bringup/config/ekf_local.yaml`：本地状态估计参数
- `src/robot_bringup/config/nav2_common.yaml`：导航参数骨架
- `src/rtabmap_ros/README.md`：`rtabmap_ros` 中文说明与细节

## 7. 常见问题

1. 找不到 `RTABMap 0.23.4`
先执行 `./scripts/build_rtabmap_0234.sh`，再 `source ./scripts/use_rtabmap_0234_env.sh`。

2. `robot_bringup` 找不到
检查是否成功 `colcon build` 且已 `source install/setup.bash`。

3. 没有 `/map` 或 `map->odom`
检查 RTAB-Map 输入话题（里程计/点云/视觉）和 TF 外参是否完整。

4. 轨迹发散或跳变
优先检查时间同步、IMU 质量、传感器外参、EKF 配置对应关系。

---

建议上手顺序：

1. 先 `mapping` 跑通并确认数据库落盘
2. 再 `localization` 验证 `map->odom` 稳定
3. 最后 `navigation` 联调到底盘控制
