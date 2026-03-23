# 测试启动说
使用脚本启动

## 环境信息

- 平台: Jetson (aarch64), Linux 5.15.148-tegra
- ROS2: Humble
- 雷达: Livox MID360
- SLAM: RTAB-Map (ICP 策略, 纯激光建图)
- 项目路径: `/home/wheeltec/wzy`

## 1. 编译

```bash
cd ~/wzy
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

已编译的关键包 (共17个):
- `livox_ros_driver2` — Livox MID360 雷达驱动
- `robot_localization` — EKF 多传感器融合 (当前未启用)
- `rtabmap_slam` / `rtabmap_odom` / `rtabmap_launch` — RTAB-Map SLAM
- `rtabmap_sync` / `rtabmap_util` / `rtabmap_viz` — RTAB-Map 工具链
- `rtsp_camera_bridge` — RTSP 相机桥接

## 2. Demo 测试 (rosbag 回放)

需要有效的 rosbag 数据文件，详见 `rosbag.md`。

### 终端 A — 启动 demo 节点

```bash
cd ~/wzy
source /opt/ros/humble/setup.bash
source install/setup.bash
bash scripts/check_mapping.sh demo-up --gui
```

### 终端 B — 播放 rosbag

```bash
cd ~/wzy
source /opt/ros/humble/setup.bash
source install/setup.bash
bash scripts/check_mapping.sh demo-play bags/demo_mapping_bag
```

### 终端 C — 状态检查

```bash
cd ~/wzy
source /opt/ros/humble/setup.bash
source install/setup.bash
bash scripts/check_mapping.sh status
```

## 3. 实机测试 (real-up)

一键启动雷达 + RTAB-Map 建图全链路。

### 启动命令

```bash
cd ~/wzy
bash scripts/run_mapping.sh
```

`run_mapping.sh` 是包装脚本，内部流程:
1. 设置环境变量 (`LIDAR_CMD_POINTCLOUD2`)
2. source ROS2 和工作区
3. 调用 `check_mapping.sh real-up`
   → `start_all_mapping.sh` (雷达管理 + 话题检测)
   → `start_rtabmap_mapping.sh` (RTAB-Map 节点启动)

### 启动后的节点

| 节点 | 功能 |
|------|------|
| `/livox_lidar_publisher` | Livox MID360 点云驱动 (PointCloud2 模式) |
| `/rtabmap/icp_odometry` | ICP 点云帧间匹配里程计 (~10Hz) |
| `/rtabmap/rtabmap` | RTAB-Map SLAM 主节点 (1Hz 地图更新) |

### TF 树结构 (当前)

```
map → odom        (RTAB-Map SLAM 发布)
  odom → livox_frame  (ICP Odometry 发布)
```

> 注: 当前定位使用 RTAB-Map 自带的 ICP Odometry，未启用 robot_localization (EKF)。

### 关键话题

| 话题 | 类型 | 来源 |
|------|------|------|
| `/livox/lidar` | sensor_msgs/PointCloud2 | livox_ros_driver2 |
| `/odometry/local` | nav_msgs/Odometry | icp_odometry |
| `/map` | nav_msgs/OccupancyGrid | rtabmap |
| `/rtabmap/cloud_map` | sensor_msgs/PointCloud2 | rtabmap |

### 常用环境变量

可在启动前通过 `export` 设置覆盖默认值:

```bash
# 可视化 (默认: RVIZ=false, RTABMAP_VIZ=true)
export RVIZ=true

# ICP 模式 (auto=自动检测外部odom, on=强制ICP, off=使用外部odom)
export USE_ICP_MODE=auto

# 雷达话题 (有外部去畸变节点时用 deskewed)
export LIDAR_TOPIC=/livox/lidar

# 里程计话题
export ODOM_TOPIC=/odometry/local

# 机器人坐标系 (auto=自动检测)
export FRAME_ID=auto
```

### 停止

```bash
# Ctrl+C 终止 run_mapping.sh 后，确认清理残留进程:
pkill -f livox_ros_driver2
pkill -f livox_lidar_publisher
pkill -f rtabmap
```

## 4. 单独测试雷达驱动

```bash
cd ~/wzy
source /opt/ros/humble/setup.bash
source install/setup.bash

# PointCloud2 模式 (RTAB-Map 兼容)
ros2 launch livox_ros_driver2 rviz_MID360_launch.py

# 验证
ros2 topic hz /livox/lidar    # 预期 ~10-15Hz
```

雷达配置文件: `src/livox_ros_driver2/config/MID360_config.json`
- 雷达 IP: 192.168.168.20
- 主机 IP: 192.168.168.50

## 5. 状态诊断

```bash
# 查看所有相关节点
ros2 node list | grep -E 'livox|rtabmap'

# 查看话题频率
ros2 topic hz /livox/lidar
ros2 topic hz /odometry/local

# 查看 TF 树
ros2 run tf2_tools view_frames

# 检查脚本内置状态
bash scripts/check_mapping.sh status
```

## 6. 已知问题

- demo rosbag (`demo_mapping.db3`) 数据文件损坏，需重新传输
- `robot_bringup` 包缺少 `package.xml` 和 `CMakeLists.txt`，colcon 无法识别
- 残留 livox 进程可能导致重复 publisher，启动前需确认清理干净
- 使用 `--symlink-install` 编译时，`find -type f` 需加 `-o -type l` 才能匹配启动文件
