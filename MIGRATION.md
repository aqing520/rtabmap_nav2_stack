# 项目依赖与迁移说明

更新时间：2026-08-06

## 1. 结论

这个项目**可以迁移**，但不能只复制当前仓库就直接运行。

| 功能 | 迁移难度 | 主要外部依赖 |
|---|---|---|
| 纯激光建图 | 中 | ROS 2 Humble、Livox SDK2、PCL/OpenCV |
| CUDA MPPI 导航 | 中高 | Nav2、STVL、CUDA MPPI、底盘驱动 |

最需要注意的外部内容有四项：

1. `/home/wheeltec/wheeltec_ros2`：当前使用的 Nav2 和底盘相关包；
2. `/home/wheeltec/cuda_robotics_ws`：CUDA MPPI 控制器；
3. 系统安装的 Livox SDK2；
4. 项目外独立运行的底盘驱动。

仓库内已经包含 Livox ROS 驱动、FAST-LIO、RTAB-Map 0.23.4 和启动配置，因此建图部分相对容易迁移。

## 2. 当前运行环境

已验证环境：

```text
Ubuntu 22.04
ROS 2 Humble
Jetson Orin / arm64
JetPack 6.2
CUDA 12.6
OpenCV 4.8
PCL 1.12
```

不同架构、CUDA 或 ROS 版本下，应重新编译，不能直接复制当前：

```text
build/
install/
log/
roslog/
```

## 3. 仓库内已有组件

以下源码已经包含在本项目中：

```text
src/livox_ros_driver2             MID360 ROS驱动
src/FAST_LIO_ROS2                 激光惯导里程计
src/rtabmap_ros                   RTAB-Map ROS 0.23.4
third_party/rtabmap-0.23.4        RTAB-Map核心源码
src/robot_bringup                 建图和导航启动配置
src/map_paint_editor_plugin       PGM地图编辑工具
```

目标机器通过以下脚本重新编译：

```bash
bash scripts/do_build_all.sh
```

## 4. 必需的外部依赖

### 4.1 ROS 2 和系统软件包

推荐安装：

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git pkg-config \
  python3-colcon-common-extensions python3-rosdep \
  libopencv-dev libpcl-dev libeigen3-dev \
  libceres-dev libboost-dev libapr1-dev \
  python3-numpy python3-opencv python3-pyqt5 \
  python3-serial python3-yaml \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-rviz2 \
  ros-humble-robot-localization \
  ros-humble-spatio-temporal-voxel-layer \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-cv-bridge
```

然后安装其余 rosdep 依赖：

```bash
source /opt/ros/humble/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  -r -y \
  --skip-keys "cuda_mppi_controller ament_python"
```

`cuda_mppi_controller` 无法通过 rosdep 安装，需要单独迁移。

### 4.2 Livox SDK2

仓库只有 ROS 驱动源码，目标机器还必须安装 Livox SDK2。

编译时会查找：

```text
/usr/local/lib/liblivox_lidar_sdk_shared.so
/usr/local/include/livox_lidar_api.h
/usr/local/include/livox_lidar_def.h
```

安装后检查：

```bash
sudo ldconfig
ldconfig -p | grep livox_lidar_sdk
```

### 4.3 CUDA MPPI

当前导航使用：

```text
cuda_mppi_controller
```

当前外部工作空间：

```text
/home/wheeltec/cuda_robotics_ws
```

源码基于：

```text
https://github.com/rsasaki0109/CudaRobotics.git
commit: 132addd9f24477ec36e4e39e1bf4776765d056a8
```

当前源码还有一处未提交的 Humble 兼容修改：

```text
ros2_ws/src/cuda_mppi_controller/CMakeLists.txt
```

该修改会在 Humble 下跳过不兼容的 `controller_benchmark`。迁移前应提交该修改或导出补丁，否则新机器可能编译失败。

构建完成后检查：

```bash
source /目标路径/cuda_robotics_ws/install/setup.bash
ros2 pkg prefix cuda_mppi_controller
```

也可以让 `robot.sh` 加载自定义位置：

```bash
export CUDA_ROBOTICS_SETUP=/目标路径/cuda_robotics_ws/install/setup.bash
```

### 4.4 Nav2 和底盘驱动

当前机器优先使用：

```text
/home/wheeltec/wheeltec_ros2/install/setup.bash
```

其中包含当前使用的 Nav2 覆盖版本和 Wheeltec 底盘相关包。

只用 apt 安装的 Nav2 也可能运行，但需要重新验证 CUDA MPPI、STVL 和行为树的兼容性。为了完全复现当前效果，建议一起迁移或重新构建 `wheeltec_ros2`。

本项目只发布：

```text
/cmd_vel
```

`robot.sh` 不负责启动底盘电机驱动。迁移后必须另外启动底盘节点，并确认：

```bash
ros2 topic info /cmd_vel -v
```

真实运行时 `/cmd_vel` 必须存在底盘订阅者。

## 5. 雷达和地图配置

### 5.1 MID360 网络

当前雷达配置：

```text
主机IP：192.168.168.50
雷达IP：192.168.168.20
```

配置文件：

```text
src/livox_ros_driver2/config/MID360_config.json
```

迁移后检查：

```bash
ip addr
ping 192.168.168.20
```

如果新机器网卡地址不同，需要修改 JSON 文件。

### 5.2 FAST-LIO 外参

外参配置：

```text
src/FAST_LIO_ROS2/config/mid360.yaml
```

更换机器人或雷达安装位置后，需要重新核对 `extrinsic_T` 和 `extrinsic_R`。

### 5.3 地图数据库

统一入口使用：

```text
db/rtabmap.db
```

`db/` 被 Git 忽略，因此地图不会随 `git clone` 自动迁移，需要单独复制：

```bash
rsync -av db/ 新机器:/目标项目路径/db/
```

## 6. 推荐迁移步骤

### 第一步：复制源码和地图

不复制编译产物：

```bash
rsync -av \
  --exclude .git/ \
  --exclude build/ \
  --exclude install/ \
  --exclude log/ \
  --exclude roslog/ \
  --exclude third_party/rtabmap-0.23.4/build_local/ \
  --exclude third_party/rtabmap-0.23.4/install/ \
  /原机器/rtabmap_nav2_stack/ \
  /新机器/rtabmap_nav2_stack/
```

### 第二步：准备外部环境

依次准备：

1. Ubuntu 22.04 和 ROS 2 Humble；
2. Livox SDK2；
3. `wheeltec_ros2` 或兼容的 Nav2；
4. `cuda_robotics_ws`；
5. 实际底盘驱动；
6. MID360 静态网卡地址。

### 第三步：编译本项目

```bash
cd /目标路径/rtabmap_nav2_stack

source /opt/ros/humble/setup.bash
source /目标路径/wheeltec_ros2/install/setup.bash
source /目标路径/cuda_robotics_ws/install/setup.bash

bash scripts/do_build_all.sh
source install/setup.bash
```

内存不足时：

```bash
JOBS=2 WORKERS=2 HEAVY_JOBS=1 bash scripts/do_build_all.sh
```

## 7. 迁移后检查

检查包：

```bash
ros2 pkg prefix livox_ros_driver2
ros2 pkg prefix fast_lio
ros2 pkg prefix rtabmap_slam
ros2 pkg prefix nav2_bringup
ros2 pkg prefix spatio_temporal_voxel_layer
ros2 pkg prefix cuda_mppi_controller
```

检查建图数据链：

```bash
bash robot.sh map rviz:=false

ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /cloud_registered_body
ros2 topic hz /Odometry
```

检查导航：

```bash
bash robot.sh nav enable_rviz:=false

ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 topic info /map -v
ros2 topic info /cmd_vel -v
```

首次测试应架空驱动轮或断开底盘速度订阅，确认 TF、点云、地图和规划正常后再允许机器人运动。

## 8. 最终迁移清单

```text
[ ] Ubuntu 22.04 / ROS 2 Humble
[ ] PCL、OpenCV、Eigen、Ceres等系统依赖
[ ] Livox SDK2
[ ] MID360网卡和IP配置
[ ] wheeltec_ros2 / 兼容Nav2
[ ] CUDA MPPI及Humble兼容修改
[ ] 底盘驱动
[ ] 当前项目源码
[ ] db/rtabmap.db
[ ] 在目标机器重新编译
[ ] 架空轮子完成首次导航验证
```
