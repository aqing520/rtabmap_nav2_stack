# RTAB-Map Demo Rosbag 回放使用说明

本文档说明如何在当前工作区使用 rosbag 回放进行 RTAB-Map 单机测试。

## 1. 前提条件

- 工作区已完成编译。
- 已执行过依赖安装（含 `image_transport_plugins`）。
- 本仓库中已有可用 bag 数据。

当前已验证的示例路径：

- `bags/demo_mapping_bag/metadata.yaml`
- `bags/demo_mapping_bag/demo_mapping.db3`

## 2. 终端分工

建议使用 3 个终端：

- 终端 A：启动 demo 节点
- 终端 B：播放 rosbag
- 终端 C：状态检查

所有终端都先执行：

```bash
cd ~/ROS2/rtabmap_nav2_stack
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 3. 启动步骤

### 步骤 1：启动 RTAB-Map demo（终端 A）

```bash
bash scripts/check_mapping.sh demo-up --gui
```

说明：

- `--gui` 会打开 `rtabmap_viz` 和 `rviz2`。
- 若不需要图形界面，可去掉 `--gui`。

### 步骤 2：播放 bag（终端 B）

循环播放（推荐调试时使用）：

```bash
bash scripts/check_mapping.sh demo-play bags/demo_mapping_bag
```

只播放一遍：

```bash
bash scripts/check_mapping.sh demo-play bags/demo_mapping_bag --no-loop
```

## 4. 运行中检查（终端 C）

### 节点检查

```bash
ros2 node list
```

正常情况下可看到：

- `/rgbd_sync`
- `/rtabmap`
- `/rosbag2_player`（仅播放期间存在）
- `/rtabmap_viz`（使用 `--gui` 时）
- `/rviz2`（使用 `--gui` 时）

### 关键话题检查

```bash
ros2 topic list | grep -E '^/rgbd_image$|^/map$|^/jn0/base_scan$'
```

```bash
timeout 8s ros2 topic echo --once /rgbd_image
```

```bash
timeout 8s ros2 topic echo --once /map
```

如果都能收到消息，说明同步与建图链路正常。

## 5. 为什么看起来“机器人自己在动”

这是 rosbag 回放导致的正常现象：

- 播放器在重放历史传感器/里程计数据。
- 可视化中轨迹变化是历史数据驱动，不是你实时控制机器人。

可以用下面命令确认没有控制话题输出：

```bash
ros2 topic list | grep cmd_vel
```

如果没有 `/cmd_vel` 或 `/cmd_vel_safe`，就不是控制指令在驱动。

## 6. 常见问题

### 6.1 `Bag path not found`

说明路径不对，先查实际文件：

```bash
find bags -maxdepth 4 \( -name metadata.yaml -o -name '*.db3' \)
```

然后把正确路径传给 `demo-play`。

### 6.2 `/rosbag2_player` 查不到

若你用了 `--no-loop`，播放结束后该节点会自动退出，这是正常现象。

### 6.3 `rgbd_sync` 启动时报插件相关错误

请确认已安装：

```bash
sudo apt update
sudo apt install ros-humble-image-transport-plugins
```

## 7. 停止方法

- 在运行 `demo-up` 的终端按 `Ctrl+C`。
- 在运行 `demo-play` 的终端按 `Ctrl+C`。

如需彻底清理，可执行：

```bash
pkill -f ros2\ launch || true
pkill -f ros2\ bag || true
```

## 8. 一键最小流程（复制即用）

终端 A：

```bash
cd ~/ROS2/rtabmap_nav2_stack
source /opt/ros/humble/setup.bash
source install/setup.bash
bash scripts/check_mapping.sh demo-up --gui
```

终端 B：

```bash
cd ~/ROS2/rtabmap_nav2_stack
source /opt/ros/humble/setup.bash
source install/setup.bash
bash scripts/check_mapping.sh demo-play bags/demo_mapping_bag
```

终端 C：

```bash
cd ~/ROS2/rtabmap_nav2_stack
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 node list
ros2 topic hz /rgbd_image
ros2 topic hz /map
```
