# Robot Bringup 启动说明

## 前置条件

```bash
# 编译（首次或代码更新后）
cd ~/wzy
colcon build --packages-select robot_bringup --symlink-install
source install/setup.bash
```

## 建图模式

启动完整建图链路：Livox 雷达 → ICP 里程计 → EKF 融合 → RTAB-Map SLAM

```bash
ros2 launch robot_bringup mapping.launch.py
```

### 数据链路

```
/livox/lidar (PointCloud2)
      ↓
icp_odometry → /odometry/lio
      ↓
EKF (+ /livox/imu) → /odometry/local
      ↓
rtabmap (SLAM 建图)
```

### TF 树

```
map → odom → base_footprint → base_link
 ↑      ↑            ↓
rtabmap  EKF     livox_frame
```

### 常用参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `start_livox` | `true` | 是否启动 Livox 驱动（外部已启动时设 false） |
| `rviz` | `false` | 是否启动 RViz |
| `rtabmap_viz` | `true` | 是否启动 rtabmap 自带可视化 |
| `scan_cloud_topic` | `/livox/lidar` | 点云输入话题 |
| `imu_topic` | `/livox/imu` | IMU 话题 |
| `frame_id` | `base_footprint` | 机器人基坐标系 |

### 示例

```bash
# 默认启动（含 Livox 驱动 + rtabmap_viz）
ros2 launch robot_bringup mapping.launch.py

# 不启动 Livox（已有雷达在运行）
ros2 launch robot_bringup mapping.launch.py start_livox:=false

# 开启 RViz，关闭 rtabmap_viz
ros2 launch robot_bringup mapping.launch.py rviz:=true rtabmap_viz:=false
```

## 启动后验证

```bash
# 检查节点
ros2 node list
# 应包含: /livox_lidar_publisher /icp_odometry /ekf_local_filter /rtabmap/rtabmap

# 检查 EKF 是否激活
ros2 lifecycle get /ekf_local_filter
# 应输出: active [3]

# 检查关键话题有数据
ros2 topic hz /odometry/lio        # ICP 输出
ros2 topic hz /odometry/local      # EKF 融合输出

# 查看当前位姿
ros2 topic echo /odometry/local --once

# 查看 TF 树
ros2 run tf2_tools view_frames
```

## 停止

直接 `Ctrl+C` 即可终止所有节点。

如有残留进程：

```bash
pkill -f livox_lidar_publisher
pkill -f icp_odometry
pkill -f rtabmap
pkill -f ekf_node
```

## 配置文件

- EKF 参数: `src/robot_bringup/config/ekf_local.yaml`
- Launch 文件: `src/robot_bringup/launch/mapping.launch.py`
