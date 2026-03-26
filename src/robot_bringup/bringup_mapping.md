# Robot Bringup 建图启动说明

## 前置条件

```bash
# 编译（首次或代码更新后）
cd ~/ROS2/rtabmap_nav2_stack
colcon build --packages-select robot_bringup --symlink-install
source install/setup.bash
```

## 建图模式

启动完整建图链路：Livox MID360 雷达 → FAST-LIO 里程计 → RTAB-Map SLAM

```bash
ros2 launch robot_bringup fastlio_mapping.launch.py
```

### 数据链路

```
/livox/lidar (PointCloud2) + /livox/imu (Imu)
              ↓
      FAST-LIO → /Odometry (nav_msgs/Odometry)
              ↓           + TF: odom → base_footprint
  /cloud_registered_body (去畸变点云)
              ↓
      rtabmap_slam (SLAM 建图)
              ↓
    /map + TF: map → odom + rtabmap.db
```

### TF 树

```
map ──→ odom ──→ base_footprint ──→ base_link ──→ livox_frame
 ↑        ↑          (static)         (static)
rtabmap  FAST-LIO
```

### 常用参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `start_livox` | `true` | 是否启动 Livox 驱动（外部已启动时设 false） |
| `rviz` | `false` | 是否启动 RViz |
| `rtabmap_viz` | `true` | 是否启动 rtabmap 自带可视化 |
| `scan_cloud_topic` | `/cloud_registered_body` | 点云输入话题（FAST-LIO 去畸变后的点云） |
| `imu_topic` | `/livox/imu` | IMU 话题 |
| `frame_id` | `base_footprint` | 机器人基坐标系 |
| `delete_db_on_start` | `true` | 启动时删除旧 RTAB-Map 数据库（全新建图） |

### 示例

```bash
# 默认启动（含 Livox 驱动 + rtabmap_viz）
ros2 launch robot_bringup fastlio_mapping.launch.py

# 不启动 Livox（已有雷达在运行）
ros2 launch robot_bringup fastlio_mapping.launch.py start_livox:=false

# 开启 RViz，关闭 rtabmap_viz
ros2 launch robot_bringup fastlio_mapping.launch.py rviz:=true rtabmap_viz:=false
```

## 启动后验证

```bash
# 检查节点
ros2 node list
# 应包含: /livox_lidar_publisher /fast_lio /rtabmap/rtabmap

# 检查关键话题频率
ros2 topic hz /Odometry                  # FAST-LIO 输出，预期 ~10-20 Hz
ros2 topic hz /cloud_registered_body     # 去畸变点云

# 查看当前位姿
ros2 topic echo /Odometry --once

# 查看 TF 树（生成 frames.pdf）
ros2 run tf2_tools view_frames
# 预期链路: map → odom → base_footprint → base_link → livox_frame
```

## 停止

直接 `Ctrl+C` 即可终止所有节点。

如有残留进程：

```bash
pkill -f livox_lidar_publisher
pkill -f fast_lio
pkill -f rtabmap
```

## 配置文件

- FAST-LIO 配置: `fast_lio` 包内 `config/mid360.yaml`
- Launch 文件: `src/robot_bringup/launch/fastlio_mapping.launch.py`
