# RViz 手动多路径点导航技术报告

> 项目：xz/rtabmap_nav2_stack  
> 日期：2026-06-10  
> 模块：robot_bringup / Nav2 FollowWaypoints / RViz Publish Point

---

## 1. 背景

原需求是实现“多路路径点”导航。初版采用 `routes.yaml` 路径库方式预先配置多条路线，但现场使用更需要在 RViz 中根据当前地图直接手动取点，并立即执行。因此本次将功能调整为：

- 不依赖固定路径库。
- 在 RViz 中通过 `Publish Point` 工具手动点击多个路径点。
- 将点序列绑定到指定地图标识和坐标系。
- 使用 Nav2 `/follow_waypoints` action 按顺序执行路径点。
- 在 RViz 中显示已选点和连线，便于确认路径顺序。

---

## 2. 功能目标

本功能面向现场调试和半自动导航任务，目标如下：

| 目标 | 实现方式 |
|---|---|
| RViz 手动取点 | 订阅 `/clicked_point` |
| 指定地图 | 启动参数 `waypoint_map_id`、`database_path` |
| 限定坐标系 | 启动参数 `waypoint_map_frame_id`，默认 `map` |
| 路径点可视化 | 发布 `/multi_waypoint_route/markers` |
| 路径执行 | 调用 Nav2 `/follow_waypoints` |
| 运行控制 | 通过 `/multi_waypoint_route/command` 控制 start/undo/clear/pause/resume/skip/cancel |

---

## 3. 本次新增与修改文件

### 3.1 新增节点

```text
src/robot_bringup/scripts/multi_waypoint_route_node.py
```

职责：

- 订阅 RViz 点击点 `/clicked_point`。
- 检查点击点 `frame_id` 是否等于指定 `map_frame_id`。
- 保存当前手动点选的路径点序列。
- 自动根据相邻点计算 yaw。
- 发布当前点列表 `/multi_waypoint_route/points`。
- 发布 RViz MarkerArray `/multi_waypoint_route/markers`。
- 调用 Nav2 `/follow_waypoints` 执行路径。

### 3.2 新增单独启动文件

```text
src/robot_bringup/launch/multi_waypoint_routes.launch.py
```

用于只启动 RViz 手动路径点节点，适合单独调试。

### 3.3 修改总启动文件

```text
src/robot_bringup/launch/bringup.launch.py
```

新增参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `start_multi_waypoint_routes` | `false` | 是否启动 RViz 手动路径点节点 |
| `waypoint_map_id` | `site_a` | 当前路径点所属地图标识 |
| `waypoint_map_frame_id` | `map` | RViz 点击点要求使用的坐标系 |

同时调整 RViz 启动逻辑：当 `start_multi_waypoint_routes:=true` 时，即使未显式传 `enable_rviz:=true`，也会自动启动 RViz。

### 3.4 新增便捷启动脚本

```text
scripts/start_multi_waypoint_navigation.sh
```

作用：

- 自动 source `/opt/ros/humble/setup.bash`。
- 自动 source `/home/wheeltec/xz/rtabmap_nav2_stack/install/setup.bash`。
- 避免终端环境只 source `wheeltec_ros2` 时出现 `Package 'robot_bringup' not found`。
- 默认使用 `site_workspace` 地图，也可通过参数指定。

### 3.5 修改安装与依赖

```text
src/robot_bringup/CMakeLists.txt
src/robot_bringup/package.xml
```

新增运行依赖：

- `action_msgs`
- `geometry_msgs`
- `nav2_msgs`
- `std_msgs`
- `visualization_msgs`

---

## 4. 系统架构

```text
RViz Publish Point
    -> /clicked_point
    -> multi_waypoint_route_node.py
        -> 校验 map_frame_id
        -> 缓存点序列
        -> 发布 /multi_waypoint_route/points
        -> 发布 /multi_waypoint_route/markers
        -> 调用 /follow_waypoints
    -> Nav2 waypoint_follower
    -> controller_server
    -> /cmd_vel_nav
    -> collision_monitor
    -> /cmd_vel
```

其中 Nav2 仍由原系统负责规划、控制与避障。本节点只负责路径点采集、管理和下发，不替代 Nav2 的局部规划器或控制器。

---

## 5. 关键话题与接口

| 名称 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/clicked_point` | `geometry_msgs/msg/PointStamped` | 订阅 | RViz `Publish Point` 工具输出 |
| `/multi_waypoint_route/command` | `std_msgs/msg/String` | 订阅 | 控制命令 |
| `/multi_waypoint_route/status` | `std_msgs/msg/String` | 发布 | 当前状态 JSON |
| `/multi_waypoint_route/points` | `std_msgs/msg/String` | 发布 | 当前点列表 JSON |
| `/multi_waypoint_route/markers` | `visualization_msgs/msg/MarkerArray` | 发布 | RViz 点和连线显示 |
| `/follow_waypoints` | `nav2_msgs/action/FollowWaypoints` | Action Client | Nav2 路径点执行接口 |

状态示例：

```json
{
  "state": "EDITING",
  "map_id": "site_a",
  "map_frame_id": "map",
  "point_count": 3,
  "current_waypoint": 0,
  "detail": "added point 2 on map site_a"
}
```

---

## 6. 地图指定机制

启动导航时通过 `database_path` 指定 RTAB-Map 数据库：

```bash
database_path:=/data/maps/site_a/rtabmap.db
```

路径点节点通过以下参数标记和校验当前地图：

```bash
waypoint_map_id:=site_a
waypoint_map_frame_id:=map
```

说明：

- `waypoint_map_id` 用于状态上报和人工区分地图，例如 `site_a`、`site_workspace`。
- `waypoint_map_frame_id` 用于校验 RViz 点击点坐标系。
- 如果 RViz 点击点的 `frame_id` 不是 `map`，节点会拒绝该点并在 `/multi_waypoint_route/status` 中提示。

这样可以避免在 RViz Fixed Frame 配错或地图坐标系不一致时误采路径点。

---

## 7. 使用方法

### 7.1 推荐启动方式

```bash
source /opt/ros/humble/setup.bash
source /home/wheeltec/xz/rtabmap_nav2_stack/install/setup.bash

ros2 launch robot_bringup bringup.launch.py \
  mode:=navigation \
  database_path:=/data/maps/site_a/rtabmap.db \
  start_multi_waypoint_routes:=true \
  waypoint_map_id:=site_a \
  waypoint_map_frame_id:=map
```

当 `start_multi_waypoint_routes:=true` 时，RViz 会自动启动。

### 7.2 便捷脚本启动

```bash
cd /home/wheeltec/xz/rtabmap_nav2_stack
bash scripts/start_multi_waypoint_navigation.sh --map site_workspace --rviz
```

脚本会自动 source 正确工作空间，避免包找不到的问题。

### 7.3 RViz 操作

1. RViz 左上角 `Fixed Frame` 设置为 `map`。
2. 确认地图和机器人位姿正常显示。
3. 添加 `MarkerArray` 显示：
   - `Add`
   - `By topic`
   - 选择 `/multi_waypoint_route/markers`
4. 选择工具栏 `Publish Point`。
5. 按顺序在地图上点击多个路径点。
6. RViz 中应显示点编号和连线。

### 7.4 执行路径

```bash
ros2 topic pub --once /multi_waypoint_route/command std_msgs/msg/String "{data: 'start'}"
```

### 7.5 常用控制命令

```bash
# 撤销最后一个点
ros2 topic pub --once /multi_waypoint_route/command std_msgs/msg/String "{data: 'undo'}"

# 清空所有点
ros2 topic pub --once /multi_waypoint_route/command std_msgs/msg/String "{data: 'clear'}"

# 暂停当前路径
ros2 topic pub --once /multi_waypoint_route/command std_msgs/msg/String "{data: 'pause'}"

# 继续执行
ros2 topic pub --once /multi_waypoint_route/command std_msgs/msg/String "{data: 'resume'}"

# 跳过当前点
ros2 topic pub --once /multi_waypoint_route/command std_msgs/msg/String "{data: 'skip'}"

# 取消路径
ros2 topic pub --once /multi_waypoint_route/command std_msgs/msg/String "{data: 'cancel'}"
```

### 7.6 查看状态

```bash
ros2 topic echo /multi_waypoint_route/status
ros2 topic echo /multi_waypoint_route/points
```

---

## 8. 执行逻辑

### 8.1 点采集

每次 RViz 发布 `/clicked_point` 后，节点执行：

```text
读取 PointStamped
    -> 检查 header.frame_id 是否等于 map_frame_id
    -> 保存 x/y/z
    -> 发布 points JSON
    -> 发布 MarkerArray
```

### 8.2 yaw 计算

RViz `Publish Point` 只能提供位置，不能提供朝向。因此节点在执行前自动计算 yaw：

- 中间点：朝向下一个路径点。
- 最后一个点：沿用上一个点到最后一个点的方向。
- 只有一个点时：使用 `default_final_yaw`，默认 `0.0`。

### 8.3 路径执行

收到 `start` 后：

```text
检查点数量
    -> 转换为 PoseStamped[]
    -> 填充 header.frame_id = map_frame_id
    -> 调用 Nav2 FollowWaypoints
    -> 根据 feedback 更新 current_waypoint
    -> 根据 result 发布 FINISHED / FAILED / CANCELED
```

---

## 9. 验证结果

已执行以下检查：

```bash
python3 -m py_compile \
  src/robot_bringup/scripts/multi_waypoint_route_node.py \
  src/robot_bringup/launch/multi_waypoint_routes.launch.py \
  src/robot_bringup/launch/bringup.launch.py
```

结果：通过。

```bash
colcon build --packages-select robot_bringup
```

结果：通过。

```bash
source /opt/ros/humble/setup.bash
source /home/wheeltec/xz/rtabmap_nav2_stack/install/setup.bash
ros2 pkg prefix robot_bringup
```

结果：

```text
/home/wheeltec/xz/rtabmap_nav2_stack/install/robot_bringup
```

```bash
ros2 launch robot_bringup multi_waypoint_routes.launch.py map_id:=site_workspace map_frame_id:=map --show-args
```

结果：参数解析正常。

---

## 10. 常见问题

### 10.1 Package 'robot_bringup' not found

原因：当前终端没有 source `xz/rtabmap_nav2_stack` 工作空间。

解决：

```bash
source /opt/ros/humble/setup.bash
source /home/wheeltec/xz/rtabmap_nav2_stack/install/setup.bash
```

或使用脚本：

```bash
bash scripts/start_multi_waypoint_navigation.sh --map site_workspace --rviz
```

### 10.2 启动后没有 RViz

当前已修改为：`start_multi_waypoint_routes:=true` 时自动启动 RViz。

如果仍未出现，检查终端是否有 `rviz2` 报错，或显式传：

```bash
enable_rviz:=true
```

### 10.3 RViz 点击后没有点和连线

检查：

- RViz `Fixed Frame` 是否为 `map`。
- 是否使用的是 `Publish Point` 工具。
- 是否添加了 `/multi_waypoint_route/markers` 的 `MarkerArray` 显示。
- `/multi_waypoint_route/status` 是否提示 frame 不匹配。

### 10.4 点了路径但机器人不走

检查：

- Nav2 是否已启动并激活。
- `/follow_waypoints` action server 是否存在。
- `/multi_waypoint_route/status` 是否显示 `FAILED`。
- `/cmd_vel_nav` 和 `/cmd_vel` 是否有输出。

---

## 11. 后续可扩展方向

当前版本重点满足现场手动取点与执行。后续可扩展：

- 将 RViz 手动取点结果保存为 YAML，形成可复用路线。
- 增加路线命名和多路线管理。
- 增加服务接口，替代字符串命令。
- 增加点位朝向手动指定功能。
- 增加到点等待时间、拍照、语音提示等 waypoint task executor。
- 在 RViz 中提供交互式 marker，支持拖拽调整点位。

---

## 12. 结论

本次改造完成了从“固定路径库”到“RViz 现场手动路径点”的功能转型。新方案更适合调试、演示和现场临时任务，能够在指定地图下快速采集路径点并调用 Nav2 执行，同时保留状态反馈、可视化和基础控制能力。
