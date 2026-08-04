# RViz 给目标点后不出现 Path 排查 README

适用场景：

- RViz 中使用 `Nav2 Goal` 或 `2D Goal Pose` 给目标点后，机器人不动。
- RViz 中没有显示全局路径 `Path`。
- 终端没有明显报错，但 Nav2 没有规划结果。

本文按现场最快排查顺序整理。建议不要一上来改参数，先判断是显示问题、Nav2 未激活、TF/地图问题，还是目标点确实不可达。

## 1. 先确认是不是 RViz 显示问题

RViz 不显示 Path，不一定代表 Nav2 没有规划。先确认 Path 显示项是否存在并订阅正确话题。

在 RViz 中检查：

1. Fixed Frame 设为 `map`。
2. 添加 `Path` 显示项。
3. Topic 选择 `/plan`。
4. 添加 `Map` 显示项，Topic 选择 `/map`。
5. 添加 `RobotModel` 或 `TF`，确认机器人位姿在地图中正常。
6. 添加 `Costmap` 或 `Map` 显示项查看 `/global_costmap/costmap`。

终端确认 `/plan` 是否真的有数据：

```bash
ros2 topic echo --once /plan
```

如果 `/plan` 有数据但 RViz 不显示，优先检查 RViz 的 `Path` topic、Fixed Frame、显示项是否勾选。

如果 `/plan` 没有数据，继续下面的排查。

## 2. 确认 Nav2 节点是否已激活

Nav2 节点未进入 `active` 状态时，给目标点不会正常规划。

查看节点：

```bash
ros2 node list | grep -E "planner_server|controller_server|bt_navigator|waypoint_follower|global_costmap|local_costmap"
```

查看生命周期状态：

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /global_costmap/global_costmap
ros2 lifecycle get /local_costmap/local_costmap
```

正常应看到 `active`。

如果不是 `active`：

```bash
ros2 launch robot_bringup bringup.launch.py \
  mode:=navigation \
  database_path:=/data/maps/site_a/rtabmap.db \
  enable_rviz:=true
```

并检查启动终端中是否有参数文件错误、插件加载失败、TF 超时等日志。

## 3. 确认 TF 链路完整

本项目导航依赖：

```text
map -> odom -> base_footprint -> base_link -> livox_frame
```

检查关键 TF：

```bash
ros2 run tf2_ros tf2_echo map base_footprint
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint base_link
```

如果 `map -> base_footprint` 不通，Nav2 不知道机器人在地图中的位置，通常不会生成有效路径。

常见原因：

- RTAB-Map 没有发布 `map -> odom`。
- FAST-LIO 没有发布 `odom -> base_footprint`。
- `use_sim_time` 不一致导致 TF 时间异常。
- `base_footprint` 和配置中的 `robot_base_frame` 不一致。

本项目默认配置中：

```yaml
global_frame: map
robot_base_frame: base_footprint
odom_topic: /Odometry
```

如果现场改了 frame 名称，需要同步修改 `nav2_common.yaml` 或启动参数。

## 4. 确认地图和全局代价地图正常

检查地图：

```bash
ros2 topic echo --once /map
ros2 topic hz /map
```

检查全局代价地图：

```bash
ros2 topic echo --once /global_costmap/costmap
ros2 topic hz /global_costmap/costmap
```

如果 `/map` 没有数据：

- 确认 `database_path` 指向存在的 RTAB-Map 数据库。
- 确认启动模式是 `navigation` 或 `localization`，不是只启动了 RViz。
- 查看 RTAB-Map 节点日志。

如果 `/map` 有数据但 `/global_costmap/costmap` 没有数据：

- 检查 `global_costmap` lifecycle 是否 active。
- 检查 Nav2 参数文件是否加载成功。
- 检查 static layer、STVL 或 inflation layer 是否加载失败。

## 5. 检查目标点是否落在障碍物或未知区域内

本项目 `nav2_common.yaml` 中全局规划器配置为：

```yaml
allow_unknown: false
```

这表示目标点或路径经过未知区域时，Nav2 可能不会规划路径。

现场处理：

1. 在 RViz 显示 `/global_costmap/costmap`。
2. 目标点不要点在黑色未知区、墙体或障碍物膨胀区内。
3. 先点一个离机器人 1-2 米、明显在可通行白色区域内的目标点做验证。
4. 如果这个近点可以出 Path，说明 Nav2 正常，远点不可达多半是地图/代价地图阻断。

## 6. 清除代价地图后重新给目标点

动态障碍物、旧点云残留或膨胀层残留可能造成暂时无法规划。

清除全局和局部代价地图：

```bash
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
```

然后重新在 RViz 中给目标点。

也可以直接用本项目脚本测试一个简单目标点：

```bash
cd /home/wheeltec/xz/rtabmap_nav2_stack
source /opt/ros/humble/setup.bash
source install/setup.bash
bash scripts/send_goal.sh 1.0 0.0 0
```

如果脚本可以触发规划，而 RViz 不行，重点检查 RViz 工具和目标话题。

## 7. 查看 planner 和 bt_navigator 日志

打开一个终端实时看日志：

```bash
ros2 topic echo /rosout | grep -E "planner_server|bt_navigator|global_costmap|Navfn|failed|Failed|exception|Exception|TF"
```

常见日志含义：

| 日志关键词 | 可能原因 |
|---|---|
| `Timed out waiting for transform` | TF 不完整或时间戳异常 |
| `Start is outside map` | 机器人定位不在地图范围内 |
| `Goal is outside map` | 目标点超出地图范围 |
| `failed to create plan` | 目标不可达、未知区阻断、障碍物阻断 |
| `plugin failed to load` | 参数中的插件未编译、未 source 或插件名错误 |
| `No valid path found` | 全局代价地图中没有可通行路径 |

## 8. 快速判断表

| 现象 | 优先处理 |
|---|---|
| `/plan` 有数据但 RViz 不显示 | 检查 RViz Path topic 是否为 `/plan`，Fixed Frame 是否为 `map` |
| Nav2 lifecycle 不是 active | 看启动终端错误，重新启动 bringup |
| `tf2_echo map base_footprint` 失败 | 修复 RTAB-Map/FAST-LIO TF 链路 |
| `/map` 没数据 | 检查 `database_path` 和 RTAB-Map 启动 |
| `/global_costmap/costmap` 没数据 | 检查 costmap lifecycle 和插件加载 |
| 近点能规划，远点不能规划 | 地图未知区或障碍物阻断 |
| 清图后恢复 | 点云残留或动态障碍物残留 |

## 9. 推荐现场恢复流程

按下面顺序执行，通常可以快速定位：

```bash
source /opt/ros/humble/setup.bash
source /home/wheeltec/xz/rtabmap_nav2_stack/install/setup.bash

ros2 lifecycle get /planner_server
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic echo --once /map
ros2 topic echo --once /global_costmap/costmap

ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
```

然后在 RViz 中先给一个近距离、空旷区域目标点。如果近点有 Path，再逐步测试远点和禁行区边界。
