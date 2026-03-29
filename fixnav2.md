# Nav2 启动问题排查与修复记录

## 问题背景

在 `add/rtk` 分支上执行 `ros2 launch robot_bringup nav2.launch.py` 后，Nav2 所有 lifecycle 节点停留在 `unconfigured` 状态，无法接收导航目标。

## 问题 1：`/map` 话题无 publisher

**现象**：global_costmap 的 static_layer 订阅 `/map`，但无 publisher。

**原因**：RTAB-Map 运行在 `rtabmap` namespace 下，地图发布到 `/rtabmap/map` 而非 `/map`。

**修复**：在 `nav2.launch.py` 的 RTAB-Map launch arguments 中添加 `'map_topic': '/map'`，将 map 话题 remap 到全局 `/map`，不改变 namespace。

**文件**：`src/robot_bringup/launch/nav2.launch.py`

## 问题 2：MPPI controller 缺少 critics

**现象**：`controller_server` configure 阶段报错 `No critics defined for FollowPath`。

**原因**：`nav2_common.yaml` 中 MPPI controller 配置缺少 `critics` 列表。

**修复**：将 controller 从 MPPI 改为 DWB（`dwb_core::DWBLocalPlanner`），配置更简单稳定。

**文件**：`src/robot_bringup/config/nav2_common.yaml`

## 问题 3：RewrittenYaml 参数传递失败（核心问题）

**现象**：即使 yaml 中 critics 配置正确，controller_server 仍然报 `No critics defined`。

**原因**：`nav2_bringup` 的 `navigation_launch.py` 使用 `RewrittenYaml` 重写参数文件。重写后的临时文件内容正确（可通过 `ps aux` 找到 `--params-file /tmp/tmpXXX` 后 `cat` 确认），但 controller_server 无法正确解析嵌套参数。升级 nav2 包到 1.1.20 后问题依旧。

**排查过程**：
1. 检查 `/tmp/tmpXXX` 参数文件 -> 内容正确
2. `ros2 param get /controller_server FollowPath.critics` -> `Parameter not set`
3. `ros2 param get /controller_server FollowPath.plugin` -> `dwb_core::DWBLocalPlanner`（说明部分参数能读到，但 critics 列表丢失）

**修复**：创建 `nav2fix.launch.py`，绕开 `navigation_launch.py`，直接启动各 Nav2 节点。

## 问题 4：整体 yaml 传给单节点导致类型冲突

**现象**：`nav2fix.launch.py` 直接启动 controller_server 并传入完整 `nav2_common.yaml`，节点 SIGABRT（exit code -6）。

**原因**：完整 yaml 包含所有节点的参数，controller_server 解析到其他节点的参数时出现类型冲突。错误信息：`parameter 'height' has invalid type: Wrong parameter type, parameter {height} is of type {integer}, setting it to {double} is not allowed.`

**修复**：在 `nav2fix.launch.py` 中使用 `OpaqueFunction`，用 Python 加载 yaml 后按节点名拆分，每个节点只收到自己的参数段，写入独立的临时文件再传给节点。

## 问题 5：Nav2 升级后 BT 插件库名变更

**现象**：`bt_navigator` configure 失败，`Could not load library: libnav2_drive_on_heading_action_bt_node.so`。

**原因**：`apt upgrade` 将 nav2 从 1.1.18 升级到 1.1.20 后，部分 BT 插件库名发生了变化。`nav2_drive_on_heading_action_bt_node` 改为 `nav2_drive_on_heading_bt_node`（去掉 `_action`）。

**排查**：`find /opt/ros/humble/lib -name "libnav2_drive_on_heading*"` 确认实际库名。

**修复**：修改 `nav2_common.yaml` 中 `bt_navigator.plugin_lib_names` 对应条目。

**文件**：`src/robot_bringup/config/nav2_common.yaml`

## 问题 6：behavior_server lifecycle transition 超时

**现象**：`lifecycle_manager` 报 `Failed to change state for node: behavior_server`。

**原因**：behavior_server 在等待 TF（`map -> odom`），RTAB-Map 定位尚未完成时 lifecycle 转换超时。

**修复**：lifecycle_manager 增加 `bond_timeout: 10.0` 和 `attempt_respawn_reconnection: True`。

## 最终方案

使用 `nav2fix.launch.py` 替代 `nav2.launch.py` 中对 `navigation_launch.py` 的调用：

```bash
ros2 launch robot_bringup nav2fix.launch.py
```

关键改动：
- 用 `OpaqueFunction` + `yaml.safe_load` 拆分参数文件，每个 Nav2 节点只接收自己的参数
- 直接启动 7 个 Nav2 节点 + lifecycle_manager，不经过 `RewrittenYaml`
- lifecycle_manager 增加超时容忍

## 涉及文件

| 文件 | 改动 |
|------|------|
| `src/robot_bringup/launch/nav2.launch.py` | 添加 `map_topic: '/map'` remap |
| `src/robot_bringup/launch/nav2fix.launch.py` | 新增，直接启动 Nav2 节点 |
| `src/robot_bringup/config/nav2_common.yaml` | MPPI -> DWB，修复 BT 插件名 |
