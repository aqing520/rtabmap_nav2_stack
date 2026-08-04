# 局部代价地图动态障碍残留优化技术报告

## 1. 背景

当前 xz 项目的导航链路中，局部代价地图使用 `/cloud_registered_body` 点云构建障碍信息。原方案基于 Nav2 默认 `nav2_costmap_2d::VoxelLayer`，通过点云标记障碍，并依赖 raytrace 对空闲区域进行清除。

实测问题集中在动态障碍离开后，局部 costmap 中仍保留残留障碍，尤其在导航途中更明显。残留障碍会影响 MPPI 局部轨迹采样，使机器人出现绕行、停顿、恢复动作频繁或局部规划质量下降。

## 2. 原方案问题分析

原局部 costmap 的主要机制是：

- `mark_cloud` 负责将点云障碍写入局部代价地图。
- `clear_cloud` 负责通过射线清除障碍。
- 障碍能否清除，强依赖后续点云是否提供穿过原障碍位置的有效空闲射线。

该机制对静态环境表现稳定，但对动态障碍存在局限：

- 动态障碍离开后，传感器未必再次完整扫过其原位置。
- 点云稀疏、遮挡、机器人运动姿态变化都会导致 raytrace 清除不充分。
- Nav2 默认 `VoxelLayer` 没有真正的时间衰减机制，障碍不会仅因时间流逝自动消失。
- 导航途中如果 costmap 尚未触发恢复清图，残留会继续参与局部规划。

## 3. 优化方案

将局部 costmap 的障碍层从 `nav2_costmap_2d::VoxelLayer` 替换为 `spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer`。

STVL 相比默认 VoxelLayer 的核心优势是增加了时间维度。障碍 voxel 不只依赖 raytrace 清除，还可以根据 `voxel_decay` 自动衰减。这样动态障碍离开后，即使没有完美的清除射线，残留也会在设定时间内逐步消失。

本次在局部 costmap 和全局 costmap 中均使用 STVL。局部 STVL 负责近距离动态避障，衰减更快；全局 STVL 负责把较大范围内的动态障碍反馈给全局规划，衰减更慢、voxel 更粗，以减少计算压力和路径抖动。

## 4. 配置变更

涉及文件：

- `src/robot_bringup/config/nav2_common.yaml`

全局 costmap 分层调整为：

```yaml
plugins: ['static_layer', 'stvl_layer', 'inflation_layer']
```

局部 costmap 分层调整为：

局部 costmap 插件由：

```yaml
plugins: ['voxel_layer', 'inflation_layer']
voxel_layer:
  plugin: nav2_costmap_2d::VoxelLayer
```

替换为：

```yaml
plugins: ['stvl_layer', 'inflation_layer']
stvl_layer:
  plugin: spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer
```

主要参数：

```yaml
voxel_decay: 4.0
decay_model: 0
voxel_size: 0.07
obstacle_range: 3.5
publish_voxel_map: false
mapping_mode: false
```

点云来源仍为：

```yaml
topic: /cloud_registered_body
data_type: PointCloud2
```

标记源 `lidar_mark` 只负责写入障碍，清除源 `lidar_clear` 只负责清除障碍。两者使用同一个点云话题，但职责分离，便于后续分别调整标记高度、清除视场和衰减行为。

## 5. 参数设计说明

局部 `voxel_decay: 4.0`

动态障碍在没有持续观测后约 4 秒进入衰减清除。该值偏保守，避免真实障碍因短暂漏检过快消失。

全局 `voxel_decay: 8.0`

全局层衰减更慢，用于让全局规划在较大范围内短时间记住动态障碍；同时避免动态障碍离开后长期污染全局路径。

局部 `voxel_size: 0.07`

相比 0.05 m 的地图分辨率略粗，降低 STVL 的 voxel 数量和 CPU 压力。若算力充足且需要更细障碍边界，可调回 0.05；若 CPU 压力大，可调到 0.10。

全局 `voxel_size: 0.10`

全局层使用更粗 voxel，降低维护较大范围点云障碍的计算压力。

局部 `obstacle_range: 3.5`

局部避障只保留近距离障碍，减少远距离动态点云对 MPPI 的干扰，也控制 STVL 维护规模。

全局 `obstacle_range: 6.0`

全局层保留更远的动态障碍，用于触发全局路径绕行。

`publish_voxel_map: false`

关闭 voxel map 发布，减少调试外的额外开销。

`model_type: 1`

按 3D LiDAR 点云模型配置，适配当前 `/cloud_registered_body` 输入。

`horizontal_fov_angle: 6.28`

按近似 360 度点云处理。如果后续确认输入不是全向聚合点云，而是 Livox 单帧非全向视场，应按实际 FOV 收窄，避免过度清除。

## 6. 预期效果

- 动态障碍离开后，局部 costmap 残留会随时间自动衰减。
- 导航途中不必完全依赖恢复行为或全量清图。
- MPPI 受到旧障碍残留影响降低，局部路径应更连续。
- 局部地图对短时点云遮挡更宽容。

## 7. 计算开销与风险

STVL 会维护带时间信息的 3D voxel，计算压力通常高于默认 VoxelLayer。当前方案通过以下方式控制开销：

- 局部 costmap 使用较小范围和较快衰减。
- 全局 costmap 使用较大范围、较粗 voxel 和较慢衰减。
- `voxel_size` 使用 0.07 m。
- `obstacle_range` 限制为 3.5 m。
- 不发布 voxel map。

需要重点观察：

- `controller_server` 是否频繁提示控制循环 missed rate。
- `/local_costmap/costmap` 发布频率是否明显下降。
- CPU 占用是否显著增加。
- 动态障碍是否出现过早消失，导致避障不稳。

## 8. 验证建议

启动导航后检查：

```bash
ros2 param get /local_costmap/local_costmap plugins
ros2 topic hz /local_costmap/costmap
ros2 topic hz /cloud_registered_body
```

观察 RViz：

- 动态障碍进入局部和全局 costmap 后是否正常标记。
- 动态障碍离开后，局部 3 到 5 秒内消失，全局约 8 秒左右消失。
- 机器人导航途中是否减少无意义绕行或停顿。

如果清除仍慢：

```yaml
voxel_decay: 3.0
```

如果 CPU 压力偏高：

```yaml
voxel_size: 0.10
obstacle_range: 3.0
```

如果出现真实障碍过早消失：

```yaml
voxel_decay: 5.0
```

## 9. 回退方式

如果 STVL 运行效果不满足预期，可将局部 costmap 插件恢复为原 `nav2_costmap_2d::VoxelLayer`，并恢复 `mark_cloud` / `clear_cloud` 配置。全局 costmap 可移除 `stvl_layer`，恢复为静态地图和膨胀层，或恢复旧的 `nav2_costmap_2d::VoxelLayer` 动态障碍层。

## 10. 结论

本方案针对动态障碍残留问题，从“依赖射线清除”调整为“射线清除 + 时间衰减”。对于导航途中动态障碍频繁出现的场景，STVL 更符合问题本质。建议先用当前局部快衰减、全局慢衰减的组合实测，再根据 CPU、清除速度和路径稳定性微调 `voxel_decay`、`voxel_size` 和 `obstacle_range`。
