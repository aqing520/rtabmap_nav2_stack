# HDL Global Localization 技术报告

> 适用版本：hdl_global_localization-humble（ROS2 Humble）  
> 机器人平台：Wheeltec AMR + Livox MID360 + FAST-LIO + RTAB-Map + Nav2  
> 场景：室内约 15m × 11m 平坦地面  
> 文档日期：2026-05-17

---

## 1. 概述

`hdl_global_localization` 是一个无需初始位置估计的**全局点云定位**软件包，由 Kenji Koide（AIST）开发。其核心功能是：在机器人**完全不知道自身位置**的情况下，通过将当前激光雷达扫描与预先构建的三维点云地图进行匹配，求解出机器人在地图坐标系下的六自由度（6-DOF）位姿，并以 `/initialpose` 的形式提供给上层导航栈。

在本系统中，该包承担**导航启动时的初始位姿自动确定**职责，替代了传统的人工在 RViz 中点击设置初始位置的操作。

### 1.1 解决的核心问题

RTAB-Map 在定位模式（localization mode）启动时，需要一个初始位姿来快速与数据库中的节点建立回环。没有 `/initialpose` 时：
- RTAB-Map 以 `--RGBD/StartAtOrigin true` 从地图原点出发，发布一个错误的 `map→odom` TF
- Nav2 可能基于错误位置规划路径
- RTAB-Map 需要通过多帧累积自行完成回环，耗时且不可靠

使用 hdl_global_localization 后，系统启动约 1~2 分钟内自动发布正确的 `/initialpose`，RTAB-Map 收到后立即重置到正确位置。

---

## 2. 系统集成架构

### 2.1 三阶段启动时序

```
t=0s ──────────────────────────────────────────────────────────────────
  [Phase 1] bringup.launch.py (autostart=false)
    ├── Livox MID360 驱动
    ├── FAST-LIO → /cloud_registered_body + /Odometry + odom→base_footprint TF
    ├── RTAB-Map 定位模式 → 从 rtabmap.db 加载地图，以原点发布 map→odom TF
    └── Nav2（已配置，未激活）

  [并行] hdl_global_localization_node
    └── 等待 /set_global_map 和 /query 服务调用

t=10s ─────────────────────────────────────────────────────────────────
  [Phase 2] global_localization_node.py
    ├── 加载地图 PCD → open3d 预降采样(0.2m) → 发送至 hdl 节点
    ├── hdl 节点：FPFH 特征提取（地图）
    ├── 接收 /cloud_registered_body 第一帧扫描
    ├── hdl 节点：FPFH 特征提取（扫描）+ RANSAC 匹配
    └── 发布 /initialpose → RTAB-Map 收到 → 重置到正确位置

t=70~130s ─────────────────────────────────────────────────────────────
  [Phase 3] 激活 Nav2 lifecycle manager (command=0 STARTUP)
    └── 规划器、控制器、代价地图全部激活 → 可接受导航目标点
```

### 2.2 话题与 TF 数据流

```
Livox MID360
    │ /livox/lidar  /livox/imu
    ▼
FAST-LIO ────────────────────────────────────────────────────────────┐
    │ /cloud_registered_body (去畸变点云, base_footprint坐标系)        │
    │ /Odometry                                                       │
    │ TF: odom → base_footprint                                       │
    ▼                                                                 │
RTAB-Map (定位模式)                                                    │
    │ TF: map → odom  ←── 由 /initialpose 重置                        │
    │ /map (占用栅格)                                                   │
    ▼                                                                 │
Nav2                                                                  │
    │ /cmd_vel_nav → collision_monitor → /cmd_vel                     │
    ▼                                                                 │
机器人底盘                               ┌──────────────────────────┘
                                         │ /cloud_registered_body
                                         ▼
                              hdl_global_localization_node
                                         │ /initialpose
                                         └──→ RTAB-Map
```

最终 TF 主链：
```
map → odom → base_footprint → base_link → livox_frame
 ↑      ↑         ↑               ↑
RTAB  FAST-LIO  (static)       (static)
```

---

## 3. ROS2 节点接口

### 3.1 节点信息

| 项目 | 内容 |
|------|------|
| 节点名 | `hdl_global_localization_node` |
| 默认引擎 | **FPFH_RANSAC**（本系统已修改，原默认为 BBS） |
| 配置目录 | `<package_share>/config/` |

### 3.2 服务接口

| 服务名 | 消息类型 | 说明 |
|--------|----------|------|
| `/set_engine` | `SetGlobalLocalizationEngine` | 切换定位引擎（BBS / FPFH_RANSAC） |
| `/set_global_map` | `SetGlobalMap` | 发送全局点云地图（sensor_msgs/PointCloud2） |
| `/query` | `QueryGlobalLocalization` | 传入当前扫描，返回位姿候选列表 |

### 3.3 服务 QoS 与 /initialpose 兼容性

RTAB-Map 订阅 `/initialpose` 使用 **VOLATILE + RELIABLE** QoS。`global_localization_node.py` 在 RTAB-Map 已经订阅后才发布（Phase 2 约在启动 70s 后），因此使用默认 QoS 即可实时送达，无需 TRANSIENT_LOCAL。

### 3.4 Query 响应结构

```
# Response
float64[] inlier_fractions   # FPFH_RANSAC: 内点比例 (0~1)；BBS: 栅格命中得分（绝对值）
float64[] errors             # FPFH_RANSAC: 匹配误差(m)；BBS: 与 inlier_fraction 相同
geometry_msgs/Pose[] poses   # 候选位姿（位置 + 四元数，6-DOF）
```

**注意**：BBS 引擎返回的 `inlier_fraction` 和 `error` 数值相同（均为 BBS 得分），不是 0~1 的比例值，不可与 FPFH_RANSAC 的结果直接比较。

---

## 4. FPFH_RANSAC 引擎详解

### 4.1 算法原理

FPFH_RANSAC 是基于 **三维点特征直方图（FPFH）** 和 **随机采样一致性（RANSAC）** 的全局配准方法，源自 PCL 的 `SampleConsensusPrerejective` 实现，全程 OpenMP 多线程并行。

**完整算法流程：**

```
① 地图预处理（set_global_map 时执行，仅一次）
   地图点云（已由 Python 端预降采样至 ~9000 点）
       │
       ├─ hdl 节点内部再降采样（globalmap_downsample_resolution=0.2m）
       │
       ├─ 法向量估计（NormalEstimationOMP，半径=0.5m）
       │   每点在 0.5m 半径内找邻域，用 PCA 估计法向量
       │
       ├─ FPFH 特征提取（FPFHEstimationOMP，半径=1.5m）
       │   以法向量为基础，计算点对角度关系 → 33维直方图
       │
       └─ 用 FLANN 对地图 FPFH 特征建 KD 树（用于快速近邻查询）

② 查询（query 时执行）
   当前扫描（已降采样）
       │
       ├─ 同样做法向量估计 + FPFH 特征提取
       │
       └─ RANSAC 主循环（最大 100,000 次迭代，matching_budget=10,000）
              │
              ├─ 预计算：为扫描中每个点在地图 FPFH KD 树中查 k=2 个最近邻
              │
              ├─ 随机采样 3 个 源点-目标点 对应关系
              │
              ├─ 多边形一致性预拒绝（CorrespondenceRejectorPoly）
              │   检查3点构成的多边形形状相似度 > 0.5，不满足则跳过
              │
              ├─ SVD 求解刚体变换矩阵 T（4×4）
              │
              ├─ 计算内点比例：将扫描点用 T 变换后与地图点比较
              │   距离 < max_correspondence_distance(1.0m) 的点为内点
              │   inlier_fraction = 内点数 / 扫描总点数
              │
              └─ 若 inlier_fraction > 0.25 → 记录为候选结果
                 最终选 inlier_fraction 最高的结果返回
```

### 4.2 FPFH 特征描述子原理

FPFH（Fast Point Feature Histogram）描述局部几何形状：

1. 对点 p 及其 k 邻域内每个点 q，计算 3 个角度特征：
   - α：法向量与连线方向的角度
   - φ：切平面内的方位角
   - θ：切平面法向量夹角
2. 对这 3 个角度分别建立 11 bin 直方图，拼接成 **33 维**特征向量
3. 具有旋转不变性，对点密度变化有一定鲁棒性
4. 使用 FLANN（最快近似最近邻库）做特征匹配

### 4.3 参数配置（本系统调优后）

#### config_base.json（降采样分辨率）

```json
{
  "base": {
    "globalmap_downsample_resolution": 0.2,
    "query_downsample_resolution": 0.2
  }
}
```

#### C++ 代码默认值（`global_localization_fpfh_ransac.hpp`）

| 参数 | **原始默认值** | **本系统调优值** | 说明 |
|------|--------------|----------------|------|
| `normal_estimation_radius` | 2.0 m | **0.5 m** | 适合室内尺度，2m 半径在小场景特征不区分 |
| `search_radius` | 8.0 m | **1.5 m** | 关键！原来 8m ≈ 地图总长度，每个点特征几乎相同 |
| `max_correspondence_distance` | 1.0 m | 1.0 m | 未变 |
| `similarity_threshold` | 0.5 | 0.5 | 未变 |
| `correspondence_randomness` | 2 | 2 | 未变 |
| `max_iterations` | 100,000 | 100,000 | 未变 |
| `matching_budget` | 10,000 | 10,000 | 未变 |
| `min_inlier_fraction` | 0.25 | 0.25 | 未变 |

**调优原因**：原参数为室外大场景（数百米）设计：
- `search_radius=8m` 在 12m 宽的室内房间中，几乎覆盖整个地图，导致所有点 FPFH 特征趋于一致，无法区分不同位置，RANSAC 匹配误匹配率极高
- `normal_estimation_radius=2m` 对于 0.2m 间距的点云过大，法向量受远处点影响
- 调整后 `search_radius=1.5m`（约 7.5 × voxel_size），特征描述的是真正的局部几何

### 4.4 点云规模与耗时关系

| 场景 | 地图降采样后点数 | 地图 FPFH 耗时 | RANSAC 耗时 |
|---|---|---|---|
| 原参数（voxel=0.5m，radius=8m） | ~1999 点 | ~14-54 秒 | ~1 秒 |
| 调优参数（voxel=0.2m，radius=1.5m） | ~9000 点 | ~30-90 秒 | ~5-15 秒 |

点数增加但搜索半径缩小，邻域内点数减少，总体计算量相当。

---

## 5. BBS 引擎详解

### 5.1 算法原理

BBS（Branch-and-Bound Search）基于 **Hess et al., ICRA 2016**（Google Cartographer 回环检测算法），将全局定位转化为 2D 搜索问题，通过分支定界法找最优位姿，理论上保证全局最优。

```
① 地图预处理
   3D 地图点云
       │
       └─ Z 轴切片（map_min_z=2.0m ~ map_max_z=2.4m，提取天花板附近特征）
           → 2D 点集
           → 构建多分辨率占用栅格金字塔（6 层）
               第0层：0.5m/格（精细）
               第5层：16m/格（粗糙）

② 查询
   当前扫描
       └─ Z 轴切片（scan_min_z=-0.2m ~ scan_max_z=0.2m，提取地面附近点）
           → 2D 点集

   旋转分辨率：θ_res = arccos(1 - r²/(2d²)) ≈ 0.033 rad（约1.9°）
       其中 r=0.5m（地图分辨率），d=15m（最大探测距离）

   搜索范围：tx∈[-50,50]m，ty∈[-50,50]m，θ∈[-π,π]
   在第5层枚举所有初始变换 → 计算得分 → 最大优先队列

   Branch-and-Bound 主循环：
       取队列顶 → 得分低于当前最优则剪枝
       是叶节点(第0层) → 更新最优
       否则分裂为 4 个子节点（坐标细化2倍）→ 压入队列
```

### 5.2 参数（config_bbs.json）

| 参数 | 值 | 说明 |
|------|-----|------|
| `map_min_z / map_max_z` | 2.0 ~ 2.4 m | 地图切片高度（天花板/墙顶区域） |
| `scan_min_z / scan_max_z` | -0.2 ~ 0.2 m | 扫描切片高度（近地面区域） |
| `map_resolution` | 0.5 m | 占用栅格分辨率 |
| `map_pyramid_level` | 6 | 金字塔层数（决定搜索粗细） |
| `max_range` | 15.0 m | 扫描点有效距离 |
| `min_tx/max_tx`, `min_ty/max_ty` | ±50 m | 平移搜索范围 |

**本系统不使用 BBS 的原因**：本场景地图 z 范围为 [-0.35, 2.56]m，z=2.0~2.4m 区域点数极少（接近天花板），导致地图切片后有效点不足，定位失败率高。

---

## 6. 引擎对比

| 特性 | FPFH_RANSAC | BBS |
|------|-------------|-----|
| 维度 | 3D（6-DOF） | 2D（x, y, yaw） |
| 算法类型 | 特征匹配 + RANSAC | 占用栅格 + 分支定界 |
| 耗时（调优后） | 40~110 秒 | 1~3 秒 |
| 对噪声鲁棒性 | 中等（RANSAC 剔除异常值） | 高（栅格计分平滑） |
| 全局最优保证 | 否（随机采样） | 是（穷举+剪枝） |
| 需要法向量 | 是 | 否 |
| 返回 z/roll/pitch | 是（需手动归零） | 否（天然为 0） |
| 适用场景 | 室内外通用 | 平坦室内，z 切片有效点充足 |
| 本系统选择 | **√ 使用** | ✗ 不用 |

---

## 7. 地图准备流程

### 7.1 导出 PCD

建图完成后执行一次：

```bash
python3 scripts/extract_pcd_from_db.py
# 默认读 /data/maps/site_a/rtabmap.db
# 输出到 cloud_map/rtabmap_<timestamp>_cloud.pcd
```

脚本处理流程：
```
rtabmap.db (SQLite)
    ├─ Node 表：读取所有节点的优化后位姿（3×4 float32 变换矩阵）
    └─ Data 表：读取压缩激光扫描（zlib 压缩的 OpenCV Mat，格式 kXYZINormal）
        │
        ├─ 对每帧：zlib 解压 → 解析 OpenCV Mat → 提取 xyz
        ├─ 应用 scan_info 中的 local_transform（传感器外参）
        ├─ 应用节点位姿（变换到地图坐标系）
        └─ 所有帧合并 → 写出 PCD（binary float32）+ PLY（ascii）
```

### 7.2 Python 端预降采样（关键优化）

原始 PCD 可能有数十万点（本系统约 587,964 点），若直接通过 ROS2 服务发送：
- 消息大小 ≈ 587,964 × 12 bytes = **7MB**
- 通过 DDS 传输耗时长达 **225 秒**

优化方案：在 Python 端使用 open3d 预降采样后再发送：

```python
pcd = o3d.io.read_point_cloud(pcd_path)          # 587,964 点
pcd_down = pcd.voxel_down_sample(0.2)             # → ~9,000 点
# 消息大小: 9000 × 12 bytes ≈ 108 KB → 发送时间 < 2 秒
```

hdl 节点收到后会再做一次 0.2m 降采样（config_base.json），但因为已经预处理，实际效果不变。

---

## 8. Python 客户端实现细节

### 8.1 ROS2 服务同步调用与死锁问题

ROS2 的服务调用机制要求 `spin()` 持续运行才能处理服务响应，但 `spin()` 在回调中是阻塞的。若在 `spin()` 的回调内直接调用同步服务（`client.call()`），会产生死锁：

```
[问题] spin() 正在执行回调 → 调用 call() 等待响应 → 
       响应需要 spin() 处理 → spin() 被回调阻塞 → 死锁
```

**本系统解决方案**：将 `spin()` 放到独立的后台线程，主线程用 `call_async()` + 轮询完成同步调用：

```python
# 后台线程运行 spin
spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
spin_thread.start()

# 主线程"同步"调用服务（实际是 async + 轮询）
def _call_sync(self, client, request):
    future = client.call_async(request)
    while not future.done():
        time.sleep(0.01)   # 让出 GIL，让后台 spin 线程处理响应
    return future.result()

# 退出时正确清理顺序（顺序错误会导致 std::terminate 崩溃）
rclpy.shutdown()           # 1. 通知 spin() 退出
spin_thread.join(timeout=3.0)  # 2. 等待 spin 线程完成
node.destroy_node()        # 3. 最后销毁节点
```

### 8.2 仅提取 x, y, yaw（地面机器人约束）

FPFH_RANSAC 返回完整 6-DOF 位姿（含 z, roll, pitch）。对于地面机器人，z/roll/pitch 来自 RANSAC 的 3D 噪声，直接发给 RTAB-Map 会导致定位混乱（实测 z=2.57m，roll=-2.8rad）。

必须手动将其归零：

```python
# 从 RANSAC 结果中只取 x, y，从四元数中只取 yaw
yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

out.pose.pose.position.x = float(pose.position.x)
out.pose.pose.position.y = float(pose.position.y)
out.pose.pose.position.z = 0.0          # 强制归零
out.pose.pose.orientation.x = 0.0       # roll=0
out.pose.pose.orientation.y = 0.0       # pitch=0
out.pose.pose.orientation.z = math.sin(yaw / 2)
out.pose.pose.orientation.w = math.cos(yaw / 2)
```

### 8.3 重试机制

首次查询可能因 DDS 遗留请求等原因返回空结果，客户端支持最多 3 次重试：

```python
for attempt in range(1, max_retries + 1):
    arrived = self._result_event.wait(timeout=30.0)
    if not arrived:
        # 超时：FAST-LIO 未发布扫描
        return False
    res = self._call_sync(self._query_cli, req)
    if res.poses:
        break
    # 无结果：重置事件，等待下一帧扫描重试
    self._done = False
    self._result_event.clear()
```

---

## 9. 一键启动脚本架构

### 9.1 进程管理与清理

脚本中所有子进程在一个独立进程组（setsid）中启动，`Ctrl+C` 时整组销毁，不留孤儿进程：

```bash
# 用 setsid 让 bringup 在独立进程组里运行
setsid ros2 launch robot_bringup bringup.launch.py ... &
BRINGUP_PGID=$!  # setsid 后 PGID == PID

cleanup() {
    kill -- -"$BRINGUP_PGID"   # 杀整个进程组（含 rtabmap/fastlio/livox）
    kill "$HDL_PID"
    sleep 2
    # 兜底：强制清理残留进程
    pkill -9 -f "fastlio_mapping|livox_ros_driver2_node|rtabmap$|hdl_global_localization_node"
}
trap cleanup EXIT INT TERM
```

**历史问题**：最初使用 `kill $PID` 只杀 ros2 launch 父进程，rtabmap、fastlio、livox 等子进程变为孤儿继续占用 CPU，导致下次启动时多个进程竞争资源，RANSAC 耗时从 1 分钟暴增至 5 分钟以上。

### 9.2 Nav2 激活命令

Nav2 lifecycle manager 的 STARTUP 命令编号为 **0**（非 1）：

```bash
# command=0: STARTUP（configure + activate 所有节点）
# command=1: PAUSE
# command=2: RESUME
# command=3: RESET
ros2 service call /lifecycle_manager_navigation/manage_nodes \
    nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"
```

---

## 10. 实测数据记录

### 10.1 调优前（原始参数）

| 项目 | 值 |
|------|-----|
| 地图降采样 voxel | 0.5m → 1117 点 |
| 法向量半径 | 2.0 m |
| FPFH 搜索半径 | **8.0 m**（等于地图宽度的 2/3！） |
| RANSAC 结果 | inlier=76.8（实为 BBS 得分），位置偏差 >3m |
| 引擎实际执行 | BBS（set_engine 有时未生效） |

### 10.2 调优后（本系统参数）

| 项目 | 值 |
|------|-----|
| 地图点数（Python 预处理后） | 367,711 → ~9,000 点（0.2m voxel） |
| 扫描点数（hdl 内部处理后） | ~4,500 → ~300 点 |
| 法向量半径 | 0.5 m |
| FPFH 搜索半径 | 1.5 m |
| 地图 FPFH 计算耗时 | 约 30~90 秒（ARM CPU，OMP 并行） |
| RANSAC 耗时 | 约 5~15 秒 |
| 发送地图耗时 | < 2 秒（优化前：225 秒） |
| 定位结果（典型） | inlier=1.000，err≈0.25m |
| 总计（Phase 2） | 约 40~110 秒 |

---

## 11. 已知问题与后续优化方向

### 11.1 y 轴精度问题

实测 y 方向偏差约 3m（预期 y=0，实测 y≈-3.4）。可能原因：
- 场景中存在对称特征（相似走廊/墙壁），RANSAC 匹配到错误位置
- inlier=0.962 较高但非最优，说明在错误位置也有很好的几何一致性

**改进方向**：
- 增加 RANSAC 迭代次数（matching_budget: 10,000 → 50,000）
- 配合 ICP 精细配准作为后处理
- 使用 BBS 作为粗定位初值，再用 FPFH 精化

### 11.2 z 切片参数（BBS 专用）

本场景 z 范围 [-0.35, 2.56]m，BBS 默认切片 z∈[2.0, 2.4]m（天花板区域），点数极少。
若要启用 BBS，需调整为 z∈[1.5, 2.0]m 或其他有效高度段。

### 11.3 地图 PCD 手动维护

目前需要手动运行 `extract_pcd_from_db.py` 导出 PCD，然后才能运行定位脚本。后续可在 `start_with_global_localization.sh` 中加入自动检测逻辑：若 PCD 比 DB 文件旧则自动重新导出。

---

## 12. 文件结构索引

```
rtabmap_nav2_stack/
├── src/
│   └── hdl_global_localization-humble/          # 核心C++包（ROS2 Humble适配）
│       ├── src/
│       │   ├── hdl_global_localization_node_ros2.cpp  # ROS2主节点（已改默认引擎为FPFH_RANSAC）
│       │   └── hdl_global_localization/
│       │       ├── engines/
│       │       │   ├── global_localization_fpfh_ransac.cpp  # FPFH引擎实现
│       │       │   └── global_localization_bbs.cpp          # BBS引擎实现
│       │       ├── ransac/
│       │       │   └── ransac_pose_estimation.cpp           # RANSAC核心（基于PCL SampleConsensusPrerejective）
│       │       └── bbs/
│       │           └── bbs_localization.cpp                 # BBS核心（基于Cartographer回环检测）
│       ├── include/hdl_global_localization/
│       │   └── engines/
│       │       └── global_localization_fpfh_ransac.hpp      # 参数定义（已调优搜索半径）
│       └── config/
│           ├── config_base.json    # 降采样分辨率（已改为0.2m）
│           └── config_bbs.json     # BBS引擎参数
│
├── scripts/
│   ├── start_with_global_localization.sh  # 一键启动（三阶段 + 进程组清理）
│   ├── global_localization_node.py        # Python客户端（预降采样+发布initialpose）
│   └── extract_pcd_from_db.py             # 从rtabmap.db导出全局点云PCD
│
└── cloud_map/
    └── rtabmap_<timestamp>_cloud.pcd      # 导出的全局地图（建图后手动生成）
```

---

*基于 hdl_global_localization-humble 源码分析及本机调试实测，2026-05-17*
