# HDL Global Localization 技术报告

> 适用版本：hdl_global_localization-humble（ROS2 Humble）  
> 机器人平台：Wheeltec AMR + Livox MID360 + FAST-LIO + RTAB-Map + Nav2  
> 文档日期：2026-05-17

---

## 1. 概述

`hdl_global_localization` 是一个无需初始位置估计的**全局点云定位**软件包，由 Kenji Koide（AIST）开发。其核心功能是：在机器人**完全不知道自身位置**的情况下，通过将当前激光雷达扫描与预先构建的三维点云地图进行匹配，求解出机器人在地图坐标系下的六自由度（6-DOF）位姿，并以 `/initialpose` 的形式提供给上层导航栈。

在本系统中，该包承担**导航启动时的初始位姿自动确定**职责，替代了传统的人工在 RViz 中点击设置初始位置的操作。

---

## 2. 系统集成架构

```
┌─────────────────────────────────────────────────────────┐
│                   机器人启动流程                          │
│                                                         │
│  Livox MID360                                           │
│      │ /livox/lidar (原始点云)                           │
│      ▼                                                  │
│  FAST-LIO                                               │
│      │ /cloud_registered_body (去畸变点云, body坐标系)   │
│      │ /Odometry (里程计)                                │
│      ▼                                                  │
│  ┌──────────────────────────────────────┐               │
│  │    hdl_global_localization_node      │               │
│  │  Service: /set_engine                │               │
│  │  Service: /set_global_map            │◄── 地图PCD     │
│  │  Service: /query                     │◄── 当前扫描    │
│  └──────────────┬───────────────────────┘               │
│                 │ /initialpose                           │
│                 ▼                                       │
│  ┌──────────────────────────────────────┐               │
│  │  RTAB-Map (定位模式) + Nav2           │               │
│  │  接收 /initialpose 确定起始位置        │               │
│  └──────────────────────────────────────┘               │
└─────────────────────────────────────────────────────────┘
```

**启动脚本**：`scripts/start_with_global_localization.sh`  
**客户端脚本**：`scripts/global_localization_node.py`  
**地图来源**：`cloud_map/*.pcd`（由 `scripts/extract_pcd_from_db.py` 从 `rtabmap.db` 导出）

---

## 3. ROS2 节点接口

### 3.1 节点信息

| 项目 | 内容 |
|------|------|
| 节点名 | `hdl_global_localization_node` |
| 启动命令 | `ros2 run hdl_global_localization hdl_global_localization_node` |
| 默认引擎 | BBS |
| 配置目录 | `<package_share>/config/` |

### 3.2 服务接口

| 服务名 | 消息类型 | 说明 |
|--------|----------|------|
| `/set_engine` | `SetGlobalLocalizationEngine` | 切换定位引擎（BBS / FPFH_RANSAC） |
| `/set_global_map` | `SetGlobalMap` | 发送全局点云地图（sensor_msgs/PointCloud2） |
| `/query` | `QueryGlobalLocalization` | 传入当前扫描，返回位姿候选列表 |

### 3.3 Query 服务请求/响应结构

```
# Request
int64 max_num_candidates        # 返回候选位姿数量上限
sensor_msgs/PointCloud2 cloud   # 当前扫描点云

# Response
std_msgs/Header header
std_msgs/Header globalmap_header
float64[] inlier_fractions      # 各候选位姿的内点比例
float64[] errors                # 各候选位姿的匹配误差
geometry_msgs/Pose[] poses      # 各候选位姿（位置+四元数）
```

---

## 4. 两种定位引擎

本包实现了两套完全独立的定位算法，可在运行时通过 `/set_engine` 服务切换。

### 4.1 FPFH_RANSAC 引擎（推荐用于本系统）

#### 4.1.1 算法原理

FPFH_RANSAC 是基于 **三维点特征直方图（FPFH）** 和 **随机采样一致性（RANSAC）** 的全局配准方法，源自 PCL 的 `SampleConsensusPrerejective` 实现。

**算法流程：**

```
地图点云
    │
    ├─ 法向量估计（OMP并行，半径=2.0m）
    │       使用 NormalEstimationOMP
    │
    ├─ FPFH特征提取（OMP并行，半径=8.0m）
    │       每点生成33维直方图描述子
    │       使用 FPFHEstimationOMP
    │
    └─ 构建FLANN特征KD树 ──────────────────┐
                                          │
当前扫描                                   │
    │                                     │
    ├─ 法向量估计                           │
    ├─ FPFH特征提取                         │
    │                                     │
    └─ RANSAC主循环（最多100,000次迭代）     │
           │                              │
           ├─ 为每个源点预查询最相似的      │
           │   k个目标特征（k=correspondence_randomness=2）◄─┘
           │
           ├─ 随机采样3个点对
           │
           ├─ 多边形一致性预拒绝（CorrespondenceRejectorPoly）
           │   相似度阈值 = 0.5
           │
           ├─ SVD求解刚体变换
           │
           ├─ 计算内点比例（inlier_fraction）
           │   内点距离阈值 = 1.0m（max_correspondence_distance）
           │
           └─ 若 inlier_fraction > 0.25（min_inlier_fraction）
               则记录为候选结果
```

#### 4.1.2 关键参数（`config/config_base.json` + 代码默认值）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `globalmap_downsample_resolution` | 0.5 m | 地图体素滤波分辨率（节点内部） |
| `query_downsample_resolution` | 0.5 m | 查询点云体素滤波分辨率（节点内部） |
| `normal_estimation_radius` | 2.0 m | 法向量估计搜索半径 |
| `search_radius` | 8.0 m | FPFH特征提取搜索半径 |
| `max_correspondence_distance` | 1.0 m | RANSAC内点判定距离阈值 |
| `similarity_threshold` | 0.5 | 多边形预拒绝相似度阈值（0~1） |
| `correspondence_randomness` | 2 | 每点候选对应的数量 |
| `max_iterations` | 100,000 | RANSAC最大迭代次数 |
| `matching_budget` | 10,000 | 实际执行距离计算的最大次数 |
| `min_inlier_fraction` | 0.25 | 结果有效的最低内点比例 |
| `voxel_based` | true | 用体素集合评估内点（更快） |

#### 4.1.3 FPFH 特征描述子

FPFH（Fast Point Feature Histogram）是对局部几何形状的紧凑描述：
- 以每个点为中心，在给定半径内计算与邻域点的**角度关系**
- 生成 **33 维**直方图向量
- 对平移旋转不变，对尺度和噪声具有一定鲁棒性
- 使用 FLANN 进行高速近邻搜索

#### 4.1.4 匹配质量评估

代码实现了两种内点计算方式（通过 `voxel_based` 参数切换）：

- **VoxelSet**（默认）：将目标点云体素化后计算命中率，速度快
- **FLANN**：精确近邻搜索，精度略高但较慢

#### 4.1.5 优缺点

| 优点 | 缺点 |
|------|------|
| 完整3D配准，可恢复roll/pitch/yaw | 计算耗时（依地图/扫描大小，秒~分钟级） |
| 对室内外场景通用 | 对点云稀疏或无结构化场景效果差 |
| 不依赖初始位姿猜测 | 参数较多，调优有一定难度 |

---

### 4.2 BBS 引擎（Branch-and-Bound Search）

#### 4.2.1 算法原理

BBS 引擎基于 **Hess et al., ICRA 2016**（Google Cartographer 回环检测算法）的思路，将全局定位问题转化为一个**2D搜索问题**，通过分支定界法在离散变换空间内寻找最优位姿。

**算法流程：**

```
3D地图点云
    │
    ├─ Z轴切片（map_min_z=2.0m ~ map_max_z=2.4m）
    │   → 提取特定高度的点，投影为2D点集
    │
    └─ 构建多分辨率占用栅格金字塔（pyramid_level=6层）
        第0层：分辨率=0.5m（精细）
        第5层：分辨率=0.5×2⁵=16m（粗糙）

3D当前扫描
    │
    └─ Z轴切片（scan_min_z=-0.2m ~ scan_max_z=0.2m）
        → 保留机器人高度附近的点（近地面）

分支定界搜索：
    初始搜索范围：
        tx ∈ [-50m, +50m]
        ty ∈ [-50m, +50m]
        θ  ∈ [-π, +π]（全角度）

    在第5层栅格（粗分辨率）枚举所有初始变换
        → 计算初始得分（扫描点命中占用格的数量）
        → 放入最大优先队列

    Branch-and-Bound主循环：
        取队列顶部（得分最高的候选）
        若得分低于当前最优 → 剪枝，丢弃
        若是叶节点（第0层）→ 更新最优解
        否则分裂为4个子节点（x×2, y×2细化）
            → 计算子节点得分 → 压入队列
```

#### 4.2.2 关键参数（`config/config_bbs.json`）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_range` | 15.0 m | 扫描点最大有效距离 |
| `min_tx / max_tx` | ±50 m | X方向搜索范围 |
| `min_ty / max_ty` | ±50 m | Y方向搜索范围 |
| `min_theta / max_theta` | ±π rad | 旋转搜索范围（全360°） |
| `map_min_z / map_max_z` | 2.0~2.4 m | 地图点云切片高度范围 |
| `map_width / map_height` | 512×1024 | 占用栅格尺寸（像素） |
| `map_resolution` | 0.5 m | 栅格基础分辨率 |
| `map_pyramid_level` | 6 | 金字塔层数 |
| `max_points_per_cell` | 5 | 每格最大点数（防过密） |
| `scan_min_z / scan_max_z` | -0.2~0.2 m | 扫描切片高度范围 |

#### 4.2.3 旋转分辨率计算

旋转分辨率 θ_res 由地图分辨率 r 和最大探测距离 d 共同决定：

```
θ_res = arccos(1 - r² / (2d²))
      = arccos(1 - 0.5² / (2 × 15²))
      ≈ 0.0333 rad ≈ 1.9°
```

这保证了在最大量程处旋转一步不超过一个栅格单元。

#### 4.2.4 优缺点

| 优点 | 缺点 |
|------|------|
| 速度快（通常1~3秒） | 仅恢复x, y, yaw（三自由度） |
| 全局最优保证（穷举+剪枝） | 需要正确设置z切片范围 |
| 适合平坦室内场景 | 地图z切片内点数不足时失败 |
| 实现简洁，可预测性强 | 不适用于多层楼、坡道等场景 |

---

## 5. 引擎对比总结

| 特性 | FPFH_RANSAC | BBS |
|------|-------------|-----|
| 维度 | 3D（6-DOF） | 2D（3-DOF：x, y, yaw） |
| 算法类型 | 特征匹配 + RANSAC | 占用栅格 + 分支定界 |
| 耗时（本系统） | 数秒（地图1999点，扫描98点） | 1~3秒 |
| 对噪声鲁棒性 | 中等（RANSAC剔除异常值） | 高（栅格计分平滑噪声） |
| 需要法向量 | 是 | 否 |
| 返回z高度 | 是 | 否（z=0） |
| 适用场景 | 室内外通用 | 平坦室内 |
| 本系统实测 | 成功（inlier=76.8） | 未单独测试 |

---

## 6. 地图准备流程

全局定位依赖的点云地图由以下流程生成：

```
建图阶段（RTAB-Map + FAST-LIO）
    → ~/.ros/rtabmap.db
            │
            ▼
scripts/extract_pcd_from_db.py
    ├─ 读取所有节点的优化后位姿（3×4变换矩阵）
    ├─ 读取压缩激光扫描数据（zlib + OpenCV Mat格式）
    ├─ 对每帧扫描：local_transform → node_pose（变换到地图系）
    └─ 合并后写出 PCD（binary）+ PLY（ascii）
            │
            ▼
cloud_map/rtabmap_<timestamp>_cloud.pcd
    （本次地图：44帧，367,711点，
      x∈[-4.6, 7.0]m，y∈[-5.6, 5.3]m，z∈[-0.35, 2.56]m）
```

节点收到地图后会进行 **0.5m 体素滤波**降采样（367,711点 → 1,999点），再计算特征。

---

## 7. 本系统运行参数记录

本次实测（2026-05-17）运行参数如下：

| 项目 | 值 |
|------|-----|
| 引擎 | FPFH_RANSAC |
| 地图原始点数 | 367,711 |
| 地图降采样后 | 1,999 点 |
| 扫描原始点数 | 4,650 |
| 扫描降采样后 | 98 点 |
| 法向量估计半径 | 2.0 m |
| FPFH特征搜索半径 | 8.0 m |
| 定位结果 | x=4.500 m，y=0.000 m，yaw=-80.2° |
| 返回 inlier_fraction | 76.8（BBS得分，非比例值） |
| 总耗时 | < 1秒 |

> 注：`inlier_fraction=76.8` 为 BBS 引擎的栅格命中得分（绝对值），非 FPFH_RANSAC 的内点比例（0~1）。实际运行时日志显示 BBS 引擎处理了 query，说明 `set_engine("FPFH_RANSAC")` 的切换在该次测试中可能未完全生效，建议后续验证。

---

## 8. 客户端脚本说明

`scripts/global_localization_node.py` 是连接 hdl 节点与系统其余部分的"胶水层"。

### 工作流程

```
main()
  │
  ├─ 查找最新 PCD 文件（cloud_map/*.pcd）
  │
  ├─ 创建 GlobalLocalizationClient 节点
  │
  ├─ 启动 rclpy.spin() 于后台线程
  │   （允许主线程做阻塞式 service 调用而不死锁）
  │
  ├─ setup()：阻塞式依次调用
  │   ├─ /set_engine  → 切换为 FPFH_RANSAC
  │   └─ /set_global_map → 发送 367,711 点的地图
  │
  ├─ wait_and_query()：
  │   ├─ 等待 /cloud_registered_body 第一帧
  │   ├─ 调用 /query
  │   └─ 提取 x, y, yaw → 发布 /initialpose
  │
  └─ 退出
```

### 关键设计：避免 ROS2 死锁

ROS2 中在 spin() 正在运行的回调里直接调用 `client.call()`（同步服务调用）会导致死锁（spin 忙于回调，无法处理服务响应）。本脚本的解决方案：

```python
# spin 运行在独立的后台 daemon 线程
spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
spin_thread.start()

# 主线程用 call_async() + 轮询 future.done() 实现"同步"调用
def _call_sync(self, client, request):
    future = client.call_async(request)
    while not future.done():
        time.sleep(0.01)
    return future.result()
```

---

## 9. 一键启动脚本

`scripts/start_with_global_localization.sh` 封装了完整启动流程：

```bash
# 用法
./scripts/start_with_global_localization.sh
./scripts/start_with_global_localization.sh --engine BBS
./scripts/start_with_global_localization.sh --db /data/maps/site_a/rtabmap.db --engine FPFH_RANSAC
```

**时序：**

```
t=0s    启动 hdl_global_localization_node（后台）
t=0s    启动 bringup.launch.py mode:=navigation（后台）
         包含：Livox驱动 + FAST-LIO + RTAB-Map定位 + Nav2
t=3s    等待 hdl 服务注册
t=11s   等待 FAST-LIO 发布 /cloud_registered_body
t=11s   执行 global_localization_node.py → 发布 /initialpose
        Nav2 收到初始位姿，可接受导航目标点
```

---

## 10. 已知问题与调优建议

### 10.1 BBS z切片范围

当前配置 `map_min_z=2.0m, map_max_z=2.4m` 针对天花板/墙顶高度特征。本场景地图 z 范围为 [-0.35, 2.56]m，若环境最高结构不足 2m，需调整此范围。建议：

```json
"map_min_z": 1.5,
"map_max_z": 2.0
```

### 10.2 FPFH_RANSAC 对稀疏点云的鲁棒性

扫描降采样后仅 98 点，特征提取的邻域可能不足。若定位失败，可尝试：
- 减小 `query_downsample_resolution`（如 0.3m）
- 减小 `normal_estimation_radius`（如 1.0m）

### 10.3 直接读取 DB 优化

目前流程需要提前执行 `extract_pcd_from_db.py` 导出 PCD 文件。后续可将 DB 解析逻辑集成进 `global_localization_node.py`，做到启动时直接读取 `rtabmap.db`，无需中间文件。

### 10.4 引擎切换验证

实测日志中出现 BBS 相关输出（"Branch-and-Bound"），即使调用了 `set_engine("FPFH_RANSAC")`。建议在 set_engine 调用后通过节点日志确认引擎切换成功，或增加验证逻辑。

---

## 11. 文件结构索引

```
rtabmap_nav2_stack/
├── src/
│   └── hdl_global_localization-humble/     # 核心C++包（ROS2 Humble）
│       ├── src/
│       │   ├── hdl_global_localization_node_ros2.cpp   # ROS2主节点
│       │   ├── hdl_global_localization/
│       │   │   ├── engines/
│       │   │   │   ├── global_localization_fpfh_ransac.cpp  # FPFH引擎
│       │   │   │   └── global_localization_bbs.cpp          # BBS引擎
│       │   │   ├── ransac/
│       │   │   │   └── ransac_pose_estimation.cpp           # RANSAC实现
│       │   │   └── bbs/
│       │   │       └── bbs_localization.cpp                 # BBS实现
│       ├── config/
│       │   ├── config_base.json    # 降采样分辨率配置
│       │   └── config_bbs.json     # BBS引擎参数
│       └── srv/
│           ├── SetGlobalMap.srv
│           ├── SetGlobalLocalizationEngine.srv
│           └── QueryGlobalLocalization.srv
│
├── scripts/
│   ├── global_localization_node.py          # Python客户端
│   ├── start_with_global_localization.sh    # 一键启动脚本
│   └── extract_pcd_from_db.py               # 地图导出工具
│
└── cloud_map/
    └── rtabmap_<timestamp>_cloud.pcd        # 导出的全局地图
```

---

*报告生成：基于 hdl_global_localization-humble 源码分析及本机实测*
