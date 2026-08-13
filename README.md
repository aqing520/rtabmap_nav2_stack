# RTAB-Map Nav2 Stack

项目根目录的 `robot.sh` 是纯激光建图、导航和点云重定位的统一入口。
当前系统使用 Livox MID360、FAST-LIO、RTAB-Map 和 Nav2 CUDA MPPI，
不启动相机、视觉定位或 GPS；`rel` 模式按需启动 HDL 点云全局重定位。

## 1. 建图

### 1.1 启动方法

进入项目根目录后执行：

```bash
bash robot.sh map
```

脚本会自动加载 ROS 2 和当前工作空间环境，实际启动：

```bash
ros2 launch robot_bringup fastlio_mapping.launch.py
```

默认关键参数为：

```text
start_livox=true
start_camera=false
sensor_profile=lidar_only
use_sim_time=false
rviz=true
rtabmap_viz=false
delete_db_on_start=true
database_path=<项目根目录>/db/rtabmap.db
```

> **注意：** 每次执行 `bash robot.sh map` 都会删除同名旧数据库并重新建图。
> 如需保留旧地图，请先备份 `db/rtabmap.db`。

不需要 RViz 时：

```bash
bash robot.sh map rviz:=false
```

### 1.2 建图逻辑

```text
Livox MID360
  ├── /livox/lidar
  └── /livox/imu
           ↓
        FAST-LIO
  ├── /Odometry
  └── /cloud_registered_body
           ↓
   RTAB-Map（mapping 模式）
  ├── 发布 /map
  ├── 发布 map → odom
  └── 保存 db/rtabmap.db
```

- FAST-LIO 使用激光点云和 IMU 计算连续里程计。
- RTAB-Map 不使用图像，订阅去畸变点云 `/cloud_registered_body` 和
  FAST-LIO 里程计 `/Odometry`。
- 建图数据库固定保存在项目根目录的 `db/rtabmap.db`。
- `/map` 是 RTAB-Map 根据当前建图数据自动生成的，不依赖 PGM 文件。

建图完成后使用 `Ctrl+C` 正常结束，等待 RTAB-Map 关闭并写完数据库。

### 1.3 建图结果检查

```bash
# 数据库应存在且非空
ls -lh db/rtabmap.db

# 检查关键数据
ros2 topic hz /cloud_registered_body
ros2 topic hz /Odometry
ros2 topic echo /map --once
```

从数据库提取三维点云：

```bash
python3 scripts/extract_pcd_from_db.py
```

该脚本默认读取 `db/rtabmap.db`，点云输出到 `db/pcd/`：

```text
db/pcd/rtabmap_<时间戳>_cloud.pcd
```

也可以通过第二个参数指定其他输出目录：

```bash
python3 scripts/extract_pcd_from_db.py \
  db/rtabmap.db \
  /其他输出目录
```

如需导出可编辑的二维 PGM/YAML 地图：

```bash
bash scripts/export_rtabmap_map_offline.sh \
  db/rtabmap.db \
  db/pgm_map/map
```

输出文件为：

```text
db/pgm_map/map.pgm
db/pgm_map/map.yaml
```

离线编辑命令：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch map_paint_editor_plugin map_paint_editor.launch.py \
  load_yaml_path:=db/pgm_map/map.yaml \
  save_yaml_path:=db/pgm_map/map.yaml
```

当前 `robot.sh nav` 默认直接使用 RTAB-Map 数据库生成的 `/map`，因此编辑后的
PGM 地图暂时不会参与导航，除非导航改为 `use_edited_map=true`。

## 2. 导航

### 2.1 启动方法

确保已经完成建图，并且以下数据库存在：

```text
db/rtabmap.db
```

启动导航：

```bash
bash robot.sh nav
```

脚本会自动：

1. 启动前执行 `pkill -f ros2`，清理上一轮 ROS 2 启动进程并等待 1 秒。
2. 检查 `db/rtabmap.db` 是否存在且非空。
3. 加载 `/opt/ros/humble/setup.bash`。
4. 加载 `~/cuda_robotics_ws/install/setup.bash`，或
   `CUDA_ROBOTICS_SETUP` 指定的 CUDA 工作空间。
5. 检查 `cuda_mppi_controller` 是否可用。
6. 以 `autostart=false` 启动 `robot_bringup/bringup.launch.py`。
7. 确认 `/cloud_registered_body` 和 `/Odometry` 都只有一个 publisher，
   并连续检查点云、里程计及 `odom → base_footprint` TF 的时间戳新鲜度。
8. 检查通过后才激活 Collision Monitor 和 Nav2。

默认关键参数为：

```text
mode=navigation
nav2_controller=cuda_mppi
sensor_profile=lidar_only
start_livox=true
start_camera=false
enable_gps=false
autostart=false（由 robot.sh 检查通过后手动激活）
enable_rviz=true
enable_collision_monitor=true
use_edited_map=false
database_path=<项目根目录>/db/rtabmap.db
```

不启动 RViz：

```bash
bash robot.sh nav enable_rviz:=false
```

临时关闭碰撞监控：

```bash
bash robot.sh nav enable_collision_monitor:=false
```

### 2.2 导航逻辑

```text
Livox 点云和 IMU
        ↓
     FAST-LIO
  ├── /Odometry
  └── /cloud_registered_body
        ↓
RTAB-Map（localization 模式）
  ├── 读取 db/rtabmap.db
  ├── 发布 /map
  └── 发布 map → odom
        ↓
      Nav2
  ├── 全局规划：Navfn
  ├── 局部控制：CUDA MPPI
  ├── 点云代价地图：STVL
  └── 输出 /cmd_vel_nav
        ↓
 Collision Monitor
        ↓
     /cmd_vel
```

`nav` 模式不启动视觉或 HDL 全局重定位，但 RTAB-Map 必须保持基础激光
定位模式，用于读取数据库并提供 Nav2 所需的 `map → odom`。机器人被搬动
后需要全局搜索位置时，使用后面的 `rel` 模式。

`map`、`nav`、`rel` 三种模式都会在启动前执行 `pkill -f ros2`。该工作流
假设机器人只运行本项目；如果同一台机器还运行其他 ROS 2 任务，它们也可能
被一并停止。默认等待 1 秒后再启动新栈，可通过下面的变量调整：

```bash
ROBOT_ROS2_CLEANUP_WAIT=2.0 bash robot.sh nav
```

`nav/rel` 另外要求连续 5 组数据的消息年龄不超过 0.5 秒。检查失败时
Nav2 保持 inactive，本次 launch 进程组会被停止。阈值可临时调整：

```bash
ROBOT_SENSOR_CHECK_TIMEOUT=30.0 \
ROBOT_SENSOR_MAX_AGE=0.5 \
ROBOT_SENSOR_REQUIRED_SAMPLES=5 \
bash robot.sh nav
```

只在明确知道风险的调试场景下，才可关闭 `nav/rel` 的数据新鲜度检查。
启动前的 `pkill -f ros2` 仍然会执行：

```bash
ROBOT_STARTUP_CHECKS=false bash robot.sh nav
```

### 2.3 Nav2 参数

默认读取：

```text
src/robot_bringup/config/nav2_cuda_mppi.yaml
```

主要配置：

- 局部控制器：`cuda_mppi_controller::CudaMppiController`
- 运动模型：`DiffDrive`
- 控制频率：`15 Hz`
- 最大线速度：`0.60 m/s`
- 最大角速度：`1.00 rad/s`
- 全局规划器：`nav2_navfn_planner/NavfnPlanner`
- 局部代价地图：`odom` 坐标系，订阅 `/cloud_registered_body`
- 全局代价地图：`map` 坐标系，静态层订阅 `/map`
- 点云障碍层：`spatio_temporal_voxel_layer`

因为 `use_edited_map=false`，导航不会启动 `map_server`，也不会读取
`db/pgm_map/map.yaml`。当前地图来源为：

```text
db/rtabmap.db → RTAB-Map → /map → Nav2
```

### 2.4 导航检查

```bash
# 检查主要节点
ros2 node list

# 检查定位和地图
ros2 topic echo /map --once
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered_body

# 检查 Nav2 控制器
ros2 node info /controller_server

# 检查速度链路
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel
```

### 2.5 纯点云重定位启动

机器人被搬动、不能继续使用上一次初始位置时，执行：

```bash
bash robot.sh rel
```

`rel` 模式要求以下两个地图产物来自同一次建图：

```text
db/rtabmap.db
db/pcd/*.pcd
```

`rel` 默认先调用 `scripts/extract_pcd_from_db.py`，从当前
`db/rtabmap.db` 自动导出一份带时间戳的新 PCD 到 `db/pcd`，然后使用刚导出
的最新文件进行 HDL 匹配。

默认的 `legacy_fpfh_v1` 配置会保留地图和实时查询的全部点：不做高度裁剪、
目标点数缩减或体素降采样。这里的“不降采样”与缓存 PCD 的二进制压缩存储
不是一回事；缓存文件仍使用无损的 binary-compressed 格式，读取后的点数和
坐标不变。由其他 HDL 引擎使用的基础 `0.2m` 降采样默认值保持不变。

从旧的降采样配置切换后，需要为同一张 PCD 重建一次 FPFH 缓存：

```bash
PCD_PATH=/absolute/path/to/map.pcd \
FORCE_REBUILD_CACHE=true \
bash robot.sh cache
```

如果已经准备好指定的 PCD，可以显式传入；此时跳过自动导出：

```bash
PCD_PATH=/absolute/path/to/map.pcd bash robot.sh rel
```

仅调试时如需跳过自动导出并使用 `db/pcd` 中已有的最新 PCD：

```bash
RELOCALIZATION_EXPORT_PCD=false bash robot.sh rel
```

启动顺序为：

```text
从 db/rtabmap.db 自动导出当前重定位 PCD
              ↓
Livox + FAST-LIO + RTAB-Map + HDL
              ↓
Nav2 保持 inactive
              ↓
确认点云、里程计、odom→base_footprint TF 连续新鲜
              ↓
HDL 地图特征加载完成后丢弃加载期间缓存的旧扫描
              ↓
等待新的 /cloud_registered_body 与全局 PCD 匹配
              ↓
等待 RTAB-Map 订阅者建立
              ↓
单次 TRANSIENT_LOCAL 发布 /initialpose
              ↓
相信已发布的 /initialpose，不做 localization_pose/TF 二次确认
              ↓
短暂等待消息交付后客户端退出并清除 latch
              ↓
激活 Collision Monitor 和 Nav2
```

自动重定位失败时，Nav2 会继续保持 inactive，终端提示用户在 RViz 中使用
`2D Pose Estimate` 手动发布 `/initialpose`。脚本收到新的人工位姿后直接
激活 Collision Monitor 和 Nav2，不再检查 `/localization_pose`、协方差或
`map→base_footprint` TF 是否与初始位姿一致。按 Ctrl+C 可以放弃人工定位
并停止系统。

自动发布不是周期发布，也不再连续发布三次。客户端先等待至少一个
`/initialpose` 订阅者，再发布一次 transient-local 消息。短暂等待 DDS
交付后客户端退出，publisher 被销毁，缓存的 latch 随之消失。

如需在自动重定位失败后直接退出，不等待人工位姿：

```bash
MANUAL_INITIALPOSE_FALLBACK=false bash robot.sh rel
```

默认还会拒绝使用修改时间早于数据库的 PCD，避免数据库和点云地图不一致。
仅调试旧地图时可以临时：

```bash
ALLOW_STALE_PCD=true bash robot.sh rel
```

常用参数：

```bash
RELOCALIZATION_ENGINE=FPFH_RANSAC \
RELOCALIZATION_MIN_INLIER=0.98 \
RELOCALIZATION_MAX_RETRIES=1 \
bash robot.sh rel enable_rviz:=false
```

## 3. 编译说明

### 3.1 编译方法

所有命令均在项目根目录执行。开始前确认没有其他编译进程：

```bash
pgrep -af 'colcon build|cmake --build'
```

首次编译或彻底重建：

```bash
source /opt/ros/humble/setup.bash

CLEAN_BUILD=1 \
JOBS=8 \
WORKERS=4 \
HEAVY_JOBS=1 \
bash scripts/do_build_all.sh
```

日常增量编译只需将 `CLEAN_BUILD` 改为 `0`：

```bash
CLEAN_BUILD=0 bash scripts/do_build_all.sh
```

编译完成后加载环境：

```bash
source install/setup.bash
```

内存不足时可降低并行度：

```bash
JOBS=4 WORKERS=2 HEAVY_JOBS=1 bash scripts/do_build_all.sh
```

### 3.2 本次编译问题

`rtabmap_util` 链接阶段出现以下错误，导致依赖它的包被跳过：

```text
librtabmap_util_plugins.so: undefined reference to
rtabmap_util::MapsManager::MapsManager()
```

检查确认：

- `MapsManager.cpp` 和 CMake 配置正常。
- 源码与旧工作空间中的可编译版本一致。
- 不需要修改源码或复制旧二进制库。
- 根据日志判断，很可能是重复或并发构建时产生了共享库链接竞争。

确认没有其他编译进程后，串行重编该包：

```bash
source /opt/ros/humble/setup.bash
source scripts/use_rtabmap_0234_env.sh

colcon build \
  --executor sequential \
  --parallel-workers 1 \
  --symlink-install \
  --allow-overriding rtabmap_util \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DOpenCV_DIR=/usr/lib/cmake/opencv4 \
  --packages-select rtabmap_util
```

本次串行重编成功，且没有修改 `rtabmap_util` 源码。随后执行增量编译即可继续构建剩余包：

```bash
CLEAN_BUILD=0 bash scripts/do_build_all.sh
```

### 3.3 失败时先检查

1. 不要同时运行多个 `colcon build`。
2. 查看 `log/latest_build/<包名>/stderr.log`。
3. 先单独重编失败包，再继续增量编译。
4. 不要复制其他工作空间的 `build/`、`install/` 或动态库。
