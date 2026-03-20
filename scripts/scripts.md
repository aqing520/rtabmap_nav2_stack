# RTAB-Map ROS2 业务脚本指南 (Scripts Guide)

> 本文档详细说明了 `scripts/` 目录下各个自动化运维与启动脚本的作用和调用关系。

## 0. ⚠️ 关于 `robot_bringup` 的现状说明

**重要提示：**
当前项目中的 `src/robot_bringup` 目录（包含 Launch 文件和 Nav2/EKF 的 `.yaml` 配置文件）**目前处于未启用状态**。
* **原因**：该目录缺少 ROS 2 标准的功能包元数据文件（即 `package.xml` 和 `CMakeLists.txt`），因此无法被 `colcon build` 识别和编译为有效的 ROS 包。
* **现状**：如果你强行运行 `ros2 launch robot_bringup ...`，系统会报错 `PackageNotFoundError`。当前系统真正的实车硬件接驳、异常恢复以及核心建图流程，**完全由本 `scripts/` 目录下的 Shell 脚本接管和驱动**。

---

## 1. 脚本调用关系树

整个脚本体系呈清晰的“总-分”结构，调用链如下：

```text
运维工程师 / 开发者
 ├──> 🛠️ check_mapping.sh (多功能诊断与快捷入口)
 │    ├──> status (日常“体检”，无需调用其他脚本)
 │    ├──> demo-up / demo-play (拉起纯仿真离线算法池，播放数据集)
 │    └──> real-up (一键实车建图)
 │         └──> 🚀 start_all_mapping.sh (总控大管家) 
 │              ├──> (内部逻辑) 探测雷达、杀除僵尸进程、监控话题防断流
 │              └──> 🧠 start_rtabmap_mapping.sh (算法配置核心) 
 │                   └──> (拉起真正的底层算法) ros2 launch rtabmap_launch ...
 │
 ├──> 📦 build_rtabmap_0234.sh (在需要重新编译源码时调用)
 │
 ├──> 🔗 use_rtabmap_0234_env.sh (挂载局部环境变量，供 colcon 编译时寻找隔离库)
 │
 └──> ☁️ export_cloud_map.sh (建图结束后，独立运行生成通用 .ply 点云)
```

---

## 2. 核心脚本详解与使用姿势

### 🚀 2.1 `start_all_mapping.sh` (总控管家脚本)
* **定位**：对外部环境和物理硬件的强力整合器。
* **职责**：
  * **环境扫雷**：清理上次异常退出的雷达进程，防止端口被锁死。
  * **容错与自愈**：不会傻等，如果探测到雷达断流（线松了或崩溃），它会自动介入并重启雷达驱动。
  * **调度**：在数据流（Lidar、Odom）铺路完成后，正式请出 `start_rtabmap_mapping.sh`。
* **用法**：
  ```bash
  # 直接在实车上跑建图
  bash scripts/start_all_mapping.sh
  ```

### 🧠 2.2 `start_rtabmap_mapping.sh` (高阶算法配置器)
* **定位**：SLAM 专属的“参数调音台”。
* **职责**：
  * **解耦硬件**：只关心数学算法，不关心外界硬件死活。它接收上级传来的话题名，动态探测 TF 坐标树。
  * **注入参数**：向底层的 RTAB-Map 注入专门针对 LiDAR 调优过的数十个 ICP（迭代最近点）匹配参数，以及空间对齐策略。
* **用法**：
  *(通常不需要用户手动执行，由 `start_all_mapping.sh` 自动传参调用)*

### 🛠️ 2.3 `check_mapping.sh` (调试与诊断的瑞士军刀)
* **定位**：算法调参和现场查错的神器。
* **用法**：
  ```bash
  # 1. 现场出问题了，看一眼是雷达挂了还是算法没拉起来
  bash scripts/check_mapping.sh status

  # 2. 回到工位，播放录好的小车跑图数据包进行离线调参
  bash scripts/check_mapping.sh demo-up
  bash scripts/check_mapping.sh demo-play bags/demo_mapping/demo_mapping.db3

  # 3. 准备就绪，一键快捷开启实车建图（等同于手敲 start_all_mapping）
  bash scripts/check_mapping.sh real-up
  ```

### ☁️ 2.4 `export_cloud_map.sh` (独立后处理工具)
* **定位**：极其纯粹的离线转换工具。
* **职责**：在建图使命结束后，把厚重的、只给机器看的 `.db` 关系型数据库，抽离并转换为人眼与 3D 软件（如 Unity、Blender、MeshLab）普遍认同的 `.ply` 彩色点云文件。
* **用法**：
  ```bash
  # 默认无参运行（自动去 ~/.ros/ 捞取最新的 rtabmap.db 并导出到 cloud_map/）
  bash scripts/export_cloud_map.sh

  # 指定特定的商业大包进行转换
  bash scripts/export_cloud_map.sh /data/maps/site_a/rtabmap.db
  ```

### 📦 2.5 编译基建脚本
* **`build_rtabmap_0234.sh`**：隔离编译第三方的 RTAB-Map 0.23.4 纯净核心。
* **`use_rtabmap_0234_env.sh`**：**仅在触发 colcon build 编译时使用**。它向 CMake 暴露上述库的绝对路径，一旦打包出节点二进制文件（通过 RPATH 机制烧录位置），它在运行态下就彻底完成了历史使命。
