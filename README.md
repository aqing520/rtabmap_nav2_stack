# RTAB-Map 导航项目（从零到一）
官方链接：https://github.com/introlab/rtabmap_ros

本仓库用于落地一套分层清晰的 ROS2 导航方案：

- `RTAB-Map` 负责全局建图/回环/重定位（发布 `map -> odom`）
- `robot_localization (EKF)` 负责本地连续里程计（发布 `odom -> base_footprint`）
- `Nav2` 负责规划、控制与避障执行
## 0.编译
```bash
do_build_all.sh  #直接编译就行
```

## 1. 仓库结构

```text
rtabmap_nav2_stack/                 # 工作空间
├── src/                            # ROS 2 工作空间源码目录
│   ├── robot_bringup/              # (未启用) 高级导航启动包组，包含 EKF 和 Nav2 的精细配置
│   │   ├── launch/                 # 启动脚本存放目录
│   │   │   ├── bringup.launch.py   # 总启动入口，控制全自动导航的各个模块
│   │   │   └── rtabmap_bridge.launch.py # 负责将 RTAB-Map 的输出桥接到 Nav2 栈
│   │   └── config/                 # 核心参数配置目录
│   │       ├── ekf_local.yaml      # 扩展卡尔曼滤波(EKF)配置，用于融合 Lidar, IMU, 轮速里程计
│   │       └── nav2_common.yaml    # Navigation 2 配置
│   └── rtabmap_ros/                # 官方 RTAB-Map ROS 2 包装层
│       ├── rtabmap_launch/         # （done）通用 Launch 入口，  目前的脚本走的就是这个
│       ├── rtabmap_slam/           # （done）SLAM 核心节点，负责建图、回环检测、图优化与重定位
│       ├── rtabmap_odom/           # （done）视觉/深度/激光前端里程计节点，提供局部连续位姿估计
│       ├── rtabmap_sync/           # （done）多传感器时间同步节点，将 RGB、Depth、Scan、IMU 整理成统一输入
│       ├── rtabmap_util/           # （done）点云、栅格、图像和 TF 处理工具节点集合
│       ├── rtabmap_msgs/           # （done）RTAB-Map  ROS 2 消息与服务类型定义
│       ├── rtabmap_conversions/    # （done）RTAB-Map C++ 核心数据结构与 ROS 消息/TF/OpenCV 之间的转换库
│       ├── rtabmap_rviz_plugins/   # （done）RViz 中显示地图图结构、点云、回环和调试信息的插件
│       ├── rtabmap_viz/            # （done）RTAB-Map 自带可视化界面节点，便于查看节点图、回环和局部地图
│       ├── rtabmap_costmap_plugins/# （done）给 Nav2提供3D体素地图能力，
│       ├── rtabmap_python/         # （done）Python 绑定与脚本接口，便于离线分析和轻量二次开发
│       ├── rtabmap_ros/            # （done）元包/聚合包，用于统一依赖导出和整体发布
│       ├── rtsp_camera_bridge/     # （不用理，不是原生包）RTSP 相机桥接节点，把网络视频流接入 ROS 图像话题
│       ├── rtabmap_examples/       # （done）单体传感器或典型设备（如 Realsense）的使用示例
│       └── rtabmap_demos/          # （done）完整机器人的离线建图仿真与演示程序
├── third_party/                    #
│   └── rtabmap-0.23.4/             # RTAB-Map C++ 核心算法源码，保证版本一致性
├── scripts/                        # 编译和配置脚本
│   ├── build_rtabmap_0234.sh       # （done）隔离编译 RTAB-Map 核心层脚本
│   └── use_rtabmap_0234_env.sh     # （done）供 colcon 编译时挂载核心库路径的环境脚本
```

## 2.建图过程  

`src/robot_bringup/` 目前是本地 `launch/config` 目录，不是独立 ROS 包。
下面只画本项目里实际参与建图的 ROS 包，以及它们和外部雷达驱动输入的关系。

### 2.1  包协作过程

```mermaid
flowchart LR
    IN["外部传感器输入
Livox / IMU / 相机"] --> RL["robot_localization
ekf_node"]
    IN --> LA["rtabmap_launch"]

    subgraph R1["当前实际主链"]
        RL --> ODOM_LOCAL["/odometry/local"]
        ODOM_LOCAL --> SL["rtabmap_slam"]
        LA -->|启动核心节点| SL
    end

    subgraph R2["RGBD / 双目模式下的独立同步"]
        LA --> SYNC_NODE["rtabmap_sync
rgbd_sync / stereo_sync"]
        SYNC_NODE --> SL
    end

    subgraph R3["rtabmap_slam 的内部依赖"]
        SL -.继承同步基类.-> SYNC_BASE["rtabmap_sync
CommonDataSubscriber"]
        SL -.消息接口.-> MSG["rtabmap_msgs"]
        SL -.数据转换.-> CONV["rtabmap_conversions"]
        SL -.地图/工具.-> UTIL["rtabmap_util"]
    end

    LA -.可选前端里程计.-> ODOM["rtabmap_odom"]
    LA -.可选可视化.-> VIZ["rtabmap_viz / RViz"]
```

- 当前实际主链：`robot_localization -> /odometry/local -> rtabmap_slam`
- `rtabmap_launch`：负责启动 `rtabmap_slam`，并按传感器模式决定是否先起独立同步节点
- 独立同步节点：只有在 `lidar_rgbd` / `lidar_stereo` 这类模式下，`rtabmap_launch` 才会先拉起 `rtabmap_sync`
- 代码级同步：即使没有独立同步节点，`rtabmap_slam` 内部仍然通过 `rtabmap_sync::CommonDataSubscriber` 处理订阅和同步
- `rtabmap_odom`：提供局部里程计/局部定位能力，但当前工程里被显式关闭，当前主链不使用它
- 当前桥接配置：`visual_odometry = false`，`icp_odometry = false`

### 2.2  关键数据流向

```mermaid
flowchart LR
    subgraph A["当前实际使用的支路"]
        W["/sensors/wheel/odom"] --> EKF["robot_localization
ekf_node"]
        LIO["/odometry/lio"] --> EKF
        IMU1["/sensors/imu/data"] --> EKF
        EKF --> ODOM_LOCAL["/odometry/local"]

        CLOUD1["/sensors/lidar/points_deskewed"] --> SLAM1["rtabmap_slam"]
        IMU2["/sensors/imu/data"] --> SLAM1
        ODOM_LOCAL --> SLAM1
        SLAM1 --> MAP1["/map"]
        SLAM1 --> TF1["TF: map -> odom"]
        SLAM1 --> DB1["rtabmap.db"]
    end

    subgraph B["可选的 rtabmap_odom 支路"]
        SENSOR["图像 / 点云 / 激光"] --> ODOM_NODE["rtabmap_odom"]
        IMU3["/sensors/imu/data"] --> ODOM_NODE
        ODOM_NODE --> ODOM_TOPIC["odom / odom_info"]
        ODOM_TOPIC --> SLAM2["rtabmap_slam"]
        SLAM2 --> MAP2["/map"]
        SLAM2 --> TF2["TF: map -> odom"]
    end
```

- 当前实际使用：`robot_localization` 把 `/sensors/wheel/odom + /odometry/lio + /sensors/imu/data` 融合成 `/odometry/local`，再喂给 `rtabmap_slam`
- 可选支路：如果启用 `rtabmap_odom`，它会自己发布 `odom / odom_info`，再把这一路结果给 `rtabmap_slam`
- 你当前工程配置里 `visual_odometry = false`、`icp_odometry = false`，所以现在走的是第一条，不是第二条

最终 TF 主链：

```text
map -> odom -> base_footprint -> base_link -> lidar/imu
```
