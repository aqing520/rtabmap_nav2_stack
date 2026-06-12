# 禁行区前端接口

本包提供了前端与后端禁行区数据交互的接口，包括服务和消息类型。

## 目录结构

```
forbidden_area_layer/
├── msg/               # 消息类型定义
│   └── ForbiddenArea.msg
├── srv/               # 服务类型定义
│   ├── GetForbiddenAreas.srv
│   └── SetForbiddenAreas.srv
├── src/               # 源代码
│   ├── forbidden_area_layer.cpp    # 禁行区层实现
│   ├── forbidden_area_layer.hpp    # 禁行区层头文件
│   └── forbidden_area_server.cpp   # 禁行区服务器实现
├── scripts/           # 脚本文件
│   └── forbidden_area_client.py    # 前端客户端示例
├── launch/            # 启动文件
│   └── forbidden_area_server.launch.py
├── CMakeLists.txt     # CMake 配置文件
├── package.xml        # 包配置文件
└── forbidden_area_layer.xml  # 插件描述文件
```

## 消息类型

### ForbiddenArea.msg

```
geometry_msgs/Point[] points
```

- `points`: 禁行区的顶点坐标列表

## 服务类型

### GetForbiddenAreas.srv

```
---
forbidden_area_layer/ForbiddenArea[] areas
```

- 响应：返回当前所有禁行区的列表

### SetForbiddenAreas.srv

```
forbidden_area_layer/ForbiddenArea[] areas
---
bool success
string message
```

- 请求：设置禁行区列表
- 响应：操作结果和消息

## 前端接口使用方法

### 1. 启动禁行区服务器

```bash
ros2 launch forbidden_area_layer forbidden_area_server.launch.py
```

### 2. 使用 Python 客户端

```python
from forbidden_area_layer.srv import GetForbiddenAreas, SetForbiddenAreas
from forbidden_area_layer.msg import ForbiddenArea
from geometry_msgs.msg import Point

# 创建客户端
client = rclpy.create_node('forbidden_area_client')
get_client = client.create_client(
    GetForbiddenAreas,
    '/global_costmap/global_costmap/get_forbidden_areas')
set_client = client.create_client(
    SetForbiddenAreas,
    '/global_costmap/global_costmap/set_forbidden_areas')

# 获取禁行区
get_request = GetForbiddenAreas.Request()
future = get_client.call_async(get_request)
rclpy.spin_until_future_complete(client, future)
areas = future.result().areas

# 设置禁行区
set_request = SetForbiddenAreas.Request()
# 创建禁行区
area = ForbiddenArea()
point1 = Point()
point1.x = 0.0
point1.y = 0.0
point1.z = 0.0
area.points.append(point1)
# 添加其他点...
set_request.areas.append(area)

future = set_client.call_async(set_request)
rclpy.spin_until_future_complete(client, future)
success = future.result().success
message = future.result().message
```

### 3. 使用示例脚本

```bash
ros2 run forbidden_area_layer forbidden_area_client.py
```

## 与导航系统集成

1. 在 `nav2_common.yaml` 中配置禁行区层：

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ['static_layer', 'obstacle_layer', 'forbidden_area_layer', 'inflation_layer']
      forbidden_area_layer:
        plugin: forbidden_area_layer::ForbiddenAreaLayer
        enabled: true
        forbidden_areas: [0.0, 0.0, 2.0, 2.0, 3.0, 3.0, 5.0, 5.0]
```

2. 启动导航系统和禁行区服务器：

```bash
# 启动导航系统，显式使用带禁行区插件的参数文件
ros2 launch robot_bringup bringup.launch.py \
  nav2_params_file:=/home/wheeltec/xz/rtabmap_nav2_stack/install/robot_bringup/share/robot_bringup/config/nav2_forbidden_area.yaml

# 启动禁行区服务器
ros2 launch forbidden_area_layer forbidden_area_server.launch.py
```

3. 前端可以通过服务接口动态修改禁行区数据

## 注意事项

- 禁行区数据会实时更新到成本地图中
- 前端修改禁行区后，导航规划会自动避开新的禁行区
- 建议在修改禁行区后重新规划路径
