#!/usr/bin/env python3
"""
禁行区演示服务端
独立运行（不依赖 nav2/costmap），提供 get/set_forbidden_areas 服务。
启动时预加载几个演示矩形禁行区，供 RViz 可视化测试使用。

用法：
  python3 forbidden_area_demo_server.py
"""

import threading
import rclpy
from rclpy.node import Node
from forbidden_area_layer.srv import GetForbiddenAreas, SetForbiddenAreas
from forbidden_area_layer.msg import ForbiddenArea
from geometry_msgs.msg import Point


GET_SERVICE = "/global_costmap/get_forbidden_areas"
SET_SERVICE = "/global_costmap/set_forbidden_areas"


def _make_rect(x1, y1, x2, y2):
    area = ForbiddenArea()
    for x, y in [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]:
        p = Point(); p.x = float(x); p.y = float(y); p.z = 0.0
        area.points.append(p)
    return area


# 预加载的演示禁行区 [x1, y1, x2, y2]（单位：米）
DEMO_AREAS = [
    (1.0,  1.0,  3.0,  3.0),
    (-2.0, 0.5, -0.5,  2.5),
    (0.5, -3.0,  2.5, -1.5),
]


class ForbiddenAreaDemoServer(Node):
    def __init__(self):
        super().__init__("forbidden_area_demo_server")
        self._lock = threading.Lock()
        self._areas = [_make_rect(*r) for r in DEMO_AREAS]

        self.create_service(GetForbiddenAreas, GET_SERVICE, self._get_cb)
        self.create_service(SetForbiddenAreas, SET_SERVICE, self._set_cb)

        self.get_logger().info(
            f"Demo server ready. Preloaded {len(self._areas)} forbidden areas.")
        self.get_logger().info(
            "Use manage_forbidden_areas.py to change areas at runtime.")

    def _get_cb(self, _req, resp):
        with self._lock:
            resp.areas = list(self._areas)
        return resp

    def _set_cb(self, req, resp):
        new = [a for a in req.areas if len(a.points) >= 3]
        with self._lock:
            self._areas = new
        resp.success = True
        resp.message = f"Updated {len(new)} forbidden areas"
        self.get_logger().info(resp.message)
        return resp


def main():
    rclpy.init()
    node = ForbiddenAreaDemoServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
