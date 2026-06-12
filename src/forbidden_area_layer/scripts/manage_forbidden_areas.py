#!/usr/bin/env python3
"""
禁行区管理脚本
通过 set_forbidden_areas service 向 ForbiddenAreaLayer 写入矩形禁行区。

用法：
  python3 manage_forbidden_areas.py              # 使用脚本内默认区域
  python3 manage_forbidden_areas.py clear        # 清空所有禁行区
"""

import sys
import rclpy
from rclpy.node import Node
from forbidden_area_layer.srv import SetForbiddenAreas
from forbidden_area_layer.msg import ForbiddenArea
from geometry_msgs.msg import Point

SET_SERVICE = '/global_costmap/set_forbidden_areas'


def make_rect_area(x1, y1, x2, y2) -> ForbiddenArea:
    """将两个对角坐标转换为矩形多边形消息（4 顶点）。"""
    area = ForbiddenArea()
    for x, y in [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]:
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = 0.0
        area.points.append(p)
    return area


def main():
    rclpy.init()
    node = Node('manage_forbidden_areas')

    client = node.create_client(SetForbiddenAreas, SET_SERVICE)

    node.get_logger().info(f'Waiting for {SET_SERVICE} service...')
    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error(f'{SET_SERVICE} service not available')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    req = SetForbiddenAreas.Request()

    if len(sys.argv) > 1 and sys.argv[1] == 'clear':
        # 发送空列表以清空所有禁行区
        node.get_logger().info('Clearing all forbidden areas')
    else:
        # 默认矩形禁行区列表，按需修改 [x1, y1, x2, y2]
        rectangles = [
            (0.0, 0.0, 2.0, 2.0),
            (3.0, 3.0, 5.0, 5.0),
        ]
        req.areas = [make_rect_area(*r) for r in rectangles]
        node.get_logger().info(f'Setting {len(req.areas)} forbidden area(s)')

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        res = future.result()
        node.get_logger().info(
            f'{"OK" if res.success else "FAIL"}: {res.message}')
    else:
        node.get_logger().error('Service call returned no result')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
