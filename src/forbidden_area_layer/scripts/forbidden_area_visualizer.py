#!/usr/bin/env python3
"""
禁行区可视化节点
定期调用 get_forbidden_areas 服务，将禁行区以 MarkerArray 发布到 RViz。

每个禁行区发布两种 Marker：
  - TRIANGLE_LIST（扇形三角剖分）：填充的半透明红色多边形
  - LINE_STRIP：红色边框轮廓
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, TransformStamped
from std_msgs.msg import ColorRGBA
from tf2_ros import StaticTransformBroadcaster
from forbidden_area_layer.srv import GetForbiddenAreas


FRAME_ID   = "map"
TOPIC      = "/forbidden_areas_markers"
POLL_HZ    = 1.0          # 每秒轮询一次
Z_HEIGHT   = 0.05         # 稍高于地面，避免被地图遮挡
FILL_ALPHA = 0.35
LINE_ALPHA = 0.9
LINE_WIDTH = 0.05         # 轮廓线宽（米）


def _color(r, g, b, a) -> ColorRGBA:
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


def _pt(x, y, z=Z_HEIGHT) -> Point:
    p = Point()
    p.x, p.y, p.z = float(x), float(y), float(z)
    return p


def _centroid(pts):
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    return cx, cy


def _fill_marker(idx: int, pts) -> Marker:
    """扇形三角剖分 → TRIANGLE_LIST 填充多边形。"""
    m = Marker()
    m.header.frame_id = FRAME_ID
    m.ns   = "forbidden_fill"
    m.id   = idx
    m.type = Marker.TRIANGLE_LIST
    m.action = Marker.ADD
    m.scale.x = m.scale.y = m.scale.z = 1.0
    m.color = _color(0.9, 0.1, 0.1, FILL_ALPHA)

    cx, cy = _centroid(pts)
    n = len(pts)
    for i in range(n):
        j = (i + 1) % n
        m.points.append(_pt(cx, cy))
        m.points.append(_pt(pts[i][0], pts[i][1]))
        m.points.append(_pt(pts[j][0], pts[j][1]))
    return m


def _outline_marker(idx: int, pts) -> Marker:
    """LINE_STRIP 轮廓。"""
    m = Marker()
    m.header.frame_id = FRAME_ID
    m.ns   = "forbidden_outline"
    m.id   = idx
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.scale.x = LINE_WIDTH
    m.color = _color(0.9, 0.1, 0.1, LINE_ALPHA)

    for x, y in pts:
        m.points.append(_pt(x, y))
    m.points.append(_pt(pts[0][0], pts[0][1]))   # 闭合
    return m


def _label_marker(idx: int, pts) -> Marker:
    """区域编号文字标签。"""
    cx, cy = _centroid(pts)
    m = Marker()
    m.header.frame_id = FRAME_ID
    m.ns   = "forbidden_label"
    m.id   = idx
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.pose.position = _pt(cx, cy, Z_HEIGHT + 0.3)
    m.pose.orientation.w = 1.0
    m.scale.z = 0.25
    m.color = _color(1.0, 1.0, 1.0, 1.0)
    m.text = f"禁行区 {idx + 1}"
    return m


class ForbiddenAreaVisualizer(Node):
    def __init__(self):
        super().__init__("forbidden_area_visualizer")
        self._pub = self.create_publisher(MarkerArray, TOPIC, 10)
        self._cli = self.create_client(
            GetForbiddenAreas,
            "/global_costmap/get_forbidden_areas")
        self._timer = self.create_timer(1.0 / POLL_HZ, self._poll)
        self._prev_count = -1

        # 发布静态 TF：map → map（确保 map frame 在 RViz 中存在）
        self._tf_broadcaster = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "map"
        tf.child_frame_id = "map"
        tf.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(tf)

        self.get_logger().info(
            f"ForbiddenAreaVisualizer started → topic: {TOPIC}")

    def _poll(self):
        if not self._cli.service_is_ready():
            return

        fut = self._cli.call_async(GetForbiddenAreas.Request())
        fut.add_done_callback(self._on_response)

    def _on_response(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().warn(f"Service call failed: {e}")
            return

        areas = result.areas
        n = len(areas)
        now = self.get_clock().now().to_msg()

        array = MarkerArray()

        # 先发 DELETE_ALL 清除上一帧多余的 marker
        if self._prev_count > n:
            del_m = Marker()
            del_m.header.stamp = now
            del_m.header.frame_id = FRAME_ID
            del_m.action = Marker.DELETEALL
            array.markers.append(del_m)

        for idx, area in enumerate(areas):
            pts = [(p.x, p.y) for p in area.points]
            if len(pts) < 3:
                continue
            for m in [_fill_marker(idx, pts),
                      _outline_marker(idx, pts),
                      _label_marker(idx, pts)]:
                m.header.stamp = now
                array.markers.append(m)

        self._pub.publish(array)

        if n != self._prev_count:
            self.get_logger().info(f"Published {n} forbidden area(s)")
            self._prev_count = n


def main():
    rclpy.init()
    node = ForbiddenAreaVisualizer()
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
