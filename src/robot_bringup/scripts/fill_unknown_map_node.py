#!/usr/bin/env python3
"""Publish an OccupancyGrid with only small enclosed unknown holes cleared."""

from collections import deque

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


UNKNOWN = -1
FREE_MAX = 20
OCCUPIED_MIN = 50


class FillUnknownMap(Node):
    def __init__(self):
        super().__init__('fill_unknown_map')

        self.input_topic = self.declare_parameter('input_topic', '/map').value
        self.output_topic = self.declare_parameter('output_topic', '/map_fixed').value
        self.max_fill_area_m2 = float(self.declare_parameter('max_fill_area_m2', 4.0).value)
        self.obstacle_keepout_m = float(self.declare_parameter('obstacle_keepout_m', 0.7).value)

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.publisher = self.create_publisher(OccupancyGrid, self.output_topic, qos)
        self.subscription = self.create_subscription(
            OccupancyGrid,
            self.input_topic,
            self.map_callback,
            qos,
        )
        self.get_logger().info(
            'Filling only enclosed unknown map holes: %s -> %s, max_fill_area=%.2fm^2, obstacle_keepout=%.2fm'
            % (
                self.input_topic,
                self.output_topic,
                self.max_fill_area_m2,
                self.obstacle_keepout_m,
            )
        )

    def map_callback(self, msg: OccupancyGrid) -> None:
        width = msg.info.width
        height = msg.info.height
        if width == 0 or height == 0 or msg.info.resolution <= 0.0:
            return

        data = list(msg.data)
        max_fill_cells = max(0, int(round(self.max_fill_area_m2 / (msg.info.resolution ** 2))))
        keepout_cells = max(0, int(round(self.obstacle_keepout_m / msg.info.resolution)))

        near_obstacle = self._distance_mask(data, width, height, lambda value: value >= OCCUPIED_MIN, keepout_cells)
        fill_mask = self._enclosed_unknown_holes(data, width, height, max_fill_cells, near_obstacle)

        filled = 0
        for index, value in enumerate(data):
            if value == UNKNOWN and fill_mask[index]:
                data[index] = 0
                filled += 1

        fixed = OccupancyGrid()
        fixed.header = msg.header
        fixed.info = msg.info
        fixed.data = data
        self.publisher.publish(fixed)

        if filled:
            self.get_logger().debug('Filled %d unknown cells in %s' % (filled, self.output_topic))

    @staticmethod
    def _enclosed_unknown_holes(data, width, height, max_fill_cells, near_obstacle):
        visited = [False] * len(data)
        fill_mask = [False] * len(data)

        for start, value in enumerate(data):
            if value != UNKNOWN or visited[start]:
                continue

            component = []
            queue = deque([start])
            visited[start] = True
            touches_border = False
            touches_obstacle_keepout = False
            touches_free = False

            while queue:
                index = queue.popleft()
                component.append(index)

                x = index % width
                y = index // width
                if x == 0 or x == width - 1 or y == 0 or y == height - 1:
                    touches_border = True
                if near_obstacle[index]:
                    touches_obstacle_keepout = True

                for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
                    if nx < 0 or nx >= width or ny < 0 or ny >= height:
                        continue
                    next_index = ny * width + nx
                    next_value = data[next_index]

                    if next_value == UNKNOWN and not visited[next_index]:
                        visited[next_index] = True
                        queue.append(next_index)
                    elif 0 <= next_value <= FREE_MAX:
                        touches_free = True

            if (
                touches_free
                and not touches_border
                and not touches_obstacle_keepout
                and len(component) <= max_fill_cells
            ):
                for index in component:
                    fill_mask[index] = True

        return fill_mask

    @staticmethod
    def _distance_mask(data, width, height, seed_predicate, max_distance):
        mask = [False] * len(data)
        queue = deque()

        for index, value in enumerate(data):
            if seed_predicate(value):
                mask[index] = True
                queue.append((index, 0))

        while queue:
            index, distance = queue.popleft()
            if distance >= max_distance:
                continue

            x = index % width
            y = index // width
            for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
                if nx < 0 or nx >= width or ny < 0 or ny >= height:
                    continue
                next_index = ny * width + nx
                if mask[next_index]:
                    continue
                mask[next_index] = True
                queue.append((next_index, distance + 1))

        return mask


def main():
    rclpy.init()
    node = FillUnknownMap()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
