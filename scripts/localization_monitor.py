#!/usr/bin/env python3
"""
Localization monitor — detect when RTAB-Map localization is wrong.

Detection methods (ICP localization mode, no visual features):
1. Covariance spike  — /localization_pose covariance suddenly grows
2. Pose jump         — position changes too fast (wrong match)
3. Proximity links   — odom_cache stops linking to database nodes (lost tracking)
4. Closest node      — closest database node distance grows (drifting away)

NOTE: loop_closure_id is ALWAYS 0 in pure ICP mode (no BoW dictionary).
      This is normal. Use odom_cache proximity links instead.

Usage:
    python3 scripts/localization_monitor.py [--threshold 2.0] [--jump-threshold 0.5]
"""

import sys
import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rtabmap_msgs.msg import Info


class LocalizationMonitor(Node):
    def __init__(self, threshold: float = 2.0, jump_threshold: float = 0.5,
                 max_closest: float = 5.0):
        super().__init__('localization_monitor')
        self.cov_threshold = threshold        # meters (std-dev)
        self.jump_threshold = jump_threshold  # meters per update
        self.max_closest = max_closest        # max closest node distance

        # state
        self.last_x = None
        self.last_y = None
        self.last_proximity_count = 0
        self.zero_proximity_frames = 0
        self.frame_count = 0

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization_pose',
            self._on_pose, qos,
        )
        self.create_subscription(
            Info,
            '/info',
            self._on_info, qos,
        )

        # periodic status (every 10s)
        self.create_timer(10.0, self._status_tick)

        self.get_logger().info(
            f'Monitor started: cov_thr={threshold}m, jump_thr={jump_threshold}m, '
            f'max_closest={max_closest}m'
        )
        self.get_logger().info(
            'NOTE: loop_closure_id=0 is NORMAL in ICP-only mode. '
            'Monitoring odom_cache proximity links instead.'
        )

    def _on_pose(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        cov = msg.pose.covariance
        var_x, var_y = cov[0], cov[7]
        std_xy = math.sqrt(max(var_x, var_y))

        # 1) Covariance spike
        if std_xy > self.cov_threshold:
            self.get_logger().warn(
                f'[COV] σ_x={math.sqrt(var_x):.3f}m, '
                f'σ_y={math.sqrt(var_y):.3f}m  (>{self.cov_threshold}m)'
            )

        # 2) Pose jump
        if self.last_x is not None:
            dx = x - self.last_x
            dy = y - self.last_y
            dist = math.sqrt(dx*dx + dy*dy)
            if dist > self.jump_threshold:
                self.get_logger().warn(
                    f'[JUMP] {dist:.3f}m in one update '
                    f'(thr={self.jump_threshold}m) — possible wrong match!'
                )

        self.last_x, self.last_y = x, y

    def _on_info(self, msg: Info):
        self.frame_count += 1

        # Count proximity links (type=2) in odom_cache
        prox_count = 0
        if msg.odom_cache and msg.odom_cache.links:
            for link in msg.odom_cache.links:
                if link.type == 2:  # proximity link to database node
                    prox_count += 1

        # 3) Proximity links lost
        if prox_count == 0 and self.last_proximity_count > 0:
            self.get_logger().warn(
                f'[LOST] All proximity links lost! '
                f'Was {self.last_proximity_count}, now 0 — localization broken!'
            )
        elif prox_count == 0:
            self.zero_proximity_frames += 1
        else:
            if self.zero_proximity_frames > 10:
                self.get_logger().info(
                    f'[RECOVER] Proximity links restored: {prox_count}'
                )
            self.zero_proximity_frames = 0

        self.last_proximity_count = prox_count

        # 4) Closest node distance
        closest_dist = self._extract_stat(msg, 'Memory/Closest_node_distance/m')
        if closest_dist is not None and closest_dist > self.max_closest:
            self.get_logger().warn(
                f'[DRIFT] Closest DB node is {closest_dist:.1f}m away '
                f'(thr={self.max_closest}m) — robot may be unmapped area!'
            )

    def _extract_stat(self, msg: Info, key: str):
        """Extract a value from stats_keys/stats_values."""
        try:
            idx = msg.stats_keys.index(key)
            return msg.stats_values[idx]
        except (ValueError, IndexError):
            return None

    def _status_tick(self):
        if self.last_x is not None:
            self.get_logger().info(
                f'[STATUS] pos=({self.last_x:.2f},{self.last_y:.2f}) '
                f'prox_links={self.last_proximity_count} '
                f'frames={self.frame_count}'
            )
        else:
            self.get_logger().info('[STATUS] Waiting for data ...')


def main():
    parser = argparse.ArgumentParser(description='Localization failure detector')
    parser.add_argument('--threshold', type=float, default=2.0,
                        help='Covariance std-dev threshold (m), default: 2.0')
    parser.add_argument('--jump-threshold', type=float, default=0.5,
                        help='Pose jump threshold (m), default: 0.5')
    parser.add_argument('--max-closest', type=float, default=5.0,
                        help='Max closest DB node distance (m), default: 5.0')
    args, _ = parser.parse_known_args(sys.argv[1:])

    rclpy.init()
    node = LocalizationMonitor(
        threshold=args.threshold,
        jump_threshold=args.jump_threshold,
        max_closest=args.max_closest,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
