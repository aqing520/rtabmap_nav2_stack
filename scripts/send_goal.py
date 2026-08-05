#!/usr/bin/env python3
import argparse
import math
import sys

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_to_quat(yaw):
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def shortest_angle(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


def clamp(value, low, high):
    return max(low, min(high, value))


class GoalSender(Node):
    def __init__(self, args):
        super().__init__("send_goal_with_yaw_guard")
        self.args = args
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, args.cmd_topic, 10)
        self.nav_client = ActionClient(self, NavigateToPose, args.action_name)

    def current_pose(self):
        deadline = self.get_clock().now() + Duration(seconds=self.args.tf_timeout)
        last_error = None
        while rclpy.ok() and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.args.map_frame,
                    self.args.base_frame,
                    rclpy.time.Time(),
                )
                t = tf.transform.translation
                yaw = quat_to_yaw(tf.transform.rotation)
                return float(t.x), float(t.y), yaw
            except TransformException as exc:
                last_error = exc
        raise RuntimeError("TF unavailable: %s -> %s (%s)" % (
            self.args.map_frame,
            self.args.base_frame,
            last_error,
        ))

    def send_navigation_goal(self):
        if not self.nav_client.wait_for_server(timeout_sec=self.args.action_timeout):
            raise RuntimeError("action server unavailable: %s" % self.args.action_name)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.args.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.args.x
        goal.pose.pose.position.y = self.args.y
        goal.pose.pose.position.z = 0.0
        qz, qw = yaw_to_quat(self.args.yaw)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(
            "Sending NavigateToPose: x=%.3f y=%.3f yaw=%.1fdeg"
            % (self.args.x, self.args.y, math.degrees(self.args.yaw))
        )
        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("NavigateToPose goal rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            return 0
        raise RuntimeError("NavigateToPose failed with status %d" % int(result.status))

    def rotate_in_place(self):
        rate = self.create_rate(self.args.rate)
        zero = Twist()
        self.get_logger().info(
            "Near-goal yaw only: rotating in place to %.1fdeg via %s"
            % (math.degrees(self.args.yaw), self.args.cmd_topic)
        )

        start = self.get_clock().now()
        while rclpy.ok():
            _, _, current_yaw = self.current_pose()
            error = shortest_angle(self.args.yaw, current_yaw)
            if abs(error) <= self.args.yaw_tolerance:
                for _ in range(5):
                    self.cmd_pub.publish(zero)
                    rclpy.spin_once(self, timeout_sec=0.02)
                self.get_logger().info("Yaw reached: error=%.3frad" % error)
                return 0

            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > self.args.rotate_timeout:
                self.cmd_pub.publish(zero)
                raise RuntimeError("rotate timeout, remaining yaw error=%.3frad" % error)

            cmd = Twist()
            speed = clamp(abs(error) * self.args.kp, self.args.min_wz, self.args.max_wz)
            cmd.angular.z = math.copysign(speed, error)
            self.cmd_pub.publish(cmd)
            rate.sleep()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("yaw_deg", nargs="?", type=float, default=0.0)
    parser.add_argument("--near-xy", type=float, default=0.25)
    parser.add_argument("--yaw-tolerance", type=float, default=0.20)
    parser.add_argument("--min-wz", type=float, default=0.12)
    parser.add_argument("--max-wz", type=float, default=0.45)
    parser.add_argument("--kp", type=float, default=0.8)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--rotate-timeout", type=float, default=20.0)
    parser.add_argument("--tf-timeout", type=float, default=3.0)
    parser.add_argument("--action-timeout", type=float, default=5.0)
    parser.add_argument("--cmd-topic", default="/cmd_vel_nav")
    parser.add_argument("--action-name", default="/navigate_to_pose")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_footprint")
    args = parser.parse_args()
    args.yaw = math.radians(args.yaw_deg)
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = GoalSender(args)
    try:
        x, y, _ = node.current_pose()
        distance = math.hypot(args.x - x, args.y - y)
        if distance <= args.near_xy:
            status = node.rotate_in_place()
        else:
            status = node.send_navigation_goal()
        return status
    except Exception as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
