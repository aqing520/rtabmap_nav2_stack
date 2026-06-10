#!/usr/bin/env python3
"""Collect RViz clicked points and execute them with Nav2 FollowWaypoints."""

import json
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class MultiWaypointRoute(Node):
    def __init__(self):
        super().__init__('multi_waypoint_route')

        self.map_id = self.declare_parameter('map_id', 'site_a').value
        self.map_frame_id = self.declare_parameter('map_frame_id', 'map').value
        self.clicked_point_topic = self.declare_parameter('clicked_point_topic', '/clicked_point').value
        self.command_topic = self.declare_parameter('command_topic', '/multi_waypoint_route/command').value
        self.status_topic = self.declare_parameter('status_topic', '/multi_waypoint_route/status').value
        self.points_topic = self.declare_parameter('points_topic', '/multi_waypoint_route/points').value
        self.markers_topic = self.declare_parameter('markers_topic', '/multi_waypoint_route/markers').value
        self.action_name = self.declare_parameter('action_name', '/follow_waypoints').value
        self.wait_for_server_sec = float(self.declare_parameter('wait_for_server_sec', 5.0).value)
        self.default_final_yaw = float(self.declare_parameter('default_final_yaw', 0.0).value)
        self.allow_empty_frame_id = bool(self.declare_parameter('allow_empty_frame_id', False).value)

        self.points = []
        self.current_waypoint = 0
        self.route_start_index = 0
        self.active_goal_handle = None
        self.canceling_for_resume = False
        self.suppress_next_cancel_status = False
        self.state = 'IDLE'

        self.action_client = ActionClient(self, FollowWaypoints, self.action_name)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.points_pub = self.create_publisher(String, self.points_topic, 1)
        self.marker_pub = self.create_publisher(MarkerArray, self.markers_topic, 1)
        self.command_sub = self.create_subscription(String, self.command_topic, self.command_callback, 10)
        self.clicked_point_sub = self.create_subscription(
            PointStamped,
            self.clicked_point_topic,
            self.clicked_point_callback,
            10,
        )
        self.create_timer(2.0, self.publish_points)

        self.publish_status('IDLE', 'ready')
        self.publish_points()
        self.get_logger().info(
            'RViz waypoint collector ready: map_id=%s, frame=%s, click topic=%s, command topic=%s'
            % (self.map_id, self.map_frame_id, self.clicked_point_topic, self.command_topic)
        )

    def clicked_point_callback(self, msg):
        frame_id = msg.header.frame_id
        if not frame_id and self.allow_empty_frame_id:
            frame_id = self.map_frame_id
        if frame_id != self.map_frame_id:
            self.publish_status(
                'FAILED',
                'ignored point in frame %s, expected %s for map %s'
                % (frame_id or '<empty>', self.map_frame_id, self.map_id),
            )
            return

        point = {
            'x': float(msg.point.x),
            'y': float(msg.point.y),
            'z': float(msg.point.z),
        }
        self.points.append(point)
        self.publish_status('EDITING', 'added point %d on map %s' % (len(self.points) - 1, self.map_id))
        self.publish_points()

    def command_callback(self, msg):
        try:
            command = self._parse_command(msg.data.strip())
        except (json.JSONDecodeError, TypeError) as exc:
            self.publish_status('FAILED', 'invalid command: %s' % exc)
            return

        if command == 'start':
            self.start_route(0)
        elif command == 'cancel':
            self.cancel_route('canceled')
        elif command == 'pause':
            self.pause_route()
        elif command == 'resume':
            self.resume_route()
        elif command == 'skip':
            self.skip_waypoint()
        elif command == 'clear':
            self.clear_points()
        elif command == 'undo':
            self.undo_point()
        elif command == 'list':
            self.publish_points()
        elif command == 'status':
            self.publish_status(self.state, 'status requested')
        else:
            self.publish_status('FAILED', 'unknown command: %s' % command)

    @staticmethod
    def _parse_command(text):
        if not text:
            return 'status'
        if text.startswith('{'):
            data = json.loads(text)
            return data.get('command', data.get('cmd', 'start'))
        return text

    def start_route(self, start_index=0):
        if len(self.points) < 1:
            self.publish_status('FAILED', 'no RViz points collected')
            return
        if start_index >= len(self.points):
            self.publish_status('FINISHED', 'route already complete')
            return

        self.current_waypoint = start_index
        self.route_start_index = start_index

        if self.active_goal_handle is not None:
            self.canceling_for_resume = True
            self.active_goal_handle.cancel_goal_async().add_done_callback(
                lambda _future: self._send_route_goal(start_index)
            )
            return

        self._send_route_goal(start_index)

    def _send_route_goal(self, start_index):
        if not self.action_client.wait_for_server(timeout_sec=self.wait_for_server_sec):
            self.publish_status('FAILED', 'action server unavailable: %s' % self.action_name)
            return

        self.route_start_index = start_index
        goal = FollowWaypoints.Goal()
        goal.poses = self._build_poses()[start_index:]
        now = self.get_clock().now().to_msg()
        for pose in goal.poses:
            pose.header.stamp = now

        self.state = 'RUNNING'
        self.publish_status('RUNNING', 'started %d RViz points from waypoint %d' % (len(goal.poses), start_index))
        future = self.action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def _build_poses(self):
        poses = []
        for index, point in enumerate(self.points):
            yaw = self._yaw_for_point(index)
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame_id
            pose.pose.position.x = point['x']
            pose.pose.position.y = point['y']
            pose.pose.position.z = point['z']
            pose.pose.orientation.z = math.sin(yaw * 0.5)
            pose.pose.orientation.w = math.cos(yaw * 0.5)
            poses.append(pose)
        return poses

    def _yaw_for_point(self, index):
        if index + 1 < len(self.points):
            current = self.points[index]
            next_point = self.points[index + 1]
            return math.atan2(next_point['y'] - current['y'], next_point['x'] - current['x'])
        if index > 0:
            previous = self.points[index - 1]
            current = self.points[index]
            return math.atan2(current['y'] - previous['y'], current['x'] - previous['x'])
        return self.default_final_yaw

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.active_goal_handle = None
            self.publish_status('FAILED', 'goal rejected')
            return

        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.current_waypoint = self.route_start_index + int(feedback.current_waypoint)
        self.publish_status('RUNNING', 'current waypoint %d' % self.current_waypoint)

    def result_callback(self, future):
        self.active_goal_handle = None
        result = future.result()
        if self.canceling_for_resume:
            self.canceling_for_resume = False
            return
        if self.suppress_next_cancel_status and result.status == GoalStatus.STATUS_CANCELED:
            self.suppress_next_cancel_status = False
            return

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.current_waypoint = len(self.points)
            self.publish_status('FINISHED', 'RViz route complete')
        elif result.status == GoalStatus.STATUS_CANCELED:
            self.publish_status('CANCELED', 'route canceled')
        else:
            missed = list(getattr(result.result, 'missed_waypoints', []))
            self.publish_status('FAILED', 'route failed: missed=%s' % missed)

    def cancel_route(self, reason):
        if self.active_goal_handle is None:
            self.publish_status('IDLE', 'no active route')
            return
        self.state = 'CANCELED'
        self.suppress_next_cancel_status = False
        self.active_goal_handle.cancel_goal_async()
        self.publish_status('CANCELED', reason)

    def pause_route(self):
        if self.active_goal_handle is None:
            self.publish_status('IDLE', 'no active route')
            return
        self.state = 'PAUSED'
        self.suppress_next_cancel_status = True
        self.active_goal_handle.cancel_goal_async()
        self.publish_status('PAUSED', 'paused at waypoint %d' % self.current_waypoint)

    def resume_route(self):
        if not self.points:
            self.publish_status('FAILED', 'no route to resume')
            return
        self.start_route(self.current_waypoint)

    def skip_waypoint(self):
        if not self.points:
            self.publish_status('FAILED', 'no active route to skip')
            return
        self.start_route(self.current_waypoint + 1)

    def clear_points(self):
        if self.active_goal_handle is not None:
            self.cancel_route('cleared active route')
        self.points = []
        self.current_waypoint = 0
        self.route_start_index = 0
        self.publish_status('EDITING', 'cleared all RViz points')
        self.publish_points()

    def undo_point(self):
        if not self.points:
            self.publish_status('EDITING', 'no point to undo')
            return
        removed = self.points.pop()
        self.current_waypoint = min(self.current_waypoint, len(self.points))
        self.publish_status('EDITING', 'removed point x=%.3f y=%.3f' % (removed['x'], removed['y']))
        self.publish_points()

    def publish_status(self, state, detail):
        self.state = state
        message = {
            'state': state,
            'map_id': self.map_id,
            'map_frame_id': self.map_frame_id,
            'point_count': len(self.points),
            'current_waypoint': self.current_waypoint,
            'detail': detail,
        }
        self.status_pub.publish(String(data=json.dumps(message, ensure_ascii=False)))
        self.get_logger().info('%s: %s' % (state, detail))

    def publish_points(self):
        message = {
            'map_id': self.map_id,
            'map_frame_id': self.map_frame_id,
            'points': self.points,
        }
        self.points_pub.publish(String(data=json.dumps(message, ensure_ascii=False)))
        self.publish_markers()

    def publish_markers(self):
        markers = MarkerArray()
        delete_marker = Marker()
        delete_marker.header.frame_id = self.map_frame_id
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = 'multi_waypoint_route'
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)

        line = Marker()
        line.header = delete_marker.header
        line.ns = 'multi_waypoint_route'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.04
        line.color.r = 0.1
        line.color.g = 0.7
        line.color.b = 1.0
        line.color.a = 0.9
        line.lifetime = Duration(seconds=0).to_msg()

        for index, point in enumerate(self.points):
            point_marker = Marker()
            point_marker.header = delete_marker.header
            point_marker.ns = 'multi_waypoint_route_points'
            point_marker.id = index + 1
            point_marker.type = Marker.SPHERE
            point_marker.action = Marker.ADD
            point_marker.pose.position.x = point['x']
            point_marker.pose.position.y = point['y']
            point_marker.pose.position.z = point['z'] + 0.08
            point_marker.pose.orientation.w = 1.0
            point_marker.scale.x = 0.16
            point_marker.scale.y = 0.16
            point_marker.scale.z = 0.16
            point_marker.color.r = 1.0
            point_marker.color.g = 0.35
            point_marker.color.b = 0.1
            point_marker.color.a = 0.95
            markers.markers.append(point_marker)

            label = Marker()
            label.header = delete_marker.header
            label.ns = 'multi_waypoint_route_labels'
            label.id = index + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = point['x']
            label.pose.position.y = point['y']
            label.pose.position.z = point['z'] + 0.35
            label.pose.orientation.w = 1.0
            label.scale.z = 0.22
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = str(index)
            markers.markers.append(label)

            ros_point = PointStamped()
            ros_point.point.x = point['x']
            ros_point.point.y = point['y']
            ros_point.point.z = point['z'] + 0.05
            line.points.append(ros_point.point)

        if self.points:
            markers.markers.append(line)
        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = MultiWaypointRoute()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
