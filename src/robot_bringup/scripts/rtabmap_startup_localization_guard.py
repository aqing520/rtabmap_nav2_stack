#!/usr/bin/env python3
"""Gate Nav2 activation on an accepted RTAB-Map startup localization."""

import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node
from rtabmap_msgs.msg import Info


HIGHEST_HYPOTHESIS_KEY = 'Loop/Highest_hypothesis_value/'
VISUAL_INLIERS_KEY = 'Loop/Visual_inliers/'
VISUAL_INLIERS_RATIO_KEY = 'Loop/Visual_inliers_ratio/'
REJECTED_HYPOTHESIS_KEY = 'Loop/RejectedHypothesis/'
OPTIMIZATION_ERROR_RATIO_KEY = 'Loop/Optimization_max_error_ratio/'


class StartupLocalizationGuard(Node):
    """Wait for RTAB-Map localization, then activate Nav2."""

    def __init__(self):
        super().__init__('rtabmap_startup_localization_guard')

        self.info_topic = self.declare_parameter('info_topic', 'info').value
        self.localization_pose_topic = self.declare_parameter(
            'localization_pose_topic', 'localization_pose'
        ).value
        self.navigation_manager_service = self.declare_parameter(
            'navigation_manager_service',
            '/lifecycle_manager_navigation/manage_nodes',
        ).value

        self.sensor_wait_timeout_sec = max(
            1.0,
            float(self.declare_parameter('sensor_wait_timeout_sec', 30.0).value),
        )
        self.localization_timeout_sec = max(
            1.0,
            float(self.declare_parameter('localization_timeout_sec', 20.0).value),
        )
        self.service_wait_timeout_sec = max(
            1.0,
            float(self.declare_parameter('service_wait_timeout_sec', 30.0).value),
        )

        self.min_hypothesis = max(
            0.0,
            float(self.declare_parameter('min_hypothesis', 0.11).value),
        )
        self.min_visual_inliers = max(
            0,
            int(self.declare_parameter('min_visual_inliers', 15).value),
        )
        self.min_visual_inliers_ratio = max(
            0.0,
            float(self.declare_parameter('min_visual_inliers_ratio', 0.0).value),
        )
        self.min_best_second_ratio = max(
            1.0,
            float(self.declare_parameter('min_best_second_ratio', 1.0).value),
        )
        self.max_optimization_error_ratio = max(
            0.0,
            float(
                self.declare_parameter(
                    'max_optimization_error_ratio', 3.0
                ).value
            ),
        )
        self.max_linear_variance = max(
            0.0,
            float(self.declare_parameter('max_linear_variance', 1.0).value),
        )
        self.max_yaw_variance = max(
            0.0,
            float(self.declare_parameter('max_yaw_variance', 1.0).value),
        )
        self.required_confirmations = max(
            1,
            int(self.declare_parameter('required_confirmations', 1).value),
        )
        self.allow_last_pose_fallback = bool(
            self.declare_parameter('allow_last_pose_fallback', False).value
        )

        self.started_at = time.monotonic()
        self.first_info_at = None
        self.timeout_handled = False
        self.sensor_timeout_reported = False

        self.state = 'WAITING_LOCALIZATION'
        self.confirmations = 0
        self.accepted_loop_id = 0
        self.max_observed_hypothesis = 0.0
        self.max_observed_inliers = 0
        self.latest_pose = None

        self.activation_reason = ''
        self.activation_started_at = None
        self.activation_steps = []
        self.activation_step_index = 0
        self.activation_future = None
        self.shutdown_timer = None

        self.info_sub = self.create_subscription(
            Info,
            self.info_topic,
            self.info_callback,
            10,
        )
        self.localization_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.localization_pose_topic,
            self.localization_pose_callback,
            10,
        )

        self.navigation_client = self.create_client(
            ManageLifecycleNodes,
            self.navigation_manager_service,
        )

        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info(
            'Startup localization guard enabled: info=%s pose=%s timeout=%.1fs '
            'min_hypothesis=%.3f min_inliers=%d fallback_to_last_pose=%s'
            % (
                self.info_topic,
                self.localization_pose_topic,
                self.localization_timeout_sec,
                self.min_hypothesis,
                self.min_visual_inliers,
                self.allow_last_pose_fallback,
            )
        )

    def info_callback(self, msg):
        if self.state not in ('WAITING_LOCALIZATION',):
            return

        if self.first_info_at is None:
            self.first_info_at = time.monotonic()
            self.get_logger().info(
                'First RTAB-Map info received; startup localization window is open.'
            )

        stats = dict(zip(msg.stats_keys, msg.stats_values))
        score = float(stats.get(HIGHEST_HYPOTHESIS_KEY, 0.0))
        inliers = int(stats.get(VISUAL_INLIERS_KEY, 0.0))
        inliers_ratio = float(stats.get(VISUAL_INLIERS_RATIO_KEY, 0.0))
        rejected = float(stats.get(REJECTED_HYPOTHESIS_KEY, 0.0))
        optimization_error_ratio = float(
            stats.get(OPTIMIZATION_ERROR_RATIO_KEY, 0.0)
        )
        best_second_ratio = self._best_second_ratio(msg)

        self.max_observed_hypothesis = max(
            self.max_observed_hypothesis, score
        )
        self.max_observed_inliers = max(self.max_observed_inliers, inliers)

        if msg.loop_closure_id <= 0:
            return

        failures = []
        if score < self.min_hypothesis:
            failures.append(
                'hypothesis %.3f < %.3f' % (score, self.min_hypothesis)
            )
        if inliers < self.min_visual_inliers:
            failures.append(
                'visual inliers %d < %d'
                % (inliers, self.min_visual_inliers)
            )
        if inliers_ratio < self.min_visual_inliers_ratio:
            failures.append(
                'inlier ratio %.3f < %.3f'
                % (inliers_ratio, self.min_visual_inliers_ratio)
            )
        if (
            self.min_best_second_ratio > 1.0
            and best_second_ratio < self.min_best_second_ratio
        ):
            failures.append(
                'best/second ratio %.3f < %.3f'
                % (best_second_ratio, self.min_best_second_ratio)
            )
        if rejected > 0.5:
            failures.append('RTAB-Map marked the hypothesis rejected')
        if (
            self.max_optimization_error_ratio > 0.0
            and optimization_error_ratio
            > self.max_optimization_error_ratio
        ):
            failures.append(
                'optimization error ratio %.3f > %.3f'
                % (
                    optimization_error_ratio,
                    self.max_optimization_error_ratio,
                )
            )

        if failures:
            self.get_logger().warning(
                'RTAB-Map loop candidate %d was not accepted by startup guard: %s'
                % (msg.loop_closure_id, '; '.join(failures))
            )
            return

        self.confirmations += 1
        self.accepted_loop_id = msg.loop_closure_id
        self.get_logger().info(
            'Accepted RTAB-Map startup localization: node=%d score=%.3f '
            'inliers=%d ratio=%.3f confirmation=%d/%d'
            % (
                msg.loop_closure_id,
                score,
                inliers,
                inliers_ratio,
                self.confirmations,
                self.required_confirmations,
            )
        )
        self._try_activate_from_localization()

    def localization_pose_callback(self, msg):
        self.latest_pose = msg
        if self.state == 'WAITING_LOCALIZATION':
            self._try_activate_from_localization()

    def timer_callback(self):
        now = time.monotonic()

        if self.state == 'WAITING_LOCALIZATION':
            if self.first_info_at is None:
                if (
                    not self.sensor_timeout_reported
                    and now - self.started_at >= self.sensor_wait_timeout_sec
                ):
                    self.sensor_timeout_reported = True
                    self.get_logger().error(
                        'No RTAB-Map info received after %.1fs. Nav2 remains '
                        'inactive; check RGB-D, LiDAR, odometry and RTAB-Map synchronization.'
                        % self.sensor_wait_timeout_sec
                    )
                return

            if (
                not self.timeout_handled
                and now - self.first_info_at >= self.localization_timeout_sec
            ):
                self.timeout_handled = True
                self._handle_localization_timeout()
            return

        if self.state == 'ACTIVATING':
            self._drive_activation(now)

    def _try_activate_from_localization(self):
        if self.state != 'WAITING_LOCALIZATION':
            return
        if self.confirmations < self.required_confirmations:
            return
        if self.latest_pose is None:
            self.get_logger().info(
                'Loop closure accepted; waiting for RTAB-Map localization_pose.'
            )
            return

        pose_ok, reason = self._localization_pose_is_valid(self.latest_pose)
        if not pose_ok:
            self.get_logger().warning(
                'Loop closure accepted, but localization covariance is not '
                'ready: %s' % reason
            )
            return

        self._request_activation(
            'RTAB-Map localized on node %d' % self.accepted_loop_id
        )

    def _handle_localization_timeout(self):
        self.get_logger().warning(
            'Startup localization timed out after %.1fs: best hypothesis=%.3f, '
            'max visual inliers=%d.'
            % (
                self.localization_timeout_sec,
                self.max_observed_hypothesis,
                self.max_observed_inliers,
            )
        )

        if self.allow_last_pose_fallback:
            self.get_logger().warning(
                'Falling back to the last localization pose stored in the '
                'RTAB-Map database.'
            )
            self._request_activation('last saved RTAB-Map pose fallback')
        else:
            self.get_logger().error(
                'Nav2 remains inactive. Move the robot to a visually distinctive '
                'area, wait for a later RTAB-Map match, or publish /initialpose manually.'
            )

    def _request_activation(self, reason):
        if self.state != 'WAITING_LOCALIZATION':
            return

        self.state = 'ACTIVATING'
        self.activation_reason = reason
        self.activation_started_at = time.monotonic()
        self.activation_steps = [
            (
                'Nav2 navigation',
                self.navigation_client,
                self.navigation_manager_service,
            )
        ]
        self.activation_step_index = 0
        self.activation_future = None

        self.get_logger().info(
            'Localization gate passed (%s). Activating lifecycle nodes...'
            % reason
        )

    def _drive_activation(self, now):
        if self.activation_step_index >= len(self.activation_steps):
            self.state = 'ACTIVE'
            self.get_logger().info(
                'Navigation startup complete: %s' % self.activation_reason
            )
            if self.shutdown_timer is None:
                self.shutdown_timer = self.create_timer(
                    1.0, self._shutdown_after_success
                )
            return

        label, client, service_name = self.activation_steps[
            self.activation_step_index
        ]

        if self.activation_future is not None:
            return

        if not client.service_is_ready():
            if (
                now - self.activation_started_at
                >= self.service_wait_timeout_sec
            ):
                self.state = 'FAILED'
                self.get_logger().error(
                    'Timed out waiting for lifecycle service %s. %s was not activated.'
                    % (service_name, label)
                )
            return

        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        self.get_logger().info(
            'Calling %s to activate %s...' % (service_name, label)
        )
        self.activation_future = client.call_async(request)
        self.activation_future.add_done_callback(
            lambda future, step_label=label: self._activation_done(
                future, step_label
            )
        )

    def _activation_done(self, future, label):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - ROS future exceptions vary
            self.state = 'FAILED'
            self.get_logger().error(
                'Lifecycle activation call for %s failed: %s' % (label, exc)
            )
            return

        if not response.success:
            self.state = 'FAILED'
            self.get_logger().error(
                'Lifecycle manager reported failure while activating %s.'
                % label
            )
            return

        self.get_logger().info('%s activated.' % label)
        self.activation_step_index += 1
        self.activation_future = None
        self.activation_started_at = time.monotonic()

    def _localization_pose_is_valid(self, msg):
        covariance = msg.pose.covariance
        linear_variances = (covariance[0], covariance[7])
        yaw_variance = covariance[35]
        values = linear_variances + (yaw_variance,)

        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            return False, 'covariance contains invalid values'
        if max(linear_variances) > self.max_linear_variance:
            return (
                False,
                'linear variance %.3f > %.3f'
                % (max(linear_variances), self.max_linear_variance),
            )
        if yaw_variance > self.max_yaw_variance:
            return (
                False,
                'yaw variance %.3f > %.3f'
                % (yaw_variance, self.max_yaw_variance),
            )
        return True, 'ok'

    @staticmethod
    def _best_second_ratio(msg):
        candidates = sorted(
            (
                float(value)
                for key, value in zip(
                    msg.posterior_keys, msg.posterior_values
                )
                if key > 0 and value > 0.0
            ),
            reverse=True,
        )
        if len(candidates) < 2 or candidates[1] <= 0.0:
            return math.inf
        return candidates[0] / candidates[1]

    def _shutdown_after_success(self):
        if rclpy.ok():
            self.get_logger().info(
                'Startup localization guard finished; RTAB-Map continues normal localization.'
            )
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = StartupLocalizationGuard()
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
