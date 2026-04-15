#!/usr/bin/env python3
import math

import numpy as np
import rospy
import tf.transformations as tf_trans
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String


class VisualServoController:
    def __init__(self):
        rospy.init_node("visual_servo_controller", anonymous=True)

        self.offset_z = rospy.get_param("~geometry/offset_z", -1.00)
        self.offset_x = rospy.get_param("~geometry/offset_x", 0.0)
        self.offset_y = rospy.get_param("~geometry/offset_y", -0.616)
        self.target_dock_dist = rospy.get_param("~geometry/target_dock_dist", 0.0)

        self.k_v_x = rospy.get_param("~pid/k_p_x", 0.5)
        self.k_v_y = rospy.get_param("~pid/k_p_y", 0.8)
        self.k_w_z = rospy.get_param("~pid/k_p_z", 1.0)
        self.k_i_x = rospy.get_param("~pid/k_i_x", 0.05)
        self.k_i_y = rospy.get_param("~pid/k_i_y", 0.05)
        self.k_i_z = rospy.get_param("~pid/k_i_z", 0.05)
        self.k_d_x = rospy.get_param("~pid/k_d_x", 0.1)
        self.k_d_y = rospy.get_param("~pid/k_d_y", 0.1)
        self.k_d_z = rospy.get_param("~pid/k_d_z", 0.1)
        self.yaw_priority_lateral = rospy.get_param("~pid/yaw_priority_lateral", 0.06)
        self.yaw_priority_yaw = rospy.get_param("~pid/yaw_priority_yaw", 0.05)
        self.final_lateral_slow_abs_err_x = rospy.get_param("~pid/final_lateral_slow_abs_err_x", 0.10)
        self.final_lateral_slow_band = rospy.get_param("~pid/final_lateral_slow_band", 0.08)
        self.final_lateral_slow_scale = rospy.get_param("~pid/final_lateral_slow_scale", 0.4)

        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_yaw = 0.0
        self.integral_limit = 0.5
        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.last_error_yaw = 0.0
        self.last_time = rospy.Time.now()

        self.max_v_x = rospy.get_param("~limits/max_v_x", 0.15)
        self.max_v_y = rospy.get_param("~limits/max_v_y", 0.10)
        self.max_w_z = rospy.get_param("~limits/max_w_z", 0.3)

        self.dist_tolerance = rospy.get_param("~tolerance/dist", 0.05)
        self.yaw_tolerance = rospy.get_param("~tolerance/yaw", 0.05)
        self.lat_tolerance = rospy.get_param("~tolerance/lateral", 0.03)
        self.dist_tolerance_relaxed = rospy.get_param("~tolerance/dist_relaxed", self.dist_tolerance * 1.5)
        self.yaw_tolerance_relaxed = rospy.get_param("~tolerance/yaw_relaxed", self.yaw_tolerance * 1.4)
        self.lat_tolerance_relaxed = rospy.get_param("~tolerance/lateral_relaxed", self.lat_tolerance * 1.5)

        self.last_aruco_time = None
        self.aruco_timeout = rospy.get_param("~timeout/aruco_lost", 2.0)

        self.last_cmd_vx = 0.0
        self.last_cmd_vy = 0.0
        self.last_cmd_wz = 0.0
        self.filter_alpha = rospy.get_param("~filter/alpha", 0.3)

        self.stable_count = 0
        self.stable_threshold = rospy.get_param("~stable/threshold", 15)
        self.is_locked = False
        self.enabled = False

        self.enable_gait = rospy.get_param("~gait/enable", True)
        self.motion_mode = "PID"
        self.gait_active = False
        self.gait_type = None
        self.gait_phase = 0
        self.gait_phase_start_time = None

        self.gait_trigger_lat = rospy.get_param("~gait/trigger_lateral", 0.08)
        self.gait_trigger_near_dist = rospy.get_param("~gait/trigger_near_dist", 0.25)
        self.gait_trigger_yaw = rospy.get_param("~gait/trigger_yaw", 0.12)
        self.gait_trigger_min_abs_err_x = rospy.get_param("~gait/trigger_min_abs_err_x", 0.12)
        self.gait_max_abs_yaw = rospy.get_param("~gait/max_abs_yaw", 0.30)
        self.gait_disable_after_stable_count = rospy.get_param("~gait/disable_after_stable_count", 3)
        self.gait_cooldown = rospy.get_param("~gait/cooldown", 2.0)
        self.last_gait_end_time = None

        self.gait_back_time = rospy.get_param("~gait/back_time", 1.2)
        self.gait_turn_time = rospy.get_param("~gait/turn_time", 0.8)
        self.gait_forward_time = rospy.get_param("~gait/forward_time", 1.5)

        self.gait_back_speed = rospy.get_param("~gait/back_speed", -0.08)
        self.gait_forward_speed = rospy.get_param("~gait/forward_speed", 0.06)
        self.gait_turn_speed = rospy.get_param("~gait/turn_speed", 0.20)
        self.stop_burst_duration = rospy.get_param("~stop_burst/duration", 0.5)
        self.stop_burst_until = None
        self.last_charging_state = None

        self.aruco_sub = rospy.Subscriber("/charging_goal_base", PoseStamped, self.callback)
        self.action_sub = rospy.Subscriber("/charging_action", String, self.action_callback)
        self.state_sub = rospy.Subscriber("/charging_state", String, self.state_callback)
        self.vel_pub = rospy.Publisher("/servo_cmd_vel", Twist, queue_size=10)
        self.lock_pub = rospy.Publisher("/docking_locked", Bool, queue_size=10)

        self.publish_idle_outputs()
        rospy.loginfo("visual_servo_controller started, waiting for START")

    def arm_stop_burst(self, duration=None):
        burst_duration = self.stop_burst_duration if duration is None else duration
        self.stop_burst_until = rospy.Time.now() + rospy.Duration(max(0.0, burst_duration))

    def run_stop_burst(self):
        if self.stop_burst_until is None:
            return

        if rospy.Time.now() <= self.stop_burst_until:
            self.vel_pub.publish(Twist())
            self.lock_pub.publish(Bool(data=False))
            return

        self.stop_burst_until = None

    def publish_idle_outputs(self):
        self.vel_pub.publish(Twist())
        self.lock_pub.publish(Bool(data=False))

    def reset_pid_state(self):
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_yaw = 0.0
        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.last_error_yaw = 0.0
        self.last_time = rospy.Time.now()

    def reset_gait_state(self):
        self.motion_mode = "PID"
        self.gait_active = False
        self.gait_type = None
        self.gait_phase = 0
        self.gait_phase_start_time = None
        self.last_gait_end_time = rospy.Time.now()

    def reset_control_state(self):
        self.reset_pid_state()
        self.reset_gait_state()
        self.stable_count = 0
        self.is_locked = False
        self.last_aruco_time = None
        self.last_cmd_vx = 0.0
        self.last_cmd_vy = 0.0
        self.last_cmd_wz = 0.0
        self.stop_burst_until = None

    def action_callback(self, msg):
        action = msg.data.strip().upper()

        if action == "START":
            self.enabled = True
            self.reset_control_state()
            self.publish_idle_outputs()
            rospy.loginfo("visual servo enabled by START")
            return

        if action == "STOP":
            self.enabled = False
            self.reset_control_state()
            self.publish_idle_outputs()
            self.arm_stop_burst()
            rospy.loginfo("visual servo disabled by STOP")
            return

        if action == "UNDOCK":
            self.enabled = False
            self.reset_control_state()
            self.publish_idle_outputs()
            rospy.loginfo("visual servo disabled by UNDOCK")
            return

        rospy.logwarn(f"unknown charging_action received: {msg.data}")

    def state_callback(self, msg):
        state = msg.data.strip().upper()
        if state == self.last_charging_state:
            return
        self.last_charging_state = state

        if state in ("DOCKED", "POST_DOCK_DONE", "FAILED", "UNDOCKING", "IDLE"):
            self.enabled = False
            self.last_aruco_time = None
            self.publish_idle_outputs()

            if state in ("POST_DOCK_DONE", "FAILED", "IDLE"):
                self.arm_stop_burst()

            rospy.loginfo(f"visual servo auto-disabled by charging_state={state}")

    def start_gait(self, gait_name):
        self.motion_mode = "GAIT"
        self.gait_active = True
        self.gait_type = gait_name
        self.gait_phase = 0
        self.gait_phase_start_time = rospy.Time.now()
        self.stable_count = 0
        self.reset_pid_state()
        rospy.logwarn(f"switching to gait mode: {gait_name}")

    def select_gait(self, error_dist, error_lat, error_yaw):
        if not self.enable_gait:
            return None

        if self.last_gait_end_time is not None:
            if (rospy.Time.now() - self.last_gait_end_time).to_sec() < self.gait_cooldown:
                return None

        if self.stable_count >= self.gait_disable_after_stable_count:
            return None
        if abs(error_lat) < self.gait_trigger_lat:
            return None
        if abs(error_dist) > self.gait_trigger_near_dist:
            return None
        if abs(error_dist) < self.gait_trigger_min_abs_err_x:
            return None
        if abs(error_yaw) > self.gait_max_abs_yaw:
            return None

        if error_lat > 0.0:
            if error_yaw >= self.gait_trigger_yaw:
                return "LEFT_BACK_FORWARD"
            return "BACKWARD_LEFT_FORWARD"

        if error_yaw <= -self.gait_trigger_yaw:
            return "RIGHT_BACK_FORWARD"
        return "BACKWARD_RIGHT_FORWARD"

    def execute_gait(self):
        cmd = Twist()
        now = rospy.Time.now()
        elapsed = (now - self.gait_phase_start_time).to_sec()

        if self.gait_type == "LEFT_BACK_FORWARD":
            if self.gait_phase == 0:
                cmd.linear.x = self.gait_back_speed
                cmd.angular.z = abs(self.gait_turn_speed)
                if elapsed >= self.gait_back_time:
                    self.gait_phase = 1
                    self.gait_phase_start_time = now
            elif self.gait_phase == 1:
                cmd.linear.x = self.gait_forward_speed
                if elapsed >= self.gait_forward_time:
                    self.reset_gait_state()
                    return Twist()

        elif self.gait_type == "RIGHT_BACK_FORWARD":
            if self.gait_phase == 0:
                cmd.linear.x = self.gait_back_speed
                cmd.angular.z = -abs(self.gait_turn_speed)
                if elapsed >= self.gait_back_time:
                    self.gait_phase = 1
                    self.gait_phase_start_time = now
            elif self.gait_phase == 1:
                cmd.linear.x = self.gait_forward_speed
                if elapsed >= self.gait_forward_time:
                    self.reset_gait_state()
                    return Twist()

        elif self.gait_type == "BACKWARD_LEFT_FORWARD":
            if self.gait_phase == 0:
                cmd.linear.x = self.gait_back_speed
                if elapsed >= self.gait_back_time:
                    self.gait_phase = 1
                    self.gait_phase_start_time = now
            elif self.gait_phase == 1:
                cmd.angular.z = abs(self.gait_turn_speed)
                if elapsed >= self.gait_turn_time:
                    self.gait_phase = 2
                    self.gait_phase_start_time = now
            elif self.gait_phase == 2:
                cmd.linear.x = self.gait_forward_speed
                if elapsed >= self.gait_forward_time:
                    self.reset_gait_state()
                    return Twist()

        elif self.gait_type == "BACKWARD_RIGHT_FORWARD":
            if self.gait_phase == 0:
                cmd.linear.x = self.gait_back_speed
                if elapsed >= self.gait_back_time:
                    self.gait_phase = 1
                    self.gait_phase_start_time = now
            elif self.gait_phase == 1:
                cmd.angular.z = -abs(self.gait_turn_speed)
                if elapsed >= self.gait_turn_time:
                    self.gait_phase = 2
                    self.gait_phase_start_time = now
            elif self.gait_phase == 2:
                cmd.linear.x = self.gait_forward_speed
                if elapsed >= self.gait_forward_time:
                    self.reset_gait_state()
                    return Twist()

        cmd.linear.x = np.clip(cmd.linear.x, -self.max_v_x, self.max_v_x)
        cmd.linear.y = 0.0
        cmd.angular.z = np.clip(cmd.angular.z, -self.max_w_z, self.max_w_z)

        self.last_cmd_vx = cmd.linear.x
        self.last_cmd_vy = cmd.linear.y
        self.last_cmd_wz = cmd.angular.z
        return cmd

    def callback(self, msg):
        if not self.enabled:
            return

        if self.is_locked:
            self.vel_pub.publish(Twist())
            self.lock_pub.publish(Bool(data=True))
            return

        self.last_aruco_time = rospy.Time.now()

        dock_x = msg.pose.position.x
        dock_y = msg.pose.position.y

        orientation_q = msg.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        (_, _, yaw) = tf_trans.euler_from_quaternion(orientation_list)

        error_dist = dock_x - self.target_dock_dist
        error_lat = dock_y
        error_yaw = yaw + 1.5707

        if not self.gait_active:
            selected_gait = self.select_gait(error_dist, error_lat, error_yaw)
            if selected_gait is not None:
                self.start_gait(selected_gait)

        if self.gait_active:
            cmd = self.execute_gait()
            self.vel_pub.publish(cmd)
            self.lock_pub.publish(Bool(data=False))
            rospy.loginfo_throttle(
                0.5,
                f"[GAIT] {self.gait_type} phase={self.gait_phase} | "
                f"dist:{error_dist:.3f} lat:{error_lat:.3f} yaw:{math.degrees(error_yaw):.1f}deg | "
                f"vx:{cmd.linear.x:.2f} wz:{cmd.angular.z:.2f}",
            )
            return

        current_time = rospy.Time.now()
        dt = max((current_time - self.last_time).to_sec(), 0.001)

        derivative_x = (error_dist - self.last_error_x) / dt
        derivative_y = (error_lat - self.last_error_y) / dt
        derivative_yaw = (error_yaw - self.last_error_yaw) / dt

        self.last_error_x = error_dist
        self.last_error_y = error_lat
        self.last_error_yaw = error_yaw
        self.last_time = current_time

        self.integral_x += error_dist * dt
        self.integral_y += error_lat * dt
        self.integral_yaw += error_yaw * dt

        self.integral_x = np.clip(self.integral_x, -self.integral_limit, self.integral_limit)
        self.integral_y = np.clip(self.integral_y, -self.integral_limit, self.integral_limit)
        self.integral_yaw = np.clip(self.integral_yaw, -self.integral_limit, self.integral_limit)

        cmd = Twist()

        if abs(error_dist) > self.dist_tolerance:
            cmd.linear.x = (
                self.k_v_x * error_dist
                + self.k_i_x * self.integral_x
                + self.k_d_x * derivative_x
            )

        cmd.linear.y = (
            -self.k_v_y * error_lat
            - self.k_i_y * self.integral_y
            - self.k_d_y * derivative_y
        )
        cmd.angular.z = (
            -self.k_w_z * error_yaw
            - self.k_i_z * self.integral_yaw
            - self.k_d_z * derivative_yaw
        )

        # When lateral error is already small, suppress sideways motion and let yaw settle first.
        if abs(error_lat) < self.yaw_priority_lateral and abs(error_yaw) > self.yaw_priority_yaw:
            cmd.linear.y = 0.0

        if (
            abs(error_dist) < self.final_lateral_slow_abs_err_x
            and abs(error_lat) < self.final_lateral_slow_band
        ) or (self.stable_count > 0 and abs(error_lat) < self.final_lateral_slow_band):
            cmd.linear.y *= self.final_lateral_slow_scale

        cmd.linear.x = np.clip(cmd.linear.x, -self.max_v_x, self.max_v_x)
        cmd.linear.y = np.clip(cmd.linear.y, -self.max_v_y, self.max_v_y)
        cmd.angular.z = np.clip(cmd.angular.z, -self.max_w_z, self.max_w_z)

        cmd.linear.x = self.filter_alpha * cmd.linear.x + (1 - self.filter_alpha) * self.last_cmd_vx
        cmd.linear.y = self.filter_alpha * cmd.linear.y + (1 - self.filter_alpha) * self.last_cmd_vy
        cmd.angular.z = self.filter_alpha * cmd.angular.z + (1 - self.filter_alpha) * self.last_cmd_wz

        self.last_cmd_vx = cmd.linear.x
        self.last_cmd_vy = cmd.linear.y
        self.last_cmd_wz = cmd.angular.z

        is_converged = (
            abs(error_dist) < self.dist_tolerance
            and abs(error_lat) < self.lat_tolerance
            and abs(error_yaw) < self.yaw_tolerance
        )
        is_near_converged = (
            abs(error_dist) < self.dist_tolerance_relaxed
            and abs(error_lat) < self.lat_tolerance_relaxed
            and abs(error_yaw) < self.yaw_tolerance_relaxed
        )

        if is_converged:
            self.stable_count += 1
        elif is_near_converged:
            self.stable_count = max(0, self.stable_count - 1)
        else:
            self.stable_count = 0

        if self.stable_count >= self.stable_threshold:
            cmd = Twist()
            self.reset_pid_state()
            self.is_locked = True
            self.enabled = False
            self.last_aruco_time = None
            self.lock_pub.publish(Bool(data=True))
            rospy.loginfo(
                "docking locked: dist=%.2fcm lat=%.2fcm yaw=%.2fdeg",
                error_dist * 100.0,
                error_lat * 100.0,
                math.degrees(error_yaw),
            )
        else:
            self.lock_pub.publish(Bool(data=False))
            rospy.loginfo_throttle(
                0.5,
                f"dock_x:{dock_x:.3f}m | err_x:{error_dist:.3f}m | lat:{error_lat:.3f}m | "
                f"yaw:{math.degrees(error_yaw):.1f}deg | "
                f"stable:{self.stable_count}/{self.stable_threshold} | "
                f"vx:{cmd.linear.x:.2f} vy:{cmd.linear.y:.2f} w:{cmd.angular.z:.2f}",
            )

        self.vel_pub.publish(cmd)


if __name__ == "__main__":
    try:
        controller = VisualServoController()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            controller.run_stop_burst()

            if controller.enabled and controller.last_aruco_time is not None:
                time_since_last = (rospy.Time.now() - controller.last_aruco_time).to_sec()

                if time_since_last > controller.aruco_timeout and not controller.is_locked:
                    controller.vel_pub.publish(Twist())
                    controller.lock_pub.publish(Bool(data=False))
                    controller.reset_pid_state()
                    controller.reset_gait_state()
                    controller.stable_count = 0
                    rospy.logwarn_throttle(
                        2, f"AprilTag lost for {time_since_last:.1f}s, stopping servo output"
                    )

            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("visual_servo_controller stopped")
