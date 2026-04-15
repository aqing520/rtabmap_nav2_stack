#!/usr/bin/env python3
import shlex
import subprocess
from enum import Enum

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String


class ChargingState(Enum):
    IDLE = 0
    SEARCHING = 1
    ALIGNING = 2
    RETREATING = 3
    DOCKED = 4
    FAILED = 5
    POST_DOCK_DONE = 6
    UNDOCKING = 7


class ChargingMode(Enum):
    STANDBY = 0
    ACTIVE = 1


class ChargingStateMachine:
    def __init__(self):
        rospy.init_node("charging_state_machine", anonymous=True)

        self.mode = ChargingMode.STANDBY
        self.state = ChargingState.IDLE
        self.retry_count = 0
        self.max_retries = 300

        self.search_timeout = rospy.get_param("~timeout/search", 10.0)
        self.align_timeout = rospy.get_param("~timeout/align", 30.0)
        self.aruco_lost_threshold = rospy.get_param("~timeout/aruco_lost", 1.0)
        self.state_start_time = None

        self.aruco_detected = False
        self.last_aruco_time = None
        self.docking_locked = False
        self.servo_cmd = Twist()

        self.post_dock_enabled = rospy.get_param("~post_dock/enabled", True)
        self.post_dock_forward_distance = max(
            0.0, rospy.get_param("~post_dock/forward_distance", 1.0)
        )
        self.post_dock_forward_speed = abs(
            rospy.get_param("~post_dock/forward_speed", 0.20)
        )
        self.post_dock_lie_down_command = (
            rospy.get_param("~post_dock/lie_down_command", "").strip()
        )
        self.post_dock_forward_duration = self._compute_post_dock_duration()
        self.undock_back_distance = max(
            0.0, rospy.get_param("~post_dock/undock_back_distance", 1.0)
        )
        self.undock_back_speed = abs(
            rospy.get_param("~post_dock/undock_back_speed", 0.20)
        )
        self.undock_prepare_delay = max(
            0.0, rospy.get_param("~post_dock/undock_prepare_delay", 3.0)
        )
        self.undock_back_duration = self._compute_undock_duration()
        self.post_dock_move_done = False
        self.post_dock_stop_sent = False
        self.post_dock_action_done = False
        self.post_dock_process = None
        self.undock_started = False
        self.undock_stop_sent = False
        self.stop_published_for_state = None
        self.stop_burst_duration = rospy.get_param("~stop_burst/duration", 0.5)
        self.stop_burst_until = None

        self.cmd_vel_pub = rospy.Publisher("/charging_cmd_vel", Twist, queue_size=10)
        self.state_pub = rospy.Publisher("/charging_state", String, queue_size=10, latch=True)

        self.action_sub = rospy.Subscriber("/charging_action", String, self.action_callback)
        self.aruco_sub = rospy.Subscriber("/aruco_poses", PoseStamped, self.aruco_callback)
        self.servo_cmd_sub = rospy.Subscriber("/servo_cmd_vel", Twist, self.servo_cmd_callback)
        self.lock_sub = rospy.Subscriber("/docking_locked", Bool, self.lock_callback)

        self.state_pub.publish(self.state.name)
        rospy.loginfo("charging_state_machine started in STANDBY, waiting for START")

    def _compute_post_dock_duration(self):
        if self.post_dock_forward_distance <= 0.0 or self.post_dock_forward_speed <= 0.0:
            return 0.0
        return self.post_dock_forward_distance / self.post_dock_forward_speed

    def _compute_undock_duration(self):
        if self.undock_back_distance <= 0.0 or self.undock_back_speed <= 0.0:
            return 0.0
        return self.undock_back_distance / self.undock_back_speed

    def _publish_stop(self):
        self.cmd_vel_pub.publish(Twist())

    def _publish_stop_once(self):
        if self.stop_published_for_state == self.state:
            return
        self._publish_stop()
        self.stop_published_for_state = self.state

    def _arm_stop_burst(self, duration=None):
        burst_duration = self.stop_burst_duration if duration is None else duration
        self.stop_burst_until = rospy.Time.now() + rospy.Duration(max(0.0, burst_duration))

    def _run_stop_burst(self):
        if self.stop_burst_until is None:
            return

        if rospy.Time.now() <= self.stop_burst_until:
            self._publish_stop()
            return

        self.stop_burst_until = None

    def _stop_post_dock_process(self):
        if self.post_dock_process is None:
            return

        if self.post_dock_process.poll() is None:
            try:
                self.post_dock_process.terminate()
                rospy.loginfo("post-dock process terminated")
            except Exception as exc:
                rospy.logwarn(f"failed to terminate post-dock process: {exc}")

        self.post_dock_process = None

    def _reset_runtime_flags(self):
        self.retry_count = 0
        self.aruco_detected = False
        self.last_aruco_time = None
        self.docking_locked = False
        self.servo_cmd = Twist()
        self.state_start_time = None

    def _reset_post_dock_flags(self):
        self.post_dock_move_done = False
        self.post_dock_stop_sent = False
        self.post_dock_action_done = False

    def _reset_undock_flags(self):
        self.undock_started = False
        self.undock_stop_sent = False

    def _enter_standby(self, reason, burst_stop=False):
        rospy.loginfo(reason)
        self.mode = ChargingMode.STANDBY
        self._stop_post_dock_process()
        self._publish_stop()
        if burst_stop:
            self._arm_stop_burst()
        self._reset_runtime_flags()
        self._reset_post_dock_flags()
        self._reset_undock_flags()

        if self.state != ChargingState.IDLE:
            self.transition_to(ChargingState.IDLE)
        else:
            self.state_pub.publish(self.state.name)

    def _start_charging(self):
        if self.mode == ChargingMode.ACTIVE and self.state != ChargingState.IDLE:
            rospy.loginfo("received START, restarting charging workflow")
        else:
            rospy.loginfo("received START, entering active charging workflow")

        self.mode = ChargingMode.ACTIVE
        self._stop_post_dock_process()
        self._publish_stop()
        self._reset_runtime_flags()
        self._reset_post_dock_flags()
        self._reset_undock_flags()
        self.transition_to(ChargingState.SEARCHING)

    def _trigger_post_dock_sequence(self, reason):
        rospy.loginfo(reason)
        self._publish_stop()
        self.transition_to(ChargingState.DOCKED)

    def _trigger_lie_down(self):
        if not self.post_dock_lie_down_command:
            rospy.logwarn("post_dock lie_down_command is empty, skipping")
            return

        try:
            self.post_dock_process = subprocess.Popen(
                shlex.split(self.post_dock_lie_down_command), close_fds=True
            )
            rospy.loginfo(f"started post-dock command: {self.post_dock_lie_down_command}")
        except Exception as exc:
            self.post_dock_process = None
            rospy.logerr(f"failed to start post-dock command: {exc}")

    def _start_undocking(self):
        if self.state != ChargingState.POST_DOCK_DONE:
            rospy.logwarn(
                f"received UNDOCK but current state is {self.state.name}, expected POST_DOCK_DONE"
            )
            return

        self._publish_stop()
        self._stop_post_dock_process()
        self._reset_undock_flags()
        self.docking_locked = False
        self.transition_to(ChargingState.UNDOCKING)

    def action_callback(self, msg):
        action = msg.data.strip().upper()

        if action == "START":
            self._start_charging()
            return

        if action == "STOP":
            self._enter_standby("received STOP, aborting charging workflow", burst_stop=True)
            return

        if action == "UNDOCK":
            self._start_undocking()
            return

        rospy.logwarn(f"unknown charging_action received: {msg.data}")

    def aruco_callback(self, _msg):
        self.aruco_detected = True
        self.last_aruco_time = rospy.Time.now()

    def servo_cmd_callback(self, msg):
        self.servo_cmd = msg

    def lock_callback(self, msg):
        self.docking_locked = msg.data

    def check_aruco_lost(self):
        if self.last_aruco_time is None:
            return True
        return (rospy.Time.now() - self.last_aruco_time).to_sec() > self.aruco_lost_threshold

    def check_timeout(self):
        if self.state_start_time is None:
            return False

        elapsed = (rospy.Time.now() - self.state_start_time).to_sec()
        if self.state == ChargingState.SEARCHING:
            return elapsed > self.search_timeout
        if self.state == ChargingState.ALIGNING:
            return elapsed > self.align_timeout
        return False

    def transition_to(self, new_state):
        rospy.loginfo(f"state transition: {self.state.name} -> {new_state.name}")
        self.state = new_state
        self.state_start_time = rospy.Time.now()
        self.stop_published_for_state = None
        if new_state in (ChargingState.IDLE, ChargingState.POST_DOCK_DONE, ChargingState.FAILED):
            self._arm_stop_burst()
        if new_state == ChargingState.DOCKED:
            self._reset_post_dock_flags()
        if new_state == ChargingState.UNDOCKING:
            self._reset_undock_flags()
        self.state_pub.publish(new_state.name)

    def state_idle(self):
        pass

    def state_searching(self):
        cmd = Twist()

        if self.aruco_detected and not self.check_aruco_lost():
            rospy.loginfo("tag detected, switching to ALIGNING")
            self.transition_to(ChargingState.ALIGNING)
            return

        if self.check_timeout():
            rospy.logwarn("search timed out")
            self.retry_count += 1
            if self.retry_count >= self.max_retries:
                self.transition_to(ChargingState.FAILED)
            else:
                rospy.loginfo(f"retry search ({self.retry_count}/{self.max_retries})")
                self.state_start_time = rospy.Time.now()
            return

        cmd.angular.z = 0.3
        self.cmd_vel_pub.publish(cmd)
        rospy.loginfo_throttle(2, "searching for AprilTag")

    def state_aligning(self):
        if self.docking_locked:
            self._trigger_post_dock_sequence("docking locked, switching to DOCKED")
            return

        if self.check_aruco_lost():
            rospy.logwarn("tag lost during ALIGNING, switching to RETREATING")
            self.transition_to(ChargingState.RETREATING)
            return

        if self.check_timeout():
            rospy.logwarn("align timed out")
            self.retry_count += 1
            if self.retry_count >= self.max_retries:
                self.transition_to(ChargingState.FAILED)
            else:
                rospy.loginfo(f"retry align ({self.retry_count}/{self.max_retries})")
                self.transition_to(ChargingState.SEARCHING)
            return

        self.cmd_vel_pub.publish(self.servo_cmd)

    def state_docked(self):
        if not self.post_dock_enabled:
            self._publish_stop()
            rospy.loginfo_throttle(5, "docked, waiting in place")
            return

        if not self.post_dock_move_done:
            elapsed = (rospy.Time.now() - self.state_start_time).to_sec()
            if elapsed < self.post_dock_forward_duration:
                cmd = Twist()
                cmd.linear.x = self.post_dock_forward_speed
                self.cmd_vel_pub.publish(cmd)
                rospy.loginfo_throttle(
                    1,
                    "post-dock forward move %.2fm at %.2fm/s"
                    % (self.post_dock_forward_distance, self.post_dock_forward_speed),
                )
                return

            self.post_dock_move_done = True
            if not self.post_dock_stop_sent:
                self._publish_stop()
                self.post_dock_stop_sent = True
            rospy.loginfo("post-dock forward move complete")

        if not self.post_dock_action_done:
            self._trigger_lie_down()
            self.post_dock_action_done = True
            self.transition_to(ChargingState.POST_DOCK_DONE)

    def state_post_dock_done(self):
        self._publish_stop_once()
        rospy.loginfo_throttle(5, "post-dock action complete")

    def state_undocking(self):
        if not self.undock_started:
            self.undock_started = True
            rospy.loginfo(
                "entered UNDOCKING, waiting %.1fs before backing up %.2fm at %.2fm/s",
                self.undock_prepare_delay,
                self.undock_back_distance,
                self.undock_back_speed,
            )

        elapsed = (rospy.Time.now() - self.state_start_time).to_sec()
        if elapsed < self.undock_prepare_delay:
            self._publish_stop()
            rospy.loginfo_throttle(
                1,
                "undocking: waiting %.1fs before backing up"
                % self.undock_prepare_delay,
            )
            return

        if elapsed < self.undock_prepare_delay + self.undock_back_duration:
            cmd = Twist()
            cmd.linear.x = -self.undock_back_speed
            self.cmd_vel_pub.publish(cmd)
            rospy.loginfo_throttle(
                1,
                "undocking: backing up %.2fm at %.2fm/s"
                % (self.undock_back_distance, self.undock_back_speed),
            )
            return

        if not self.undock_stop_sent:
            self._publish_stop()
            self.undock_stop_sent = True

        rospy.loginfo("undocking complete, returning to STANDBY")
        self._enter_standby("undocking finished")

    def state_failed(self):
        self._publish_stop_once()
        rospy.logerr_throttle(5, f"charging failed after {self.max_retries} retries")

    def state_retreating(self):
        cmd = Twist()
        cmd.linear.x = -0.2
        self.cmd_vel_pub.publish(cmd)

        if (rospy.Time.now() - self.state_start_time).to_sec() > 2.5:
            self._publish_stop()
            self.transition_to(ChargingState.SEARCHING)

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self._run_stop_burst()

            if self.mode == ChargingMode.STANDBY:
                self.state_idle()
                rate.sleep()
                continue

            if self.docking_locked and self.state not in (
                ChargingState.IDLE,
                ChargingState.DOCKED,
                ChargingState.POST_DOCK_DONE,
                ChargingState.UNDOCKING,
                ChargingState.FAILED,
            ):
                self._trigger_post_dock_sequence("received docking lock signal, switching to DOCKED")

            if self.state == ChargingState.IDLE:
                self.state_idle()
            elif self.state == ChargingState.SEARCHING:
                self.state_searching()
            elif self.state == ChargingState.ALIGNING:
                self.state_aligning()
            elif self.state == ChargingState.RETREATING:
                self.state_retreating()
            elif self.state == ChargingState.DOCKED:
                self.state_docked()
            elif self.state == ChargingState.POST_DOCK_DONE:
                self.state_post_dock_done()
            elif self.state == ChargingState.UNDOCKING:
                self.state_undocking()
            elif self.state == ChargingState.FAILED:
                self.state_failed()

            rate.sleep()


if __name__ == "__main__":
    try:
        machine = ChargingStateMachine()
        machine.run()
    except rospy.ROSInterruptException:
        pass
