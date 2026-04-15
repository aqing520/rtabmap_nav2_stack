#!/usr/bin/env python3
"""Subscribe to apriltag_ros AprilTagDetectionArray and republish one tag as PoseStamped for pose_filter."""
import rospy
from geometry_msgs.msg import PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray


def main():
    rospy.init_node("apriltag_pose_bridge", anonymous=True)
    target_id = int(rospy.get_param("~target_tag_id", 88))
    pub = rospy.Publisher("/aruco_pose_raw", PoseStamped, queue_size=10)

    def cb(msg):
        for det in msg.detections:
            if not det.id or int(det.id[0]) != target_id:
                continue
            pwc = det.pose
            out = PoseStamped()
            out.header = pwc.header
            out.pose = pwc.pose.pose
            pub.publish(out)
            return

    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, cb, queue_size=1)
    rospy.loginfo(
        "apriltag_pose_bridge: /tag_detections -> /aruco_pose_raw (tag id %s)", target_id
    )
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
