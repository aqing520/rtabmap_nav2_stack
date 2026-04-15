#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped

class PoseKalmanFilter:
    def __init__(self):
        rospy.init_node('pose_filter', anonymous=True)
        
        # 卡尔曼滤波器（6状态：x,y,z,vx,vy,vz）
        self.kf = cv2.KalmanFilter(6, 3)
        self.kf.transitionMatrix = np.eye(6, dtype=np.float32)
        self.kf.transitionMatrix[0, 3] = 1
        self.kf.transitionMatrix[1, 4] = 1
        self.kf.transitionMatrix[2, 5] = 1
        self.kf.measurementMatrix = np.eye(6, dtype=np.float32)[:3, :]
        self.kf.processNoiseCov = np.eye(6, dtype=np.float32) * 0.01
        self.kf.measurementNoiseCov = np.eye(3, dtype=np.float32) * 0.1
        self.is_initialized = False
        
        self.sub = rospy.Subscriber("/aruco_pose_raw", PoseStamped, self.callback)
        self.pub = rospy.Publisher("/aruco_poses", PoseStamped, queue_size=10)
        
        rospy.loginfo("卡尔曼滤波节点启动")
    
    def callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        
        measurement = np.array([[np.float32(x)], [np.float32(y)], [np.float32(z)]])
        
        if not self.is_initialized:
            self.kf.statePre = np.array([[x], [y], [z], [0], [0], [0]], dtype=np.float32)
            self.kf.statePost = self.kf.statePre
            self.is_initialized = True
            smooth_x, smooth_y, smooth_z = x, y, z
        else:
            self.kf.predict()
            estimated = self.kf.correct(measurement)
            smooth_x = estimated[0][0]
            smooth_y = estimated[1][0]
            smooth_z = estimated[2][0]
        
        # 发布平滑后的位姿
        filtered_msg = PoseStamped()
        filtered_msg.header = msg.header
        filtered_msg.pose.position.x = smooth_x
        filtered_msg.pose.position.y = smooth_y
        filtered_msg.pose.position.z = smooth_z
        filtered_msg.pose.orientation = msg.pose.orientation
        
        self.pub.publish(filtered_msg)

if __name__ == '__main__':
    try:
        filter_node = PoseKalmanFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
