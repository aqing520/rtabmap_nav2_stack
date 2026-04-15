#!/usr/bin/env python3
"""
充电目标坐标转换节点 - 修正版
功能：
1. 接收相机坐标系(camera_link)下的ArUco位姿
2. 应用几何偏移，计算充电底座位置
3. 转换到机器人本体坐标系(base_link)
4. (可选) 转换到全局地图坐标系(map)

TF链路依赖：
- 方案A: map -> odom -> laser_link -> base_link -> camera_link
- 方案B: map -> odom -> base_link -> laser_link / camera_link
"""
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import numpy as np

class ChargingPoseTransformer:
    def __init__(self):
        rospy.init_node('charging_pose_transformer', anonymous=True)
        
        # ================= TF监听器 =================
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))  # 增加缓存时间到10秒
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # ================= 参数配置 =================
        # 是否转换到全局地图坐标系（需要SLAM系统正常运行）
        self.use_map_frame = rospy.get_param('~use_map_frame', False)
        
        # 几何偏移：充电底座相对二维码的偏移量（在相机坐标系下）
        self.offset_x = rospy.get_param('~geometry/offset_x', 0.0)
        self.offset_y = rospy.get_param('~geometry/offset_y', -0.616)
        self.offset_z = rospy.get_param('~geometry/offset_z', -1.00)
        
        # ================= 发布器和订阅器 =================
        # 发布base_link坐标系下的充电目标（用于视觉伺服）
        self.dock_pub_base = rospy.Publisher("/charging_goal_base", PoseStamped, queue_size=10)
        
        # 发布map坐标系下的充电目标（用于全局导航，可选）
        self.dock_pub_map = rospy.Publisher("/charging_goal_map", PoseStamped, queue_size=10)
        
        # 订阅滤波后的ArUco位姿（camera_link坐标系）
        self.aruco_sub = rospy.Subscriber("/aruco_poses", PoseStamped, self.callback)
        
        # 订阅充电状态
        self.charging_state = "IDLE"
        self.state_sub = rospy.Subscriber("/charging_state", String, self.state_callback)
        
        # ================= 状态变量 =================
        self.tf_fail_count = 0  # TF查询失败计数
        self.last_valid_pose_base = None  # 缓存最后一个有效的base_link坐标
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("充电目标坐标转换节点已启动")
        rospy.loginfo(f"模式: {'使用全局地图坐标系(map)' if self.use_map_frame else '使用本体坐标系(base_link)'}")
        rospy.loginfo(f"几何偏移: x={self.offset_x}m, y={self.offset_y}m, z={self.offset_z}m")
        rospy.loginfo("=" * 60)
        
        # 等待TF树建立
        rospy.sleep(1.0)
        self.check_tf_tree()

    def check_tf_tree(self):
        """启动时检查TF树完整性"""
        try:
            # 检查 camera_link -> base_link
            self.tf_buffer.lookup_transform(
                "base_link", "camera_link", 
                rospy.Time(0), rospy.Duration(3.0)
            )
            rospy.loginfo("✓ TF链路检查通过: camera_link -> base_link")
            
            # 检查 base_link -> map (可选)
            if self.use_map_frame:
                try:
                    self.tf_buffer.lookup_transform(
                        "map", "base_link",
                        rospy.Time(0), rospy.Duration(3.0)
                    )
                    rospy.loginfo("✓ TF链路检查通过: base_link -> map")
                except:
                    rospy.logwarn("⚠ 警告: map->base_link 变换不可用，请确认SLAM系统已启动")
                    rospy.logwarn("  将仅发布base_link坐标系下的充电目标")
                    
        except Exception as e:
            rospy.logerr(f"❌ TF链路检查失败: {e}")
            rospy.logerr("  请检查:")
            rospy.logerr("  1. launch文件中是否发布了 base_link->camera_link 静态TF")
            rospy.logerr("  2. 是否删除了手动的 map->base_link 静态TF")
            rospy.logerr("  3. SLAM系统(Fast-LIO)是否正常运行")

    def state_callback(self, msg):
        """充电状态回调"""
        self.charging_state = msg.data

    def callback(self, msg):
        """
        ArUco位姿回调函数
        输入: camera_link坐标系下的二维码位姿
        输出: base_link和map坐标系下的充电底座位姿
        """
        # 如果已经对接完成，停止处理
        if self.charging_state == "DOCKED":
            return
        
        try:
            # ========== 步骤1：应用几何偏移 ==========
            # 在相机坐标系下，计算充电底座位置
            dock_pose_camera = PoseStamped()
            dock_pose_camera.header = msg.header
            dock_pose_camera.header.stamp = rospy.Time.now()
            
            # 应用偏移量
            dock_pose_camera.pose.position.x = msg.pose.position.x + self.offset_x
            dock_pose_camera.pose.position.y = msg.pose.position.y + self.offset_y
            dock_pose_camera.pose.position.z = msg.pose.position.z + self.offset_z
            
            # 姿态保持不变（假设充电底座与二维码朝向一致）
            dock_pose_camera.pose.orientation = msg.pose.orientation
            
            # ========== 步骤2：转换到base_link坐标系 ==========
            try:
                # 查询 camera_link -> base_link 的变换
                transform_cam_to_base = self.tf_buffer.lookup_transform(
                    "base_link",  # 目标坐标系
                    dock_pose_camera.header.frame_id,  # 源坐标系（camera_link）
                    rospy.Time(0),  # 使用最新的变换
                    rospy.Duration(1.0)  # 超时时间
                )
                
                # 执行坐标变换
                dock_pose_base = tf2_geometry_msgs.do_transform_pose(
                    dock_pose_camera, transform_cam_to_base
                )
                dock_pose_base.header.frame_id = "base_link"
                dock_pose_base.header.stamp = rospy.Time.now()
                
                # 缓存有效位姿（用于TF临时失效时）
                self.last_valid_pose_base = dock_pose_base
                
                # 发布base_link坐标系下的充电目标
                self.dock_pub_base.publish(dock_pose_base)
                
                # 日志输出（限流1Hz）
                rospy.loginfo_throttle(1.0, 
                    f"[base_link] 充电目标 → "
                    f"x:{dock_pose_base.pose.position.x:.3f}m, "
                    f"y:{dock_pose_base.pose.position.y:.3f}m, "
                    f"z:{dock_pose_base.pose.position.z:.3f}m")
                
                # 重置失败计数
                self.tf_fail_count = 0
                
            except (tf2_ros.LookupException, 
                    tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                # TF查询失败
                self.tf_fail_count += 1
                
                # 每10次失败输出一次警告
                if self.tf_fail_count % 10 == 0:
                    rospy.logwarn(
                        f"⚠ camera_link->base_link 变换失败 (第{self.tf_fail_count}次)\n"
                        f"  错误: {e}\n"
                        f"  请检查静态TF是否正确发布"
                    )
                
                # 使用缓存的位姿（如果有）
                if self.last_valid_pose_base is not None:
                    self.dock_pub_base.publish(self.last_valid_pose_base)
                    rospy.loginfo_throttle(5.0, "使用缓存的充电目标位姿")
                
                return  # 不继续处理map坐标系
            
            # ========== 步骤3：（可选）转换到map坐标系 ==========
            if self.use_map_frame:
                try:
                    # 查询 base_link -> map 的变换
                    transform_base_to_map = self.tf_buffer.lookup_transform(
                        "map",  # 目标坐标系
                        "base_link",  # 源坐标系
                        rospy.Time(0),
                        rospy.Duration(1.0)
                    )
                    
                    # 执行坐标变换
                    dock_pose_map = tf2_geometry_msgs.do_transform_pose(
                        dock_pose_base, transform_base_to_map
                    )
                    dock_pose_map.header.frame_id = "map"
                    dock_pose_map.header.stamp = rospy.Time.now()
                    
                    # 发布map坐标系下的充电目标
                    self.dock_pub_map.publish(dock_pose_map)
                    
                    rospy.loginfo_throttle(1.0, 
                        f"[map] 充电目标 → "
                        f"X:{dock_pose_map.pose.position.x:.2f}m, "
                        f"Y:{dock_pose_map.pose.position.y:.2f}m")
                    
                except (tf2_ros.LookupException, 
                        tf2_ros.ConnectivityException, 
                        tf2_ros.ExtrapolationException) as e:
                    # map坐标系变换失败（通常是SLAM系统未启动）
                    rospy.logwarn_throttle(5.0, 
                        f"⚠ base_link->map 变换失败: {e}\n"
                        f"  请确认SLAM系统（Fast-LIO + PGO）已启动\n"
                        f"  当前仅发布base_link坐标系下的充电目标"
                    )
        
        except Exception as e:
            rospy.logerr(f"❌ 位姿转换异常: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())

if __name__ == '__main__':
    try:
        transformer = ChargingPoseTransformer()
        rospy.loginfo("等待ArUco检测数据...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点已关闭")