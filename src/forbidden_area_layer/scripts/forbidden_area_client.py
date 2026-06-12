#!/usr/bin/env python3
"""
禁行区客户端脚本
用于前端调用后端的禁行区数据
"""

import rclpy
from rclpy.node import Node
from forbidden_area_layer.srv import GetForbiddenAreas, SetForbiddenAreas
from forbidden_area_layer.msg import ForbiddenArea
from geometry_msgs.msg import Point

GET_SERVICE = '/global_costmap/get_forbidden_areas'
SET_SERVICE = '/global_costmap/set_forbidden_areas'

class ForbiddenAreaClient(Node):
    def __init__(self):
        super().__init__('forbidden_area_client')
        self.get_areas_client = self.create_client(GetForbiddenAreas, GET_SERVICE)
        self.set_areas_client = self.create_client(SetForbiddenAreas, SET_SERVICE)
        
        # 等待服务可用
        while not self.get_areas_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{GET_SERVICE} service not available, waiting...')
        while not self.set_areas_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{SET_SERVICE} service not available, waiting...')
    
    def get_forbidden_areas(self):
        """获取禁行区列表"""
        request = GetForbiddenAreas.Request()
        future = self.get_areas_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result():
            return future.result().areas
        else:
            self.get_logger().error('Failed to get forbidden areas')
            return []
    
    def set_forbidden_areas(self, areas):
        """设置禁行区列表"""
        request = SetForbiddenAreas.Request()
        request.areas = areas
        
        future = self.set_areas_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result():
            return future.result().success, future.result().message
        else:
            self.get_logger().error('Failed to set forbidden areas')
            return False, 'Service call failed'

def create_forbidden_area(points):
    """创建禁行区消息"""
    area = ForbiddenArea()
    for point in points:
        p = Point()
        p.x = point[0]
        p.y = point[1]
        p.z = 0.0
        area.points.append(p)
    return area

def main():
    rclpy.init()
    client = ForbiddenAreaClient()
    
    # 测试获取禁行区
    print("\n=== 获取禁行区 ===")
    areas = client.get_forbidden_areas()
    print(f"获取到 {len(areas)} 个禁行区")
    for i, area in enumerate(areas):
        print(f"禁行区 {i+1}: {[(p.x, p.y) for p in area.points]}")
    
    # 测试设置禁行区
    print("\n=== 设置禁行区 ===")
    new_areas = [
        create_forbidden_area([[0, 0], [2, 0], [2, 2], [0, 2]]),
        create_forbidden_area([[3, 3], [5, 3], [5, 5], [3, 5]])
    ]
    success, message = client.set_forbidden_areas(new_areas)
    print(f"设置禁行区: {success}, 消息: {message}")
    
    # 再次获取禁行区，验证设置是否成功
    print("\n=== 验证禁行区 ===")
    areas = client.get_forbidden_areas()
    print(f"获取到 {len(areas)} 个禁行区")
    for i, area in enumerate(areas):
        print(f"禁行区 {i+1}: {[(p.x, p.y) for p in area.points]}")
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
