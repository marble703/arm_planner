#!/usr/bin/env python3
# 测试关节状态服务的脚本
# 这个脚本调用get_joint_states服务，获取轨迹中每个关键点的关节角度

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from fairino3_v6_planner.srv import GetJointStates
import time
import math
import numpy as np

class JointStateClientNode(Node):
    def __init__(self):
        super().__init__('joint_state_client')
        self.get_logger().info('初始化关节状态客户端节点')
        
        # 创建服务客户端
        self.client = self.create_client(GetJointStates, 'get_joint_states')
        
        # 等待服务可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务可用...')
        
        self.get_logger().info('服务已连接！')
    
    def send_request(self, x, y, z, roll=0.0, pitch=math.pi/2, yaw=0.0):
        """发送关节状态请求"""
        # 创建请求
        request = GetJointStates.Request()
        
        # 设置目标姿势
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'base_link'
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        
        # 使用欧拉角计算四元数
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        pose.pose.orientation.w = cy * cp * cr + sy * sp * sr
        pose.pose.orientation.x = cy * cp * sr - sy * sp * cr
        pose.pose.orientation.y = sy * cp * sr + cy * sp * cr
        pose.pose.orientation.z = sy * cp * cr - cy * sp * sr
        
        request.target_pose = pose
        
        # 发送请求
        self.get_logger().info(f'发送请求: 位置({x}, {y}, {z}), 方向(roll={roll}, pitch={pitch}, yaw={yaw})')
        future = self.client.call_async(request)
        
        # 等待响应
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()
    
    def print_response(self, response):
        """打印响应"""
        if response.success:
            self.get_logger().info('服务调用成功!')
            self.get_logger().info(f'状态消息: {response.status_message}')
            
            # 打印轨迹关键点的关节状态
            if len(response.trajectory_joint_states) > 0:
                # 获取关节名称（从第一个关键点获取）
                joint_names = response.trajectory_joint_states[0].name
                
                # 打印关键点数量
                self.get_logger().info(f'轨迹关键点数量: {len(response.trajectory_joint_states)}')
                
                # 打印每个关键点的关节状态
                for i in range(len(response.trajectory_joint_states)):
                    point = response.trajectory_joint_states[i]
                    self.get_logger().info(f'第 {i+1} 个关键点的关节状态:')
                    for j, (name, position) in enumerate(zip(joint_names, point.position)):
                        self.get_logger().info(f'  {name}: {position:.6f} rad ({math.degrees(position):.2f} 度)')

            else:
                self.get_logger().warn('响应中没有关节状态数据')
        else:
            self.get_logger().error(f'服务调用失败: {response.status_message}')

def main():
    """主函数"""
    rclpy.init()
    
    # 创建客户端节点
    node = JointStateClientNode()
    
    try:
        # 测试不同的位置
        test_positions = [
            (0.3, 0.0, 0.5),    # 正前方
            # (0.3, 0.2, 0.5),    # 右前方
            # (0.3, -0.2, 0.5),   # 左前方
            # (0.3, 0.0, 0.3),    # 正前方低位置
            # (0.3, 0.0, 0.4)     # 正前方高位置
        ]
        
        for i, (x, y, z) in enumerate(test_positions):
            print(f"\n\n测试 {i+1}/{len(test_positions)}: 位置({x}, {y}, {z})")
            response = node.send_request(x, y, z)
            node.print_response(response)
            time.sleep(1)  # 等待1秒再进行下一个测试
        
    except KeyboardInterrupt:
        pass
    finally:
        # 清理并关闭
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
