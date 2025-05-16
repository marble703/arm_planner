#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from math import pi
import time
import numpy as np

class PoseGoalPublisher(Node):
    def __init__(self):
        super().__init__('pose_goal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'target_pose', 10)
        self.get_logger().info('位姿目标发布节点已创建')
        
    def publish_pose(self, x, y, z, qx, qy, qz, qw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'发布目标位姿: 位置 ({x}, {y}, {z}), 四元数 ({qx}, {qy}, {qz}, {qw})')

def main():
    rclpy.init()
    node = PoseGoalPublisher()
    
    # 等待一些时间确保连接建立
    time.sleep(2.0)
    
    try:
        # 一开始等待，确保所有节点已完全准备好
        print("等待系统准备就绪...")
        time.sleep(5.0)
        
        # 示例1: 机械臂前方可达位置（约25厘米前方，高度30厘米）
        print("发送目标1：前方姿态")
        node.publish_pose(0.25, 0.25, 0.6, 0.0, 0.0, 0.0, 1.0)  # 姿态保持竖直
        
        # 等待规划和执行
        time.sleep(5.0)
        
        # # 示例2: 机械臂侧前方位置
        # print("发送目标2：侧前方姿态")
        # node.publish_pose(0.22, 0.15, 0.28, 0.0, -0.383, 0.0, 0.924)  # 轻微倾斜姿态
        
        # # 等待规划和执行
        # time.sleep(5.0)
        
        # # 示例3: 机械臂右侧略高位置
        # print("发送目标3：右侧略高姿态") 
        # node.publish_pose(0.18, -0.18, 0.35, 0.0, 0.383, 0.0, 0.924)  # 另一个倾斜姿态

    except Exception as e:
        node.get_logger().error(f'发生错误: {str(e)}')
    finally:
        # 清理资源
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
