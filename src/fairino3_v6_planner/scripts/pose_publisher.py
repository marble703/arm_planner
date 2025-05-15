#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf_transformations

class PosePublisherNode(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        
        # 创建位姿发布器
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            'target_pose',
            10)
            
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('位姿发布节点已启动，将每秒发布一次位姿')
        
        # 设置一个默认位姿
        self.pose = PoseStamped()
        self.pose.header.frame_id = 'base_link'
        self.pose.pose.position.x = 0.3
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 0.5
        
        # 计算四元数表示的水平姿态
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        self.pose.pose.orientation.x = q[0]
        self.pose.pose.orientation.y = q[1]
        self.pose.pose.orientation.z = q[2]
        self.pose.pose.orientation.w = q[3]
        
    def timer_callback(self):
        # 更新时间戳
        self.pose.header.stamp = self.get_clock().now().to_msg()
        
        # 发布位姿
        self.pose_publisher.publish(self.pose)
        self.get_logger().info(f'发布位姿: 坐标 ({self.pose.pose.position.x}, {self.pose.pose.position.y}, {self.pose.pose.position.z})')
    
    def update_pose(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """更新目标位姿"""
        self.pose.pose.position.x = x
        self.pose.pose.position.y = y
        self.pose.pose.position.z = z
        
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        self.pose.pose.orientation.x = q[0]
        self.pose.pose.orientation.y = q[1]
        self.pose.pose.orientation.z = q[2]
        self.pose.pose.orientation.w = q[3]
        
        self.get_logger().info(f'设置新位姿: 坐标 ({x}, {y}, {z}), 欧拉角 ({roll}, {pitch}, {yaw})')

def main(args=None):
    rclpy.init(args=args)
    
    pose_publisher = PosePublisherNode()
    
    if len(sys.argv) >= 4:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
        
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        
        if len(sys.argv) >= 7:
            roll = float(sys.argv[4])
            pitch = float(sys.argv[5])
            yaw = float(sys.argv[6])
        
        pose_publisher.update_pose(x, y, z, roll, pitch, yaw)
    
    rclpy.spin(pose_publisher)
    
    pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
