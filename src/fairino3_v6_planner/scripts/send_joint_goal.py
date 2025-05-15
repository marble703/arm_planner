#!/usr/bin/env python3
# 导入必要的库
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class JointGoalPublisher(Node):
    def __init__(self):
        super().__init__('joint_goal_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.get_logger().info('关节目标发布节点已创建')
        
    def publish_joints(self, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        msg.position = positions
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'发布目标关节位置: {positions}')

def main():
    rclpy.init()
    node = JointGoalPublisher()
    
    # 等待一些时间确保连接建立
    time.sleep(2.0)
    
    try:
        # 示例1: 使用SRDF中定义的pos1关节配置
        node.publish_joints([2.4468, -1.6214, 1.5465, -1.5877, -1.6368, 0.0])

        # 等待10秒，给规划和执行留出时间
        time.sleep(10.0)
        
        # 示例2: 使用SRDF中定义的pos2关节配置
        node.publish_joints([0.8606, -1.4189, 1.5465, -1.7227, -1.5018, 0.0])
        
        # 继续等待
        time.sleep(10.0)
        
        # 示例3: 回到初始姿态
        node.publish_joints([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    except Exception as e:
        node.get_logger().error(f'发生错误: {str(e)}')
    finally:
        # 清理资源
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
