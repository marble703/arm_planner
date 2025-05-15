#!/usr/bin/env python3
"""
测试脚本：先发布初始关节位姿，再发布目标末端位姿进行规划。
外部传入参数：
  --start j1 j2 j3 j4 j5 j6
  --target x y z qx qy qz qw

使用示例：
ros2 run fairino3_v6_planner test_start_and_goal.py --start 0 0 0 0 0 0 --target 0.3 0.0 0.4 0 0 0 1
  
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import argparse
import time

class TestStartAndGoal(Node):
    def __init__(self, start_positions, target_pose):
        super().__init__('test_start_and_goal')
        # 发布关节状态
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        # 发布末端位姿目标
        self.pose_pub = self.create_publisher(PoseStamped, 'target_pose', 10)
        self.start_positions = start_positions
        self.target_pose = target_pose
        self.get_logger().info(f'初始化：起始关节 {start_positions}, 目标位姿 {target_pose}')

    def run(self):
        # 等待系统准备
        self.get_logger().info('等待1秒，保证连接建立...')
        time.sleep(1.0)
        # 发布初始关节状态多次，确保PlanningSceneMonitor更新
        msg_js = JointState()
        msg_js.name = ['j1','j2','j3','j4','j5','j6']
        for i in range(5):
            msg_js.header.stamp = self.get_clock().now().to_msg()
            msg_js.position = self.start_positions
            self.joint_pub.publish(msg_js)
            time.sleep(0.2)
        self.get_logger().info('初始关节位姿已发布')
        # 等待规划场景刷新
        time.sleep(1.0)
        # 发布目标位姿
        msg_pose = PoseStamped()
        msg_pose.header.stamp = self.get_clock().now().to_msg()
        msg_pose.header.frame_id = 'base_link'
        x,y,z,qx,qy,qz,qw = self.target_pose
        msg_pose.pose.position.x = x
        msg_pose.pose.position.y = y
        msg_pose.pose.position.z = z
        msg_pose.pose.orientation.x = qx
        msg_pose.pose.orientation.y = qy
        msg_pose.pose.orientation.z = qz
        msg_pose.pose.orientation.w = qw
        self.pose_pub.publish(msg_pose)
        self.get_logger().info(f'发布目标位姿: position=({x},{y},{z}), orientation=({qx},{qy},{qz},{qw})')


def main():
    parser = argparse.ArgumentParser(description='测试：先设置当前关节，再发布目标位姿')
    parser.add_argument('--start', type=float, nargs=6, required=True,
                        help='6个关节角度 (rad)')
    parser.add_argument('--target', type=float, nargs=7, required=True,
                        help='目标位姿: x y z qx qy qz qw')
    args = parser.parse_args()

    rclpy.init()
    node = TestStartAndGoal(args.start, args.target)
    try:
        node.run()
        # 稍作等待再退出
        time.sleep(2.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
