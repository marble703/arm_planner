#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

class PosePublisher(Node):
    def __init__(self, pose_file: str, interval: float = 2):
        super().__init__('pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.interval = interval  # 发布间隔（秒）

        # 读取文件中所有的位姿数据
        with open(pose_file, 'r') as f:
            self.poses = self.parse_poses(f.readlines())

    def parse_poses(self, lines):
        poses = []
        for line in lines:
            line = line.strip().strip(',').strip('()')
            if not line:
                continue
            values = [float(v) for v in line.split(',')]
            if len(values) != 7:
                self.get_logger().warn(f"行格式不合法，跳过：{line}")
                continue
            poses.append(values)
        return poses

    def publish_loop(self):
        for pose_data in self.poses:
            x, y, z, qw, qx, qy, qz = pose_data
            msg = PoseStamped()
            msg.header.frame_id = "base_link"
            msg.header.stamp = self.get_clock().now().to_msg()

            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = z
            msg.pose.orientation.w = qw
            msg.pose.orientation.x = qx
            msg.pose.orientation.y = qy
            msg.pose.orientation.z = qz

            self.publisher_.publish(msg)
            self.get_logger().info(f"发布目标位姿: {pose_data}")
            time.sleep(self.interval)  # 使用 time.sleep 控制节奏（单线程）

def main():
    rclpy.init()
    node = PosePublisher('../test_data/001.txt', interval=1.0)
    node.publish_loop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
