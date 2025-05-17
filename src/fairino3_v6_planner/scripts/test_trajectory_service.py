#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from fairino3_v6_planner.srv import GetTrajectoryPoses
import time

class TrajectoryPosesClient(Node):
    def __init__(self):
        super().__init__('trajectory_poses_client')
        self.client = self.create_client(GetTrajectoryPoses, 'get_trajectory_poses')
        
        self.get_logger().info('轨迹关键点服务客户端已创建，等待服务可用...')
        
        # 确保服务可用
        while not self.client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('等待服务可用...')
        
        self.get_logger().info('服务已可用')
    
    def send_request(self, x, y, z, qx, qy, qz, qw):
        # 创建请求
        request = GetTrajectoryPoses.Request()
        request.target_pose = PoseStamped()
        request.target_pose.header.stamp = self.get_clock().now().to_msg()
        request.target_pose.header.frame_id = 'base_link'
        
        request.target_pose.pose.position.x = x
        request.target_pose.pose.position.y = y
        request.target_pose.pose.position.z = z
        request.target_pose.pose.orientation.x = qx
        request.target_pose.pose.orientation.y = qy
        request.target_pose.pose.orientation.z = qz
        request.target_pose.pose.orientation.w = qw
        
        self.get_logger().info(f'发送请求: 位置 ({x}, {y}, {z}), 四元数 ({qx}, {qy}, {qz}, {qw})')
        
        # 发送请求
        future = self.client.call_async(request)
        return future
        
def main():
    rclpy.init()
    client_node = TrajectoryPosesClient()
    
    try:
        # 发送请求到前方可达位置
        future = client_node.send_request(0.25, 0.25, 0.6, 0.0, 0.0, 0.0, 1.0)
        
        # 等待结果
        while rclpy.ok():
            rclpy.spin_once(client_node)
            if future.done():
                try:
                    response = future.result()
                    
                    if response.success:
                        client_node.get_logger().info('请求成功')
                        client_node.get_logger().info(f'轨迹点数量: {len(response.trajectory_poses.poses)}')
                        client_node.get_logger().info(f'服务消息: {response.status_message}')
                        
                        # 打印一些轨迹点信息
                        for i, pose in enumerate(response.trajectory_poses.poses):
                            client_node.get_logger().info(
                                f'点 {i}: 位置 ({pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f})'
                            )
                    else:
                        client_node.get_logger().error(f'请求失败: {response.status_message}')
                except Exception as e:
                    client_node.get_logger().error(f'处理响应时发生错误: {str(e)}')
                break
    
    except Exception as e:
        client_node.get_logger().error(f'发生错误: {str(e)}')
    
    finally:
        client_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
