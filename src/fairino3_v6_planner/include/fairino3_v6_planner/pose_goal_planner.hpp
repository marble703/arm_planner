#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace fairino3_v6_planner {

class PoseGoalPlanner: public rclcpp::Node {
public:
    explicit PoseGoalPlanner(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );
    ~PoseGoalPlanner() = default;

private:
    bool debug_;

    // 最大轨迹点数
    size_t max_points_;

    // 回调函数：处理接收到的位姿目标
    void poseGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // MoveIt组件
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>
        planning_scene_interface_;

    // 规划和执行运动计划
    bool planToPose(const geometry_msgs::msg::PoseStamped& target_pose);
    bool executePlan();

    // ROS 2订阅者
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    // ROS 2发布者
    rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr
        display_trajectory_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
        trajectory_poses_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
        planning_success_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
        planning_status_publisher_;

    // 配置参数
    std::string planning_group_;
    std::string end_effector_link_;

    // 当前运动计划
    moveit::planning_interface::MoveGroupInterface::Plan current_plan_;

    // 提取轨迹关键点的工具函数
    geometry_msgs::msg::PoseArray
    extractTrajectoryPoses(const moveit_msgs::msg::RobotTrajectory& trajectory);
};

} // namespace fairino3_v6_planner
