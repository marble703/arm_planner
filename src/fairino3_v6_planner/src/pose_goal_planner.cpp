#include "fairino3_v6_planner/pose_goal_planner.hpp"
#include <chrono>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <tf2_eigen/tf2_eigen.hpp>
#include <thread>
#include "fairino3_v6_planner/srv/get_trajectory_poses.hpp"

namespace fairino3_v6_planner {

PoseGoalPlanner::PoseGoalPlanner(const rclcpp::NodeOptions& options):
    Node("fairino3_v6_planner", options) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 从参数获取配置
    this->declare_parameter("planning_group", "fairino3_v6_group");
    this->declare_parameter("end_effector_link", "wrist3_link");
    this->declare_parameter("debug", false);    // 默认值为false
    this->declare_parameter("max_points", 100); // 默认值为100

    planning_group_ = this->get_parameter("planning_group").as_string();
    end_effector_link_ = this->get_parameter("end_effector_link").as_string();
    debug_ = this->get_parameter("debug").as_bool();
    max_points_ =
        static_cast<size_t>(this->get_parameter("max_points").as_int());

    // 调试
    RCLCPP_DEBUG(
        this->get_logger(),
        "从配置中读取到的max_points: %zu",
        max_points_
    );
    RCLCPP_DEBUG(
        this->get_logger(),
        "从配置中读取到的debug: %s",
        debug_ ? "true" : "false"
    );

    RCLCPP_INFO(
        this->get_logger(),
        "初始化位姿规划器，规划组: %s，末端执行器: %s，最大轨迹点数: %zu，调试模式: %s",
        planning_group_.c_str(),
        end_effector_link_.c_str(),
        max_points_,
        debug_ ? "开启" : "关闭"
    );

    // 创建位姿目标订阅
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "target_pose",
        10,
        std::bind(
            &PoseGoalPlanner::poseGoalCallback,
            this,
            std::placeholders::_1
        )
    );

    // 创建结果发布者
    display_trajectory_publisher_ =
        this->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
            "display_trajectory",
            10
        );
    planning_success_publisher_ =
        this->create_publisher<std_msgs::msg::Bool>("planning_success", 10);
    planning_status_publisher_ =
        this->create_publisher<std_msgs::msg::String>("planning_status", 10);
        
    // 创建轨迹关键点服务
    trajectory_poses_service_ = this->create_service<fairino3_v6_planner::srv::GetTrajectoryPoses>(
        "get_trajectory_poses",
        std::bind(
            &PoseGoalPlanner::handleGetTrajectoryPoses,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    RCLCPP_INFO(
        this->get_logger(),
        "已订阅 'target_pose' 话题，创建结果发布话题，并提供'get_trajectory_poses'服务"
    );

    // 等待一段时间以确保MoveIt组件完全初始化
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 初始化MoveIt组件
    // 在一个单独的线程中创建move_group，避免阻塞主线程
    std::thread([this]() {
        // 延迟创建，确保节点完全初始化
        std::this_thread::sleep_for(std::chrono::seconds(3));
        try {
            move_group_ = std::make_shared<
                moveit::planning_interface::MoveGroupInterface>(
                this->shared_from_this(),
                planning_group_
            );
            if (debug_) {
                RCLCPP_INFO(this->get_logger(), "已创建 MoveGroupInterface");
            }
            planning_scene_interface_ = std::make_shared<
                moveit::planning_interface::PlanningSceneInterface>();
            if (debug_) {
                RCLCPP_INFO(this->get_logger(), "已创建 PlanningSceneInterface");
            }
            // 设置规划参数
            move_group_->setPlanningTime(5.0);
            move_group_->setNumPlanningAttempts(10);
            move_group_->setMaxVelocityScalingFactor(0.5);
            move_group_->setMaxAccelerationScalingFactor(0.5);
            move_group_->setEndEffectorLink(end_effector_link_);
            if (debug_) {
                RCLCPP_INFO(this->get_logger(), "已设置规划参数");
            }
            RCLCPP_INFO(this->get_logger(), "MoveIt组件初始化完成");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(
                this->get_logger(),
                "MoveIt组件初始化失败: %s",
                e.what()
            );
        }
    }).detach();

    RCLCPP_INFO(this->get_logger(), "位姿规划器初始化中...");
}

void PoseGoalPlanner::poseGoalCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg
) {
    RCLCPP_INFO(this->get_logger(), "\n\n\n收到新的目标位姿");

    // 检查MoveGroup是否已初始化
    if (!move_group_) {
        std_msgs::msg::String status_msg;
        status_msg.data = "MoveGroup尚未初始化，请稍后再试";
        planning_status_publisher_->publish(status_msg);
        RCLCPP_WARN(
            this->get_logger(),
            "MoveGroup尚未初始化，忽略当前目标位姿请求"
        );
        return;
    }

    // 确保位姿参考坐标系匹配
    if (msg->header.frame_id.empty()) {
        msg->header.frame_id = "base_link";
        RCLCPP_WARN(
            this->get_logger(),
            "目标位姿没有指定参考坐标系，默认使用 'base_link'"
        );
    }

    // 规划到目标位姿并发布结果
    if (!planToPose(*msg)) {
        RCLCPP_ERROR(this->get_logger(), "无法规划到目标位姿");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "规划完成，已发布轨迹");
    // 由于不使用 moveit 执行，跳过执行阶段，仅发布规划结果
    return;
}

bool PoseGoalPlanner::planToPose(
    const geometry_msgs::msg::PoseStamped& target_pose
) {
    moveit::core::RobotStatePtr current_robot_state =
        move_group_->getCurrentState();
    if (current_robot_state) {
        std::stringstream ss;
        current_robot_state->printStatePositions(ss);
        if (debug_) {
            RCLCPP_DEBUG(
                this->get_logger(),
                "当前机械臂状态 (关节位置): %s",
                ss.str().c_str()
            );
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "无法获取当前机械臂状态以记录初始状态");
    }
    if (debug_) {
        RCLCPP_DEBUG(
            this->get_logger(),
            "目标位姿: %f, %f, %f",
            target_pose.pose.position.x,
            target_pose.pose.position.y,
            target_pose.pose.position.z
        );
    }

    RCLCPP_INFO(this->get_logger(), "规划到目标位姿");

    move_group_->setPoseTarget(target_pose);

    // 进行运动规划
    bool success =
        (move_group_->plan(current_plan_)
         == moveit::core::MoveItErrorCode::SUCCESS);

    // 发布规划结果状态
    std_msgs::msg::Bool success_msg;
    success_msg.data = success;
    planning_success_publisher_->publish(success_msg);

    std_msgs::msg::String status_msg;
    status_msg.data = success ? "规划成功" : "规划失败";
    planning_status_publisher_->publish(status_msg);

    if (success) {
        RCLCPP_INFO(this->get_logger(), "运动规划成功");

        // 发布轨迹用于显示
        moveit_msgs::msg::DisplayTrajectory display_trajectory;
        display_trajectory.trajectory_start = current_plan_.start_state_;
        display_trajectory.trajectory.push_back(current_plan_.trajectory_);
        if (debug_) {
            RCLCPP_DEBUG(
                this->get_logger(),
                "生成的轨迹路径点数量: %zu",
                display_trajectory.trajectory.size()
            );
        }
        display_trajectory_publisher_->publish(display_trajectory);

        // 提取并发布轨迹关键点
        geometry_msgs::msg::PoseArray trajectory_poses =
            extractTrajectoryPoses(current_plan_.trajectory_);
        trajectory_poses.header.stamp = this->now();
        trajectory_poses.header.frame_id = target_pose.header.frame_id;

        if (debug_) {
            RCLCPP_DEBUG(this->get_logger(), "提取的轨迹关键点: ");
            int i = 0;
            for (const auto& pose: trajectory_poses.poses) {
                RCLCPP_DEBUG(
                    this->get_logger(),
                    "关键点:%d \n %f, %f, %f\n",
                    i++,
                    pose.position.x,
                    pose.position.y,
                    pose.position.z
                );
            }
        }

        // 不再发布轨迹关键点，改为通过服务提供
    } else {
        RCLCPP_ERROR(this->get_logger(), "运动规划失败");
    }

    return success;
}

bool PoseGoalPlanner::executePlan() {
    RCLCPP_INFO(this->get_logger(), "执行运动计划");

    bool success =
        (move_group_->execute(current_plan_)
         == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(this->get_logger(), "运动执行成功");
    } else {
        RCLCPP_ERROR(this->get_logger(), "运动执行失败");
    }

    return success;
}

// 提取轨迹关键点
geometry_msgs::msg::PoseArray PoseGoalPlanner::extractTrajectoryPoses(
    const moveit_msgs::msg::RobotTrajectory& trajectory
) {
    geometry_msgs::msg::PoseArray pose_array;

    if (trajectory.joint_trajectory.points.empty()
        && trajectory.multi_dof_joint_trajectory.points.empty())
    {
        RCLCPP_WARN(this->get_logger(), "轨迹中没有任何点");
        return pose_array;
    }

    // 获取轨迹中的关键点
    size_t num_points = 0;
    if (!trajectory.joint_trajectory.points.empty()) {
        num_points = trajectory.joint_trajectory.points.size();
    } else if (!trajectory.multi_dof_joint_trajectory.points.empty()) {
        num_points = trajectory.multi_dof_joint_trajectory.points.size();
    }

    if (num_points == 0) {
        RCLCPP_WARN(this->get_logger(), "轨迹中没有有效的点");
        return pose_array;
    }

    // 如果轨迹点太多，仅选取部分关键点
    // 计算采样步长
    size_t step = 1;
    if (num_points > max_points_) {
        step = num_points / (max_points_ - 1) + 1;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "轨迹点总数: %zu, 步长: %zu",
        num_points,
        step
    );

    // 获取当前机器人状态并克隆用于轨迹点计算
    moveit::core::RobotStatePtr robot_state = move_group_->getCurrentState();
    if (!robot_state) {
        RCLCPP_WARN(this->get_logger(), "无法获取当前机器人状态");
        return pose_array;
    }

    for (size_t i = 0; i < num_points; i += step) {
        // 基于轨迹中的关节位置设置机器人状态
        if (!trajectory.joint_trajectory.points.empty()) {
            try {
                robot_state->setJointGroupPositions(
                    planning_group_,
                    trajectory.joint_trajectory.points[i].positions
                );
                robot_state->update(); // 更新所有变换
            } catch (const std::exception& e) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "设置机器人状态时出错: %s",
                    e.what()
                );
                continue;
            }
        } else if (!trajectory.multi_dof_joint_trajectory.points.empty()) {
            // 处理多自由度轨迹点 (如果有)
            RCLCPP_INFO(this->get_logger(), "处理多自由度轨迹点");
            continue;
        }

        try {
            // 计算末端执行器位姿
            const Eigen::Isometry3d& end_effector_pose =
                robot_state->getGlobalLinkTransform(end_effector_link_);

            // 转换为几何消息
            geometry_msgs::msg::Pose pose;
            pose = tf2::toMsg(end_effector_pose);
            pose_array.poses.push_back(pose);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(
                this->get_logger(),
                "计算末端执行器位姿时出错: %s",
                e.what()
            );
        }
    }

    // 确保包含最后一个点
    if (!pose_array.poses.empty() && num_points > 0
        && (num_points - 1) % step != 0) {
        size_t last_idx = num_points - 1;
        try {
            if (!trajectory.joint_trajectory.points.empty()) {
                robot_state->setJointGroupPositions(
                    planning_group_,
                    trajectory.joint_trajectory.points[last_idx].positions
                );
                robot_state->update();

                const Eigen::Isometry3d& end_effector_pose =
                    robot_state->getGlobalLinkTransform(end_effector_link_);

                geometry_msgs::msg::Pose pose = tf2::toMsg(end_effector_pose);
                pose_array.poses.push_back(pose);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(
                this->get_logger(),
                "添加最后一个轨迹点时出错: %s",
                e.what()
            );
        }
    }

    RCLCPP_INFO(
        this->get_logger(),
        "提取了 %zu 个轨迹关键点",
        pose_array.poses.size()
    );
    return pose_array;
}

void PoseGoalPlanner::handleGetTrajectoryPoses(
    const std::shared_ptr<fairino3_v6_planner::srv::GetTrajectoryPoses::Request> request,
    std::shared_ptr<fairino3_v6_planner::srv::GetTrajectoryPoses::Response> response
) {
    RCLCPP_INFO(this->get_logger(), "收到轨迹关键点服务请求");

    // 检查MoveGroup是否已初始化
    if (!move_group_) {
        if (!initializationAttempted_) {
            RCLCPP_INFO(this->get_logger(), "MoveGroup未初始化，尝试初始化");
            initializeMoveItComponents();
            
            // 给一些时间让初始化完成
            std::this_thread::sleep_for(std::chrono::seconds(3));
            
            if (!move_group_) {
                response->success = false;
                response->status_message = "无法初始化MoveGroup组件";
                RCLCPP_ERROR(this->get_logger(), "服务无法执行：MoveGroup组件初始化失败");
                return;
            }
        } else {
            response->success = false;
            response->status_message = "MoveGroup组件未初始化";
            RCLCPP_ERROR(this->get_logger(), "服务无法执行：MoveGroup组件未初始化");
            return;
        }
    }

    // 确保位姿参考坐标系匹配
    geometry_msgs::msg::PoseStamped target_pose = request->target_pose;
    if (target_pose.header.frame_id.empty()) {
        target_pose.header.frame_id = "base_link";
        RCLCPP_WARN(
            this->get_logger(),
            "目标位姿没有指定参考坐标系，默认使用 'base_link'"
        );
    }

    // 进行运动规划
    bool planning_success = planToPose(target_pose);
    
    // 设置响应
    response->success = planning_success;
    
    if (planning_success) {
        // 提取并返回轨迹关键点
        response->trajectory_poses = extractTrajectoryPoses(current_plan_.trajectory_);
        response->trajectory_poses.header.stamp = this->now();
        response->trajectory_poses.header.frame_id = target_pose.header.frame_id;
        response->status_message = "规划成功";
        
        RCLCPP_INFO(
            this->get_logger(),
            "返回 %zu 个轨迹关键点",
            response->trajectory_poses.poses.size()
        );
    } else {
        response->status_message = "规划失败";
        RCLCPP_ERROR(this->get_logger(), "无法规划到目标位姿");
    }
}

void PoseGoalPlanner::initializeMoveItComponents() {
    RCLCPP_INFO(this->get_logger(), "开始初始化MoveIt组件...");
    initializationAttempted_ = true;
    
    try {
        move_group_ = std::make_shared<
            moveit::planning_interface::MoveGroupInterface>(
            this->shared_from_this(),
            planning_group_
        );
        if (debug_) {
            RCLCPP_INFO(this->get_logger(), "已创建 MoveGroupInterface");
        }
        
        planning_scene_interface_ = std::make_shared<
            moveit::planning_interface::PlanningSceneInterface>();
        if (debug_) {
            RCLCPP_INFO(this->get_logger(), "已创建 PlanningSceneInterface");
        }
        
        // 设置规划参数
        move_group_->setPlanningTime(5.0);
        move_group_->setNumPlanningAttempts(10);
        move_group_->setMaxVelocityScalingFactor(0.5);
        move_group_->setMaxAccelerationScalingFactor(0.5);
        move_group_->setEndEffectorLink(end_effector_link_);
        
        if (debug_) {
            RCLCPP_INFO(this->get_logger(), "已设置规划参数");
        }
        
        RCLCPP_INFO(this->get_logger(), "MoveIt组件初始化完成");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(
            this->get_logger(),
            "MoveIt组件初始化失败: %s",
            e.what()
        );
    }
}
} // namespace fairino3_v6_planner

// 主函数入口点
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    auto planner_node =
        std::make_shared<fairino3_v6_planner::PoseGoalPlanner>(options);

    RCLCPP_INFO(planner_node->get_logger(), "fairino3_v6规划器节点已启动");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(planner_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
