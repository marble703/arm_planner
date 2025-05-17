# Arm Planner 项目

本项目基于 ROS 2（Humble）和 MoveIt 2 搭建机械臂运动规划系统，包含以下主要模块：

1. **fairino_description**：机器人模型描述包，包含 URDF/xacro 文件，用于定义机械臂的几何和物理属性。此包原本包含多个机械臂描述，由于本项目仅需要规划一种机械臂故仅保留 `fairino3_v6` 描述文件。
2. **fairino3_v6_moveit2_config**：MoveIt 2 配置包，使用 `moveit_setup_assistant` 自动生成的配置（SRDF、关节限位、运动学、控制器、OMPL 规划器等），并包含 `move_group` 和 `rviz2` 的启动文件。
3. **fairino3_v6_planner**：自定义规划器节点和测试脚本：
   - C++ 实现 `PoseGoalPlanner` 节点，订阅目标位姿 (`target_pose`)，调用 MoveIt 2 接口进行路径规划，并发布规划轨迹。
   - 提供轨迹关键点服务 (`get_trajectory_poses`)，接收目标位姿，返回规划轨迹的关键点。
   - 提供关节状态服务 (`get_joint_states`)，接收目标位姿，返回规划得到的关节角度（JointState）。
   - `scripts/` 目录下包含测试脚本：
     - `send_pose_goal.py`：发布笛卡尔空间目标位姿（`target_pose`）。
     - `send_joint_goal.py`：发布关节空间目标 (`joint_states`) 消息，用于测试关节规划。
     - `test_start_and_goal.py`：综合测试，先发布初始关节状态，再发布目标位姿，完成一次端到端的规划流程。
     - `test_trajectory_service.py`：测试轨迹关键点服务，发送目标位姿并接收轨迹关键点。
     - `test_joint_states_service.py`：测试关节状态服务，发送目标位姿并接收对应的关节角度。
   - `config/` 目录下包含可配置参数：
     - `config.yaml`：包含规划器配置参数，如最大轨迹点数量(max_points)等。

---

## 开发规划

- 添加关节限位
- 添加避障 
- 添加 MoveIt 初始化重试

## 编译与依赖

```bash
cd ~/arm/arm_planner
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select fairino3_v6_planner fairino3_v6_moveit2_config fairino_description
source install/setup.bash
```

> 请确保已安装 `robot_state_publisher`, `xacro`, `moveit_ros_move_group`, `moveit_simple_controller_manager` 等 ROS 2 运行时包。

---

## 启动方法

运行以下命令，启动 MoveIt 2 `move_group`、RViz2 和自定义规划节点：

```bash
source install/setup.bash
ros2 launch fairino3_v6_planner pose_goal_planner.launch.py
```

- `use_sim_time` 参数默认为 `true`，可通过 `-p use_sim_time:=false` 切换至系统时钟。
- `with_rviz` 参数默认为 `false`, 可通过 `with_rviz:=true` 启动可视化界面
- 启动可视化界面后可在 RViz2 中查看机器人模型并使用 Motion Planning 面板进行交互式规划。

---

## 测试脚本使用方法

### 1. 发布笛卡尔空间位姿目标

```bash
python3 src/fairino3_v6_planner/scripts/send_pose_goal.py
```

- 脚本内已设置三个示例目标位姿，可根据需求修改脚本中的坐标和四元数。

### 2. 发布关节空间目标

```bash
python3 src/fairino3_v6_planner/scripts/send_joint_goal.py
```

- 使用 SRDF 中定义的 `pos1`、`pos2` 等预设关节角，实现关节空间测试。

### 3. 综合测试：先设起始关节，再发目标位姿

```bash
ros2 run fairino3_v6_planner test_start_and_goal.py \
  --start j1 j2 j3 j4 j5 j6 \
  --target x y z qx qy qz qw
```

- `--start`：6 个关节角度 (rad)
- `--target`：目标位姿 (位置 x,y,z + 四元数 qx,qy,qz,qw)

示例：
```bash
ros2 run fairino3_v6_planner test_start_and_goal.py \
  --start 0 0 0 0 0 0 \
  --target 0.3 0.0 0.4 0 0 0 1
```

运行后，脚本会：
1. 多次发布初始 `joint_states`，更新 MoveIt 起始状态；
2. 发布目标位姿触发规划；
3. 输出规划结果并在 RViz2 中显示轨迹。

### 4. 测试轨迹关键点服务

```bash
python3 src/fairino3_v6_planner/scripts/test_trajectory_service.py
```

- 脚本创建一个服务客户端，向 `get_trajectory_poses` 服务发送带有目标位姿的请求
- 接收并打印规划结果，包括轨迹关键点数量和部分关键点位置

---

## ROS2 接口

本包暴露了以下 ROS 2 话题，可供外部节点发布/订阅，灵活触发规划并获取结果：

### 订阅（Trigger Planning）
- `/target_pose` (geometry_msgs/PoseStamped)：接收末端期望位姿，收到消息后触发笛卡尔空间规划。
  示例：
  ```bash
  ros2 topic pub /target_pose geometry_msgs/msg/PoseStamped \
    "{ header: { frame_id: 'base_link' }, pose: { position: { x: 0.3, y: 0.0, z: 0.4 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } } }"
  ```
- `/joint_states` (sensor_msgs/JointState)：可选，发布一组关节角度以更新起始状态后再触发位姿规划。
  示例：
  ```bash
  ros2 topic pub /joint_states sensor_msgs/msg/JointState \
    "{ header: { frame_id: 'base_link' }, name: ['j1','j2','j3','j4','j5','j6'], position: [0,0,0,0,0,0] }"
  ```

### 发布（Planning Results）
- `/display_trajectory` (moveit_msgs/DisplayTrajectory)：规划结果完整轨迹，可在 RViz 中可视化。
- `/trajectory_poses` (geometry_msgs/PoseArray)：提取后的若干关键末端位姿点，用于外部可视化或后续处理。
- `/planning_success` (std_msgs/Bool)：规划是否成功（true/false）。
- `/planning_status` (std_msgs/String)：规划状态文字描述，例如 "规划成功" 或 "规划失败"。

### 服务接口
- `/get_trajectory_poses` (fairino3_v6_planner/srv/GetTrajectoryPoses)：提供轨迹关键点的服务
  - 请求：目标位姿 (geometry_msgs/PoseStamped)
  - 响应：轨迹关键点数组 (geometry_msgs/PoseArray)、成功标志 (bool) 和状态消息 (string)

要查看消息，可以使用：
```bash
ros2 topic echo /planning_success
ros2 topic echo /planning_status
ros2 service list | grep trajectory  # 查看可用服务
```

---

## 可用服务

### 1. 轨迹关键点服务 (GetTrajectoryPoses)

获取从当前位置到目标位置的离散轨迹点序列：

```
# 请求
ros2 service call /get_trajectory_poses fairino3_v6_planner/srv/GetTrajectoryPoses "{target_pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.3, y: 0.0, z: 0.5}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}}"
```

**服务定义**:
```
# 请求：目标位姿
geometry_msgs/PoseStamped target_pose
---
# 响应：轨迹关键点、成功标志和状态消息
geometry_msgs/PoseArray trajectory_poses
bool success
string status_message
```

### 2. 关节状态服务 (GetJointStates)

获取从当前位置到目标位置的整个轨迹中，每个关键点的关节角度：

```
# 请求
ros2 service call /get_joint_states fairino3_v6_planner/srv/GetJointStates "{target_pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.3, y: 0.0, z: 0.5}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}}"
```

**服务定义**:
```
# 请求：目标位姿
geometry_msgs/PoseStamped target_pose
---
# 响应：轨迹中每个关键点的关节状态数组、成功标志和状态消息
sensor_msgs/JointState[] trajectory_joint_states
bool success
string status_message
```

也可以使用测试脚本进行服务测试：

```bash
# 测试轨迹关键点服务
ros2 run fairino3_v6_planner test_trajectory_service.py

# 测试关节状态服务
ros2 run fairino3_v6_planner test_joint_states_service.py
```

---

## 配置参数

规划器支持通过配置文件调整参数，主要配置参数包含在 `src/fairino3_v6_planner/config/config.yaml` 文件中：

```yaml
/**:
  ros__parameters:
    # 是否开启调试模式
    debug: false
    # 最大轨迹点数量，用于限制发布的轨迹关键点数量
    max_points: 30
```

可以通过修改配置文件调整这些参数，或在启动节点时通过参数覆盖：

```bash
ros2 launch fairino3_v6_planner pose_goal_planner.launch.py max_points:=50
```

---

## 项目结构

```
arm_planner/
├─ src/
│  ├─ fairino_description/           # URDF/xacro
│  ├─ fairino3_v6_moveit2_config/    # MoveIt2 配置、launch
│  └─ fairino3_v6_planner/           # 规划节点、脚本、launch
└─ README.md                         # 本文档
```