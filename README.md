# Arm Planner 项目

本项目基于 ROS 2（Humble）和 MoveIt 2 搭建机械臂运动规划系统，包含以下主要模块：

1. **fairino_description**：机器人模型描述包，包含 URDF/xacro 文件，用于定义机械臂的几何和物理属性。此包原本包含多个机械臂描述，由于本项目仅需要规划一种机械臂故仅保留 `fairino3_v6` 描述文件。
2. **fairino3_v6_moveit2_config**：MoveIt 2 配置包，使用 `moveit_setup_assistant` 自动生成的配置（SRDF、关节限位、运动学、控制器、OMPL 规划器等），并包含 `move_group` 和 `rviz2` 的启动文件。
3. **fairino3_v6_planner**：自定义规划器节点和测试脚本：
   - C++ 实现 `PoseGoalPlanner` 节点，订阅目标位姿 (`target_pose`)，调用 MoveIt 2 接口进行路径规划，并发布规划轨迹。
   - `scripts/` 目录下包含三种测试脚本：
     - `send_pose_goal.py`：发布笛卡尔空间目标位姿（`target_pose`）。
     - `send_joint_goal.py`：发布关节空间目标 (`joint_states`) 消息，用于测试关节规划。
     - `test_start_and_goal.py`：综合测试，先发布初始关节状态，再发布目标位姿，完成一次端到端的规划流程。
   - `config/` 目录下包含可配置参数：
     - `config.yaml`：包含规划器配置参数，如最大轨迹点数量(max_points)等。

---

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
- `with_rviz` 参数默认为 `false`, 可通过 `with_rviz:=true` 关闭可视化界面
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

---

## 外部话题接口

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

要查看并调试这些话题，可以使用：
```bash
ros2 topic echo /planning_success
ros2 topic echo /planning_status
ros2 topic echo /trajectory_poses
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