# Arm Planner 项目

本项目基于 ROS 2（Humble）和 MoveIt 2 搭建机械臂运动规划系统，包含以下主要模块：

1. **fairino_description**：机器人模型描述包，包含 URDF/xacro 文件，用于定义机械臂的几何和物理属性。
2. **fairino3_v6_moveit2_config**：MoveIt 2 配置包，自动生成的配置（SRDF、关节限位、运动学、控制器、OMPL 规划器等），并包含 `move_group` 和 `rviz2` 的启动文件。
3. **fairino3_v6_planner**：自定义规划器节点和测试脚本：
   - C++ 实现 `PoseGoalPlanner` 节点，订阅目标位姿 (`target_pose`)，调用 MoveIt 2 接口进行路径规划，并发布规划轨迹。
   - `scripts/` 目录下包含三种测试脚本：
     - `send_pose_goal.py`：发布笛卡尔空间目标位姿（`target_pose`）。
     - `send_joint_goal.py`：发布关节空间目标 (`joint_states`) 消息，用于测试关节规划。
     - `test_start_and_goal.py`：综合测试，先发布初始关节状态，再发布目标位姿，完成一次端到端的规划流程。

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
ros2 launch fairino3_v6_planner pose_goal_planner.launch.py
```

- `use_sim_time` 参数默认为 `true`，可通过 `-p use_sim_time:=false` 切换至系统时钟。
- 启动后可在 RViz2 中查看机器人模型并使用 Motion Planning 面板进行交互式规划。

---

## 测试脚本使用方法

### 1. 发布笛卡尔空间位姿目标

```bash
# 直接运行脚本（确保已 source 环境）
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

## 项目结构

```
arm_planner/
├─ src/
│  ├─ fairino_description/           # URDF/xacro
│  ├─ fairino3_v6_moveit2_config/    # MoveIt2 配置、launch
│  └─ fairino3_v6_planner/           # 规划节点、脚本、launch
└─ README.md                         # 本文档
```

更多细节请参考各包的 `package.xml`、`CMakeLists.txt` 和源码注释。祝你使用愉快！