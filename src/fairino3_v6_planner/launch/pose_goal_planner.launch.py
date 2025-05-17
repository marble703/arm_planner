from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
import yaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # 是否使用RViz
    with_rviz = LaunchConfiguration('with_rviz', default='false')

    # 获取各项参数
    fairino3_v6_moveit2_config_dir = get_package_share_directory('fairino3_v6_moveit2_config')
    fairino_description_dir = get_package_share_directory('fairino_description')
    fairino3_v6_planner_dir = get_package_share_directory('fairino3_v6_planner')
    
    # 加载机器人描述文件
    robot_urdf_path = os.path.join(fairino_description_dir, 'urdf', 'fairino3_v6.urdf')
    
    # 读取URDF文件内容
    with open(robot_urdf_path, 'r') as file:
        robot_description_config = file.read()
    
    # 设置机器人描述参数
    robot_description = {'robot_description': robot_description_config}
    
    # 加载SRDF文件
    srdf_file = os.path.join(fairino3_v6_moveit2_config_dir, 'config', 'fairino3_v6_robot.srdf')
    with open(srdf_file, 'r') as file:
        semantic_description_config = file.read()
    
    # 设置语义描述参数
    robot_description_semantic = {'robot_description_semantic': semantic_description_config}
    
    # 加载运动学配置
    kinematics_yaml = os.path.join(fairino3_v6_moveit2_config_dir, 'config', 'kinematics.yaml')
    with open(kinematics_yaml, 'r') as file:
        kinematics_config = yaml.safe_load(file)
        
    # 加载关节限制配置
    joint_limits_yaml = os.path.join(fairino3_v6_moveit2_config_dir, 'config', 'joint_limits.yaml')
    with open(joint_limits_yaml, 'r') as file:
        joint_limits = yaml.safe_load(file)
        
    # 加载OMPL规划器配置
    ompl_yaml = os.path.join(fairino3_v6_moveit2_config_dir, 'config', 'ompl_planning.yaml')
    with open(ompl_yaml, 'r') as file:
        ompl_config = yaml.safe_load(file)
    
    # 加载规划器自定义配置
    planner_yaml = os.path.join(fairino3_v6_planner_dir, 'config', 'config.yaml')
    with open(planner_yaml, 'r') as file:
        planner_yaml_content = yaml.safe_load(file)
        
    # 尝试从嵌套结构中提取参数，如果失败则直接使用顶层参数
    planner_config = {}
    
    # 先检查ROS 2标准格式 (/**:ros__parameters)
    if planner_yaml_content and '/**' in planner_yaml_content and 'ros__parameters' in planner_yaml_content['/**']:
        planner_config = planner_yaml_content['/**']['ros__parameters']
    # 如果上面的格式不存在，尝试直接从顶层读取
    elif planner_yaml_content:
        for key in ['debug', 'max_points']:
            if key in planner_yaml_content:
                planner_config[key] = planner_yaml_content[key]
    
    # 规划器配置
    planning_pipelines_config = {
        'default_planning_pipeline': 'ompl',
        'planning_pipelines': {
            'ompl': {
                'planning_plugin': 'ompl_interface/OMPLPlanner',
                'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
                'start_state_max_bounds_error': 0.1
            }
        },
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1
        }
    }
    
    # 加载控制器配置
    controllers_yaml = os.path.join(fairino3_v6_moveit2_config_dir, 'config', 'moveit_controllers.yaml')
    with open(controllers_yaml, 'r') as file:
        controllers_config = yaml.safe_load(file)
    
    moveit_controllers = {
        'moveit_simple_controller_manager': controllers_config['moveit_simple_controller_manager'],
        'moveit_controller_manager': controllers_config['moveit_controller_manager']
    }
    
    # 启动robot_state_publisher节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description,
                    {'use_sim_time': use_sim_time}]
    )
    
    # 配置和启动move_group节点
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            joint_limits,
            ompl_config,
            planning_pipelines_config,
            moveit_controllers,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # 包含RViz启动文件，使用RobotModel显示
    rviz_config_file = os.path.join(fairino3_v6_moveit2_config_dir, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(with_rviz),
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # 启动位姿目标规划器节点
    # 构建基本参数字典
    planner_params = {
        'use_sim_time': use_sim_time,
        'planning_group': 'fairino3_v6_group',
        'end_effector_link': 'wrist3_link',
    }
    
    # 添加从配置文件中读取的参数
    for key, value in planner_config.items():
        planner_params[key] = value
        
    pose_goal_planner_node = Node(
        package='fairino3_v6_planner',
        executable='pose_goal_planner',
        name='pose_goal_planner',
        output='screen',
        parameters=[
            planner_params,  # 合并后的参数
            robot_description,
            robot_description_semantic,
            kinematics_config,
            joint_limits,
            ompl_config,
            planning_pipelines_config,
            moveit_controllers
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'with_rviz', 
            default_value='false', 
            description='是否启动RViz'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='使用仿真时间,如果为false则使用系统时间'),

        # 添加robot_state_publisher
        robot_state_publisher_node,
        
        # 加载move_group
        move_group_node,
        
        # 加载RViz
        rviz_node,
        
        # 启动位姿规划器
        pose_goal_planner_node
    ])
