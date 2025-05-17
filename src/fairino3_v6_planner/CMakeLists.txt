cmake_minimum_required(VERSION 3.8)
project(fairino3_v6_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS on)
set(CMAKE_CXX_STANDARD 20)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

# 包含目录
include_directories(include)

# 生成自定义服务
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetTrajectoryPoses.srv"
  "srv/GetJointStates.srv"
  DEPENDENCIES geometry_msgs sensor_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 添加可执行文件
add_executable(pose_goal_planner src/pose_goal_planner.cpp)
ament_target_dependencies(pose_goal_planner
  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  tf2_geometry_msgs
  tf2_eigen
  moveit_core
  sensor_msgs
)
target_link_libraries(pose_goal_planner "${cpp_typesupport_target}")

# 安装可执行文件
install(TARGETS
  pose_goal_planner
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装Python脚本
install(PROGRAMS
  scripts/pose_publisher.py
  scripts/test_joint_states_service.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    config
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
