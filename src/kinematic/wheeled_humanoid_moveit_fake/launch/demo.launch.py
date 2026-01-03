"""
虚拟机器人 MoveIt2 Demo Launch 文件（ros2_control 风格）
无需真实机器人即可进行 Plan 和 Execute

============================================================
架构说明（方法二：模拟 ros2_control 流）
============================================================

数据流：

  ┌─────────────────────────────────────────────────────────┐
  │                    RViz2 (可视化)                        │
  │  - 显示机器人模型（通过 TF）                              │
  │  - MoveIt 插件（规划、执行界面）                          │
  └─────────────────────────────────────────────────────────┘
                           ↑
                           │ TF + /robot_description
                           │
  ┌─────────────────────────────────────────────────────────┐
  │              robot_state_publisher                       │
  │  - 订阅 /joint_states                                    │
  │  - 发布 TF 变换                                          │
  └─────────────────────────────────────────────────────────┘
                           ↑
                           │ /joint_states
                           │
  ┌─────────────────────────────────────────────────────────┐
  │         fake_joint_driver (模拟 ros2_control)            │
  │  - 发布 /joint_states（类似 joint_state_broadcaster）    │
  │  - 提供 FollowJointTrajectory action servers            │
  │    （类似 JointTrajectoryController）                    │
  └─────────────────────────────────────────────────────────┘
                           ↑
                           │ FollowJointTrajectory action
                           │
  ┌─────────────────────────────────────────────────────────┐
  │                 move_group (MoveIt)                      │
  │  - 接收规划请求                                          │
  │  - OMPL 规划                                             │
  │  - 通过 MoveItSimpleControllerManager 发送轨迹           │
  └─────────────────────────────────────────────────────────┘

启动顺序（事件驱动）：
  1. static_tf_publisher (world -> base_link)
  2. fake_joint_driver (开始发布 /joint_states)
  3. [等待 /joint_states 可用]
  4. robot_state_publisher (订阅 /joint_states, 发布 TF)
  5. [等待延迟确保状态稳定]
  6. move_group (订阅 /joint_states 用于状态验证)
  7. rviz2 (可视化)

使用方法:
  ros2 launch wheeled_humanoid_moveit_fake demo.launch.py

可选参数:
  use_rviz:=true/false  - 是否启动 RViz (默认 true)
  use_gui:=true/false   - 是否启动关节滑块GUI (默认 false)
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    RegisterEventHandler,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # ============================================================
    # 参数声明
    # ============================================================
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="是否启动 RViz"
    )
    
    use_gui_arg = DeclareLaunchArgument(
        "use_gui",
        default_value="false",
        description="是否启动 joint_state_publisher_gui（用于手动拖动关节）"
    )

    # ============================================================
    # 获取包路径
    # ============================================================
    fake_moveit_share = get_package_share_directory("wheeled_humanoid_moveit_fake")
    robot_model_share = get_package_share_directory("robot_model")
    
    # URDF 文件路径
    urdf_file = os.path.join(robot_model_share, "urdf", "wheel_robot.urdf")
    
    # 读取 URDF 内容
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    # ============================================================
    # 构建 MoveIt 配置
    # ============================================================
    moveit_config = (
        MoveItConfigsBuilder("robot_model", package_name="wheeled_humanoid_moveit_fake")
        .robot_description(file_path=urdf_file)
        .robot_description_semantic(file_path="config/robot_model.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(
            file_path=os.path.join(fake_moveit_share, "config", "moveit_controllers.yaml")
        )
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    rviz_config = os.path.join(fake_moveit_share, "config", "moveit.rviz")

    # ============================================================
    # 1. Static TF: world -> base_link
    # ============================================================
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # ============================================================
    # 2. Robot Description Publisher - 在 /robot_description 上发布 URDF
    #    作用：
    #      - 让单独启动的 rviz2 也可以通过订阅 /robot_description 获取模型
    #      - 与 move_group 等使用的 robot_description 内容保持一致
    # ============================================================
    robot_description_publisher = Node(
        package="wheeled_humanoid_moveit_fake",
        executable="robot_description_publisher.py",
        name="robot_description_publisher",
        output="screen",
        parameters=[
            {"urdf_path": urdf_file},
        ],
    )

    # ============================================================
    # 3. Fake Joint Driver - 模拟 ros2_control 控制栈
    #    - 发布 /joint_states（100Hz）
    #    - 提供 FollowJointTrajectory action servers
    # ============================================================
    fake_joint_driver = Node(
        package="wheeled_humanoid_moveit_fake",
        executable="fake_joint_driver.py",
        name="fake_joint_driver",
        output="screen",
    )

    # ============================================================
    # 4. Robot State Publisher - 发布 TF
    #    延迟 0.5 秒启动，确保 /joint_states 已经稳定
    # ============================================================
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {"publish_frequency": 100.0},  # 匹配 fake_joint_driver 频率
        ],
    )
    
    delayed_robot_state_publisher = TimerAction(
        period=0.5,
        actions=[robot_state_publisher],
    )

    # ============================================================
    # 5. MoveIt move_group Node - 核心规划节点
    #    延迟 2 秒启动，确保 /joint_states 已经稳定发布
    # ============================================================
    move_group_params = moveit_config.to_dict()
    
    move_group_params.update({
        "use_sim_time": False,
        "publish_robot_description_semantic": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        # 轨迹执行配置（宽松设置，适合仿真）
        "trajectory_execution.allowed_execution_duration_scaling": 2.0,
        "trajectory_execution.allowed_goal_duration_margin": 2.0,
        "trajectory_execution.allowed_start_tolerance": 0.1,
        "trajectory_execution.execution_duration_monitoring": False,
        # 关键：增加等待当前状态的超时时间（默认1秒太短）
        "trajectory_execution.wait_for_trajectory_completion": True,
        # 规划场景监控配置
        "planning_scene_monitor.publish_planning_scene": True,
        "planning_scene_monitor.publish_geometry_updates": True,
        "planning_scene_monitor.publish_state_updates": True,
        # 增加状态更新等待超时（解决 "couldn't receive full current joint state" 问题）
        "planning_scene_monitor.wait_for_initial_state_timeout": 10.0,
        # 当前状态监控器配置 - 关键参数
        # 增加等待关节状态的超时时间（默认1秒）
        "current_state_monitor.joint_state_qos_overrides./joint_states.subscription.reliability": "best_effort",
    })

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[move_group_params],
    )
    
    # 延迟启动 move_group，确保 joint_states 已稳定
    delayed_move_group = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg="Starting move_group (joint_states should be stable now)..."),
            move_group_node,
        ],
    )

    # ============================================================
    # 6. RViz2 - 可视化界面
    #    延迟 3 秒启动，确保 move_group 已就绪
    # ============================================================
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    
    delayed_rviz = TimerAction(
        period=3.0,
        actions=[rviz_node],
    )

    # ============================================================
    # 7. Joint State Publisher GUI（可选）
    # ============================================================
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_gui")),
    )

    # ============================================================
    # 返回 Launch Description
    # ============================================================
    return LaunchDescription([
        # 参数
        use_rviz_arg,
        use_gui_arg,
        
        # 启动信息
        LogInfo(msg="="*60),
        LogInfo(msg="Wheeled Humanoid MoveIt Fake Demo (ros2_control style)"),
        LogInfo(msg="="*60),
        LogInfo(msg="Architecture:"),
        LogInfo(msg="  fake_joint_driver -> /joint_states -> robot_state_publisher -> TF"),
        LogInfo(msg="  move_group -> FollowJointTrajectory -> fake_joint_driver"),
        LogInfo(msg="="*60),
        
        # 节点（按启动顺序，使用延迟确保依赖稳定）
        static_tf_node,
        robot_description_publisher,
        fake_joint_driver,
        delayed_robot_state_publisher,  # 延迟 0.5s
        delayed_move_group,             # 延迟 2.0s
        delayed_rviz,                   # 延迟 3.0s
        joint_state_publisher_gui,
    ])
