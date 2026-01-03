#!/usr/bin/python3
"""
Fake Joint Driver for Virtual Wheeled Humanoid Robot (ros2_control 风格)
模拟完整的 ros2_control 控制栈：
  - 发布 /joint_states（类似 joint_state_broadcaster）
  - 提供 FollowJointTrajectory action servers（类似 JointTrajectoryController）

重要：控制器名称与真实机器人 (wheeled_humanoid_moveit) 完全一致，
确保在虚拟机器人上开发的功能可以直接部署到真实机器人。

控制器名称映射：
  - left_arm_controller   -> 左臂轨迹控制 (AR5_5_07L_joint_1-7)
  - right_arm_controller  -> 右臂轨迹控制 (AR5_5_07R_joint_1-7)
  - torso_controller      -> 躯干控制 (Trunk_Joint1-4)
  - head_controller       -> 头部控制 (Head_Joint1-2)
  - left_gripper_controller  -> 左夹爪控制 (Lfinger_joint)
  - right_gripper_controller -> 右夹爪控制 (Rfinger_joint)

数据流架构：
  MoveIt move_group
      ↓ FollowJointTrajectory action
  FakeJointDriver (本节点)
      ↓ 更新内部 joint_positions
  /joint_states topic
      ↓
  robot_state_publisher → TF
      ↓
  RViz 显示 + MoveIt PlanningSceneMonitor
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import time
import threading
import numpy as np


class FakeJointDriver(Node):
    """
    模拟 ros2_control 的虚拟关节驱动器。
    
    功能：
    1. 以 100Hz 发布 /joint_states（高频确保 MoveIt 能及时收到状态）
    2. 提供 6 个 FollowJointTrajectory action server（与真实机器人控制器名称一致）
    3. 平滑插值执行轨迹，模拟真实控制器行为
    """
    
    def __init__(self):
        super().__init__('fake_joint_driver')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # ============================================================
        # 定义所有关节（与 URDF 和 MoveIt 配置一致）
        # ============================================================
        self.all_joints = [
            # Virtual mobile base joints (controlled by move_robot node)
            'virtual_x_joint', 'virtual_y_joint', 'virtual_yaw_joint',
            # Left arm (7 DOF)
            'AR5_5_07L_joint_1', 'AR5_5_07L_joint_2', 'AR5_5_07L_joint_3',
            'AR5_5_07L_joint_4', 'AR5_5_07L_joint_5', 'AR5_5_07L_joint_6',
            'AR5_5_07L_joint_7',
            # Right arm (7 DOF)
            'AR5_5_07R_joint_1', 'AR5_5_07R_joint_2', 'AR5_5_07R_joint_3',
            'AR5_5_07R_joint_4', 'AR5_5_07R_joint_5', 'AR5_5_07R_joint_6',
            'AR5_5_07R_joint_7',
            # Torso (4 DOF)
            'Trunk_Joint1', 'Trunk_Joint2', 'Trunk_Joint3', 'Trunk_Joint4',
            # Head (2 DOF)
            'Head_Joint1', 'Head_Joint2',
            # Grippers (mimic joints may exist, but we control these)
            'Lfinger_joint', 'Rfinger_joint',
        ]
        
        # 虚拟底盘关节（由 move_robot 节点控制，这里只做状态合并）
        self.virtual_base_joints = ['virtual_x_joint', 'virtual_y_joint', 'virtual_yaw_joint']
        self.virtual_base_positions = {name: 0.0 for name in self.virtual_base_joints}
        
        # 当前关节状态
        self.joint_positions = {name: 0.0 for name in self.all_joints}
        self.joint_velocities = {name: 0.0 for name in self.all_joints}
        self.joint_efforts = {name: 0.0 for name in self.all_joints}
        
        # 线程安全锁
        self.lock = threading.Lock()
        
        # 正在执行的轨迹标记
        self.executing = {}
        
        # ============================================================
        # 订阅 move_robot 发布的虚拟底盘关节状态
        # ============================================================
        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
        
        # ============================================================
        # 关节状态发布器 - 使用可靠 QoS 确保 MoveIt 能收到
        # ============================================================
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.joint_state_pub = self.create_publisher(
            JointState, 
            'joint_states', 
            qos
        )
        
        # 高频发布关节状态（100Hz，确保 MoveIt 不会超时）
        self.publish_rate = 100.0  # Hz
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_joint_states)
        
        # 立即发布多次初始状态，确保 MoveIt 能收到
        for _ in range(5):
            self.publish_joint_states()
            time.sleep(0.01)
        
        # ============================================================
        # 创建轨迹 action servers
        # 控制器名称与真实机器人完全一致！
        # ============================================================
        self.action_servers = {}
        
        controller_names = [
            'left_arm_controller',
            'right_arm_controller', 
            'torso_controller',
            'head_controller',
            'left_gripper_controller',
            'right_gripper_controller',
        ]
        
        for controller_name in controller_names:
            action_name = f'{controller_name}/follow_joint_trajectory'
            self.action_servers[controller_name] = ActionServer(
                self,
                FollowJointTrajectory,
                action_name,
                execute_callback=self.execute_trajectory_callback,
                goal_callback=self.goal_callback,
                cancel_callback=self.cancel_callback,
                callback_group=self.callback_group
            )
            self.executing[controller_name] = False
        
        self.get_logger().info('='*60)
        self.get_logger().info('Fake Joint Driver Started (ros2_control style)')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Publishing /joint_states at {self.publish_rate} Hz')
        self.get_logger().info(f'Total joints: {len(self.all_joints)}')
        self.get_logger().info(f'Virtual base joints: {self.virtual_base_joints}')
        self.get_logger().info('Action servers (FollowJointTrajectory):')
        for name in controller_names:
            self.get_logger().info(f'  - /{name}/follow_joint_trajectory')
        self.get_logger().info('='*60)

    def _joint_state_callback(self, msg: JointState):
        """处理来自 move_robot 节点的虚拟底盘关节状态"""
        with self.lock:
            for i, name in enumerate(msg.name):
                if name in self.virtual_base_joints and i < len(msg.position):
                    self.joint_positions[name] = msg.position[i]

    def publish_joint_states(self):
        """发布当前关节状态（模拟 joint_state_broadcaster）"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        with self.lock:
            msg.name = list(self.joint_positions.keys())
            msg.position = list(self.joint_positions.values())
            msg.velocity = list(self.joint_velocities.values())
            msg.effort = list(self.joint_efforts.values())
        
        self.joint_state_pub.publish(msg)
    
    def goal_callback(self, goal_request):
        """接受所有轨迹目标"""
        self.get_logger().debug('Received trajectory goal request')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """允许取消轨迹执行"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_trajectory_callback(self, goal_handle):
        """
        执行轨迹回调（模拟 JointTrajectoryController）
        
        实现平滑的轨迹插值执行，模拟真实控制器行为。
        """
        self.get_logger().info('Executing trajectory...')
        
        trajectory = goal_handle.request.trajectory
        joint_names = trajectory.joint_names
        points = trajectory.points
        
        # 空轨迹直接成功
        if len(points) == 0:
            self.get_logger().warn('Empty trajectory received')
            goal_handle.succeed()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result
        
        self.get_logger().info(f'  Joints: {joint_names}')
        self.get_logger().info(f'  Points: {len(points)}')
        
        # 获取轨迹总时长
        total_duration = (
            points[-1].time_from_start.sec + 
            points[-1].time_from_start.nanosec * 1e-9
        )
        self.get_logger().info(f'  Duration: {total_duration:.2f}s')
        
        # 准备反馈消息
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = list(joint_names)
        
        # 获取当前位置作为起点
        with self.lock:
            start_positions = [self.joint_positions.get(j, 0.0) for j in joint_names]
        
        # 执行控制循环
        control_rate = 100.0  # Hz
        control_dt = 1.0 / control_rate
        start_time = time.time()
        point_idx = 0
        
        while point_idx < len(points):
            # 检查是否被取消
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Trajectory cancelled')
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result
            
            # 当前时间
            elapsed = time.time() - start_time
            
            # 找到当前应该插值的区间
            while (point_idx < len(points) - 1):
                next_time = (
                    points[point_idx + 1].time_from_start.sec + 
                    points[point_idx + 1].time_from_start.nanosec * 1e-9
                )
                if elapsed < next_time:
                    break
                point_idx += 1
            
            # 获取当前点和下一点
            current_point = points[point_idx]
            current_time = (
                current_point.time_from_start.sec + 
                current_point.time_from_start.nanosec * 1e-9
            )
            
            if point_idx < len(points) - 1:
                next_point = points[point_idx + 1]
                next_time = (
                    next_point.time_from_start.sec + 
                    next_point.time_from_start.nanosec * 1e-9
                )
                
                # 线性插值
                if next_time > current_time:
                    alpha = (elapsed - current_time) / (next_time - current_time)
                    alpha = max(0.0, min(1.0, alpha))
                else:
                    alpha = 1.0
                
                interpolated_positions = []
                interpolated_velocities = []
                
                for j in range(len(joint_names)):
                    p0 = current_point.positions[j]
                    p1 = next_point.positions[j]
                    interpolated_positions.append(p0 + alpha * (p1 - p0))
                    
                    if current_point.velocities and next_point.velocities:
                        v0 = current_point.velocities[j] if j < len(current_point.velocities) else 0.0
                        v1 = next_point.velocities[j] if j < len(next_point.velocities) else 0.0
                        interpolated_velocities.append(v0 + alpha * (v1 - v0))
                    else:
                        interpolated_velocities.append(0.0)
            else:
                # 最后一个点
                interpolated_positions = list(current_point.positions)
                interpolated_velocities = list(current_point.velocities) if current_point.velocities else [0.0] * len(joint_names)
            
            # 更新关节状态
            with self.lock:
                for j, joint_name in enumerate(joint_names):
                    if joint_name in self.joint_positions:
                        self.joint_positions[joint_name] = interpolated_positions[j]
                        self.joint_velocities[joint_name] = interpolated_velocities[j] if j < len(interpolated_velocities) else 0.0
            
            # 发送反馈
            feedback.actual.positions = interpolated_positions
            feedback.actual.velocities = interpolated_velocities
            feedback.desired.positions = interpolated_positions
            feedback.desired.velocities = interpolated_velocities
            feedback.error.positions = [0.0] * len(joint_names)
            feedback.error.velocities = [0.0] * len(joint_names)
            goal_handle.publish_feedback(feedback)
            
            # 检查是否完成
            if elapsed >= total_duration:
                break
            
            # 控制频率
            time.sleep(control_dt)
        
        # 确保最终位置精确
        final_point = points[-1]
        with self.lock:
            for j, joint_name in enumerate(joint_names):
                if joint_name in self.joint_positions:
                    self.joint_positions[joint_name] = final_point.positions[j]
                    self.joint_velocities[joint_name] = 0.0  # 停止时速度为0
        
        self.get_logger().info('Trajectory execution completed successfully')
        
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "Trajectory executed successfully"
        return result


def main(args=None):
    rclpy.init(args=args)
    
    node = FakeJointDriver()
    
    # 使用多线程执行器以支持并发 action 执行
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        node.get_logger().info('Spinning...')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
