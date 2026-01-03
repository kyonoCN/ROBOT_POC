#!/usr/bin/env python3
"""
附着抓取演示节点 (Attached Collision Object Demo)

功能：
  - 在场景中添加一个可抓取的物体
  - 演示机械臂移动到抓取位置
  - 将物体附着到夹爪（attach）
  - 移动机械臂到新位置（物体随之移动）
  - 分离物体（detach）

RViz2 中的表现：
  - 物体初始为绿色/灰色（场景障碍物）
  - attach 后变为紫色（附着到机器人）
  - 物体会随夹爪一起移动
  - detach 后恢复为绿色/灰色

使用方法：
  1. 先启动 demo.launch.py
  2. 运行此节点: ros2 run wheeled_humanoid_moveit_fake attach_object_demo.py
  3. 按提示按 Enter 键逐步执行
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import (
    CollisionObject,
    AttachedCollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from moveit_msgs.action import MoveGroup
from std_msgs.msg import Header

import time


class AttachObjectDemo(Node):
    """附着抓取演示节点"""

    def __init__(self):
        super().__init__('attach_object_demo')
        
        # 配置参数
        self.planning_frame = "world"
        self.arm_group = "left_arm"
        self.gripper_link = "Lbase_link"  # 夹爪基座 link
        
        # 允许与附着物体接触的 links（夹爪相关的所有 link）
        self.touch_links = [
            "Lbase_link",
            "Lleft_pad", "Lright_pad",
            "Lleftinn_Link", "Lrightinn_Link",
            "Lleftout_Link", "Lrightout_Link",
            "Lleft_kckle", "Lright_kckle",
            "left_flange_link",
            "AR5_5_07L_link7",
            "AR5_5_07L_tcp",
        ]
        
        # 创建 Planning Scene 发布者
        self.planning_scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )
        
        # 创建服务客户端
        self.apply_planning_scene_client = self.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene'
        )
        
        self.get_planning_scene_client = self.create_client(
            GetPlanningScene,
            '/get_planning_scene'
        )
        
        # 等待服务可用
        self.get_logger().info("等待 MoveIt 服务...")
        self.apply_planning_scene_client.wait_for_service(timeout_sec=30.0)
        self.get_planning_scene_client.wait_for_service(timeout_sec=30.0)
        self.get_logger().info("MoveIt 服务已就绪!")
        
        # 物体名称
        self.object_name = "grasp_target"

    def create_box_collision_object(self, name: str, pose: Pose, 
                                     size: tuple = (0.05, 0.05, 0.1)) -> CollisionObject:
        """创建一个长方体碰撞对象"""
        collision_object = CollisionObject()
        collision_object.header.frame_id = self.planning_frame
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = name
        
        # 定义形状
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(size)  # [x, y, z]
        
        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD
        
        return collision_object

    def add_object_to_scene(self, pose: Pose, size: tuple = (0.05, 0.05, 0.1)):
        """添加物体到场景中作为障碍物"""
        self.get_logger().info(f"添加物体 '{self.object_name}' 到场景...")
        
        collision_object = self.create_box_collision_object(
            self.object_name, pose, size
        )
        
        # 构造 PlanningScene 消息
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.world.collision_objects.append(collision_object)
        
        # 应用到场景
        self._apply_planning_scene(planning_scene)
        self.get_logger().info(f"物体 '{self.object_name}' 已添加到场景 (绿色/灰色)")

    def attach_object(self, link: str = None):
        """将物体附着到机器人 link 上"""
        if link is None:
            link = self.gripper_link
            
        self.get_logger().info(f"将物体 '{self.object_name}' 附着到 '{link}'...")
        
        # 创建 AttachedCollisionObject
        attached_object = AttachedCollisionObject()
        attached_object.link_name = link
        attached_object.object.id = self.object_name
        attached_object.object.header.frame_id = self.planning_frame
        attached_object.object.header.stamp = self.get_clock().now().to_msg()
        attached_object.object.operation = CollisionObject.ADD
        
        # 设置允许接触的 links
        attached_object.touch_links = self.touch_links
        
        # 构造 PlanningScene 消息
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.robot_state.attached_collision_objects.append(attached_object)
        # 同时从世界中移除该物体
        remove_object = CollisionObject()
        remove_object.id = self.object_name
        remove_object.header.frame_id = self.planning_frame
        remove_object.operation = CollisionObject.REMOVE
        planning_scene.world.collision_objects.append(remove_object)
        
        # 应用到场景
        self._apply_planning_scene(planning_scene)
        self.get_logger().info(f"物体已附着! (在 RViz 中显示为紫色)")

    def detach_object(self, link: str = None):
        """从机器人 link 上分离物体"""
        if link is None:
            link = self.gripper_link
            
        self.get_logger().info(f"从 '{link}' 分离物体 '{self.object_name}'...")
        
        # 创建分离请求
        attached_object = AttachedCollisionObject()
        attached_object.link_name = link
        attached_object.object.id = self.object_name
        attached_object.object.operation = CollisionObject.REMOVE
        
        # 构造 PlanningScene 消息
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.robot_state.attached_collision_objects.append(attached_object)
        
        # 应用到场景
        self._apply_planning_scene(planning_scene)
        self.get_logger().info(f"物体已分离! (恢复为场景障碍物)")

    def remove_object_from_scene(self):
        """从场景中完全移除物体"""
        self.get_logger().info(f"从场景中移除物体 '{self.object_name}'...")
        
        collision_object = CollisionObject()
        collision_object.id = self.object_name
        collision_object.header.frame_id = self.planning_frame
        collision_object.operation = CollisionObject.REMOVE
        
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.world.collision_objects.append(collision_object)
        
        self._apply_planning_scene(planning_scene)
        self.get_logger().info(f"物体已移除")

    def _apply_planning_scene(self, planning_scene: PlanningScene):
        """应用 PlanningScene 更新"""
        request = ApplyPlanningScene.Request()
        request.scene = planning_scene
        
        future = self.apply_planning_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            if future.result().success:
                self.get_logger().debug("PlanningScene 更新成功")
            else:
                self.get_logger().error("PlanningScene 更新失败")
        else:
            self.get_logger().error("服务调用超时")
        
        # 等待场景更新
        time.sleep(0.5)

    def add_static_obstacles(self):
        """添加一些静态障碍物用于演示避障"""
        self.get_logger().info("添加静态障碍物...")
        
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        
        # 障碍物1: 桌子
        table_pose = Pose()
        table_pose.position.x = 0.5
        table_pose.position.y = 0.0
        table_pose.position.z = 0.4
        table_pose.orientation.w = 1.0
        
        table = self.create_box_collision_object(
            "table", table_pose, (0.6, 0.8, 0.02)
        )
        planning_scene.world.collision_objects.append(table)
        
        # 障碍物2: 侧面墙
        wall_pose = Pose()
        wall_pose.position.x = 0.5
        wall_pose.position.y = 0.5
        wall_pose.position.z = 0.6
        wall_pose.orientation.w = 1.0
        
        wall = self.create_box_collision_object(
            "wall", wall_pose, (0.4, 0.02, 0.4)
        )
        planning_scene.world.collision_objects.append(wall)
        
        self._apply_planning_scene(planning_scene)
        self.get_logger().info("静态障碍物已添加")


def main():
    rclpy.init()
    
    demo = AttachObjectDemo()
    
    print("\n" + "="*60)
    print("  附着抓取演示 (Attached Collision Object Demo)")
    print("="*60)
    print("\n请在 RViz2 中观察变化:")
    print("  - 绿色/灰色: 场景中的障碍物")
    print("  - 紫色: 附着到机器人的物体")
    print("\n确保 RViz2 中已启用:")
    print("  - MotionPlanning 插件")
    print("  - Scene Robot > Show Robot Collision")
    print("="*60 + "\n")
    
    try:
        # Step 1: 添加静态障碍物
        input("按 Enter 添加静态障碍物 (桌子和墙)...")
        demo.add_static_obstacles()
        
        # Step 2: 添加待抓取物体
        input("\n按 Enter 添加待抓取物体...")
        object_pose = Pose()
        object_pose.position.x = 0.4
        object_pose.position.y = 0.2
        object_pose.position.z = 0.45  # 桌面上方
        object_pose.orientation.w = 1.0
        demo.add_object_to_scene(object_pose, size=(0.04, 0.04, 0.08))
        print("物体已添加，在 RViz 中应显示为绿色/灰色")
        
        # Step 3: 附着物体
        input("\n按 Enter 将物体附着到夹爪 (模拟抓取)...")
        demo.attach_object()
        print("物体已附着，在 RViz 中应显示为紫色")
        print("现在可以在 RViz 中拖动机械臂，物体会跟随移动!")
        print("规划时会自动考虑附着物体的碰撞!")
        
        # Step 4: 等待用户操作
        input("\n按 Enter 分离物体 (模拟放置)...")
        demo.detach_object()
        print("物体已分离，恢复为场景障碍物")
        
        # Step 5: 清理
        input("\n按 Enter 清理场景并退出...")
        demo.remove_object_from_scene()
        
        print("\n演示完成!")
        
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
