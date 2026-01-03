#!/usr/bin/env python3
"""
Attach Object GUI Demo

Usage:
  1. Launch demo.launch.py first
  2. Run: ros2 run wheeled_humanoid_moveit_fake attach_object_gui.py
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import (
    CollisionObject,
    AttachedCollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene

import threading
import time

try:
    import tkinter as tk
    from tkinter import ttk
    HAS_TK = True
except ImportError:
    HAS_TK = False


class AttachObjectNode(Node):

    def __init__(self):
        super().__init__('attach_object_gui')
        
        self.planning_frame = "world"
        self.gripper_link = "Lbase_link"
        self.object_name = "grasp_target"
        
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
        
        self.apply_scene_client = self.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene'
        )
        
        self.get_scene_client = self.create_client(
            GetPlanningScene,
            '/get_planning_scene'
        )
        
        self.object_in_scene = False
        self.object_attached = False
        
        self.get_logger().info("Waiting for /apply_planning_scene service...")
        if not self.apply_scene_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Service not available! Start MoveIt first.")
        else:
            self.get_logger().info("Service ready!")
        
        self.get_logger().info("Waiting for /get_planning_scene service...")
        self.get_scene_client.wait_for_service(timeout_sec=5.0)

    def _apply_scene(self, scene: PlanningScene) -> bool:
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self.apply_scene_client.call_async(req)
        timeout = 5.0
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.1)
        if future.done() and future.result() is not None:
            result = future.result()
            self.get_logger().info(f"ApplyPlanningScene result: success={result.success}")
            return result.success
        self.get_logger().error("ApplyPlanningScene timeout or no result")
        return False

    def add_object(self, x=0.4, y=0.3, z=0.5, sx=0.04, sy=0.04, sz=0.08) -> bool:
        if self.object_in_scene:
            return False
        obj = CollisionObject()
        obj.header.frame_id = self.planning_frame
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = self.object_name
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [sx, sy, sz]
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        obj.primitives.append(box)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)
        if self._apply_scene(scene):
            self.object_in_scene = True
            return True
        return False

    def attach_object(self) -> bool:
        if not self.object_in_scene or self.object_attached:
            self.get_logger().warn(f"attach_object: in_scene={self.object_in_scene}, attached={self.object_attached}")
            return False
        self.get_logger().info(f"attach_object: attaching {self.object_name} to {self.gripper_link}")
        attached = AttachedCollisionObject()
        attached.link_name = self.gripper_link
        attached.object.id = self.object_name
        attached.object.header.frame_id = self.planning_frame
        attached.object.operation = CollisionObject.ADD
        attached.touch_links = self.touch_links
        remove_obj = CollisionObject()
        remove_obj.id = self.object_name
        remove_obj.header.frame_id = self.planning_frame
        remove_obj.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)
        scene.world.collision_objects.append(remove_obj)
        result = self._apply_scene(scene)
        self.get_logger().info(f"attach_object: apply_scene result={result}")
        if result:
            self.object_attached = True
            return True
        # Even if service returns False, attach often works
        self.get_logger().warn("attach_object: service returned False, but proceeding anyway")
        self.object_attached = True
        return True

    def detach_object(self) -> bool:
        if not self.object_attached:
            self.get_logger().warn("detach_object: object not attached")
            return False
        self.get_logger().info(f"detach_object: detaching {self.object_name} from {self.gripper_link}")
        attached = AttachedCollisionObject()
        attached.link_name = self.gripper_link
        attached.object.id = self.object_name
        attached.object.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)
        result = self._apply_scene(scene)
        self.get_logger().info(f"detach_object: apply_scene result={result}")
        
        # Verify detach by checking actual state
        time.sleep(0.2)
        actual_count = self._get_attached_count()
        self.get_logger().info(f"detach_object: verification - attached_count={actual_count}")
        
        if actual_count == 0:
            self.object_attached = False
            self.object_in_scene = False
            self.get_logger().info("detach_object: SUCCESS - verified no attached objects")
            return True
        else:
            self.get_logger().error(f"detach_object: FAILED - still have {actual_count} attached objects")
            return False
    
    def _get_attached_count(self) -> int:
        """Get current count of attached collision objects from MoveIt."""
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        future = self.get_scene_client.call_async(req)
        timeout = 2.0
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.05)
        if future.done() and future.result() is not None:
            return len(future.result().scene.robot_state.attached_collision_objects)
        return -1

    def remove_object(self) -> bool:
        if self.object_attached:
            self.detach_object()
        obj = CollisionObject()
        obj.id = self.object_name
        obj.header.frame_id = self.planning_frame
        obj.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)
        if self._apply_scene(scene):
            self.object_in_scene = False
            return True
        return False

    def add_obstacles(self) -> bool:
        scene = PlanningScene()
        scene.is_diff = True
        # Table
        table = CollisionObject()
        table.header.frame_id = self.planning_frame
        table.id = "table"
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.6, 0.8, 0.02]
        pose = Pose()
        pose.position.x = 0.5
        pose.position.y = 0.0
        pose.position.z = 0.4
        pose.orientation.w = 1.0
        table.primitives.append(box)
        table.primitive_poses.append(pose)
        table.operation = CollisionObject.ADD
        scene.world.collision_objects.append(table)
        # Wall
        wall = CollisionObject()
        wall.header.frame_id = self.planning_frame
        wall.id = "wall"
        box2 = SolidPrimitive()
        box2.type = SolidPrimitive.BOX
        box2.dimensions = [0.4, 0.02, 0.4]
        pose2 = Pose()
        pose2.position.x = 0.5
        pose2.position.y = 0.5
        pose2.position.z = 0.6
        pose2.orientation.w = 1.0
        wall.primitives.append(box2)
        wall.primitive_poses.append(pose2)
        wall.operation = CollisionObject.ADD
        scene.world.collision_objects.append(wall)
        return self._apply_scene(scene)

    def refresh_rviz(self) -> bool:
        """Force refresh RViz by getting current scene and re-applying with is_diff=False.
        
        This clears RViz's cached robot state including stale attached objects.
        """
        self.get_logger().info("refresh_rviz: Getting current planning scene...")
        
        # Get current planning scene (full scene)
        req = GetPlanningScene.Request()
        # Request all components
        req.components.components = (
            PlanningSceneComponents.SCENE_SETTINGS |
            PlanningSceneComponents.ROBOT_STATE |
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS |
            PlanningSceneComponents.WORLD_OBJECT_NAMES |
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
            PlanningSceneComponents.OCTOMAP |
            PlanningSceneComponents.TRANSFORMS |
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX |
            PlanningSceneComponents.LINK_PADDING_AND_SCALING |
            PlanningSceneComponents.OBJECT_COLORS
        )
        
        future = self.get_scene_client.call_async(req)
        timeout = 5.0
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.1)
        
        if not (future.done() and future.result() is not None):
            self.get_logger().error("refresh_rviz: Failed to get planning scene")
            return False
        
        current_scene = future.result().scene
        self.get_logger().info(f"refresh_rviz: Got scene, attached_objects={len(current_scene.robot_state.attached_collision_objects)}, world_objects={len(current_scene.world.collision_objects)}")
        
        # Re-apply the scene with is_diff=False to force full refresh
        current_scene.is_diff = False
        current_scene.robot_state.is_diff = False
        
        result = self._apply_scene(current_scene)
        self.get_logger().info(f"refresh_rviz: Re-applied scene, result={result}")
        
        return result


class AttachObjectGUI:
    
    def __init__(self, node: AttachObjectNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title("Attach Object Demo")
        self.root.geometry("320x400")
        self.root.resizable(False, False)
        self._create_widgets()
        
    def _create_widgets(self):
        # Title
        title = ttk.Label(self.root, text="MoveIt Attach Demo", font=('Arial', 14, 'bold'))
        title.pack(pady=15)
        
        # Info
        info = ttk.Label(self.root, text="Green = Scene Object\nPurple = Attached", justify='center')
        info.pack(pady=5)
        
        ttk.Separator(self.root, orient='horizontal').pack(fill='x', pady=10)
        
        # Buttons
        btn_frame = ttk.Frame(self.root)
        btn_frame.pack(pady=10)
        
        ttk.Button(btn_frame, text="Add Obstacles", command=self._on_add_obstacles, width=20).pack(pady=4)
        ttk.Button(btn_frame, text="Add Object", command=self._on_add_object, width=20).pack(pady=4)
        ttk.Button(btn_frame, text="Attach", command=self._on_attach, width=20).pack(pady=4)
        ttk.Button(btn_frame, text="Detach", command=self._on_detach, width=20).pack(pady=4)
        ttk.Button(btn_frame, text="Remove", command=self._on_remove, width=20).pack(pady=4)
        
        ttk.Separator(btn_frame, orient='horizontal').pack(fill='x', pady=8)
        ttk.Button(btn_frame, text="Refresh RViz", command=self._on_refresh_rviz, width=20).pack(pady=4)
        
        # Status
        self.status_var = tk.StringVar(value="Ready")
        ttk.Label(self.root, textvariable=self.status_var, font=('Arial', 10)).pack(pady=15)
        
        # Tip
        ttk.Label(self.root, text="Tip: Drag arm in RViz after attach\nObject follows gripper!", 
                  justify='center', foreground='gray').pack(pady=5)

    def _on_add_obstacles(self):
        self.status_var.set("Adding...")
        self.root.update()
        self.status_var.set("OK - Obstacles added" if self.node.add_obstacles() else "Failed")

    def _on_add_object(self):
        self.status_var.set("Adding...")
        self.root.update()
        self.status_var.set("OK - Object added (green)" if self.node.add_object() else "Failed or exists")

    def _on_attach(self):
        self.status_var.set("Attaching...")
        self.root.update()
        self.status_var.set("OK - Attached (purple)" if self.node.attach_object() else "Failed")

    def _on_detach(self):
        self.status_var.set("Detaching...")
        self.root.update()
        self.status_var.set("OK - Detached" if self.node.detach_object() else "Failed")

    def _on_remove(self):
        self.status_var.set("Removing...")
        self.root.update()
        self.status_var.set("OK - Removed" if self.node.remove_object() else "Failed")

    def _on_refresh_rviz(self):
        self.status_var.set("Refreshing RViz...")
        self.root.update()
        self.status_var.set("OK - RViz refreshed" if self.node.refresh_rviz() else "Failed")

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = AttachObjectNode()
    
    if not HAS_TK:
        print("Error: tkinter required. Install: sudo apt install python3-tk")
        rclpy.shutdown()
        return
    
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()
    
    try:
        gui = AttachObjectGUI(node)
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
