#!/usr/bin/env python3
"""
Test script for debugging attach/detach functionality.

Usage:
  1. Start MoveIt: ros2 launch wheeled_humanoid_moveit_fake demo.launch.py
  2. Run this script: python3 test_attach_detach.py

This script will:
  1. Add a test box to the scene
  2. Attach it to left arm
  3. Query and display MoveIt state
  4. Detach it
  5. Query and display MoveIt state again
"""

import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import (
    CollisionObject,
    AttachedCollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene


class AttachDetachTester(Node):
    def __init__(self):
        super().__init__('attach_detach_tester')
        
        self.apply_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.get_client = self.create_client(GetPlanningScene, '/get_planning_scene')
        
        self.get_logger().info("Waiting for services...")
        self.apply_client.wait_for_service(timeout_sec=10.0)
        self.get_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info("Services ready!")
        
        self.object_name = "test_box"
        self.link_name = "Lbase_link"
        self.touch_links = [
            "Lbase_link", "Lleft_pad", "Lright_pad",
            "Lleftinn_Link", "Lrightinn_Link",
            "Lleftout_Link", "Lrightout_Link",
            "left_flange_link", "AR5_5_07L_link7",
        ]

    def get_state(self) -> dict:
        """Get current planning scene state."""
        req = GetPlanningScene.Request()
        req.components.components = (
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS |
            PlanningSceneComponents.WORLD_OBJECT_NAMES
        )
        
        future = self.get_client.call_async(req)
        start = time.time()
        while not future.done() and (time.time() - start) < 5.0:
            time.sleep(0.05)
        
        if future.done() and future.result():
            scene = future.result().scene
            return {
                'attached': [(aco.object.id, aco.link_name) for aco in scene.robot_state.attached_collision_objects],
                'world': [co.id for co in scene.world.collision_objects]
            }
        return None

    def print_state(self, label: str):
        """Print current state."""
        state = self.get_state()
        print(f"\n{'='*50}")
        print(f"STATE: {label}")
        print(f"{'='*50}")
        if state:
            print(f"  Attached objects: {state['attached']}")
            print(f"  World objects: {state['world']}")
        else:
            print("  Failed to get state!")
        print(f"{'='*50}\n")

    def apply_scene(self, scene: PlanningScene) -> bool:
        """Apply a planning scene."""
        req = ApplyPlanningScene.Request()
        req.scene = scene
        
        future = self.apply_client.call_async(req)
        start = time.time()
        while not future.done() and (time.time() - start) < 5.0:
            time.sleep(0.1)
        
        if future.done() and future.result():
            return future.result().success
        return False

    def add_object(self):
        """Add test object to world."""
        print("\n>>> Adding object to world...")
        
        co = CollisionObject()
        co.header.frame_id = "world"
        co.id = self.object_name
        co.operation = CollisionObject.ADD
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.05, 0.05, 0.1]
        
        pose = Pose()
        pose.position.x = 0.4
        pose.position.y = 0.3
        pose.position.z = 0.5
        pose.orientation.w = 1.0
        
        co.primitives.append(box)
        co.primitive_poses.append(pose)
        
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(co)
        
        result = self.apply_scene(scene)
        print(f"    Result: {result}")
        return result

    def attach_object(self):
        """Attach object to robot."""
        print(f"\n>>> Attaching {self.object_name} to {self.link_name}...")
        
        # Create attached collision object
        attached = AttachedCollisionObject()
        attached.link_name = self.link_name
        attached.object.id = self.object_name
        attached.object.header.frame_id = "world"
        attached.object.operation = CollisionObject.ADD
        attached.touch_links = self.touch_links
        
        # Remove from world
        remove = CollisionObject()
        remove.id = self.object_name
        remove.header.frame_id = "world"
        remove.operation = CollisionObject.REMOVE
        
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)
        scene.world.collision_objects.append(remove)
        
        result = self.apply_scene(scene)
        print(f"    Result: {result}")
        return result

    def detach_object(self):
        """Detach object from robot."""
        print(f"\n>>> Detaching {self.object_name} from {self.link_name}...")
        
        attached = AttachedCollisionObject()
        attached.link_name = self.link_name
        attached.object.id = self.object_name
        attached.object.header.frame_id = "world"
        attached.object.operation = CollisionObject.REMOVE
        
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)
        
        result = self.apply_scene(scene)
        print(f"    Result: {result}")
        return result

    def detach_with_full_state(self):
        """Detach using full robot state (is_diff=False)."""
        print(f"\n>>> Detaching using full robot state (is_diff=False)...")
        
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = False
        scene.robot_state.attached_collision_objects = []  # Empty = no attached objects
        
        result = self.apply_scene(scene)
        print(f"    Result: {result}")
        return result

    def remove_from_world(self):
        """Remove object from world."""
        print(f"\n>>> Removing {self.object_name} from world...")
        
        co = CollisionObject()
        co.id = self.object_name
        co.header.frame_id = "world"
        co.operation = CollisionObject.REMOVE
        
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(co)
        
        result = self.apply_scene(scene)
        print(f"    Result: {result}")
        return result

    def run_test(self):
        """Run the full test sequence."""
        print("\n" + "="*60)
        print("ATTACH/DETACH TEST")
        print("="*60)
        
        # Initial state
        self.print_state("INITIAL")
        
        # Step 1: Add object
        self.add_object()
        time.sleep(0.5)
        self.print_state("AFTER ADD")
        
        # Step 2: Attach
        self.attach_object()
        time.sleep(0.5)
        self.print_state("AFTER ATTACH")
        
        input("\nPress Enter to detach...")
        
        # Step 3: Detach (method 1)
        self.detach_object()
        time.sleep(0.5)
        self.print_state("AFTER DETACH (method 1)")
        
        # Check if still attached
        state = self.get_state()
        if state and any(self.object_name in str(a) for a in state['attached']):
            print("!!! Object still attached, trying method 2...")
            
            # Step 3b: Detach (method 2)
            self.detach_with_full_state()
            time.sleep(0.5)
            self.print_state("AFTER DETACH (method 2)")
        
        # Step 4: Remove from world
        self.remove_from_world()
        time.sleep(0.5)
        self.print_state("AFTER REMOVE")
        
        print("\n" + "="*60)
        print("TEST COMPLETE")
        print("="*60)


def main():
    rclpy.init()
    node = AttachDetachTester()
    
    try:
        node.run_test()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
