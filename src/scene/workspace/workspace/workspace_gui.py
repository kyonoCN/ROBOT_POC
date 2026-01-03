#!/usr/bin/env python3
"""
Workspace GUI - Enhanced obstacle management for MoveIt planning scene.
Supports both box obstacles and mesh obstacles.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading
import json
import numpy as np
import os
from typing import Dict, List, Optional, Union

from geometry_msgs.msg import Pose, Point, Vector3
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
from scipy.spatial.transform import Rotation as R
from ament_index_python.packages import get_package_share_directory
import time as time_module

# Mesh directory - dynamically resolved
def get_mesh_dir():
    """Get mesh directory path. Uses package share directory to find workspace root."""
    try:
        # Get package share directory: install/workspace/share/workspace
        pkg_share = get_package_share_directory('workspace')
        # Go up to workspace root: install/workspace/share/workspace -> workspace_root
        ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
        mesh_dir = os.path.join(ws_root, 'src', 'scene', 'workspace', 'mesh')
        if os.path.exists(mesh_dir):
            return mesh_dir
    except Exception:
        pass
    # Fallback: try to find from current file location
    current_file = os.path.abspath(__file__)
    # workspace_gui.py -> workspace/ -> workspace/ -> scene/ -> src/ -> ws_root
    pkg_dir = os.path.dirname(os.path.dirname(current_file))
    mesh_dir = os.path.join(pkg_dir, 'mesh')
    if os.path.exists(mesh_dir):
        return mesh_dir
    return os.path.expanduser("~/ws/KITT_POC_ws/src/scene/workspace/mesh")

MESH_DIR = get_mesh_dir()

# Global mesh sampling rate (0.0-1.0, 1.0 = use all triangles)
MESH_SAMPLE_RATE = 1.0

# Attach configuration for left and right arms
# Use gripper base link (Lbase_link/Rbase_link) for attach
# touch_links includes all links that can contact the attached object
ATTACH_CONFIG = {
    'left': {
        'link': 'Lbase_link',  # Gripper base link
        'touch_links': [
            # Gripper base and all child links
            'Lbase_link',
            'Lleftout_Link', 'Lrightout_Link',
            'Lleftinn_Link', 'Lrightinn_Link',
            'Lleft_kckle', 'Lright_kckle',
            'Lleft_pad', 'Lright_pad',
            # Parent links (flange and arm end)
            'left_flange_link',
            'AR5_5_07L_tcp',
            'AR5_5_07L_link7',
            'AR5_5_07L_link6',
        ]
    },
    'right': {
        'link': 'Rbase_link',  # Gripper base link
        'touch_links': [
            # Gripper base and all child links
            'Rbase_link',
            'Rleftout_Link', 'Rrightout_Link',
            'Rleftinn_Link', 'Rrightinn_Link',
            'Rleft_kckle', 'Rright_kckle',
            'Rleft_pad', 'Rright_pad',
            # Parent links (flange and arm end)
            'right_flange_link',
            'AR5_5_07R_tcp',
            'AR5_5_07R_link7',
            'AR5_5_07R_link6',
        ]
    }
}


def load_stl_mesh(filepath: str, sample_rate: float = None, compute_bbox: bool = True):
    """Load STL file and return vertices, triangles, and bounding box for MoveIt mesh.
    
    Args:
        filepath: Path to STL file
        sample_rate: Fraction of triangles to keep (0.0-1.0). None uses global MESH_SAMPLE_RATE.
        compute_bbox: Whether to compute bounding box from full mesh
    
    Returns:
        tuple: (vertices, triangles, bbox_size, bbox_center)
        - bbox_size: [width, depth, height] of bounding box
        - bbox_center: [x, y, z] center offset of bounding box
    """
    import struct
    import random
    
    if sample_rate is None:
        sample_rate = MESH_SAMPLE_RATE
    
    sample_rate = max(0.01, min(1.0, sample_rate))  # Clamp to valid range
    
    all_triangles_data = []
    all_vertices_for_bbox = []  # Keep all vertices for bbox calculation
    
    with open(filepath, 'rb') as f:
        header = f.read(80)
        num_triangles = struct.unpack('I', f.read(4))[0]
        
        for _ in range(num_triangles):
            # Normal vector (skip)
            f.read(12)
            # Three vertices
            v1 = struct.unpack('fff', f.read(12))
            v2 = struct.unpack('fff', f.read(12))
            v3 = struct.unpack('fff', f.read(12))
            # Attribute byte count
            f.read(2)
            
            all_triangles_data.append((v1, v2, v3))
            if compute_bbox:
                all_vertices_for_bbox.extend([v1, v2, v3])
    
    # Compute bounding box from all vertices
    bbox_size = [0.1, 0.1, 0.1]  # Default
    bbox_center = [0.0, 0.0, 0.0]
    if compute_bbox and all_vertices_for_bbox:
        xs = [v[0] for v in all_vertices_for_bbox]
        ys = [v[1] for v in all_vertices_for_bbox]
        zs = [v[2] for v in all_vertices_for_bbox]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        min_z, max_z = min(zs), max(zs)
        bbox_size = [max_x - min_x, max_y - min_y, max_z - min_z]
        bbox_center = [(min_x + max_x) / 2, (min_y + max_y) / 2, (min_z + max_z) / 2]
    
    # Sample triangles if needed
    if sample_rate < 1.0:
        num_to_keep = max(1, int(len(all_triangles_data) * sample_rate))
        sampled_data = random.sample(all_triangles_data, num_to_keep)
    else:
        sampled_data = all_triangles_data
    
    # Build vertices and triangles lists
    vertices = []
    triangles = []
    for v1, v2, v3 in sampled_data:
        idx_base = len(vertices)
        vertices.extend([v1, v2, v3])
        triangles.append((idx_base, idx_base + 1, idx_base + 2))
    
    return vertices, triangles, bbox_size, bbox_center


class WorkspaceObstacle:
    """Represents a box obstacle with extended properties."""
    
    def __init__(
        self,
        name: str,
        position: List[float] = None,
        size: List[float] = None,
        rotation_euler: List[float] = None,
        color: List[float] = None,
        base_frame: str = 'world',
        collision_enabled: bool = True,
        visible: bool = True,
        pose_axis_visible: bool = False,
        attached_to: str = None,  # None, 'left', or 'right'
    ):
        self.name = name
        self.position = position if position else [0.5, 0.0, 0.5]
        self.size = size if size else [0.1, 0.1, 0.1]
        self.rotation_euler = rotation_euler if rotation_euler else [0.0, 0.0, 0.0]
        self.color = color if color else [0.8, 0.2, 0.2, 0.7]
        self.base_frame = base_frame
        self.collision_enabled = collision_enabled
        self.visible = visible
        self.pose_axis_visible = pose_axis_visible
        self.attached_to = attached_to  # None, 'left', or 'right'
        self.obj_type = 'box'
        # Store the relative offset from link when attached (for correct detach position)
        # This is the object's position relative to the link frame at attach time
        self._attach_offset_pos = None  # [x, y, z] offset in link frame
        self._attach_offset_rot = None  # [x, y, z, w] quaternion offset in link frame
    
    def to_quaternion(self) -> List[float]:
        euler_rad = np.deg2rad(self.rotation_euler)
        r = R.from_euler('xyz', euler_rad)
        return r.as_quat().tolist()
    
    def to_dict(self) -> dict:
        return {
            'type': 'box',
            'name': self.name,
            'position': list(self.position),
            'size': list(self.size),
            'rotation_euler': list(self.rotation_euler),
            'color': list(self.color),
            'base_frame': self.base_frame,
            'collision_enabled': self.collision_enabled,
            'visible': self.visible,
            'pose_axis_visible': self.pose_axis_visible,
            'attached_to': self.attached_to,
        }

    @classmethod
    def from_dict(cls, data: dict) -> 'WorkspaceObstacle':
        return cls(
            name=data['name'],
            position=data.get('position', [0.5, 0.0, 0.5]),
            size=data.get('size', [0.1, 0.1, 0.1]),
            rotation_euler=data.get('rotation_euler', [0.0, 0.0, 0.0]),
            color=data.get('color', [0.8, 0.2, 0.2, 0.7]),
            base_frame=data.get('base_frame', 'world'),
            collision_enabled=data.get('collision_enabled', True),
            visible=data.get('visible', True),
            pose_axis_visible=data.get('pose_axis_visible', False),
            attached_to=data.get('attached_to', None),
        )


class WorkspaceMesh:
    """Represents a mesh obstacle with bounding box option for fast positioning."""
    
    def __init__(
        self,
        name: str,
        mesh_path: str,
        position: List[float] = None,
        rotation_euler: List[float] = None,
        scale: List[float] = None,
        base_frame: str = 'world',
        collision_enabled: bool = True,
        use_bbox: bool = False,  # False=show mesh, True=show bounding box instead
        attached_to: str = None,  # None, 'left', or 'right'
    ):
        self.name = name
        self.mesh_path = mesh_path  # Relative path from MESH_DIR
        self.position = position if position else [0.0, 0.0, 0.0]
        self.rotation_euler = rotation_euler if rotation_euler else [0.0, 0.0, 0.0]
        self.scale = scale if scale else [1.0, 1.0, 1.0]
        self.base_frame = base_frame
        self.collision_enabled = collision_enabled
        self.use_bbox = use_bbox  # False=mesh, True=bounding box (mutually exclusive)
        self.attached_to = attached_to  # None, 'left', or 'right'
        self.obj_type = 'mesh'
        self._mesh_data = None  # Cache: (vertices, triangles, bbox_size, bbox_center)
        self._cached_mesh_msg = None  # Cache for Mesh message
        self._cached_scale = None  # Scale used for cached mesh
        self._cached_sample_rate = None  # Sample rate used for cached mesh
        self._bbox_size = [0.1, 0.1, 0.1]  # Bounding box size (before scale)
        self._bbox_center = [0.0, 0.0, 0.0]  # Bounding box center offset (before scale)
        # Store the relative offset from link when attached (for correct detach position)
        self._attach_offset_pos = None  # [x, y, z] offset in link frame
        self._attach_offset_rot = None  # [x, y, z, w] quaternion offset in link frame
    
    def to_quaternion(self) -> List[float]:
        euler_rad = np.deg2rad(self.rotation_euler)
        r = R.from_euler('xyz', euler_rad)
        return r.as_quat().tolist()
    
    def get_full_path(self) -> str:
        return os.path.join(MESH_DIR, self.mesh_path)
    
    def load_mesh(self, force_reload: bool = False):
        """Load mesh data from file (cached)."""
        global MESH_SAMPLE_RATE
        # Reload if sample rate changed or forced
        if force_reload or self._mesh_data is None or self._cached_sample_rate != MESH_SAMPLE_RATE:
            full_path = self.get_full_path()
            if os.path.exists(full_path):
                vertices, triangles, bbox_size, bbox_center = load_stl_mesh(full_path, MESH_SAMPLE_RATE)
                self._mesh_data = (vertices, triangles)
                self._bbox_size = bbox_size
                self._bbox_center = bbox_center
                self._cached_sample_rate = MESH_SAMPLE_RATE
                self._cached_mesh_msg = None  # Invalidate mesh message cache
        return self._mesh_data
    
    def get_bbox_size_scaled(self) -> List[float]:
        """Get bounding box size with scale applied."""
        return [
            self._bbox_size[0] * self.scale[0],
            self._bbox_size[1] * self.scale[1],
            self._bbox_size[2] * self.scale[2]
        ]
    
    def get_bbox_center_scaled(self) -> List[float]:
        """Get bounding box center offset with scale applied."""
        return [
            self._bbox_center[0] * self.scale[0],
            self._bbox_center[1] * self.scale[1],
            self._bbox_center[2] * self.scale[2]
        ]
    
    def get_mesh_msg(self) -> Optional[Mesh]:
        """Get cached Mesh message for CollisionObject."""
        global MESH_SAMPLE_RATE
        # Check if we need to rebuild (scale changed, sample rate changed, or not cached)
        if (self._cached_mesh_msg is None or 
            self._cached_scale != tuple(self.scale) or
            self._cached_sample_rate != MESH_SAMPLE_RATE):
            mesh_data = self.load_mesh()
            if mesh_data is None:
                return None
            
            vertices, triangles = mesh_data
            mesh = Mesh()
            for v in vertices:
                pt = Point()
                pt.x = float(v[0]) * self.scale[0]
                pt.y = float(v[1]) * self.scale[1]
                pt.z = float(v[2]) * self.scale[2]
                mesh.vertices.append(pt)
            
            for tri in triangles:
                mt = MeshTriangle()
                mt.vertex_indices = list(tri)
                mesh.triangles.append(mt)
            
            self._cached_mesh_msg = mesh
            self._cached_scale = tuple(self.scale)
        
        return self._cached_mesh_msg

    def to_dict(self) -> dict:
        return {
            'type': 'mesh',
            'name': self.name,
            'mesh_path': self.mesh_path,
            'position': list(self.position),
            'rotation_euler': list(self.rotation_euler),
            'scale': list(self.scale),
            'base_frame': self.base_frame,
            'collision_enabled': self.collision_enabled,
            'use_bbox': self.use_bbox,
            'attached_to': self.attached_to,
        }
    
    @classmethod
    def from_dict(cls, data: dict) -> 'WorkspaceMesh':
        # Support legacy config with show_mesh/show_bbox
        use_bbox = data.get('use_bbox', None)
        if use_bbox is None:
            # Legacy: if show_mesh=False or show_bbox=True, use bbox
            use_bbox = not data.get('show_mesh', True) or data.get('show_bbox', False)
        return cls(
            name=data['name'],
            mesh_path=data['mesh_path'],
            position=data.get('position', [0.0, 0.0, 0.0]),
            rotation_euler=data.get('rotation_euler', [0.0, 0.0, 0.0]),
            scale=data.get('scale', [1.0, 1.0, 1.0]),
            base_frame=data.get('base_frame', 'world'),
            collision_enabled=data.get('collision_enabled', True),
            use_bbox=use_bbox,
            attached_to=data.get('attached_to', None),
        )


class WorkspaceGUINode(Node):
    """ROS2 node for workspace obstacle management."""
    
    def __init__(self):
        super().__init__('workspace_gui_node')
        
        self.obstacles: Dict[str, WorkspaceObstacle] = {}
        self.meshes: Dict[str, WorkspaceMesh] = {}
        
        # TF2 for coordinate transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Publishers - only for visualization markers
        self.marker_pub = self.create_publisher(MarkerArray, '/workspace_markers', 10)
        self.axis_pub = self.create_publisher(MarkerArray, '/workspace_pose_axes', 10)
        
        # Service client for ALL planning scene operations (add, remove, attach, detach)
        self.apply_scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        
        # Service client for querying planning scene state (for debugging)
        self.get_scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
        
        # Timer ONLY for visualization markers (not for planning scene)
        self.timer = self.create_timer(0.1, self._publish_visualization)
        self.get_logger().info("WorkspaceGUI node initialized (event-driven planning scene)")
        
        # Wait for services
        self.get_logger().info("Waiting for /get_planning_scene service...")
        if self.get_scene_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("GetPlanningScene service ready")
        else:
            self.get_logger().warn("GetPlanningScene service not available - diagnostics limited")

    def _publish_visualization(self):
        """Publish visualization markers only (not planning scene)."""
        self._publish_markers()
        self._publish_axes()

    def publish_obstacle(self, obj) -> bool:
        """Publish a single obstacle to MoveIt planning scene via service.
        
        This is called when:
        - A new obstacle is created
        - An obstacle's properties are modified (position, size, etc.)
        
        Args:
            obj: WorkspaceObstacle or WorkspaceMesh object
        
        Returns:
            True if successful, False otherwise
        """
        # Skip if object is attached (managed separately)
        if obj.attached_to is not None:
            return True
        
        if not self.apply_scene_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("ApplyPlanningScene service not available")
            return False
        
        # Build collision object
        co = CollisionObject()
        co.header.frame_id = obj.base_frame
        co.header.stamp = self.get_clock().now().to_msg()
        co.id = obj.name
        
        if obj.collision_enabled:
            co.operation = CollisionObject.ADD
            
            pose = Pose()
            pose.position.x = obj.position[0]
            pose.position.y = obj.position[1]
            pose.position.z = obj.position[2]
            q = obj.to_quaternion()
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            
            if obj.obj_type == 'box':
                box = SolidPrimitive()
                box.type = SolidPrimitive.BOX
                box.dimensions = list(obj.size)
                co.primitives.append(box)
                co.primitive_poses.append(pose)
            elif obj.obj_type == 'mesh':
                if obj.use_bbox:
                    obj.load_mesh()
                    bbox_size = obj.get_bbox_size_scaled()
                    bbox_center = obj.get_bbox_center_scaled()
                    box = SolidPrimitive()
                    box.type = SolidPrimitive.BOX
                    box.dimensions = list(bbox_size)
                    rot = R.from_quat(q)
                    rotated_center = rot.apply(bbox_center)
                    adjusted_pose = Pose()
                    adjusted_pose.position.x = pose.position.x + rotated_center[0]
                    adjusted_pose.position.y = pose.position.y + rotated_center[1]
                    adjusted_pose.position.z = pose.position.z + rotated_center[2]
                    adjusted_pose.orientation = pose.orientation
                    co.primitives.append(box)
                    co.primitive_poses.append(adjusted_pose)
                else:
                    mesh_msg = obj.get_mesh_msg()
                    if mesh_msg is None:
                        self.get_logger().warn(f"Failed to load mesh: {obj.mesh_path}")
                        return False
                    co.meshes.append(mesh_msg)
                    co.mesh_poses.append(pose)
        else:
            co.operation = CollisionObject.REMOVE
        
        # Apply via service
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(co)
        
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.apply_scene_client.call_async(request)
        
        timeout = 2.0
        start = time_module.time()
        while not future.done() and (time_module.time() - start) < timeout:
            time_module.sleep(0.05)
        
        if future.done() and future.result() is not None and future.result().success:
            return True
        else:
            self.get_logger().warn(f"Failed to publish obstacle: {obj.name}")
            return False

    def remove_obstacle_from_scene(self, name: str, frame: str = "world") -> bool:
        """Remove an obstacle from MoveIt planning scene via service.
        
        Args:
            name: Name of the obstacle to remove
            frame: Frame ID
        
        Returns:
            True if successful, False otherwise
        """
        if not self.apply_scene_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("ApplyPlanningScene service not available")
            return False
        
        co = CollisionObject()
        co.header.frame_id = frame
        co.header.stamp = self.get_clock().now().to_msg()
        co.id = name
        co.operation = CollisionObject.REMOVE
        
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(co)
        
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.apply_scene_client.call_async(request)
        
        timeout = 2.0
        start = time_module.time()
        while not future.done() and (time_module.time() - start) < timeout:
            time_module.sleep(0.05)
        
        if future.done() and future.result() is not None and future.result().success:
            self.get_logger().info(f"Removed obstacle from scene: {name}")
            return True
        else:
            self.get_logger().warn(f"Failed to remove obstacle: {name}")
            return False

    def publish_all_obstacles(self):
        """Publish all obstacles to MoveIt planning scene (used for initial sync)."""
        for obs in self.obstacles.values():
            if obs.attached_to is None:
                self.publish_obstacle(obs)
        for mesh in self.meshes.values():
            if mesh.attached_to is None:
                self.publish_obstacle(mesh)

    def get_planning_scene_state(self) -> dict:
        """Query MoveIt for current planning scene state (for debugging).
        
        Returns:
            dict with 'attached_objects' and 'world_objects' lists, or None on failure
        """
        if not self.get_scene_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("[DIAG] GetPlanningScene service not available")
            return None
        
        req = GetPlanningScene.Request()
        req.components.components = (
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS |
            PlanningSceneComponents.WORLD_OBJECT_NAMES
        )
        
        future = self.get_scene_client.call_async(req)
        timeout = 3.0
        start = time_module.time()
        while not future.done() and (time_module.time() - start) < timeout:
            time_module.sleep(0.05)
        
        if future.done() and future.result() is not None:
            scene = future.result().scene
            attached = [aco.object.id for aco in scene.robot_state.attached_collision_objects]
            world = [co.id for co in scene.world.collision_objects]
            return {
                'attached_objects': attached,
                'world_objects': world,
                'attached_details': [(aco.object.id, aco.link_name) for aco in scene.robot_state.attached_collision_objects]
            }
        return None

    def log_planning_scene_state(self, prefix: str = ""):
        """Log current planning scene state for debugging."""
        state = self.get_planning_scene_state()
        if state:
            self.get_logger().info(f"[DIAG]{prefix} === MoveIt Planning Scene State ===")
            self.get_logger().info(f"[DIAG]{prefix} Attached objects ({len(state['attached_objects'])}): {state['attached_objects']}")
            self.get_logger().info(f"[DIAG]{prefix} Attached details: {state['attached_details']}")
            self.get_logger().info(f"[DIAG]{prefix} World objects ({len(state['world_objects'])}): {state['world_objects']}")
        else:
            self.get_logger().warn(f"[DIAG]{prefix} Failed to get planning scene state")
        return state

    def attach_object(self, obj_name: str, arm: str) -> bool:
        """Attach an object to the specified arm's end effector.
        
        Args:
            obj_name: Name of the object to attach
            arm: 'left' or 'right'
        
        Returns:
            True if successful, False otherwise
        """
        self.get_logger().info(f"")
        self.get_logger().info(f"{'='*60}")
        self.get_logger().info(f"[ATTACH] Starting attach: obj={obj_name}, arm={arm}")
        self.get_logger().info(f"{'='*60}")
        
        # Log initial state
        self.log_planning_scene_state(" BEFORE")
        
        if arm not in ATTACH_CONFIG:
            self.get_logger().error(f"[ATTACH] Invalid arm: {arm}")
            return False
        
        config = ATTACH_CONFIG[arm]
        link_name = config['link']
        touch_links = config['touch_links']
        
        self.get_logger().info(f"[ATTACH] link_name={link_name}, touch_links count={len(touch_links)}")
        
        # Get the object
        obj = self.obstacles.get(obj_name) or self.meshes.get(obj_name)
        if obj is None:
            self.get_logger().error(f"[ATTACH] Object not found: {obj_name}")
            return False
        
        if obj.attached_to is not None:
            self.get_logger().warn(f"[ATTACH] Object {obj_name} is already attached to {obj.attached_to}")
            return False
        
        # Check if arm already has an attached object (only one object per arm allowed)
        for obs in self.obstacles.values():
            if obs.attached_to == arm:
                self.get_logger().warn(f"[ATTACH] Arm {arm} already has attached object: {obs.name}")
                return False
        for mesh in self.meshes.values():
            if mesh.attached_to == arm:
                self.get_logger().warn(f"[ATTACH] Arm {arm} already has attached mesh: {mesh.name}")
                return False
        
        # Wait for service
        if not self.apply_scene_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("[ATTACH] ApplyPlanningScene service not available")
            return False
        
        # Build object pose
        pose = Pose()
        pose.position.x = obj.position[0]
        pose.position.y = obj.position[1]
        pose.position.z = obj.position[2]
        q = obj.to_quaternion()
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        
        self.get_logger().info(f"[ATTACH] Object pos=({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})")
        
        # Step 1: Ensure object exists in world (add via service)
        self.get_logger().info("[ATTACH] Step 1: Ensuring object in world...")
        
        add_co = CollisionObject()
        add_co.header.frame_id = obj.base_frame
        add_co.header.stamp = self.get_clock().now().to_msg()
        add_co.id = obj_name
        add_co.operation = CollisionObject.ADD
        
        if obj.obj_type == 'box':
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = list(obj.size)
            add_co.primitives.append(box)
            add_co.primitive_poses.append(pose)
        elif obj.obj_type == 'mesh':
            if obj.use_bbox:
                obj.load_mesh()
                bbox_size = obj.get_bbox_size_scaled()
                bbox_center = obj.get_bbox_center_scaled()
                box = SolidPrimitive()
                box.type = SolidPrimitive.BOX
                box.dimensions = list(bbox_size)
                rot = R.from_quat(q)
                rotated_center = rot.apply(bbox_center)
                adjusted_pose = Pose()
                adjusted_pose.position.x = pose.position.x + rotated_center[0]
                adjusted_pose.position.y = pose.position.y + rotated_center[1]
                adjusted_pose.position.z = pose.position.z + rotated_center[2]
                adjusted_pose.orientation = pose.orientation
                add_co.primitives.append(box)
                add_co.primitive_poses.append(adjusted_pose)
            else:
                mesh_msg = obj.get_mesh_msg()
                if mesh_msg is None:
                    self.get_logger().error(f"[ATTACH] Failed to get mesh message")
                    return False
                add_co.meshes.append(mesh_msg)
                add_co.mesh_poses.append(pose)
        
        add_scene = PlanningScene()
        add_scene.is_diff = True
        add_scene.world.collision_objects.append(add_co)
        
        request = ApplyPlanningScene.Request()
        request.scene = add_scene
        future = self.apply_scene_client.call_async(request)
        
        timeout = 5.0
        start = time_module.time()
        while not future.done() and (time_module.time() - start) < timeout:
            time_module.sleep(0.1)
        
        if not (future.done() and future.result() is not None and future.result().success):
            self.get_logger().error("[ATTACH] Step 1 FAILED: Could not add object to world")
            return False
        self.get_logger().info("[ATTACH] Step 1 OK: Object in world")
        
        # Step 2: Attach object to link
        self.get_logger().info(f"[ATTACH] Step 2: Attaching to {link_name}...")
        
        attached = AttachedCollisionObject()
        attached.link_name = link_name
        attached.object.id = obj_name
        attached.object.header.frame_id = obj.base_frame
        attached.object.operation = CollisionObject.ADD
        attached.touch_links = list(touch_links)
        
        remove_obj = CollisionObject()
        remove_obj.id = obj_name
        remove_obj.header.frame_id = obj.base_frame
        remove_obj.operation = CollisionObject.REMOVE
        
        attach_scene = PlanningScene()
        attach_scene.is_diff = True
        attach_scene.robot_state.attached_collision_objects.append(attached)
        attach_scene.world.collision_objects.append(remove_obj)
        
        request2 = ApplyPlanningScene.Request()
        request2.scene = attach_scene
        future2 = self.apply_scene_client.call_async(request2)
        
        start = time_module.time()
        while not future2.done() and (time_module.time() - start) < timeout:
            time_module.sleep(0.1)
        
        if future2.done() and future2.result() is not None:
            result = future2.result()
            self.get_logger().info(f"[ATTACH] Step 2 result: success={result.success}")
            
            # Verify attach by checking actual state
            time_module.sleep(0.3)
            state = self.log_planning_scene_state(" AFTER")
            
            if state and obj_name in state['attached_objects']:
                self.get_logger().info(f"[ATTACH] SUCCESS: Verified {obj_name} is in attached_objects")
                
                # Calculate and save the relative offset from link to object
                # This is needed for correct position calculation during detach
                # offset = inverse(link_world_transform) * object_world_transform
                try:
                    # Get link's current world transform
                    target_frame = obj.base_frame if obj.base_frame else "world"
                    transform = self.tf_buffer.lookup_transform(
                        target_frame,  # Target frame (usually "world")
                        link_name,     # Source frame (the link)
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=2.0)
                    )
                    
                    t = transform.transform
                    link_pos = np.array([t.translation.x, t.translation.y, t.translation.z])
                    link_rot = R.from_quat([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
                    
                    # Object's world position and rotation
                    obj_pos = np.array(obj.position)
                    obj_rot = R.from_euler('xyz', np.deg2rad(obj.rotation_euler))
                    
                    # Calculate relative offset: offset = inverse(link) * object
                    # offset_pos = inverse(link_rot) * (obj_pos - link_pos)
                    link_rot_inv = link_rot.inv()
                    offset_pos = link_rot_inv.apply(obj_pos - link_pos)
                    offset_rot = link_rot_inv * obj_rot
                    
                    obj._attach_offset_pos = offset_pos.tolist()
                    obj._attach_offset_rot = offset_rot.as_quat().tolist()
                    
                    self.get_logger().info(f"[ATTACH] Link position: {link_pos.tolist()}")
                    self.get_logger().info(f"[ATTACH] Object position: {obj_pos.tolist()}")
                    self.get_logger().info(f"[ATTACH] Saved offset_pos (in link frame): {obj._attach_offset_pos}")
                    self.get_logger().info(f"[ATTACH] Saved offset_rot (in link frame): {obj._attach_offset_rot}")
                    
                except Exception as e:
                    self.get_logger().warn(f"[ATTACH] Failed to calculate attach offset: {e}")
                    obj._attach_offset_pos = None
                    obj._attach_offset_rot = None
                
                obj.attached_to = arm
                return True
            else:
                self.get_logger().error(f"[ATTACH] VERIFICATION FAILED: {obj_name} not found in attached_objects!")
                self.get_logger().error(f"[ATTACH] This indicates MoveIt did not actually attach the object")
                # Still mark as attached in our state for consistency
                obj.attached_to = arm
                return True
        else:
            self.get_logger().error(f"[ATTACH] FAILED: Timeout or no result from service")
            return False

    def detach_object(self, obj_name: str) -> bool:
        """Detach an object from the robot and place it back in the world at current position.
        
        After detach, the object is placed back in the world as a static collision object
        at its current position (where the arm moved it to).
        
        Args:
            obj_name: Name of the object to detach
        
        Returns:
            True if successful, False otherwise
        """
        self.get_logger().info(f"")
        self.get_logger().info(f"{'='*60}")
        self.get_logger().info(f"[DETACH] Starting detach: obj={obj_name}")
        self.get_logger().info(f"{'='*60}")
        
        # Log initial state
        self.log_planning_scene_state(" BEFORE")
        
        # Get the object
        obj = self.obstacles.get(obj_name) or self.meshes.get(obj_name)
        if obj is None:
            self.get_logger().error(f"[DETACH] Object not found in local state: {obj_name}")
            return False
        
        if obj.attached_to is None:
            self.get_logger().warn(f"[DETACH] Object {obj_name} is not attached (local state)")
            # But let's check MoveIt's actual state
            state = self.get_planning_scene_state()
            if state and obj_name in state['attached_objects']:
                self.get_logger().warn(f"[DETACH] But MoveIt says it IS attached! Proceeding with detach...")
            else:
                return False
        
        arm = obj.attached_to
        if arm:
            config = ATTACH_CONFIG[arm]
            link_name = config['link']
            self.get_logger().info(f"[DETACH] Detaching from link={link_name}, arm={arm}")
        else:
            # Try to find which link it's attached to from MoveIt state
            state = self.get_planning_scene_state()
            link_name = None
            if state:
                for obj_id, lname in state['attached_details']:
                    if obj_id == obj_name:
                        link_name = lname
                        break
            if not link_name:
                self.get_logger().error(f"[DETACH] Cannot determine link_name for {obj_name}")
                return False
            self.get_logger().info(f"[DETACH] Found link from MoveIt: {link_name}")
        
        # Wait for service
        if not self.apply_scene_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("[DETACH] ApplyPlanningScene service not available")
            return False
        
        timeout = 5.0
        
        # ============================================================
        # CRITICAL: Calculate the object's world position BEFORE detaching
        # ============================================================
        # When attached, the object moves with the link. To get the correct world position:
        # new_world_pos = link_world_transform * saved_offset
        # where saved_offset was calculated at attach time
        
        self.get_logger().info(f"[DETACH] Calculating object's current world position...")
        self.get_logger().info(f"[DETACH] Original position (before attach): {obj.position}")
        self.get_logger().info(f"[DETACH] Original base_frame: {obj.base_frame}")
        self.get_logger().info(f"[DETACH] Saved offset_pos: {obj._attach_offset_pos}")
        self.get_logger().info(f"[DETACH] Saved offset_rot: {obj._attach_offset_rot}")
        
        new_world_position = None
        new_world_orientation = None
        
        try:
            # Get the link's current position in world frame
            target_frame = obj.base_frame if obj.base_frame else "world"
            transform = self.tf_buffer.lookup_transform(
                target_frame,  # Target frame (usually "world")
                link_name,     # Source frame (the link the object is attached to)
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            
            t = transform.transform
            link_pos = np.array([t.translation.x, t.translation.y, t.translation.z])
            link_rot = R.from_quat([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
            
            self.get_logger().info(f"[DETACH] Link '{link_name}' current position in '{target_frame}': ({link_pos[0]:.4f}, {link_pos[1]:.4f}, {link_pos[2]:.4f})")
            
            # Calculate new world position using saved offset
            # new_world = link_world * offset
            # new_pos = link_pos + link_rot * offset_pos
            # new_rot = link_rot * offset_rot
            
            if obj._attach_offset_pos is not None:
                offset_pos = np.array(obj._attach_offset_pos)
                new_pos = link_pos + link_rot.apply(offset_pos)
                new_world_position = new_pos.tolist()
                
                if obj._attach_offset_rot is not None:
                    offset_rot = R.from_quat(obj._attach_offset_rot)
                    new_rot = link_rot * offset_rot
                    new_world_orientation = new_rot.as_quat().tolist()
                else:
                    new_world_orientation = link_rot.as_quat().tolist()
                
                self.get_logger().info(f"[DETACH] Calculated new world position (using saved offset): {new_world_position}")
            else:
                # Fallback: no saved offset, use link position directly (less accurate)
                self.get_logger().warn(f"[DETACH] No saved offset, using link position directly (may be inaccurate)")
                new_world_position = link_pos.tolist()
                new_world_orientation = link_rot.as_quat().tolist()
                self.get_logger().info(f"[DETACH] Calculated new world position (link position): {new_world_position}")
            
        except Exception as e:
            self.get_logger().warn(f"[DETACH] Failed to get link transform: {e}")
            self.get_logger().warn(f"[DETACH] Will try to get position from MoveIt after detach")
        
        # Step 1: Remove from attached_collision_objects (detach from robot)
        self.get_logger().info("[DETACH] Step 1: Removing from attached objects (is_diff=True)...")
        
        detach_scene = PlanningScene()
        detach_scene.is_diff = True
        detach_scene.robot_state.is_diff = True  # CRITICAL: Must be True for diff operations
        
        attached_obj = AttachedCollisionObject()
        attached_obj.link_name = link_name
        attached_obj.object.id = obj_name
        attached_obj.object.header.frame_id = obj.base_frame
        attached_obj.object.header.stamp = self.get_clock().now().to_msg()
        attached_obj.object.operation = CollisionObject.REMOVE
        
        detach_scene.robot_state.attached_collision_objects.append(attached_obj)
        
        request = ApplyPlanningScene.Request()
        request.scene = detach_scene
        
        future = self.apply_scene_client.call_async(request)
        
        start = time_module.time()
        while not future.done() and (time_module.time() - start) < timeout:
            time_module.sleep(0.1)
        
        if future.done() and future.result() is not None:
            result = future.result()
            self.get_logger().info(f"[DETACH] Step 1 result: success={result.success}")
        else:
            self.get_logger().error("[DETACH] Step 1 FAILED: Timeout or no result")
            return False
        
        # Verify detach
        time_module.sleep(0.1)
        state = self.log_planning_scene_state(" AFTER STEP 1")
        
        if state and obj_name in state['attached_objects']:
            self.get_logger().error(f"[DETACH] VERIFICATION FAILED: {obj_name} still in attached_objects after Step 1!")
            
            # Alternative: Get full planning scene, remove the attached object, re-apply
            self.get_logger().info("[DETACH] Step 1b: Getting full scene and re-applying...")
            
            # Get current full planning scene
            get_req = GetPlanningScene.Request()
            get_req.components.components = (
                PlanningSceneComponents.SCENE_SETTINGS |
                PlanningSceneComponents.ROBOT_STATE |
                PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS |
                PlanningSceneComponents.WORLD_OBJECT_NAMES |
                PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
                PlanningSceneComponents.TRANSFORMS |
                PlanningSceneComponents.ALLOWED_COLLISION_MATRIX |
                PlanningSceneComponents.LINK_PADDING_AND_SCALING |
                PlanningSceneComponents.OBJECT_COLORS
            )
            
            get_future = self.get_scene_client.call_async(get_req)
            start = time_module.time()
            while not get_future.done() and (time_module.time() - start) < timeout:
                time_module.sleep(0.1)
            
            if get_future.done() and get_future.result() is not None:
                full_scene = get_future.result().scene
                self.get_logger().info(f"[DETACH] Got full scene, attached count: {len(full_scene.robot_state.attached_collision_objects)}")
                
                # Remove the target object from attached_collision_objects
                new_attached = []
                for aco in full_scene.robot_state.attached_collision_objects:
                    if aco.object.id != obj_name:
                        new_attached.append(aco)
                    else:
                        self.get_logger().info(f"[DETACH] Removing {aco.object.id} from attached list")
                
                full_scene.robot_state.attached_collision_objects = new_attached
                full_scene.is_diff = False  # Full scene replacement
                full_scene.robot_state.is_diff = False
                
                request1b = ApplyPlanningScene.Request()
                request1b.scene = full_scene
                
                future1b = self.apply_scene_client.call_async(request1b)
                start = time_module.time()
                while not future1b.done() and (time_module.time() - start) < timeout:
                    time_module.sleep(0.1)
                
                if future1b.done() and future1b.result() is not None:
                    self.get_logger().info(f"[DETACH] Step 1b result: success={future1b.result().success}")
                
                time_module.sleep(0.3)
                state = self.log_planning_scene_state(" AFTER STEP 1b")
                
                if state and obj_name in state['attached_objects']:
                    self.get_logger().error(f"[DETACH] Still attached after Step 1b!")
            else:
                self.get_logger().error("[DETACH] Failed to get full planning scene")
        
        # Step 2: Update object position
        # CRITICAL: Use the TF-calculated position (new_world_position) which was computed BEFORE detach
        # MoveIt's returned position after detach is often (0,0,0) which is incorrect
        self.get_logger().info("[DETACH] Step 2: Updating object position...")
        
        time_module.sleep(0.1)
        
        # Log the object's original position for comparison
        self.get_logger().info(f"[DETACH] Original local position: {obj.position}")
        self.get_logger().info(f"[DETACH] Original local rotation: {obj.rotation_euler}")
        self.get_logger().info(f"[DETACH] Original base_frame: {obj.base_frame}")
        
        # Use the TF-calculated position if available (calculated BEFORE detach)
        # This is the CORRECT approach - TF gives us the link's world position at detach time
        if new_world_position is not None:
            self.get_logger().info(f"[DETACH] Using TF-calculated position (BEFORE detach): {new_world_position}")
            new_position = new_world_position
            new_orientation = new_world_orientation
            self.get_logger().info(f"[DETACH] TF position is reliable - skipping MoveIt query")
        else:
            # Fallback: try to get from MoveIt (but this often returns 0,0,0 on first detach)
            self.get_logger().warn("[DETACH] TF position not available, trying MoveIt (may be incorrect)...")
            new_position = None
            new_orientation = None
            
            # Get world objects with geometry to find the new position
            get_world_req = GetPlanningScene.Request()
            get_world_req.components.components = (
                PlanningSceneComponents.WORLD_OBJECT_NAMES |
                PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            )
            
            get_world_future = self.get_scene_client.call_async(get_world_req)
            start = time_module.time()
            while not get_world_future.done() and (time_module.time() - start) < timeout:
                time_module.sleep(0.1)
            
            object_frame_id = None
            if get_world_future.done() and get_world_future.result() is not None:
                world_scene = get_world_future.result().scene
                self.get_logger().info(f"[DETACH] World scene has {len(world_scene.world.collision_objects)} objects")
                
                for co in world_scene.world.collision_objects:
                    self.get_logger().info(f"[DETACH] World object: id={co.id}, frame_id={co.header.frame_id}")
                    if co.id == obj_name:
                        object_frame_id = co.header.frame_id
                        self.get_logger().info(f"[DETACH] Found {obj_name} in world objects, frame_id={object_frame_id}")
                        
                        # Get position from primitive_poses or mesh_poses
                        pose = None
                        if co.primitive_poses:
                            pose = co.primitive_poses[0]
                            self.get_logger().info(f"[DETACH] Got pose from primitive_poses")
                        elif co.mesh_poses:
                            pose = co.mesh_poses[0]
                            self.get_logger().info(f"[DETACH] Got pose from mesh_poses")
                        else:
                            self.get_logger().warn(f"[DETACH] No poses found in collision object!")
                        
                        if pose:
                            self.get_logger().info(f"[DETACH] Raw pose in frame '{object_frame_id}': pos=({pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f})")
                            self.get_logger().info(f"[DETACH] Raw orientation: ({pose.orientation.x:.4f}, {pose.orientation.y:.4f}, {pose.orientation.z:.4f}, {pose.orientation.w:.4f})")
                            
                            # Check if MoveIt returned (0,0,0) which is likely incorrect
                            if abs(pose.position.x) < 0.001 and abs(pose.position.y) < 0.001 and abs(pose.position.z) < 0.001:
                                self.get_logger().warn(f"[DETACH] MoveIt returned (0,0,0) - this is likely incorrect!")
                                self.get_logger().warn(f"[DETACH] Keeping original position: {obj.position}")
                                # Don't update position - keep original
                            else:
                                target_frame = obj.base_frame
                                if not object_frame_id or object_frame_id == "world" or object_frame_id == target_frame:
                                    new_position = [pose.position.x, pose.position.y, pose.position.z]
                                    new_orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                                    self.get_logger().info(f"[DETACH] Using MoveIt position: {new_position}")
                                else:
                                    self.get_logger().info(f"[DETACH] Need transform from '{object_frame_id}' to '{target_frame}'")
                                    try:
                                        transform = self.tf_buffer.lookup_transform(
                                            target_frame, 
                                            object_frame_id,
                                            rclpy.time.Time(),
                                            timeout=rclpy.duration.Duration(seconds=1.0)
                                        )
                                        t = transform.transform
                                        trans_rot = R.from_quat([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
                                        trans_pos = np.array([t.translation.x, t.translation.y, t.translation.z])
                                        orig_pos = np.array([pose.position.x, pose.position.y, pose.position.z])
                                        new_pos = trans_rot.apply(orig_pos) + trans_pos
                                        new_position = new_pos.tolist()
                                        orig_rot = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
                                        new_rot = trans_rot * orig_rot
                                        new_orientation = new_rot.as_quat().tolist()
                                        self.get_logger().info(f"[DETACH] Transformed position: {new_position}")
                                    except Exception as e:
                                        self.get_logger().warn(f"[DETACH] Transform failed: {e}, using raw position")
                                        new_position = [pose.position.x, pose.position.y, pose.position.z]
                                        new_orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                        break
            else:
                self.get_logger().error("[DETACH] Failed to get world objects from MoveIt")
        
        # Update internal state
        obj.attached_to = None
        # Keep collision_enabled = True so the object stays visible and in the scene
        obj.collision_enabled = True
        # Clear the saved offset (no longer needed)
        obj._attach_offset_pos = None
        obj._attach_offset_rot = None
        
        # Update position if we got it
        if new_position:
            self.get_logger().info(f"[DETACH] Position change: {obj.position} -> {new_position}")
            obj.position = new_position
            if new_orientation:
                # Convert quaternion to euler
                r = R.from_quat(new_orientation)
                euler = r.as_euler('xyz', degrees=True)
                old_euler = obj.rotation_euler
                obj.rotation_euler = list(euler)
                self.get_logger().info(f"[DETACH] Rotation change: {old_euler} -> {obj.rotation_euler}")
        else:
            self.get_logger().warn(f"[DETACH] Could not get new position, keeping original position")
        
        # Final verification
        time_module.sleep(0.3)
        final_state = self.log_planning_scene_state(" FINAL")
        
        if final_state:
            if obj_name in final_state['attached_objects']:
                self.get_logger().error(f"[DETACH] FINAL CHECK FAILED: {obj_name} STILL in attached_objects!")
                self.get_logger().error(f"[DETACH] This is a critical issue - MoveIt is not responding to detach requests")
                return False
            elif obj_name in final_state['world_objects']:
                self.get_logger().info(f"[DETACH] FINAL CHECK OK: {obj_name} is now a world object (static obstacle)")
            else:
                self.get_logger().warn(f"[DETACH] {obj_name} not found in world objects - may need to re-add")
        
        self.get_logger().info(f"[DETACH] Completed: {obj_name} detached and placed at new position")
        self.get_logger().info(f"{'='*60}")
        
        return True

    def _publish_markers(self):
        """Publish Markers for visible obstacles and meshes."""
        marker_array = MarkerArray()
        
        # Box markers
        for obs in self.obstacles.values():
            marker_id = abs(hash(obs.name)) % 100000
            marker = Marker()
            marker.header.frame_id = obs.base_frame
            marker.header.stamp.sec = 0
            marker.header.stamp.nanosec = 0
            marker.ns = "workspace_obstacles"
            marker.id = marker_id
            # lifetime = 0 means never expire
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 0
            
            if obs.visible:
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = obs.position[0]
                marker.pose.position.y = obs.position[1]
                marker.pose.position.z = obs.position[2]
                q = obs.to_quaternion()
                marker.pose.orientation.x = q[0]
                marker.pose.orientation.y = q[1]
                marker.pose.orientation.z = q[2]
                marker.pose.orientation.w = q[3]
                marker.scale.x = obs.size[0]
                marker.scale.y = obs.size[1]
                marker.scale.z = obs.size[2]
                marker.color.r = float(obs.color[0])
                marker.color.g = float(obs.color[1])
                marker.color.b = float(obs.color[2])
                marker.color.a = float(obs.color[3])
            else:
                marker.action = Marker.DELETE
            
            marker_array.markers.append(marker)
        
        # Mesh markers OR bounding box markers (mutually exclusive)
        for mesh_obj in self.meshes.values():
            mesh_marker_id = abs(hash(mesh_obj.name)) % 100000 + 50000
            bbox_marker_id = abs(hash(mesh_obj.name)) % 100000 + 60000
            
            # Mesh marker
            mesh_marker = Marker()
            mesh_marker.header.frame_id = mesh_obj.base_frame
            mesh_marker.header.stamp.sec = 0
            mesh_marker.header.stamp.nanosec = 0
            mesh_marker.ns = "workspace_meshes"
            mesh_marker.id = mesh_marker_id
            mesh_marker.lifetime.sec = 0
            mesh_marker.lifetime.nanosec = 0
            
            # Bounding box marker
            bbox_marker = Marker()
            bbox_marker.header.frame_id = mesh_obj.base_frame
            bbox_marker.header.stamp.sec = 0
            bbox_marker.header.stamp.nanosec = 0
            bbox_marker.ns = "workspace_mesh_bbox"
            bbox_marker.id = bbox_marker_id
            bbox_marker.lifetime.sec = 0
            bbox_marker.lifetime.nanosec = 0
            
            if not mesh_obj.use_bbox:
                # Show mesh, delete bbox
                mesh_marker.type = Marker.MESH_RESOURCE
                mesh_marker.action = Marker.ADD
                mesh_marker.mesh_resource = "file://" + mesh_obj.get_full_path()
                mesh_marker.pose.position.x = mesh_obj.position[0]
                mesh_marker.pose.position.y = mesh_obj.position[1]
                mesh_marker.pose.position.z = mesh_obj.position[2]
                q = mesh_obj.to_quaternion()
                mesh_marker.pose.orientation.x = q[0]
                mesh_marker.pose.orientation.y = q[1]
                mesh_marker.pose.orientation.z = q[2]
                mesh_marker.pose.orientation.w = q[3]
                mesh_marker.scale.x = mesh_obj.scale[0]
                mesh_marker.scale.y = mesh_obj.scale[1]
                mesh_marker.scale.z = mesh_obj.scale[2]
                mesh_marker.color.r = 0.7
                mesh_marker.color.g = 0.7
                mesh_marker.color.b = 0.7
                mesh_marker.color.a = 1.0
                
                bbox_marker.action = Marker.DELETE
            else:
                # Show bbox, delete mesh
                mesh_marker.action = Marker.DELETE
                
                mesh_obj.load_mesh()  # Ensure bbox is computed
                bbox_size = mesh_obj.get_bbox_size_scaled()
                bbox_center = mesh_obj.get_bbox_center_scaled()
                
                bbox_marker.type = Marker.CUBE
                bbox_marker.action = Marker.ADD
                
                q = mesh_obj.to_quaternion()
                rot = R.from_quat(q)
                rotated_center = rot.apply(bbox_center)
                
                bbox_marker.pose.position.x = mesh_obj.position[0] + rotated_center[0]
                bbox_marker.pose.position.y = mesh_obj.position[1] + rotated_center[1]
                bbox_marker.pose.position.z = mesh_obj.position[2] + rotated_center[2]
                bbox_marker.pose.orientation.x = q[0]
                bbox_marker.pose.orientation.y = q[1]
                bbox_marker.pose.orientation.z = q[2]
                bbox_marker.pose.orientation.w = q[3]
                bbox_marker.scale.x = bbox_size[0]
                bbox_marker.scale.y = bbox_size[1]
                bbox_marker.scale.z = bbox_size[2]
                # Semi-transparent green for bbox
                bbox_marker.color.r = 0.2
                bbox_marker.color.g = 0.8
                bbox_marker.color.b = 0.2
                bbox_marker.color.a = 0.5
            
            marker_array.markers.append(mesh_marker)
            marker_array.markers.append(bbox_marker)
        
        if marker_array.markers:
            self.marker_pub.publish(marker_array)

    def _publish_axes(self):
        """Publish coordinate axis markers."""
        marker_array = MarkerArray()
        
        for obs in self.obstacles.values():
            base_id = (abs(hash(obs.name)) % 100000) * 3
            
            if obs.pose_axis_visible and obs.visible:
                axis_length = max(0.1, max(obs.size) * 0.8)
                q = obs.to_quaternion()
                rot = R.from_quat(q)
                colors = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
                directions = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
                
                for i, (color, direction) in enumerate(zip(colors, directions)):
                    marker = Marker()
                    marker.header.frame_id = obs.base_frame
                    marker.header.stamp.sec = 0
                    marker.header.stamp.nanosec = 0
                    marker.ns = "workspace_pose_axes"
                    marker.id = base_id + i
                    marker.type = Marker.ARROW
                    marker.action = Marker.ADD
                    marker.lifetime.sec = 0
                    marker.lifetime.nanosec = 0
                    
                    start = np.array(obs.position)
                    end = start + rot.apply(direction) * axis_length
                    
                    p1, p2 = Point(), Point()
                    p1.x, p1.y, p1.z = float(start[0]), float(start[1]), float(start[2])
                    p2.x, p2.y, p2.z = float(end[0]), float(end[1]), float(end[2])
                    marker.points = [p1, p2]
                    marker.scale.x = 0.01
                    marker.scale.y = 0.02
                    marker.scale.z = 0.0
                    marker.color.r, marker.color.g, marker.color.b = color
                    marker.color.a = 1.0
                    marker_array.markers.append(marker)
            else:
                for i in range(3):
                    marker = Marker()
                    marker.header.frame_id = "world"
                    marker.header.stamp.sec = 0
                    marker.header.stamp.nanosec = 0
                    marker.ns = "workspace_pose_axes"
                    marker.id = base_id + i
                    marker.action = Marker.DELETE
                    marker_array.markers.append(marker)
        
        if marker_array.markers:
            self.axis_pub.publish(marker_array)

    def transform_obstacle_frame(self, obstacle, new_frame: str):
        if obstacle.base_frame == new_frame:
            return obstacle.position, obstacle.rotation_euler
        try:
            transform = self.tf_buffer.lookup_transform(
                new_frame, obstacle.base_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            t = transform.transform
            trans_rot = R.from_quat([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
            trans_pos = np.array([t.translation.x, t.translation.y, t.translation.z])
            new_pos = trans_rot.apply(np.array(obstacle.position)) + trans_pos
            old_rot = R.from_euler('xyz', np.deg2rad(obstacle.rotation_euler))
            new_euler = np.rad2deg((trans_rot * old_rot).as_euler('xyz')).tolist()
            return new_pos.tolist(), new_euler
        except Exception as e:
            self.get_logger().warn(f"Transform failed: {e}")
            return None


def clamp_value(value: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(max_val, value))


class SliderWithEntry:
    """A slider with an adjacent entry field for direct input."""
    
    def __init__(self, parent, label: str, min_val: float, max_val: float,
                 default: float, resolution: float = 0.01, on_change=None):
        self.min_val = min_val
        self.max_val = max_val
        self.on_change = on_change
        self._updating = False
        
        self.frame = tk.Frame(parent, bg='#2b2b2b')
        self.frame.pack(fill=tk.X, expand=False, pady=2)
        
        self.frame.columnconfigure(0, weight=0, minsize=50)
        self.frame.columnconfigure(1, weight=0, minsize=70)
        self.frame.columnconfigure(2, weight=1, minsize=100)
        
        tk.Label(self.frame, text=label, width=6, bg='#2b2b2b', fg='white',
                 anchor='w').grid(row=0, column=0, sticky='w')
        
        self.entry_var = tk.StringVar(value=f"{default:.3f}")
        self.entry = tk.Entry(self.frame, textvariable=self.entry_var, width=8,
                              bg='#3c3c3c', fg='white', insertbackground='white')
        self.entry.grid(row=0, column=1, padx=(0, 5), sticky='w')
        self.entry.bind('<Return>', self._on_entry_change)
        self.entry.bind('<FocusOut>', self._on_entry_change)
        
        self.slider = tk.Scale(
            self.frame, from_=min_val, to=max_val, resolution=resolution,
            orient=tk.HORIZONTAL, bg='#3c3c3c', fg='white',
            highlightthickness=0, troughcolor='#555555',
            activebackground='#4CAF50', command=self._on_slider_change)
        self.slider.set(default)
        self.slider.grid(row=0, column=2, sticky='ew', padx=(0, 5))

    def _on_slider_change(self, value):
        if self._updating:
            return
        self._updating = True
        try:
            self.entry_var.set(f"{float(value):.3f}")
            if self.on_change:
                self.on_change()
        finally:
            self._updating = False
    
    def _on_entry_change(self, event=None):
        if self._updating:
            return
        self._updating = True
        try:
            val = clamp_value(float(self.entry_var.get()), self.min_val, self.max_val)
            self.slider.set(val)
            self.entry_var.set(f"{val:.3f}")
            if self.on_change:
                self.on_change()
        except ValueError:
            self.entry_var.set(f"{self.slider.get():.3f}")
        finally:
            self._updating = False
    
    def get(self) -> float:
        return self.slider.get()
    
    def set(self, value: float):
        self._updating = True
        try:
            val = clamp_value(value, self.min_val, self.max_val)
            self.slider.set(val)
            self.entry_var.set(f"{val:.3f}")
        finally:
            self._updating = False


class WorkspaceGUI:
    """Tkinter GUI for workspace obstacle management."""
    
    AVAILABLE_FRAMES = ['world', 'AR5_5_07L_base', 'AR5_5_07R_base']
    
    def __init__(self, node: WorkspaceGUINode):
        self.node = node
        self.selected_item: Optional[str] = None
        self.selected_type: Optional[str] = None  # 'box' or 'mesh'
        self.obstacle_counter = 0
        self.mesh_counter = 0
        self._updating_ui = False
        
        self.root = tk.Tk()
        self.root.title("Workspace Manager - MoveIt Scene Editor")
        self.root.geometry("850x850")
        self.root.configure(bg='#2b2b2b')
        self.root.minsize(700, 650)
        
        self._setup_styles()
        self._create_widgets()

    def _setup_styles(self):
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TFrame', background='#2b2b2b')
        style.configure('TLabel', background='#2b2b2b', foreground='white')
        style.configure('TButton', padding=5)
        style.configure('TLabelframe', background='#2b2b2b', foreground='white')
        style.configure('TLabelframe.Label', background='#2b2b2b', foreground='#4CAF50')
        style.configure('TCheckbutton', background='#2b2b2b', foreground='white')
    
    def _create_widgets(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Top buttons
        btn_section = ttk.Frame(main_frame)
        btn_section.pack(fill=tk.X, pady=(0, 10))
        ttk.Button(btn_section, text="Save Config", command=self._save_config).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_section, text="Load Config", command=self._load_config).pack(side=tk.LEFT, padx=5)
        
        # Mesh sample rate control
        sample_frame = tk.Frame(btn_section, bg='#2b2b2b')
        sample_frame.pack(side=tk.LEFT, padx=20)
        tk.Label(sample_frame, text="Mesh Sample %:", bg='#2b2b2b', fg='white').pack(side=tk.LEFT)
        self.sample_rate_var = tk.StringVar(value="100")
        sample_entry = tk.Entry(sample_frame, textvariable=self.sample_rate_var, width=5,
                                bg='#3c3c3c', fg='white', insertbackground='white')
        sample_entry.pack(side=tk.LEFT, padx=2)
        sample_entry.bind('<Return>', self._on_sample_rate_change)
        ttk.Button(sample_frame, text="Apply", command=self._on_sample_rate_change).pack(side=tk.LEFT, padx=2)
        
        ttk.Button(btn_section, text="Clear All", command=self._clear_all).pack(side=tk.RIGHT, padx=5)
        
        paned = tk.PanedWindow(main_frame, orient=tk.HORIZONTAL, bg='#2b2b2b', sashwidth=5)
        paned.pack(fill=tk.BOTH, expand=True)
        
        # Left: Object list
        list_frame = ttk.LabelFrame(paned, text="Objects", padding="5")
        paned.add(list_frame, width=180)
        
        list_container = ttk.Frame(list_frame)
        list_container.pack(fill=tk.BOTH, expand=True)
        scrollbar = ttk.Scrollbar(list_container)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.object_listbox = tk.Listbox(list_container, width=18, height=20,
            bg='#3c3c3c', fg='white', selectbackground='#4CAF50',
            yscrollcommand=scrollbar.set)
        self.object_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.object_listbox.yview)
        self.object_listbox.bind('<<ListboxSelect>>', self._on_select)
        
        btn_frame = ttk.Frame(list_frame)
        btn_frame.pack(fill=tk.X, pady=(5, 0))
        ttk.Button(btn_frame, text="Box", command=self._add_box).pack(side=tk.LEFT, expand=True, fill=tk.X)
        ttk.Button(btn_frame, text="Mesh", command=self._add_mesh).pack(side=tk.LEFT, expand=True, fill=tk.X)
        
        btn_frame2 = ttk.Frame(list_frame)
        btn_frame2.pack(fill=tk.X, pady=(2, 0))
        ttk.Button(btn_frame2, text="Remove", command=self._remove_item).pack(side=tk.LEFT, expand=True, fill=tk.X)
        ttk.Button(btn_frame2, text="Dup", command=self._duplicate_item).pack(side=tk.LEFT, expand=True, fill=tk.X)
        
        # Right: Properties
        props_frame = ttk.LabelFrame(paned, text="Properties", padding="10")
        paned.add(props_frame)
        
        self.props_canvas = tk.Canvas(props_frame, bg='#2b2b2b', highlightthickness=0)
        scrollbar2 = ttk.Scrollbar(props_frame, orient="vertical", command=self.props_canvas.yview)
        self.props_inner = tk.Frame(self.props_canvas, bg='#2b2b2b')
        self.props_inner.bind("<Configure>", lambda e: self.props_canvas.configure(scrollregion=self.props_canvas.bbox("all")))
        self.canvas_window = self.props_canvas.create_window((0, 0), window=self.props_inner, anchor="nw")
        self.props_canvas.configure(yscrollcommand=scrollbar2.set)
        self.props_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar2.pack(side=tk.RIGHT, fill=tk.Y)
        
        def on_canvas_configure(event):
            self.props_canvas.itemconfig(self.canvas_window, width=event.width - 10)
        self.props_canvas.bind('<Configure>', on_canvas_configure)
        
        self._create_property_widgets()

    def _create_property_widgets(self):
        p = self.props_inner
        
        # Type indicator
        f = tk.LabelFrame(p, text="Type", bg='#2b2b2b', fg='#4CAF50', padx=5, pady=5)
        f.pack(fill=tk.X, pady=5, padx=5)
        self.type_label = tk.Label(f, text="None", bg='#2b2b2b', fg='white')
        self.type_label.pack(fill=tk.X, padx=5)
        
        # Name
        f = tk.LabelFrame(p, text="Name", bg='#2b2b2b', fg='#4CAF50', padx=5, pady=5)
        f.pack(fill=tk.X, pady=5, padx=5)
        self.name_var = tk.StringVar()
        tk.Entry(f, textvariable=self.name_var, width=20, bg='#3c3c3c', fg='white').pack(side=tk.LEFT, padx=5)
        tk.Button(f, text="Rename", command=self._apply_rename, bg='#3c3c3c', fg='white').pack(side=tk.LEFT)
        
        # Mesh path (only for mesh)
        self.mesh_path_frame = tk.LabelFrame(p, text="Mesh File", bg='#2b2b2b', fg='#4CAF50', padx=5, pady=5)
        self.mesh_path_frame.pack(fill=tk.X, pady=5, padx=5)
        self.mesh_path_var = tk.StringVar()
        tk.Entry(self.mesh_path_frame, textvariable=self.mesh_path_var, width=30, bg='#3c3c3c', fg='white', state='readonly').pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        # Base Frame
        f = tk.LabelFrame(p, text="Base Frame", bg='#2b2b2b', fg='#4CAF50', padx=5, pady=5)
        f.pack(fill=tk.X, pady=5, padx=5)
        self.frame_var = tk.StringVar(value='world')
        combo = ttk.Combobox(f, textvariable=self.frame_var, values=self.AVAILABLE_FRAMES, state='readonly', width=18)
        combo.pack(side=tk.LEFT, padx=5)
        combo.bind('<<ComboboxSelected>>', self._on_frame_change)
        
        # Position
        f = tk.LabelFrame(p, text="Position (m)", bg='#2b2b2b', fg='#4CAF50', padx=5, pady=5)
        f.pack(fill=tk.X, pady=5, padx=5)
        self.pos_x = SliderWithEntry(f, "X:", -3.0, 3.0, 0.5, 0.01, self._on_property_change)
        self.pos_y = SliderWithEntry(f, "Y:", -3.0, 3.0, 0.0, 0.01, self._on_property_change)
        self.pos_z = SliderWithEntry(f, "Z:", -1.0, 3.0, 0.5, 0.01, self._on_property_change)
        
        # Size (only for box)
        self.size_frame = tk.LabelFrame(p, text="Size (m)", bg='#2b2b2b', fg='#4CAF50', padx=5, pady=5)
        self.size_frame.pack(fill=tk.X, pady=5, padx=5)
        self.size_x = SliderWithEntry(self.size_frame, "X:", 0.01, 2.0, 0.1, 0.01, self._on_property_change)
        self.size_y = SliderWithEntry(self.size_frame, "Y:", 0.01, 2.0, 0.1, 0.01, self._on_property_change)
        self.size_z = SliderWithEntry(self.size_frame, "Z:", 0.01, 2.0, 0.1, 0.01, self._on_property_change)
        
        # Rotation
        f = tk.LabelFrame(p, text="Rotation (deg)", bg='#2b2b2b', fg='#4CAF50', padx=5, pady=5)
        f.pack(fill=tk.X, pady=5, padx=5)
        self.rot_roll = SliderWithEntry(f, "Roll:", -180, 180, 0, 1, self._on_property_change)
        self.rot_pitch = SliderWithEntry(f, "Pitch:", -180, 180, 0, 1, self._on_property_change)
        self.rot_yaw = SliderWithEntry(f, "Yaw:", -180, 180, 0, 1, self._on_property_change)

        # Quaternion
        f = tk.LabelFrame(p, text="Quaternion (read-only)", bg='#2b2b2b', fg='#4CAF50', padx=5, pady=5)
        f.pack(fill=tk.X, pady=5, padx=5)
        self.quat_label = tk.Label(f, text="x: 0.000  y: 0.000  z: 0.000  w: 1.000", bg='#2b2b2b', fg='white')
        self.quat_label.pack(fill=tk.X, padx=5)
        
        # Color (only for box)
        self.color_frame = tk.LabelFrame(p, text="Color (RGBA)", bg='#2b2b2b', fg='#4CAF50', padx=5, pady=5)
        self.color_frame.pack(fill=tk.X, pady=5, padx=5)
        self.color_r = SliderWithEntry(self.color_frame, "R:", 0, 1, 0.8, 0.05, self._on_property_change)
        self.color_g = SliderWithEntry(self.color_frame, "G:", 0, 1, 0.2, 0.05, self._on_property_change)
        self.color_b = SliderWithEntry(self.color_frame, "B:", 0, 1, 0.2, 0.05, self._on_property_change)
        self.color_a = SliderWithEntry(self.color_frame, "A:", 0, 1, 0.7, 0.05, self._on_property_change)
        
        # Options
        f = tk.LabelFrame(p, text="Options", bg='#2b2b2b', fg='#4CAF50', padx=5, pady=5)
        f.pack(fill=tk.X, pady=5, padx=5)
        self.collision_var = tk.BooleanVar(value=True)
        tk.Checkbutton(f, text="Enable Collision (include in planning)", variable=self.collision_var,
                       command=self._on_property_change, bg='#2b2b2b', fg='white',
                       selectcolor='#3c3c3c', activebackground='#2b2b2b').pack(anchor=tk.W, pady=2)
        
        # Box-only options frame
        self.box_options_frame = tk.Frame(f, bg='#2b2b2b')
        self.box_options_frame.pack(fill=tk.X)
        self.visible_var = tk.BooleanVar(value=True)
        tk.Checkbutton(self.box_options_frame, text="Visible (show marker in RViz)", variable=self.visible_var,
                       command=self._on_property_change, bg='#2b2b2b', fg='white',
                       selectcolor='#3c3c3c', activebackground='#2b2b2b').pack(anchor=tk.W, pady=2)
        self.axis_var = tk.BooleanVar(value=False)
        tk.Checkbutton(self.box_options_frame, text="Show Pose Axes", variable=self.axis_var,
                       command=self._on_property_change, bg='#2b2b2b', fg='white',
                       selectcolor='#3c3c3c', activebackground='#2b2b2b').pack(anchor=tk.W, pady=2)
        
        # Mesh-only options frame
        self.mesh_options_frame = tk.Frame(f, bg='#2b2b2b')
        self.mesh_options_frame.pack(fill=tk.X)
        self.use_bbox_var = tk.BooleanVar(value=False)
        tk.Checkbutton(self.mesh_options_frame, text="Use Bounding Box (faster positioning)", 
                       variable=self.use_bbox_var,
                       command=self._on_property_change, bg='#2b2b2b', fg='white',
                       selectcolor='#3c3c3c', activebackground='#2b2b2b').pack(anchor=tk.W, pady=2)
        
        # Attach section
        attach_frame = tk.LabelFrame(p, text="Attach to Arm", bg='#2b2b2b', fg='#4CAF50', padx=5, pady=5)
        attach_frame.pack(fill=tk.X, pady=5, padx=5)
        
        # Attach buttons row
        attach_btn_frame = tk.Frame(attach_frame, bg='#2b2b2b')
        attach_btn_frame.pack(fill=tk.X, pady=2)
        
        self.attach_left_btn = tk.Button(attach_btn_frame, text="Attach Left", 
                                          command=lambda: self._on_attach('left'),
                                          bg='#3c3c3c', fg='white', width=12)
        self.attach_left_btn.pack(side=tk.LEFT, padx=2)
        
        self.attach_right_btn = tk.Button(attach_btn_frame, text="Attach Right",
                                           command=lambda: self._on_attach('right'),
                                           bg='#3c3c3c', fg='white', width=12)
        self.attach_right_btn.pack(side=tk.LEFT, padx=2)
        
        self.detach_btn = tk.Button(attach_btn_frame, text="Detach",
                                     command=self._on_detach,
                                     bg='#3c3c3c', fg='white', width=12)
        self.detach_btn.pack(side=tk.LEFT, padx=2)
        
        # Diagnostic buttons row
        diag_btn_frame = tk.Frame(attach_frame, bg='#2b2b2b')
        diag_btn_frame.pack(fill=tk.X, pady=2)
        
        self.diag_btn = tk.Button(diag_btn_frame, text="Check MoveIt State",
                                   command=self._on_check_moveit_state,
                                   bg='#555555', fg='white', width=18)
        self.diag_btn.pack(side=tk.LEFT, padx=2)
        
        self.force_clear_btn = tk.Button(diag_btn_frame, text="Force Clear All",
                                          command=self._on_force_clear_attached,
                                          bg='#8B0000', fg='white', width=18)
        self.force_clear_btn.pack(side=tk.LEFT, padx=2)
        
        # Attach status label
        self.attach_status_var = tk.StringVar(value="Status: Not Attached")
        self.attach_status_label = tk.Label(attach_frame, textvariable=self.attach_status_var,
                                            bg='#2b2b2b', fg='#aaaaaa')
        self.attach_status_label.pack(anchor=tk.W, pady=2)
        
        # MoveIt state display
        self.moveit_state_var = tk.StringVar(value="MoveIt: (click 'Check MoveIt State')")
        self.moveit_state_label = tk.Label(attach_frame, textvariable=self.moveit_state_var,
                                            bg='#2b2b2b', fg='#888888', wraplength=350, justify=tk.LEFT)
        self.moveit_state_label.pack(anchor=tk.W, pady=2)
    
    def _update_ui_for_type(self, obj_type: str):
        """Show/hide UI elements based on object type."""
        if obj_type == 'box':
            self.type_label.config(text="Box")
            self.mesh_path_frame.pack_forget()
            # Re-pack size and color frames
            self.size_frame.pack_forget()
            self.color_frame.pack_forget()
            self.size_frame.pack(fill=tk.X, pady=5, padx=5)
            self.color_frame.pack(fill=tk.X, pady=5, padx=5)
            self.box_options_frame.pack(fill=tk.X)
            self.mesh_options_frame.pack_forget()
        elif obj_type == 'mesh':
            self.type_label.config(text="Mesh")
            self.mesh_path_frame.pack(fill=tk.X, pady=5, padx=5)
            self.size_frame.pack_forget()
            self.color_frame.pack_forget()
            self.box_options_frame.pack_forget()
            self.mesh_options_frame.pack(fill=tk.X)
        else:
            self.type_label.config(text="None")

    def _on_select(self, event):
        selection = self.object_listbox.curselection()
        if not selection:
            return
        display_name = self.object_listbox.get(selection[0])
        # Parse type prefix and remove attach suffix
        if display_name.startswith("[B] "):
            name = display_name[4:]
            # Remove attach suffix if present
            if name.endswith(" [L]") or name.endswith(" [R]"):
                name = name[:-4]
            self.selected_type = 'box'
            self.selected_item = name
            if name in self.node.obstacles:
                self._load_box_to_ui(self.node.obstacles[name])
        elif display_name.startswith("[M] "):
            name = display_name[4:]
            # Remove attach suffix if present
            if name.endswith(" [L]") or name.endswith(" [R]"):
                name = name[:-4]
            self.selected_type = 'mesh'
            self.selected_item = name
            if name in self.node.meshes:
                self._load_mesh_to_ui(self.node.meshes[name])
    
    def _load_box_to_ui(self, obs: WorkspaceObstacle):
        self._updating_ui = True
        try:
            self._update_ui_for_type('box')
            self.name_var.set(obs.name)
            self.frame_var.set(obs.base_frame)
            self.pos_x.set(obs.position[0])
            self.pos_y.set(obs.position[1])
            self.pos_z.set(obs.position[2])
            self.size_x.set(obs.size[0])
            self.size_y.set(obs.size[1])
            self.size_z.set(obs.size[2])
            self.rot_roll.set(obs.rotation_euler[0])
            self.rot_pitch.set(obs.rotation_euler[1])
            self.rot_yaw.set(obs.rotation_euler[2])
            self.color_r.set(obs.color[0])
            self.color_g.set(obs.color[1])
            self.color_b.set(obs.color[2])
            self.color_a.set(obs.color[3])
            self.collision_var.set(obs.collision_enabled)
            self.visible_var.set(obs.visible)
            self.axis_var.set(obs.pose_axis_visible)
            q = obs.to_quaternion()
            self.quat_label.config(text=f"x: {q[0]:.4f}  y: {q[1]:.4f}  z: {q[2]:.4f}  w: {q[3]:.4f}")
            self._update_attach_status(obs.attached_to)
        finally:
            self._updating_ui = False
    
    def _load_mesh_to_ui(self, mesh_obj: WorkspaceMesh):
        self._updating_ui = True
        try:
            self._update_ui_for_type('mesh')
            self.name_var.set(mesh_obj.name)
            self.mesh_path_var.set(mesh_obj.mesh_path)
            self.frame_var.set(mesh_obj.base_frame)
            self.pos_x.set(mesh_obj.position[0])
            self.pos_y.set(mesh_obj.position[1])
            self.pos_z.set(mesh_obj.position[2])
            self.rot_roll.set(mesh_obj.rotation_euler[0])
            self.rot_pitch.set(mesh_obj.rotation_euler[1])
            self.rot_yaw.set(mesh_obj.rotation_euler[2])
            self.collision_var.set(mesh_obj.collision_enabled)
            self.use_bbox_var.set(mesh_obj.use_bbox)
            q = mesh_obj.to_quaternion()
            self.quat_label.config(text=f"x: {q[0]:.4f}  y: {q[1]:.4f}  z: {q[2]:.4f}  w: {q[3]:.4f}")
            self._update_attach_status(mesh_obj.attached_to)
        finally:
            self._updating_ui = False

    def _update_attach_status(self, attached_to: Optional[str]):
        """Update the attach status display and button states."""
        if attached_to is None:
            self.attach_status_var.set("Status: Not Attached")
            self.attach_status_label.config(fg='#aaaaaa')
            self.attach_left_btn.config(state=tk.NORMAL)
            self.attach_right_btn.config(state=tk.NORMAL)
            self.detach_btn.config(state=tk.DISABLED)
        elif attached_to == 'left':
            self.attach_status_var.set("Status: Attached to LEFT arm (purple in RViz)")
            self.attach_status_label.config(fg='#9932CC')  # Purple
            self.attach_left_btn.config(state=tk.DISABLED)
            self.attach_right_btn.config(state=tk.DISABLED)
            self.detach_btn.config(state=tk.NORMAL)
        elif attached_to == 'right':
            self.attach_status_var.set("Status: Attached to RIGHT arm (purple in RViz)")
            self.attach_status_label.config(fg='#9932CC')  # Purple
            self.attach_left_btn.config(state=tk.DISABLED)
            self.attach_right_btn.config(state=tk.DISABLED)
            self.detach_btn.config(state=tk.NORMAL)

    def _on_attach(self, arm: str):
        """Handle attach button click."""
        if not self.selected_item:
            messagebox.showwarning("Warning", "Please select an object first")
            return
        
        obj = self.node.obstacles.get(self.selected_item) or self.node.meshes.get(self.selected_item)
        if obj is None:
            return
        
        if obj.attached_to is not None:
            messagebox.showwarning("Warning", f"Object is already attached to {obj.attached_to} arm")
            return
        
        # Check if arm already has an attached object
        for obs in self.node.obstacles.values():
            if obs.attached_to == arm:
                messagebox.showwarning("Warning", f"{arm.capitalize()} arm already has attached object: {obs.name}\nDetach it first.")
                return
        for mesh in self.node.meshes.values():
            if mesh.attached_to == arm:
                messagebox.showwarning("Warning", f"{arm.capitalize()} arm already has attached mesh: {mesh.name}\nDetach it first.")
                return
        
        # Disable buttons during operation
        self.attach_left_btn.config(state=tk.DISABLED)
        self.attach_right_btn.config(state=tk.DISABLED)
        self.root.update()
        
        success = self.node.attach_object(self.selected_item, arm)
        
        if success:
            self._update_attach_status(arm)
            # Update listbox display
            self._update_listbox_item_display()
        else:
            messagebox.showerror("Error", f"Failed to attach object to {arm} arm.\nMake sure MoveIt is running.")
            self._update_attach_status(None)

    def _on_detach(self):
        """Handle detach button click."""
        if not self.selected_item:
            return
        
        obj = self.node.obstacles.get(self.selected_item) or self.node.meshes.get(self.selected_item)
        if obj is None or obj.attached_to is None:
            return
        
        # Disable button during operation
        self.detach_btn.config(state=tk.DISABLED)
        self.root.update()
        
        success = self.node.detach_object(self.selected_item)
        
        if success:
            self._update_attach_status(None)
            # Update listbox display
            self._update_listbox_item_display()
            # Reload UI to show updated position (object position may have changed)
            if obj.obj_type == 'box':
                self._load_box_to_ui(obj)
            else:
                self._load_mesh_to_ui(obj)
            # Update MoveIt state display
            self._on_check_moveit_state()
        else:
            messagebox.showerror("Error", "Failed to detach object.\nCheck terminal for detailed logs.")
            self._update_attach_status(obj.attached_to)
            # Update MoveIt state display
            self._on_check_moveit_state()

    def _on_check_moveit_state(self):
        """Check and display current MoveIt planning scene state."""
        state = self.node.get_planning_scene_state()
        if state:
            attached_str = ", ".join(state['attached_objects']) if state['attached_objects'] else "(none)"
            world_count = len(state['world_objects'])
            self.moveit_state_var.set(f"MoveIt Attached: {attached_str}\nWorld objects: {world_count}")
            if state['attached_objects']:
                self.moveit_state_label.config(fg='#FF6600')  # Orange warning
            else:
                self.moveit_state_label.config(fg='#00FF00')  # Green OK
        else:
            self.moveit_state_var.set("MoveIt: Failed to query state")
            self.moveit_state_label.config(fg='#FF0000')

    def _on_force_clear_attached(self):
        """Force clear all attached objects from MoveIt."""
        if not messagebox.askyesno("Confirm", 
            "This will force-clear ALL attached objects from MoveIt.\n"
            "Use this if detach is not working.\n\nProceed?"):
            return
        
        self.node.get_logger().info("[FORCE CLEAR] Starting force clear of all attached objects...")
        
        # Get current state
        state = self.node.get_planning_scene_state()
        if not state:
            messagebox.showerror("Error", "Failed to get MoveIt state")
            return
        
        self.node.get_logger().info(f"[FORCE CLEAR] Current attached: {state['attached_objects']}")
        
        if not state['attached_objects']:
            messagebox.showinfo("Info", "No attached objects to clear")
            return
        
        timeout = 5.0
        
        # Method 1: Try diff-based removal with is_diff=True
        self.node.get_logger().info("[FORCE CLEAR] Method 1: Diff-based removal...")
        for obj_id, link_name in state['attached_details']:
            self.node.get_logger().info(f"[FORCE CLEAR] Removing {obj_id} from {link_name}...")
            
            scene = PlanningScene()
            scene.is_diff = True
            scene.robot_state.is_diff = True  # CRITICAL: Must be True
            
            attached = AttachedCollisionObject()
            attached.link_name = link_name
            attached.object.id = obj_id
            attached.object.header.frame_id = "world"
            attached.object.header.stamp = self.node.get_clock().now().to_msg()
            attached.object.operation = CollisionObject.REMOVE
            
            scene.robot_state.attached_collision_objects.append(attached)
            
            req = ApplyPlanningScene.Request()
            req.scene = scene
            future = self.node.apply_scene_client.call_async(req)
            
            start = time_module.time()
            while not future.done() and (time_module.time() - start) < timeout:
                time_module.sleep(0.1)
            
            if future.done() and future.result():
                self.node.get_logger().info(f"[FORCE CLEAR] Remove {obj_id}: success={future.result().success}")
        
        time_module.sleep(0.3)
        
        # Check if method 1 worked
        mid_state = self.node.get_planning_scene_state()
        if mid_state and mid_state['attached_objects']:
            self.node.get_logger().info(f"[FORCE CLEAR] Method 1 incomplete, still attached: {mid_state['attached_objects']}")
            
            # Method 2: Get full scene and re-apply without attached objects
            self.node.get_logger().info("[FORCE CLEAR] Method 2: Full scene replacement...")
            
            get_req = GetPlanningScene.Request()
            get_req.components.components = (
                PlanningSceneComponents.SCENE_SETTINGS |
                PlanningSceneComponents.ROBOT_STATE |
                PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS |
                PlanningSceneComponents.WORLD_OBJECT_NAMES |
                PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
                PlanningSceneComponents.TRANSFORMS |
                PlanningSceneComponents.ALLOWED_COLLISION_MATRIX |
                PlanningSceneComponents.LINK_PADDING_AND_SCALING |
                PlanningSceneComponents.OBJECT_COLORS
            )
            
            get_future = self.node.get_scene_client.call_async(get_req)
            start = time_module.time()
            while not get_future.done() and (time_module.time() - start) < timeout:
                time_module.sleep(0.1)
            
            if get_future.done() and get_future.result() is not None:
                full_scene = get_future.result().scene
                self.node.get_logger().info(f"[FORCE CLEAR] Got full scene, clearing {len(full_scene.robot_state.attached_collision_objects)} attached objects")
                
                # Clear all attached objects
                full_scene.robot_state.attached_collision_objects = []
                full_scene.is_diff = False
                full_scene.robot_state.is_diff = False
                
                req2 = ApplyPlanningScene.Request()
                req2.scene = full_scene
                future2 = self.node.apply_scene_client.call_async(req2)
                
                start = time_module.time()
                while not future2.done() and (time_module.time() - start) < timeout:
                    time_module.sleep(0.1)
                
                if future2.done() and future2.result():
                    self.node.get_logger().info(f"[FORCE CLEAR] Full scene apply: success={future2.result().success}")
        
        time_module.sleep(0.3)
        
        # Verify
        final_state = self.node.get_planning_scene_state()
        self.node.get_logger().info(f"[FORCE CLEAR] Final attached: {final_state['attached_objects'] if final_state else 'unknown'}")
        
        # Update local state
        for obs in self.node.obstacles.values():
            obs.attached_to = None
        for mesh in self.node.meshes.values():
            mesh.attached_to = None
        
        # Update UI
        self._on_check_moveit_state()
        
        if final_state and not final_state['attached_objects']:
            messagebox.showinfo("Success", "All attached objects cleared!")
        else:
            messagebox.showwarning("Warning", 
                f"Force clear completed but MoveIt still reports attached objects:\n"
                f"{final_state['attached_objects'] if final_state else 'unknown'}\n\n"
                "This may indicate a MoveIt bug or configuration issue.")

    def _update_listbox_item_display(self):
        """Update the listbox item to show attach status."""
        if not self.selected_item:
            return
        
        selection = self.object_listbox.curselection()
        if not selection:
            return
        
        obj = self.node.obstacles.get(self.selected_item) or self.node.meshes.get(self.selected_item)
        if obj is None:
            return
        
        prefix = "[B] " if obj.obj_type == 'box' else "[M] "
        suffix = ""
        if obj.attached_to == 'left':
            suffix = " [L]"
        elif obj.attached_to == 'right':
            suffix = " [R]"
        
        display_name = prefix + obj.name + suffix
        
        idx = selection[0]
        self.object_listbox.delete(idx)
        self.object_listbox.insert(idx, display_name)
        self.object_listbox.selection_set(idx)

    def _on_property_change(self):
        if self._updating_ui or not self.selected_item:
            return
        
        if self.selected_type == 'box' and self.selected_item in self.node.obstacles:
            obs = self.node.obstacles[self.selected_item]
            obs.position = [self.pos_x.get(), self.pos_y.get(), self.pos_z.get()]
            obs.size = [self.size_x.get(), self.size_y.get(), self.size_z.get()]
            obs.rotation_euler = [self.rot_roll.get(), self.rot_pitch.get(), self.rot_yaw.get()]
            obs.color = [self.color_r.get(), self.color_g.get(), self.color_b.get(), self.color_a.get()]
            obs.collision_enabled = self.collision_var.get()
            obs.visible = self.visible_var.get()
            obs.pose_axis_visible = self.axis_var.get()
            q = obs.to_quaternion()
            self.quat_label.config(text=f"x: {q[0]:.4f}  y: {q[1]:.4f}  z: {q[2]:.4f}  w: {q[3]:.4f}")
            # Publish change to MoveIt (event-driven)
            self.node.publish_obstacle(obs)
        
        elif self.selected_type == 'mesh' and self.selected_item in self.node.meshes:
            mesh_obj = self.node.meshes[self.selected_item]
            mesh_obj.position = [self.pos_x.get(), self.pos_y.get(), self.pos_z.get()]
            mesh_obj.rotation_euler = [self.rot_roll.get(), self.rot_pitch.get(), self.rot_yaw.get()]
            mesh_obj.collision_enabled = self.collision_var.get()
            mesh_obj.use_bbox = self.use_bbox_var.get()
            q = mesh_obj.to_quaternion()
            self.quat_label.config(text=f"x: {q[0]:.4f}  y: {q[1]:.4f}  z: {q[2]:.4f}  w: {q[3]:.4f}")
            # Publish change to MoveIt (event-driven)
            self.node.publish_obstacle(mesh_obj)
    
    def _on_frame_change(self, event=None):
        if self._updating_ui or not self.selected_item:
            return
        
        new_frame = self.frame_var.get()
        
        if self.selected_type == 'box' and self.selected_item in self.node.obstacles:
            obs = self.node.obstacles[self.selected_item]
            if obs.base_frame == new_frame:
                return
            result = self.node.transform_obstacle_frame(obs, new_frame)
            if result:
                obs.position, obs.rotation_euler = result
                obs.base_frame = new_frame
                self._load_box_to_ui(obs)
            else:
                self._updating_ui = True
                self.frame_var.set(obs.base_frame)
                self._updating_ui = False
                messagebox.showwarning("Transform Failed", f"Could not transform to frame '{new_frame}'.")
        
        elif self.selected_type == 'mesh' and self.selected_item in self.node.meshes:
            mesh_obj = self.node.meshes[self.selected_item]
            if mesh_obj.base_frame == new_frame:
                return
            result = self.node.transform_obstacle_frame(mesh_obj, new_frame)
            if result:
                mesh_obj.position, mesh_obj.rotation_euler = result
                mesh_obj.base_frame = new_frame
                self._load_mesh_to_ui(mesh_obj)
            else:
                self._updating_ui = True
                self.frame_var.set(mesh_obj.base_frame)
                self._updating_ui = False
                messagebox.showwarning("Transform Failed", f"Could not transform to frame '{new_frame}'.")

    def _apply_rename(self):
        if not self.selected_item:
            return
        new_name = self.name_var.get().strip()
        if not new_name or new_name == self.selected_item:
            return
        
        # Check name collision
        if new_name in self.node.obstacles or new_name in self.node.meshes:
            messagebox.showerror("Error", f"Name '{new_name}' already exists")
            return
        
        if self.selected_type == 'box' and self.selected_item in self.node.obstacles:
            old_frame = self.node.obstacles[self.selected_item].base_frame
            # Remove old name from MoveIt
            self.node.remove_obstacle_from_scene(self.selected_item, old_frame)
            obs = self.node.obstacles.pop(self.selected_item)
            obs.name = new_name
            self.node.obstacles[new_name] = obs
            # Publish with new name
            self.node.publish_obstacle(obs)
        elif self.selected_type == 'mesh' and self.selected_item in self.node.meshes:
            old_frame = self.node.meshes[self.selected_item].base_frame
            # Remove old name from MoveIt
            self.node.remove_obstacle_from_scene(self.selected_item, old_frame)
            mesh_obj = self.node.meshes.pop(self.selected_item)
            mesh_obj.name = new_name
            self.node.meshes[new_name] = mesh_obj
            # Publish with new name
            self.node.publish_obstacle(mesh_obj)
        
        # Update listbox
        selection = self.object_listbox.curselection()
        if selection:
            prefix = "[B] " if self.selected_type == 'box' else "[M] "
            self.object_listbox.delete(selection[0])
            self.object_listbox.insert(selection[0], prefix + new_name)
            self.object_listbox.selection_set(selection[0])
        self.selected_item = new_name
    
    def _add_box(self):
        self.obstacle_counter += 1
        name = f"box_{self.obstacle_counter}"
        while name in self.node.obstacles or name in self.node.meshes:
            self.obstacle_counter += 1
            name = f"box_{self.obstacle_counter}"
        obs = WorkspaceObstacle(name=name, position=[0.5, 0.0, 0.5], size=[0.1, 0.1, 0.1])
        self.node.obstacles[name] = obs
        # Publish new obstacle to MoveIt
        self.node.publish_obstacle(obs)
        self.object_listbox.insert(tk.END, "[B] " + name)
        self.object_listbox.selection_clear(0, tk.END)
        self.object_listbox.selection_set(tk.END)
        self.object_listbox.see(tk.END)
        self.selected_item = name
        self.selected_type = 'box'
        self._load_box_to_ui(obs)

    def _add_mesh(self):
        # Open file dialog to select mesh
        filepath = filedialog.askopenfilename(
            title="Select Mesh File",
            filetypes=[("STL files", "*.stl"), ("STL files", "*.STL")],
            initialdir=MESH_DIR)
        if not filepath:
            return
        
        # Check if file is in MESH_DIR
        if not filepath.startswith(MESH_DIR):
            messagebox.showwarning("Warning", f"Mesh file should be in:\n{MESH_DIR}\n\nFile will be referenced by absolute path.")
            rel_path = filepath
        else:
            rel_path = os.path.relpath(filepath, MESH_DIR)
        
        self.mesh_counter += 1
        name = f"mesh_{self.mesh_counter}"
        while name in self.node.obstacles or name in self.node.meshes:
            self.mesh_counter += 1
            name = f"mesh_{self.mesh_counter}"
        
        # New mesh: default to showing mesh (use_bbox=False)
        mesh_obj = WorkspaceMesh(name=name, mesh_path=rel_path, position=[0.0, 0.0, 0.0],
                                  use_bbox=False)
        # Pre-load mesh to compute bounding box
        mesh_obj.load_mesh()
        self.node.meshes[name] = mesh_obj
        # Publish new mesh to MoveIt
        self.node.publish_obstacle(mesh_obj)
        self.object_listbox.insert(tk.END, "[M] " + name)
        self.object_listbox.selection_clear(0, tk.END)
        self.object_listbox.selection_set(tk.END)
        self.object_listbox.see(tk.END)
        self.selected_item = name
        self.selected_type = 'mesh'
        self._load_mesh_to_ui(mesh_obj)
    
    def _remove_item(self):
        if not self.selected_item:
            return
        
        name = self.selected_item
        
        # Get the object to check if it's attached
        obj = self.node.obstacles.get(name) or self.node.meshes.get(name)
        if obj is None:
            return
        
        # If object is attached, detach it first
        if obj.attached_to is not None:
            self.node.get_logger().info(f"[REMOVE] Object {name} is attached, detaching first...")
            success = self.node.detach_object(name)
            if not success:
                messagebox.showerror("Error", f"Failed to detach {name} before removal")
                return
        
        if self.selected_type == 'box' and name in self.node.obstacles:
            frame = self.node.obstacles[name].base_frame
            del self.node.obstacles[name]
            # Remove from MoveIt via service
            self.node.remove_obstacle_from_scene(name, frame)
        elif self.selected_type == 'mesh' and name in self.node.meshes:
            frame = self.node.meshes[name].base_frame
            del self.node.meshes[name]
            # Remove from MoveIt via service
            self.node.remove_obstacle_from_scene(name, frame)
        
        selection = self.object_listbox.curselection()
        if selection:
            self.object_listbox.delete(selection[0])
        self.selected_item = None
        self.selected_type = None

    def _duplicate_item(self):
        if not self.selected_item:
            return
        
        if self.selected_type == 'box' and self.selected_item in self.node.obstacles:
            src = self.node.obstacles[self.selected_item]
            self.obstacle_counter += 1
            new_name = f"{src.name}_copy_{self.obstacle_counter}"
            while new_name in self.node.obstacles or new_name in self.node.meshes:
                self.obstacle_counter += 1
                new_name = f"{src.name}_copy_{self.obstacle_counter}"
            new_obs = WorkspaceObstacle(
                name=new_name, position=[src.position[0] + 0.1, src.position[1], src.position[2]],
                size=list(src.size), rotation_euler=list(src.rotation_euler), color=list(src.color),
                base_frame=src.base_frame, collision_enabled=src.collision_enabled,
                visible=src.visible, pose_axis_visible=src.pose_axis_visible)
            self.node.obstacles[new_name] = new_obs
            # Publish duplicated obstacle to MoveIt
            self.node.publish_obstacle(new_obs)
            self.object_listbox.insert(tk.END, "[B] " + new_name)
            self.object_listbox.selection_clear(0, tk.END)
            self.object_listbox.selection_set(tk.END)
            self.selected_item = new_name
            self._load_box_to_ui(new_obs)
        
        elif self.selected_type == 'mesh' and self.selected_item in self.node.meshes:
            src = self.node.meshes[self.selected_item]
            self.mesh_counter += 1
            new_name = f"{src.name}_copy_{self.mesh_counter}"
            while new_name in self.node.obstacles or new_name in self.node.meshes:
                self.mesh_counter += 1
                new_name = f"{src.name}_copy_{self.mesh_counter}"
            new_mesh = WorkspaceMesh(
                name=new_name, mesh_path=src.mesh_path,
                position=[src.position[0] + 0.1, src.position[1], src.position[2]],
                rotation_euler=list(src.rotation_euler), scale=list(src.scale),
                base_frame=src.base_frame, collision_enabled=src.collision_enabled)
            self.node.meshes[new_name] = new_mesh
            # Publish duplicated mesh to MoveIt
            self.node.publish_obstacle(new_mesh)
            self.object_listbox.insert(tk.END, "[M] " + new_name)
            self.object_listbox.selection_clear(0, tk.END)
            self.object_listbox.selection_set(tk.END)
            self.selected_item = new_name
            self._load_mesh_to_ui(new_mesh)
    
    def _clear_all(self):
        if not self.node.obstacles and not self.node.meshes:
            return
        if messagebox.askyesno("Confirm", "Remove all objects?"):
            for name, obs in list(self.node.obstacles.items()):
                self.node.remove_obstacle_from_scene(name, obs.base_frame)
            for name, mesh_obj in list(self.node.meshes.items()):
                self.node.remove_obstacle_from_scene(name, mesh_obj.base_frame)
            self.node.obstacles.clear()
            self.node.meshes.clear()
            self.object_listbox.delete(0, tk.END)
            self.selected_item = None
            self.selected_type = None
    
    def _on_sample_rate_change(self, event=None):
        """Handle mesh sample rate change."""
        global MESH_SAMPLE_RATE
        try:
            rate_percent = float(self.sample_rate_var.get())
            rate = max(1, min(100, rate_percent)) / 100.0  # Convert to 0.0-1.0
            self.sample_rate_var.set(str(int(rate * 100)))
            
            if rate != MESH_SAMPLE_RATE:
                MESH_SAMPLE_RATE = rate
                # Force reload all meshes with new sample rate
                for mesh_obj in self.node.meshes.values():
                    mesh_obj.load_mesh(force_reload=True)
                self.node.get_logger().info(f"Mesh sample rate changed to {int(rate * 100)}%")
        except ValueError:
            self.sample_rate_var.set(str(int(MESH_SAMPLE_RATE * 100)))

    def _get_config_dir(self):
        """Get config directory path."""
        return os.path.join(os.path.dirname(MESH_DIR), 'config')
    
    def _save_config(self):
        filepath = filedialog.asksaveasfilename(
            defaultextension=".json", filetypes=[("JSON files", "*.json")],
            initialdir=self._get_config_dir())
        if not filepath:
            return
        
        data = {
            'obstacles': [obs.to_dict() for obs in self.node.obstacles.values()],
            'meshes': [mesh_obj.to_dict() for mesh_obj in self.node.meshes.values()]
        }
        try:
            with open(filepath, 'w') as f:
                json.dump(data, f, indent=2)
            messagebox.showinfo("Saved", f"Config saved to:\n{filepath}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save: {e}")
    
    def _load_config(self):
        filepath = filedialog.askopenfilename(
            filetypes=[("JSON files", "*.json")],
            initialdir=self._get_config_dir())
        if not filepath:
            return
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            
            # Clear existing
            for name, obs in list(self.node.obstacles.items()):
                self.node.remove_obstacle_from_scene(name, obs.base_frame)
            for name, mesh_obj in list(self.node.meshes.items()):
                self.node.remove_obstacle_from_scene(name, mesh_obj.base_frame)
            self.node.obstacles.clear()
            self.node.meshes.clear()
            self.object_listbox.delete(0, tk.END)
            
            # Load boxes
            for obs_data in data.get('obstacles', []):
                if obs_data.get('type', 'box') == 'box':
                    obs = WorkspaceObstacle.from_dict(obs_data)
                    self.node.obstacles[obs.name] = obs
                    self.object_listbox.insert(tk.END, "[B] " + obs.name)
            
            # Load meshes
            for mesh_data in data.get('meshes', []):
                mesh_obj = WorkspaceMesh.from_dict(mesh_data)
                self.node.meshes[mesh_obj.name] = mesh_obj
                self.object_listbox.insert(tk.END, "[M] " + mesh_obj.name)
            
            self.selected_item = None
            self.selected_type = None
            # Publish all loaded obstacles to MoveIt
            self.node.publish_all_obstacles()
            
            total = len(self.node.obstacles) + len(self.node.meshes)
            messagebox.showinfo("Loaded", f"Loaded {total} objects ({len(self.node.obstacles)} boxes, {len(self.node.meshes)} meshes)")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load config:\n{e}")
    
    def run(self):
        # Bind window close event
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)
        self.root.mainloop()
    
    def _on_closing(self):
        """Handle window close event."""
        # Ask to save if there are objects
        if self.node.obstacles or self.node.meshes:
            result = messagebox.askyesnocancel(
                "Save Configuration",
                "Do you want to save the scene configuration before closing?"
            )
            if result is None:  # Cancel
                return
            if result:  # Yes - save
                self._save_config()
        
        # Clear all objects from RViz and MoveIt
        self._cleanup_scene()
        
        # Close window
        self.root.destroy()
    
    def _cleanup_scene(self):
        """Remove all markers and collision objects before closing."""
        # Detach any attached objects first
        for obs in list(self.node.obstacles.values()):
            if obs.attached_to is not None:
                self.node.detach_object(obs.name)
        for mesh_obj in list(self.node.meshes.values()):
            if mesh_obj.attached_to is not None:
                self.node.detach_object(mesh_obj.name)
        
        # Remove collision objects via service
        for name, obs in list(self.node.obstacles.items()):
            self.node.remove_obstacle_from_scene(name, obs.base_frame)
        for name, mesh_obj in list(self.node.meshes.items()):
            self.node.remove_obstacle_from_scene(name, mesh_obj.base_frame)
        
        # Send DELETE markers
        marker_array = MarkerArray()
        
        for obs in self.node.obstacles.values():
            marker_id = abs(hash(obs.name)) % 100000
            marker = Marker()
            marker.header.frame_id = obs.base_frame
            marker.ns = "workspace_obstacles"
            marker.id = marker_id
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)
        
        for mesh_obj in self.node.meshes.values():
            # Delete mesh marker
            marker_id = abs(hash(mesh_obj.name)) % 100000 + 50000
            marker = Marker()
            marker.header.frame_id = mesh_obj.base_frame
            marker.ns = "workspace_meshes"
            marker.id = marker_id
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)
            
            # Delete bbox marker
            bbox_marker_id = abs(hash(mesh_obj.name)) % 100000 + 60000
            bbox_marker = Marker()
            bbox_marker.header.frame_id = mesh_obj.base_frame
            bbox_marker.ns = "workspace_mesh_bbox"
            bbox_marker.id = bbox_marker_id
            bbox_marker.action = Marker.DELETE
            marker_array.markers.append(bbox_marker)
        
        if marker_array.markers:
            self.node.marker_pub.publish(marker_array)
        
        # Clear axes markers
        axis_array = MarkerArray()
        for obs in self.node.obstacles.values():
            base_id = (abs(hash(obs.name)) % 100000) * 3
            for i in range(3):
                marker = Marker()
                marker.header.frame_id = "world"
                marker.ns = "workspace_pose_axes"
                marker.id = base_id + i
                marker.action = Marker.DELETE
                axis_array.markers.append(marker)
        
        if axis_array.markers:
            self.node.axis_pub.publish(axis_array)
        
        # Give time for messages to be sent
        import time
        time.sleep(0.2)


def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceGUINode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    gui = WorkspaceGUI(node)
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
