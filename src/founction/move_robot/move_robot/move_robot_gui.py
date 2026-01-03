#!/usr/bin/env python3
"""
Move Robot GUI - Virtual robot base movement control with position markers.

This node controls the robot chassis movement by publishing joint states for
the virtual mobile base joints defined in the URDF:
  - virtual_x_joint: Prismatic joint for X translation
  - virtual_y_joint: Prismatic joint for Y translation  
  - virtual_yaw_joint: Revolute joint for Yaw rotation

Features:
1. Velocity mode: Continuous movement in X/Y directions or rotation around Z axis
2. Position mode: Move to a specific X, Y, Yaw position over a specified duration
3. Position markers: Mark and visualize positions in RViz with coordinate axes
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import tkinter as tk
from tkinter import ttk, simpledialog
import threading
import numpy as np
import time
from typing import Dict, List

from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class PositionMarker:
    """Represents a position marker with coordinate axes."""
    def __init__(self, name: str, x: float, y: float, yaw: float, visible: bool = True):
        self.name = name
        self.x = x
        self.y = y
        self.yaw = yaw  # radians
        self.visible = visible
        self.z = 0.265  # Fixed chassis height


class MoveRobotNode(Node):
    """ROS2 node for virtual robot base movement via joint states."""
    
    VIRTUAL_X_JOINT = 'virtual_x_joint'
    VIRTUAL_Y_JOINT = 'virtual_y_joint'
    VIRTUAL_YAW_JOINT = 'virtual_yaw_joint'
    CHASSIS_Z = 0.265

    def __init__(self):
        super().__init__('move_robot_node')
        
        # Current pose (x, y, yaw) - these are joint positions
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Velocity mode state
        self.velocity_mode_active = False
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_yaw = 0.0
        
        # Position mode state
        self.position_mode_active = False
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.move_start_time = 0.0
        self.move_duration = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        
        # Position markers
        self.markers: Dict[str, PositionMarker] = {}
        self.marker_counter = 0
        # Pending delete markers (name -> base_id) - to ensure DELETE is published
        self.pending_deletes: Dict[str, int] = {}
        
        # Publishers
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/position_markers', 10)
        
        # Timer for updating (100 Hz)
        self.update_rate = 100.0
        self.timer = self.create_timer(1.0 / self.update_rate, self._update_callback)
        
        # Marker publish timer (10 Hz)
        self.marker_timer = self.create_timer(0.1, self._publish_markers)
        
        self._publish_joint_states()
        self.get_logger().info("MoveRobot node initialized")

    def _update_callback(self):
        """Timer callback to update pose and publish joint states."""
        current_time = time.time()
        dt = 1.0 / self.update_rate
        
        if self.velocity_mode_active:
            cos_yaw = np.cos(self.current_yaw)
            sin_yaw = np.sin(self.current_yaw)
            world_vel_x = self.vel_x * cos_yaw - self.vel_y * sin_yaw
            world_vel_y = self.vel_x * sin_yaw + self.vel_y * cos_yaw
            self.current_x += world_vel_x * dt
            self.current_y += world_vel_y * dt
            self.current_yaw += self.vel_yaw * dt
            self.current_yaw = np.arctan2(np.sin(self.current_yaw), np.cos(self.current_yaw))
        
        elif self.position_mode_active:
            elapsed = current_time - self.move_start_time
            if self.move_duration <= 0:
                self.current_x = self.target_x
                self.current_y = self.target_y
                self.current_yaw = self.target_yaw
                self.position_mode_active = False
            elif elapsed >= self.move_duration:
                self.current_x = self.target_x
                self.current_y = self.target_y
                self.current_yaw = self.target_yaw
                self.position_mode_active = False
            else:
                t = elapsed / self.move_duration
                self.current_x = self.start_x + (self.target_x - self.start_x) * t
                self.current_y = self.start_y + (self.target_y - self.start_y) * t
                delta_yaw = np.arctan2(np.sin(self.target_yaw - self.start_yaw), 
                                       np.cos(self.target_yaw - self.start_yaw))
                self.current_yaw = self.start_yaw + delta_yaw * t
        
        self._publish_joint_states()
    
    def _publish_joint_states(self):
        """Publish virtual joint states."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.VIRTUAL_X_JOINT, self.VIRTUAL_Y_JOINT, self.VIRTUAL_YAW_JOINT]
        msg.position = [self.current_x, self.current_y, self.current_yaw]
        msg.velocity = [0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0]
        self.joint_state_pub.publish(msg)

    def _publish_markers(self):
        """Publish position marker visualizations."""
        marker_array = MarkerArray()
        
        # First, publish DELETE for pending deletes
        for name, base_id in list(self.pending_deletes.items()):
            # Delete all 4 markers (3 axes + 1 text)
            for i in range(4):
                marker = Marker()
                marker.header.frame_id = 'world'
                marker.ns = f"position_marker_{name}" if i < 3 else f"position_marker_{name}_text"
                marker.id = base_id + i
                marker.action = Marker.DELETE
                marker_array.markers.append(marker)
        # Clear pending deletes after publishing
        self.pending_deletes.clear()
        
        # Then publish active markers
        for marker_obj in self.markers.values():
            base_id = abs(hash(marker_obj.name)) % 100000
            axis_length = 0.3
            axis_width = 0.02
            
            cos_yaw = np.cos(marker_obj.yaw)
            sin_yaw = np.sin(marker_obj.yaw)
            
            # X, Y, Z axes
            axes = [
                ([1, 0, 0], [1.0, 0.0, 0.0]),  # X axis - red
                ([0, 1, 0], [0.0, 1.0, 0.0]),  # Y axis - green
                ([0, 0, 1], [0.0, 0.0, 1.0]),  # Z axis - blue
            ]
            
            for i, (direction, color) in enumerate(axes):
                marker = Marker()
                marker.header.frame_id = 'world'
                marker.header.stamp.sec = 0
                marker.header.stamp.nanosec = 0
                marker.ns = f"position_marker_{marker_obj.name}"
                marker.id = base_id + i
                marker.lifetime.sec = 0
                marker.lifetime.nanosec = 0
                
                if marker_obj.visible:
                    marker.type = Marker.ARROW
                    marker.action = Marker.ADD
                    
                    p1 = Point()
                    p1.x = marker_obj.x
                    p1.y = marker_obj.y
                    p1.z = marker_obj.z
                    
                    dx = direction[0] * cos_yaw - direction[1] * sin_yaw
                    dy = direction[0] * sin_yaw + direction[1] * cos_yaw
                    dz = direction[2]
                    
                    p2 = Point()
                    p2.x = marker_obj.x + dx * axis_length
                    p2.y = marker_obj.y + dy * axis_length
                    p2.z = marker_obj.z + dz * axis_length
                    
                    marker.points = [p1, p2]
                    marker.scale.x = axis_width
                    marker.scale.y = axis_width * 2
                    marker.scale.z = 0.0
                    marker.color.r = color[0]
                    marker.color.g = color[1]
                    marker.color.b = color[2]
                    marker.color.a = 1.0
                else:
                    marker.action = Marker.DELETE
                
                marker_array.markers.append(marker)
            
            # Text label
            text_marker = Marker()
            text_marker.header.frame_id = 'world'
            text_marker.ns = f"position_marker_{marker_obj.name}_text"
            text_marker.id = base_id + 3
            text_marker.lifetime.sec = 0
            
            if marker_obj.visible:
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.position.x = marker_obj.x
                text_marker.pose.position.y = marker_obj.y
                text_marker.pose.position.z = marker_obj.z + 0.4
                text_marker.scale.z = 0.1
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 1.0
                text_marker.text = marker_obj.name
            else:
                text_marker.action = Marker.DELETE
            
            marker_array.markers.append(text_marker)
        
        if marker_array.markers:
            self.marker_pub.publish(marker_array)

    def add_marker(self, name: str, x: float, y: float, yaw: float) -> str:
        """Add a position marker."""
        if name in self.markers:
            self.marker_counter += 1
            name = f"{name}_{self.marker_counter}"
        self.markers[name] = PositionMarker(name, x, y, yaw)
        self.get_logger().info(f"Added marker '{name}' at ({x:.3f}, {y:.3f}, {np.rad2deg(yaw):.1f}°)")
        return name
    
    def add_marker_at_current(self) -> str:
        """Add a marker at current robot position."""
        self.marker_counter += 1
        name = f"mark_{self.marker_counter}"
        return self.add_marker(name, self.current_x, self.current_y, self.current_yaw)
    
    def remove_marker(self, name: str):
        """Remove a marker and schedule DELETE."""
        if name in self.markers:
            base_id = abs(hash(name)) % 100000
            # Add to pending deletes
            self.pending_deletes[name] = base_id
            del self.markers[name]
            self.get_logger().info(f"Removed marker '{name}'")
    
    def rename_marker(self, old_name: str, new_name: str) -> bool:
        """Rename a marker."""
        if old_name not in self.markers or new_name in self.markers:
            return False
        marker_obj = self.markers.pop(old_name)
        # Schedule delete for old name
        old_base_id = abs(hash(old_name)) % 100000
        self.pending_deletes[old_name] = old_base_id
        # Add with new name
        marker_obj.name = new_name
        self.markers[new_name] = marker_obj
        self.get_logger().info(f"Renamed marker '{old_name}' to '{new_name}'")
        return True
    
    def set_marker_visible(self, name: str, visible: bool):
        """Set marker visibility."""
        if name in self.markers:
            self.markers[name].visible = visible
    
    def start_velocity_mode(self, vel_x: float, vel_y: float, vel_yaw: float):
        self.stop_all()
        self.vel_x = vel_x
        self.vel_y = vel_y
        self.vel_yaw = vel_yaw
        self.velocity_mode_active = True
    
    def start_position_mode(self, target_x: float, target_y: float, target_yaw: float, duration: float):
        self.stop_all()
        self.target_x = target_x
        self.target_y = target_y
        self.target_yaw = target_yaw
        self.move_duration = duration
        self.start_x = self.current_x
        self.start_y = self.current_y
        self.start_yaw = self.current_yaw
        self.move_start_time = time.time()
        self.position_mode_active = True
    
    def stop_all(self):
        self.velocity_mode_active = False
        self.position_mode_active = False
        self.vel_x = self.vel_y = self.vel_yaw = 0.0
    
    def reset_pose(self):
        self.stop_all()
        self.current_x = self.current_y = self.current_yaw = 0.0


class MoveRobotGUI:
    """Tkinter GUI for robot base movement control."""
    
    def __init__(self, node: MoveRobotNode):
        self.node = node
        
        self.root = tk.Tk()
        self.root.title("Move Robot - Base Movement Control")
        self.root.geometry("520x750")
        self.root.configure(bg='#2b2b2b')
        self.root.minsize(480, 700)
        
        self._setup_styles()
        self._create_widgets()
        self._update_display()
    
    def _setup_styles(self):
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TFrame', background='#2b2b2b')
        style.configure('TLabel', background='#2b2b2b', foreground='white')
        style.configure('TButton', padding=5)
        style.configure('TLabelframe', background='#2b2b2b', foreground='white')
        style.configure('TLabelframe.Label', background='#2b2b2b', foreground='#4CAF50')
    
    def _create_widgets(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Current Pose Display
        pose_frame = tk.LabelFrame(main_frame, text="Current Pose (World Frame)", 
                                   bg='#2b2b2b', fg='#4CAF50', padx=10, pady=8)
        pose_frame.pack(fill=tk.X, pady=(0, 8))
        
        self.pose_label = tk.Label(pose_frame, text="X: 0.000 m  Y: 0.000 m  Yaw: 0.0°", 
                                   bg='#2b2b2b', fg='white', font=('Consolas', 11))
        self.pose_label.pack(fill=tk.X)
        
        self.status_label = tk.Label(pose_frame, text="Status: Idle", bg='#2b2b2b', fg='#888888')
        self.status_label.pack(fill=tk.X, pady=(3, 0))
        
        ttk.Button(pose_frame, text="Reset to Origin", command=self._reset_pose).pack(pady=(8, 0))
        
        # Mode 1: Velocity Control
        vel_frame = tk.LabelFrame(main_frame, text="Mode 1: Velocity Control", 
                                  bg='#2b2b2b', fg='#4CAF50', padx=10, pady=8)
        vel_frame.pack(fill=tk.X, pady=(0, 8))
        
        speed_frame = tk.Frame(vel_frame, bg='#2b2b2b')
        speed_frame.pack(fill=tk.X, pady=3)
        tk.Label(speed_frame, text="Linear (m/s):", bg='#2b2b2b', fg='white').pack(side=tk.LEFT)
        self.lin_vel_var = tk.StringVar(value="0.5")
        tk.Entry(speed_frame, textvariable=self.lin_vel_var, width=6, bg='#3c3c3c', fg='white').pack(side=tk.LEFT, padx=5)
        tk.Label(speed_frame, text="Angular (°/s):", bg='#2b2b2b', fg='white').pack(side=tk.LEFT, padx=(10, 0))
        self.ang_vel_var = tk.StringVar(value="30.0")
        tk.Entry(speed_frame, textvariable=self.ang_vel_var, width=6, bg='#3c3c3c', fg='white').pack(side=tk.LEFT, padx=5)
        
        btn_grid = tk.Frame(vel_frame, bg='#2b2b2b')
        btn_grid.pack(pady=8)
        tk.Button(btn_grid, text="↺ Yaw+", width=7, command=lambda: self._start_velocity('yaw+'),
                  bg='#3c3c3c', fg='white').grid(row=0, column=0, padx=2, pady=2)
        tk.Button(btn_grid, text="↑ X+", width=7, command=lambda: self._start_velocity('x+'),
                  bg='#3c3c3c', fg='white').grid(row=0, column=1, padx=2, pady=2)
        tk.Button(btn_grid, text="↻ Yaw-", width=7, command=lambda: self._start_velocity('yaw-'),
                  bg='#3c3c3c', fg='white').grid(row=0, column=2, padx=2, pady=2)
        tk.Button(btn_grid, text="← Y+", width=7, command=lambda: self._start_velocity('y+'),
                  bg='#3c3c3c', fg='white').grid(row=1, column=0, padx=2, pady=2)
        tk.Button(btn_grid, text="STOP", width=7, command=self._stop_all,
                  bg='#c62828', fg='white', font=('Arial', 9, 'bold')).grid(row=1, column=1, padx=2, pady=2)
        tk.Button(btn_grid, text="→ Y-", width=7, command=lambda: self._start_velocity('y-'),
                  bg='#3c3c3c', fg='white').grid(row=1, column=2, padx=2, pady=2)
        tk.Button(btn_grid, text="↓ X-", width=7, command=lambda: self._start_velocity('x-'),
                  bg='#3c3c3c', fg='white').grid(row=2, column=1, padx=2, pady=2)

        # Mode 2: Position Control
        pos_frame = tk.LabelFrame(main_frame, text="Mode 2: Position Control", 
                                  bg='#2b2b2b', fg='#4CAF50', padx=10, pady=8)
        pos_frame.pack(fill=tk.X, pady=(0, 8))
        
        input_frame = tk.Frame(pos_frame, bg='#2b2b2b')
        input_frame.pack(fill=tk.X)
        
        for label, var_name, default in [
            ("Target X (m):", "target_x_var", "0.0"),
            ("Target Y (m):", "target_y_var", "0.0"),
            ("Target Yaw (°):", "target_yaw_var", "0.0"),
            ("Duration (s):", "duration_var", "0.0"),
        ]:
            row = tk.Frame(input_frame, bg='#2b2b2b')
            row.pack(fill=tk.X, pady=1)
            tk.Label(row, text=label, width=13, anchor='w', bg='#2b2b2b', fg='white').pack(side=tk.LEFT)
            var = tk.StringVar(value=default)
            setattr(self, var_name, var)
            tk.Entry(row, textvariable=var, width=10, bg='#3c3c3c', fg='white').pack(side=tk.LEFT, padx=5)
        
        pos_btn_frame = tk.Frame(pos_frame, bg='#2b2b2b')
        pos_btn_frame.pack(fill=tk.X, pady=(8, 0))
        tk.Button(pos_btn_frame, text="Move to Target", width=12, command=self._start_position_move,
                  bg='#4CAF50', fg='white').pack(side=tk.LEFT, padx=2)
        tk.Button(pos_btn_frame, text="Stop", width=6, command=self._stop_all,
                  bg='#c62828', fg='white').pack(side=tk.LEFT, padx=2)
        tk.Button(pos_btn_frame, text="Set Current", width=9, command=self._set_current_as_target,
                  bg='#3c3c3c', fg='white').pack(side=tk.LEFT, padx=2)
        
        # Position Markers
        marker_frame = tk.LabelFrame(main_frame, text="Position Markers", 
                                     bg='#2b2b2b', fg='#4CAF50', padx=10, pady=8)
        marker_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 5))
        
        # Marker list
        list_frame = tk.Frame(marker_frame, bg='#2b2b2b')
        list_frame.pack(fill=tk.BOTH, expand=True)
        
        scrollbar = ttk.Scrollbar(list_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.marker_listbox = tk.Listbox(list_frame, height=6, bg='#3c3c3c', fg='white',
                                         selectbackground='#4CAF50', yscrollcommand=scrollbar.set)
        self.marker_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.marker_listbox.yview)
        self.marker_listbox.bind('<<ListboxSelect>>', self._on_marker_select)
        
        # Marker buttons row 1
        btn_row1 = tk.Frame(marker_frame, bg='#2b2b2b')
        btn_row1.pack(fill=tk.X, pady=(5, 2))
        tk.Button(btn_row1, text="Mark Current", width=11, command=self._mark_current,
                  bg='#2196F3', fg='white').pack(side=tk.LEFT, padx=2)
        tk.Button(btn_row1, text="Add Marker", width=10, command=self._add_marker_dialog,
                  bg='#4CAF50', fg='white').pack(side=tk.LEFT, padx=2)
        tk.Button(btn_row1, text="Delete", width=7, command=self._delete_marker,
                  bg='#c62828', fg='white').pack(side=tk.LEFT, padx=2)
        
        # Marker buttons row 2
        btn_row2 = tk.Frame(marker_frame, bg='#2b2b2b')
        btn_row2.pack(fill=tk.X, pady=2)
        tk.Button(btn_row2, text="Rename", width=8, command=self._rename_marker,
                  bg='#3c3c3c', fg='white').pack(side=tk.LEFT, padx=2)
        tk.Button(btn_row2, text="Show/Hide", width=9, command=self._toggle_marker_visible,
                  bg='#3c3c3c', fg='white').pack(side=tk.LEFT, padx=2)
        tk.Button(btn_row2, text="Go To", width=7, command=self._goto_marker,
                  bg='#FF9800', fg='white').pack(side=tk.LEFT, padx=2)
        
        # Marker info
        self.marker_info_label = tk.Label(marker_frame, text="Select a marker to see details", 
                                          bg='#2b2b2b', fg='#888888')
        self.marker_info_label.pack(fill=tk.X, pady=(5, 0))

    def _update_display(self):
        """Update the pose display."""
        x, y = self.node.current_x, self.node.current_y
        yaw_deg = np.rad2deg(self.node.current_yaw)
        self.pose_label.config(text=f"X: {x:+.3f} m   Y: {y:+.3f} m   Yaw: {yaw_deg:+.1f}°")
        
        if self.node.velocity_mode_active:
            self.status_label.config(text="Status: Velocity Mode Active", fg='#4CAF50')
        elif self.node.position_mode_active:
            self.status_label.config(text="Status: Moving to Target...", fg='#FFC107')
        else:
            self.status_label.config(text="Status: Idle", fg='#888888')
        
        self.root.after(50, self._update_display)
    
    def _start_velocity(self, direction: str):
        try:
            lin_vel = float(self.lin_vel_var.get())
            ang_vel = np.deg2rad(float(self.ang_vel_var.get()))
        except ValueError:
            return
        
        vel_map = {
            'x+': (lin_vel, 0, 0), 'x-': (-lin_vel, 0, 0),
            'y+': (0, lin_vel, 0), 'y-': (0, -lin_vel, 0),
            'yaw+': (0, 0, ang_vel), 'yaw-': (0, 0, -ang_vel),
        }
        if direction in vel_map:
            self.node.start_velocity_mode(*vel_map[direction])
    
    def _stop_all(self):
        self.node.stop_all()
    
    def _start_position_move(self):
        try:
            target_x = float(self.target_x_var.get())
            target_y = float(self.target_y_var.get())
            target_yaw = np.deg2rad(float(self.target_yaw_var.get()))
            duration = float(self.duration_var.get())
        except ValueError:
            return
        self.node.start_position_mode(target_x, target_y, target_yaw, duration)
    
    def _set_current_as_target(self):
        self.target_x_var.set(f"{self.node.current_x:.3f}")
        self.target_y_var.set(f"{self.node.current_y:.3f}")
        self.target_yaw_var.set(f"{np.rad2deg(self.node.current_yaw):.1f}")
    
    def _reset_pose(self):
        self.node.reset_pose()
    
    def _update_marker_list(self, select_name: str = None):
        """Update the marker listbox, optionally selecting a specific marker."""
        # Remember current selection if not specified
        if select_name is None:
            current_sel = self._get_selected_marker_name()
            if current_sel and current_sel in self.node.markers:
                select_name = current_sel
        
        self.marker_listbox.delete(0, tk.END)
        select_idx = None
        for i, (name, marker) in enumerate(self.node.markers.items()):
            vis = "●" if marker.visible else "○"
            self.marker_listbox.insert(tk.END, f"{vis} {name}")
            if name == select_name:
                select_idx = i
        
        # Restore selection
        if select_idx is not None:
            self.marker_listbox.selection_set(select_idx)
            self._update_marker_info(select_name)
    
    def _update_marker_info(self, name: str):
        """Update marker info label."""
        if name and name in self.node.markers:
            m = self.node.markers[name]
            vis_str = "visible" if m.visible else "hidden"
            self.marker_info_label.config(
                text=f"{name} ({vis_str}): X={m.x:.3f}m, Y={m.y:.3f}m, Yaw={np.rad2deg(m.yaw):.1f}°",
                fg='white')
        else:
            self.marker_info_label.config(text="Select a marker to see details", fg='#888888')
    
    def _on_marker_select(self, event):
        """Handle marker selection."""
        name = self._get_selected_marker_name()
        self._update_marker_info(name)

    def _get_selected_marker_name(self):
        """Get selected marker name."""
        selection = self.marker_listbox.curselection()
        if not selection:
            return None
        return self.marker_listbox.get(selection[0])[2:]
    
    def _mark_current(self):
        """Mark current position."""
        name = self.node.add_marker_at_current()
        self._update_marker_list(select_name=name)
    
    def _add_marker_dialog(self):
        """Show dialog to add marker at specified position."""
        dialog = tk.Toplevel(self.root)
        dialog.title("Add Position Marker")
        dialog.geometry("280x200")
        dialog.configure(bg='#2b2b2b')
        dialog.transient(self.root)
        dialog.grab_set()
        
        entries = {}
        for label, default in [("Name:", "marker"), ("X (m):", "0.0"), 
                               ("Y (m):", "0.0"), ("Yaw (°):", "0.0")]:
            row = tk.Frame(dialog, bg='#2b2b2b')
            row.pack(fill=tk.X, padx=10, pady=3)
            tk.Label(row, text=label, width=10, anchor='w', bg='#2b2b2b', fg='white').pack(side=tk.LEFT)
            var = tk.StringVar(value=default)
            tk.Entry(row, textvariable=var, width=15, bg='#3c3c3c', fg='white').pack(side=tk.LEFT)
            entries[label] = var
        
        def on_add():
            try:
                name = entries["Name:"].get().strip()
                x = float(entries["X (m):"].get())
                y = float(entries["Y (m):"].get())
                yaw = np.deg2rad(float(entries["Yaw (°):"].get()))
                if name:
                    added_name = self.node.add_marker(name, x, y, yaw)
                    self._update_marker_list(select_name=added_name)
                dialog.destroy()
            except ValueError:
                pass
        
        btn_frame = tk.Frame(dialog, bg='#2b2b2b')
        btn_frame.pack(fill=tk.X, padx=10, pady=10)
        tk.Button(btn_frame, text="Add", width=10, command=on_add, bg='#4CAF50', fg='white').pack(side=tk.LEFT, padx=5)
        tk.Button(btn_frame, text="Cancel", width=10, command=dialog.destroy, bg='#3c3c3c', fg='white').pack(side=tk.LEFT)
    
    def _delete_marker(self):
        """Delete selected marker."""
        name = self._get_selected_marker_name()
        if name:
            self.node.remove_marker(name)
            # Select next available marker
            self._update_marker_list()
            if self.node.markers:
                self.marker_listbox.selection_set(0)
                self._on_marker_select(None)
            else:
                self.marker_info_label.config(text="Select a marker to see details", fg='#888888')
    
    def _rename_marker(self):
        """Rename selected marker."""
        name = self._get_selected_marker_name()
        if not name:
            return
        new_name = simpledialog.askstring("Rename Marker", f"New name for '{name}':", 
                                          parent=self.root, initialvalue=name)
        if new_name and new_name.strip() and new_name != name:
            if self.node.rename_marker(name, new_name.strip()):
                self._update_marker_list(select_name=new_name.strip())
    
    def _toggle_marker_visible(self):
        """Toggle marker visibility."""
        name = self._get_selected_marker_name()
        if name and name in self.node.markers:
            m = self.node.markers[name]
            self.node.set_marker_visible(name, not m.visible)
            self._update_marker_list(select_name=name)
    
    def _goto_marker(self):
        """Move robot to selected marker position."""
        name = self._get_selected_marker_name()
        if name and name in self.node.markers:
            m = self.node.markers[name]
            self.target_x_var.set(f"{m.x:.3f}")
            self.target_y_var.set(f"{m.y:.3f}")
            self.target_yaw_var.set(f"{np.rad2deg(m.yaw):.1f}")
            self.duration_var.set("2.0")
            self._start_position_move()
    
    def run(self):
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)
        self.root.mainloop()
    
    def _on_closing(self):
        self.node.stop_all()
        self.root.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotNode()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    gui = MoveRobotGUI(node)
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
