#!/usr/bin/env python3
"""Launch file for workspace GUI."""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    # Get the path to the workspace_gui.py script
    # Try installed location first, then source location
    install_path = os.path.expanduser(
        '~/ws/KITT_POC_ws/install/workspace/lib/python3.10/site-packages/workspace/workspace_gui.py'
    )
    source_path = os.path.expanduser(
        '~/ws/KITT_POC_ws/src/scene/workspace/workspace/workspace_gui.py'
    )
    
    if os.path.exists(install_path):
        script_path = install_path
    else:
        script_path = source_path
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', script_path],
            name='workspace_gui',
            output='screen',
        ),
    ])
    