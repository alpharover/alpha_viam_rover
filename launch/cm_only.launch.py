from __future__ import annotations

import os
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml


def generate_launch_description() -> LaunchDescription:
    cwd = os.getcwd()
    urdf_xacro = os.path.join(cwd, "urdf", "rover.urdf.xacro")
    controllers_yaml = os.path.join(cwd, "configs", "controllers.yaml")
    try:
        import xacro  # type: ignore
        robot_desc = xacro.process_file(urdf_xacro).toxml()
    except Exception:
        robot_desc = ""

    # Load diff drive params and wrap under the controller name so controller_manager passes them through
    dd_params_path = os.path.join(cwd, "configs", "diff_drive_params.yaml")
    dd_controller_params = {}
    try:
        with open(dd_params_path, "r", encoding="utf-8") as f:
            raw = yaml.safe_load(f) or {}
        # Accept either flat or ros__parameters root
        ddp = raw.get("ros__parameters", raw)
        dd_controller_params = {"diff_drive_controller": {"ros__parameters": ddp}}
    except Exception:
        dd_controller_params = {}

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_desc}],
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                name="controller_manager",
                output="screen",
                parameters=[{"robot_description": robot_desc}, controllers_yaml, dd_controller_params],
            ),
        ]
    )
