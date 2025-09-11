from __future__ import annotations

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    share_dir = get_package_share_directory("alpha_viam_bringup")
    urdf_xacro = os.path.join(share_dir, "urdf", "rover.urdf.xacro")
    try:
        import xacro  # type: ignore
        robot_desc = xacro.process_file(urdf_xacro).toxml()
    except Exception:
        robot_desc = ""

    controllers_yaml = os.path.join(share_dir, "configs", "controllers.yaml")

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
                parameters=[{"robot_description": robot_desc}, controllers_yaml],
            ),
        ]
    )
