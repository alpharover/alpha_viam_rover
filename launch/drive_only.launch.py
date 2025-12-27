from __future__ import annotations

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def _find_repo_root() -> Path:
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "configs").is_dir() and (parent / "urdf").is_dir():
            return parent
    return Path.cwd()


def generate_launch_description() -> LaunchDescription:
    cwd = str(_find_repo_root())
    controllers_yaml = os.path.join(cwd, "configs", "controllers.yaml")
    diffdrive_yaml = os.path.join(cwd, "configs", "diff_drive.yaml")
    urdf_xacro = os.path.join(cwd, "urdf", "rover.urdf.xacro")

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": __import__("xacro").process_file(urdf_xacro).toxml()}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            name="controller_manager",
            output="screen",
            parameters=[
                {"robot_description": __import__("xacro").process_file(urdf_xacro).toxml()},
                controllers_yaml,
            ],
        ),
        # Spawn joint_state_broadcaster first
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "run",
                        "controller_manager",
                        "spawner",
                        "joint_state_broadcaster",
                        "--controller-manager",
                        "/controller_manager",
                    ],
                    output="screen",
                )
            ],
        ),
        # Then spawn simple velocity controllers for each wheel
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "run",
                        "controller_manager",
                        "spawner",
                        "left_wheel_velocity_controller",
                        "--controller-manager",
                        "/controller_manager",
                        "--param-file",
                        os.path.join(cwd, "configs", "left_wheel_forward.yaml"),
                    ],
                    output="screen",
                )
            ],
        ),
        TimerAction(
            period=3.6,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "run",
                        "controller_manager",
                        "spawner",
                        "right_wheel_velocity_controller",
                        "--controller-manager",
                        "/controller_manager",
                        "--param-file",
                        os.path.join(cwd, "configs", "right_wheel_forward.yaml"),
                    ],
                    output="screen",
                )
            ],
        ),
    ]

    return LaunchDescription(nodes)
