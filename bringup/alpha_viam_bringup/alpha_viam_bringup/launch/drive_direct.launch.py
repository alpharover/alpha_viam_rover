from launch import LaunchDescription
from launch.actions import OpaqueFunction, ExecuteProcess
from launch_ros.actions import Node
from pathlib import Path

import os


def _find_repo_root() -> Path:
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "configs").is_dir() and (parent / "urdf").is_dir():
            return parent
    return Path.cwd()


def generate_launch_description():
    def setup(context, *args, **kwargs):
        repo_root = _find_repo_root()
        nodes = []
        urdf_xacro = str(repo_root / "urdf" / "rover.urdf.xacro")
        try:
            import xacro  # type: ignore
            robot_desc_raw = xacro.process_file(urdf_xacro).toxml()
        except Exception as e:
            robot_desc_raw = ""
            print(f"[drive_direct] xacro failed: {e}")

        nodes.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_desc_raw}],
            )
        )

        nodes.append(
            ExecuteProcess(
                cmd=["python3", str(repo_root / "scripts" / "l298n_direct.py")],
                output="screen",
            )
        )
        return nodes

    return LaunchDescription([OpaqueFunction(function=setup)])

