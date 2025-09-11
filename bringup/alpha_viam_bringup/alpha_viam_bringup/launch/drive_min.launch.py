from launch import LaunchDescription
from launch.actions import OpaqueFunction, TimerAction
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    def setup(context, *args, **kwargs):
        nodes = []
        share_dir = get_package_share_directory("alpha_viam_bringup")
        urdf_xacro = os.path.join(share_dir, "urdf", "rover.urdf.xacro")
        try:
            import xacro  # type: ignore
            robot_desc_raw = xacro.process_file(urdf_xacro).toxml()
        except Exception as e:
            robot_desc_raw = ""
            print(f"[drive_min] xacro failed: {e}")

        nodes.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_desc_raw}],
            )
        )
        # Patch env for plugin discovery (l298n_hardware) in case overlay wasn't sourced
        env_patch = {}
        try:
            l298n_share = get_package_share_directory("l298n_hardware")
            l298n_prefix = os.path.dirname(os.path.dirname(l298n_share))
        except Exception:
            cwd = os.getcwd()
            candidate = os.path.join(cwd, "install", "l298n_hardware")
            l298n_prefix = candidate if os.path.isdir(candidate) else None
        if l298n_prefix:
            l298n_lib = os.path.join(l298n_prefix, "lib")
            env_patch = {
                "AMENT_PREFIX_PATH": f"{l298n_prefix}:" + os.environ.get("AMENT_PREFIX_PATH", ""),
                "LD_LIBRARY_PATH": f"{l298n_lib}:" + os.environ.get("LD_LIBRARY_PATH", ""),
            }

        nodes.append(
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                name="controller_manager",
                output="screen",
                # Pass only the controllers.yaml; get robot_description via topic
                parameters=[os.path.join(share_dir, "configs", "controllers.yaml")],
                remappings=[
                    ("~/robot_description", "/robot_description"),
                ],
                additional_env=env_patch,
            )
        )

        nodes.append(
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
            )
        )

        # Diff-drive controller is loaded/parameterized/activated by scripts to guarantee order.
        return nodes

    return LaunchDescription([OpaqueFunction(function=setup)])
