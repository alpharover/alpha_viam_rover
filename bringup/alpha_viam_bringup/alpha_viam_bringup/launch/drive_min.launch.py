from launch import LaunchDescription
from launch.actions import OpaqueFunction, TimerAction
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    def setup(context, *args, **kwargs):
        nodes = []
        urdf_xacro = os.path.join(os.getcwd(), "urdf", "rover.urdf.xacro")
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
        # Ensure plugin discovery can find l298n_hardware even if the shell overlay wasn't sourced
        l298n_prefix = os.path.join(os.getcwd(), "install", "l298n_hardware")
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
                parameters=[os.path.join(os.getcwd(), "configs", "controllers.yaml")],
                remappings=[
                    ("~/robot_description", "/robot_description"),
                ],
                env=env_patch,
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

        # Load and activate diff_drive without --param-file; params are present at load time
        nodes.append(
            TimerAction(
                period=3.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "run",
                            "controller_manager",
                            "spawner",
                            "diff_drive_controller",
                            "--controller-manager",
                            "/controller_manager",
                            "--activate",
                            "--unload-on-kill",
                        ],
                        output="screen",
                    )
                ],
            )
        )
        return nodes

    return LaunchDescription([OpaqueFunction(function=setup)])
