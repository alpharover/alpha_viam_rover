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
            print(f"[drive_forward] xacro failed: {e}")

        nodes.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_desc_raw}],
            )
        )

        # Use forward controllers YAML
        nodes.append(
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                name="controller_manager",
                output="screen",
                parameters=[os.path.join(os.getcwd(), "configs", "wheels_forward.yaml")],
                remappings=[("~/robot_description", "/robot_description")],
            )
        )

        # Spawn JSB
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
                            "--activate",
                        ],
                        output="screen",
                    )
                ],
            )
        )

        # Spawn forward controllers (with timeouts)
        for ctrl in ("left_wheel_velocity_controller", "right_wheel_velocity_controller"):
            nodes.append(
                TimerAction(
                    period=3.0 if ctrl.startswith("left") else 4.0,
                    actions=[
                        ExecuteProcess(
                            cmd=[
                                "bash",
                                "-lc",
                                f"timeout 20s ros2 run controller_manager spawner {ctrl} --controller-manager /controller_manager --activate --unload-on-kill",
                            ],
                            output="screen",
                        )
                    ],
                )
            )
        return nodes

    return LaunchDescription([OpaqueFunction(function=setup)])
