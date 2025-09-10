from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
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
            print(f"[cm_only] xacro failed: {e}")

        nodes.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_desc_raw}],
            )
        )

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
                parameters=[os.path.join(os.getcwd(), "configs", "controllers.yaml")],
                remappings=[
                    ("~/robot_description", "/robot_description"),
                ],
                env=env_patch,
            )
        )

        return nodes

    return LaunchDescription([OpaqueFunction(function=setup)])
