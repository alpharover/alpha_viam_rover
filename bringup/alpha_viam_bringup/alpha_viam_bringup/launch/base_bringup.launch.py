from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    ekf_params_arg = DeclareLaunchArgument(
        'ekf_params_file',
        default_value=os.path.join(os.getcwd(), 'configs', 'ekf.yaml'),
        description='Path to robot_localization EKF params YAML.'
    )

    urdf_xacro_arg = DeclareLaunchArgument(
        'urdf_xacro',
        default_value=os.path.join(os.getcwd(), 'urdf', 'rover.urdf.xacro'),
        description='Path to rover xacro file.'
    )

    def launch_setup(context, *args, **kwargs):
        ekf_params = LaunchConfiguration('ekf_params_file').perform(context)
        urdf_xacro = LaunchConfiguration('urdf_xacro').perform(context)

        # Build robot_description from xacro at runtime
        try:
            import xacro  # type: ignore
            robot_desc_raw = xacro.process_file(urdf_xacro).toxml()
        except Exception as e:
            robot_desc_raw = ''
            print(f"[alpha_viam_bringup] xacro processing failed: {e}")

        nodes = [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_desc_raw}],
            ),
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[ekf_params],
            ),
            Node(
                package='diagnostic_aggregator',
                executable='aggregator_node',
                name='diagnostic_aggregator',
                output='screen',
            ),
            Node(
                package='foxglove_bridge',
                executable='foxglove_bridge',
                name='foxglove_bridge',
                output='screen',
                parameters=[{'address': '0.0.0.0', 'port': 8765}],
            ),
        ]

        return nodes

    return LaunchDescription([
        ekf_params_arg,
        urdf_xacro_arg,
        OpaqueFunction(function=launch_setup),
    ])
