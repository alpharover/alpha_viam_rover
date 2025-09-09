from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.actions import ExecuteProcess

import os


def generate_launch_description():
    ekf_params_arg = DeclareLaunchArgument(
        "ekf_params_file",
        default_value=os.path.join(os.getcwd(), "configs", "ekf.yaml"),
        description="Path to robot_localization EKF params YAML.",
    )

    urdf_xacro_arg = DeclareLaunchArgument(
        "urdf_xacro",
        default_value=os.path.join(os.getcwd(), "urdf", "rover.urdf.xacro"),
        description="Path to rover xacro file.",
    )

    diagnostics_params_arg = DeclareLaunchArgument(
        "diagnostics_params_file",
        default_value=os.path.join(os.getcwd(), "configs", "diagnostics.yaml"),
        description="Path to diagnostic_aggregator params YAML.",
    )

    spawn_drive_arg = DeclareLaunchArgument(
        "spawn_drive",
        default_value="false",
        description=("Whether to spawn diff_drive_controller " "(set true once params are valid)"),
    )

    def launch_setup(context, *args, **kwargs):
        ekf_params = LaunchConfiguration("ekf_params_file").perform(context)
        urdf_xacro = LaunchConfiguration("urdf_xacro").perform(context)
        diagnostics_params = LaunchConfiguration("diagnostics_params_file").perform(context)

        # Build robot_description from xacro at runtime
        try:
            import xacro  # type: ignore

            robot_desc_raw = xacro.process_file(urdf_xacro).toxml()
        except Exception as e:
            robot_desc_raw = ""
            print(f"[alpha_viam_bringup] xacro processing failed: {e}")

        # Load simple YAML for power params and pass as parameters directly
        power_params = {}
        try:
            import yaml  # type: ignore

            with open(
                os.path.join(os.getcwd(), "configs", "power.yaml"),
                "r",
                encoding="utf-8",
            ) as f:
                y = yaml.safe_load(f)
                if isinstance(y, dict) and "power" in y and isinstance(y["power"], dict):
                    power_params = y["power"]
        except Exception as e:
            print(f"[alpha_viam_bringup] power.yaml parse failed: {e}")

        # Load IMU params and filter to known keys
        imu_params = {}
        imu_filter_params = {}
        try:
            import yaml  # type: ignore

            with open(
                os.path.join(os.getcwd(), "configs", "imu.yaml"),
                "r",
                encoding="utf-8",
            ) as f:
                y = yaml.safe_load(f)
                if isinstance(y, dict) and "imu" in y and isinstance(y["imu"], dict):
                    _raw = y["imu"]
                    if isinstance(_raw, dict):
                        allowed = {
                            "i2c_bus",
                            "address",
                            "frame_id",
                            "rate_hz",
                            "imu_topic",
                            "accel_range_g",
                            "gyro_range_dps",
                            "calibrate_gyro",
                            "calib_samples",
                        }
                        imu_params = {k: v for k, v in _raw.items() if k in allowed}
                        # Capture filter sub-params if present
                        if isinstance(_raw.get("filter"), dict):
                            imu_filter_params = _raw["filter"]
        except Exception as e:
            print(f"[alpha_viam_bringup] imu.yaml parse failed: {e}")

        # Load network params (for Foxglove port and Wi‑Fi iface)
        net_params = {"foxglove_ws_port": 8765, "wifi_iface": "wlan1"}
        try:
            import yaml  # type: ignore

            with open(
                os.path.join(os.getcwd(), "configs", "network.yaml"),
                "r",
                encoding="utf-8",
            ) as f:
                y = yaml.safe_load(f)
                if isinstance(y, dict):
                    if "foxglove_ws_port" in y:
                        net_params["foxglove_ws_port"] = int(y["foxglove_ws_port"])  # type: ignore[arg-type]
                    if "wifi_iface" in y:
                        net_params["wifi_iface"] = str(y["wifi_iface"])  # type: ignore[arg-type]
        except Exception as e:
            print(f"[alpha_viam_bringup] network.yaml parse failed: {e}")

        nodes = [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_desc_raw}],
            ),
            # INA219 power monitor (publishes bus voltage and current)
            Node(
                package="ina219_monitor",
                executable="ina219_monitor",
                name="ina219_monitor",
                output="screen",
                parameters=[
                    power_params,
                    {"i2c_bus": 1, "address": 0x40},
                ],
            ),
            # MPU-6050 IMU (publishes sensor_msgs/Imu)
            Node(
                package="mpu6050_driver",
                executable="mpu6050_node",
                name="mpu6050",
                output="screen",
                parameters=[{"rate_hz": 100.0}, imu_params],
            ),
            # Optional IMU orientation filter (Madgwick) → publishes fused orientation
            # Subscribes to /imu/data (raw) and publishes /imu/data_fused
            Node(
                package="imu_filter_madgwick",
                executable="imu_filter_madgwick_node",
                name="imu_filter",
                output="screen",
                parameters=[
                    {"use_mag": False, "world_frame": "enu"},
                    {
                        k: v
                        for k, v in imu_filter_params.items()
                        if k in ("gain", "zeta", "use_mag", "world_frame", "publish_tf")
                    },
                ],
                remappings=[
                    ("imu/data_raw", "/imu/data"),
                    ("imu/data", "/imu/data_fused"),
                ],
            ),
            # Controller manager (ros2_control)
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                name="controller_manager",
                output="screen",
                parameters=[
                    {"robot_description": robot_desc_raw},
                    os.path.join(os.getcwd(), "configs", "controllers.yaml"),
                ],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[ekf_params],
            ),
            Node(
                package="diagnostic_aggregator",
                executable="aggregator_node",
                name="diagnostic_aggregator",
                output="screen",
                parameters=[diagnostics_params],
            ),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
                parameters=[
                    {"address": "0.0.0.0", "port": int(net_params.get("foxglove_ws_port", 8765))}
                ],
            ),
            # Wi‑Fi monitor (RSSI + link_ok + diagnostics)
            Node(
                package="wifi_monitor",
                executable="wifi_monitor",
                name="wifi_monitor",
                output="screen",
                parameters=[
                    {
                        "iface": net_params.get("wifi_iface", "wlan1"),
                        "rate_hz": 0.5,
                        "include_info": False,
                    }
                ],
            ),
            # Spawn controllers (after controller_manager is up)
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
                    ),
                ],
            ),
        ]

        # Optionally spawn diff_drive_controller (gated until parameters are finalized)
        if LaunchConfiguration("spawn_drive").perform(context).lower() in (
            "true",
            "1",
            "yes",
        ):
            nodes.append(
                TimerAction(
                    period=2.5,
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
                            ],
                            output="screen",
                        )
                    ],
                )
            )

        return nodes

    # Ensure all DeclareLaunchArgument actions are added BEFORE the OpaqueFunction
    # so that LaunchConfiguration lookups succeed when launch_setup runs.
    return LaunchDescription(
        [
            ekf_params_arg,
            urdf_xacro_arg,
            diagnostics_params_arg,
            spawn_drive_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
