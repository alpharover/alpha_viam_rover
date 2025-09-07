from __future__ import annotations

from pathlib import Path
from typing import Any, Dict

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_network_cfg() -> Dict[str, Any]:
    repo_root = Path(__file__).resolve().parents[1]
    cfg_path = repo_root / "configs" / "network.yaml"
    try:
        data = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}
    except Exception:
        data = {}
    return data


def generate_launch_description() -> LaunchDescription:
    cfg = _load_network_cfg()
    default_port = str(cfg.get("foxglove_ws_port", 8765))
    default_addr = cfg.get("rover_hostname", "0.0.0.0")

    use_teleop = DeclareLaunchArgument("use_teleop", default_value="false")
    ws_port = DeclareLaunchArgument("ws_port", default_value=default_port)
    ws_address = DeclareLaunchArgument("ws_address", default_value=default_addr)

    foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "port": LaunchConfiguration("ws_port"),
                "address": LaunchConfiguration("ws_address"),
            }
        ],
    )

    teleop = Node(
        condition=lambda context: context.perform_substitution(
            LaunchConfiguration("use_teleop")
        )
        .lower()
        .startswith("t"),
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        parameters=[{"speed": 0.5, "turn": 1.0}],
        remappings=[("/cmd_vel", "/cmd_vel")],
    )

    return LaunchDescription([use_teleop, ws_port, ws_address, foxglove, teleop])

