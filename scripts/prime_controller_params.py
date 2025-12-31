#!/usr/bin/env python3
from __future__ import annotations

import sys
import yaml
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter as ParamMsg
from ros2param.api import call_set_parameters, get_parameter_value
import yaml as _yaml


KEYS = [
    "left_wheel_names",
    "right_wheel_names",
    "wheel_separation",
    "wheel_radius",
    "wheels_per_side",
    "use_stamped_vel",
    "cmd_vel_timeout",
    "publish_rate",
    "enable_odom_tf",
    "odom_frame_id",
    "base_frame_id",
    "open_loop",
]


def to_param_value(v: Any):
    # ros2param expects a YAML string input
    if isinstance(v, str):
        s = v
    else:
        s = _yaml.safe_dump(v, default_flow_style=True).strip()
    return get_parameter_value(string_value=s)


def flatten_params(ctrl: str, data: Dict[str, Any]) -> Dict[str, Any]:
    # Accept either controller-rooted or wildcard params
    params: Dict[str, Any]
    if ctrl in data and isinstance(data[ctrl], dict):
        inner = data[ctrl]
        params = inner.get("ros__parameters", inner)
    elif "**" in data or "/**" in data:  # type: ignore[operator]
        inner = data.get("/**", data.get("**", {}))
        params = inner.get("ros__parameters", inner)
    else:
        params = {}
    out: Dict[str, Any] = {}
    for k in KEYS:
        if k in params:
            out[f"{ctrl}.{k}"] = params[k]
    return out


def main() -> int:
    if len(sys.argv) < 3:
        print("usage: prime_controller_params.py <controller_name> <param_yaml>")
        return 2
    ctrl = sys.argv[1]
    yaml_path = sys.argv[2]
    with open(yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    flat = flatten_params(ctrl, data)
    # Also set params_file pointing at the installed controller-scoped YAML (if available)
    share_path = "/home/alpha_viam/alpha_viam_rover/install/alpha_viam_bringup/share/alpha_viam_bringup/configs/diff_drive.params.yaml"
    flat.setdefault(f"{ctrl}.params_file", [share_path])
    if not flat:
        print("no params to set; check YAML shape")
        return 3

    rclpy.init()
    node = Node("prime_params")
    try:
        params = []
        for name, val in flat.items():
            pm = ParamMsg()
            pm.name = name
            pm.value = to_param_value(val)
            params.append(pm)
        resp = call_set_parameters(node=node, node_name="/controller_manager", parameters=params)
        ok = all(r.successful for r in resp.results)
        print(
            ("primed " + str(len(params)) + " params on /controller_manager for " + ctrl)
            if ok
            else "set_parameters returned failure"
        )
        return 0 if ok else 6
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
