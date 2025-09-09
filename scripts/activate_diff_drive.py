#!/usr/bin/env python3
from __future__ import annotations

import sys
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterValue
from rcl_interfaces.srv import SetParameters
from controller_manager_msgs.srv import LoadController, SwitchController


class DiffDriveActivator(Node):
    def __init__(self, yaml_path: str) -> None:
        super().__init__("diff_drive_activator")
        self.yaml_path = yaml_path
        self.cm_name = "/controller_manager"
        self.ctrl_name = "diff_drive_controller"

        self.cli_load = self.create_client(LoadController, f"{self.cm_name}/load_controller")
        self.cli_switch = self.create_client(SwitchController, f"{self.cm_name}/switch_controller")

    def wait_for(self, client_name: str, client) -> None:
        if not client.wait_for_service(timeout_sec=8.0):
            raise RuntimeError(f"Service not available: {client_name}")

    def load_controller(self) -> None:
        self.wait_for("load_controller", self.cli_load)
        req = LoadController.Request()
        req.name = self.ctrl_name
        fut = self.cli_load.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=8.0)
        if not fut.done() or fut.result() is None or not fut.result().ok:
            raise RuntimeError("load_controller failed")

    def set_params(self) -> None:
        # Read YAML as flat ros__parameters mapping
        with open(self.yaml_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        params = data.get("ros__parameters", data)
        cli = self.create_client(SetParameters, f"/{self.ctrl_name}/set_parameters")
        end = time.time() + 8.0
        while time.time() < end and not cli.wait_for_service(timeout_sec=0.5):
            pass
        if not cli.service_is_ready():
            raise RuntimeError("controller parameter service not ready")

        def to_param_msg(name: str, value):
            v = ParameterValue()
            if isinstance(value, bool):
                v.type = ParameterValue.TYPE_BOOL
                v.bool_value = value
            elif isinstance(value, int):
                v.type = ParameterValue.TYPE_INTEGER
                v.integer_value = value
            elif isinstance(value, float):
                v.type = ParameterValue.TYPE_DOUBLE
                v.double_value = value
            elif isinstance(value, str):
                v.type = ParameterValue.TYPE_STRING
                v.string_value = value
            elif isinstance(value, list) and all(isinstance(x, str) for x in value):
                v.type = ParameterValue.TYPE_STRING_ARRAY
                v.string_array_value = value
            elif isinstance(value, list) and all(isinstance(x, float) for x in value):
                v.type = ParameterValue.TYPE_DOUBLE_ARRAY
                v.double_array_value = value
            elif isinstance(value, list) and all(isinstance(x, int) for x in value):
                v.type = ParameterValue.TYPE_INTEGER_ARRAY
                v.integer_array_value = value
            else:
                # Fallback to string
                v.type = ParameterValue.TYPE_STRING
                v.string_value = str(value)
            p = ParamMsg()
            p.name = name
            p.value = v
            return p

        req = SetParameters.Request()
        req.parameters = [to_param_msg(k, v) for k, v in params.items()]
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=8.0)
        if not fut.done() or fut.result() is None:
            raise RuntimeError("set_parameters call failed")
        if not all(r.successful for r in fut.result().results):
            raise RuntimeError("one or more parameters rejected")

    def activate(self) -> None:
        self.wait_for("switch_controller", self.cli_switch)
        req = SwitchController.Request()
        req.activate_controllers = [self.ctrl_name]
        req.strictness = SwitchController.Request.STRICT
        req.timeout = 3.0
        fut = self.cli_switch.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=8.0)
        if not fut.done() or fut.result() is None or not fut.result().ok:
            raise RuntimeError("switch_controller failed to activate")


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: activate_diff_drive.py <diff_drive_params.yaml>")
        return 2
    yaml_path = sys.argv[1]
    rclpy.init()
    node = DiffDriveActivator(yaml_path)
    try:
        node.get_logger().info("Loading controller...")
        node.load_controller()
        node.get_logger().info("Setting parameters...")
        node.set_params()
        node.get_logger().info("Activating controller...")
        node.activate()
        node.get_logger().info("diff_drive_controller active")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
