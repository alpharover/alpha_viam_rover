#!/usr/bin/env python3
from __future__ import annotations

import sys
import time
import yaml

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterValue
from rcl_interfaces.srv import SetParameters
from controller_manager_msgs.srv import LoadController, SwitchController, ListControllers


class ControllerActivator(Node):
    def __init__(self, diff_yaml_path: str) -> None:
        super().__init__("controller_activator")
        self.cm = "/controller_manager"
        self.diff = "diff_drive_controller"
        self.jsb = "joint_state_broadcaster"
        self.diff_yaml_path = diff_yaml_path
        self.cli_load = self.create_client(LoadController, f"{self.cm}/load_controller")
        self.cli_switch = self.create_client(SwitchController, f"{self.cm}/switch_controller")
        self.cli_list = self.create_client(ListControllers, f"{self.cm}/list_controllers")

    def wait(self, name: str, client, timeout: float = 10.0) -> None:
        end = time.time() + timeout
        while time.time() < end:
            if client.wait_for_service(timeout_sec=0.5):
                return
        raise RuntimeError(f"Service not available: {name}")

    def ensure_loaded(self, name: str) -> None:
        self.wait("load_controller", self.cli_load)
        # Check if already loaded
        try:
            self.wait("list_controllers", self.cli_list)
            fut = self.cli_list.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            if fut.done() and fut.result():
                for c in fut.result().controller:
                    if c.name == name:
                        return
        except Exception:
            pass
        # Load
        req = LoadController.Request()
        req.name = name
        fut = self.cli_load.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=8.0)
        if not fut.done() or fut.result() is None or not fut.result().ok:
            raise RuntimeError(f"load_controller failed for {name}")

    def set_diff_params(self) -> None:
        with open(self.diff_yaml_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        if "ros__parameters" in data:
            params = data["ros__parameters"]
        elif self.diff in data:
            inner = data[self.diff]
            params = inner.get("ros__parameters", inner) if isinstance(inner, dict) else {}
        else:
            params = data
        cli = self.create_client(SetParameters, f"/{self.diff}/set_parameters")
        self.wait("diff_drive set_parameters", cli)

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
        if (
            not fut.done()
            or fut.result() is None
            or not all(r.successful for r in fut.result().results)
        ):
            raise RuntimeError("diff_drive set_parameters failed")

    def activate(self) -> None:
        self.wait("switch_controller", self.cli_switch)
        req = SwitchController.Request()
        req.activate_controllers = [self.jsb, self.diff]
        req.strictness = SwitchController.Request.STRICT
        req.timeout = 3.0
        fut = self.cli_switch.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=8.0)
        if not fut.done() or fut.result() is None or not fut.result().ok:
            raise RuntimeError("switch_controller failed")


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: activate_controllers.py <diff_drive_params.yaml>")
        return 2
    rclpy.init()
    node = ControllerActivator(sys.argv[1])
    try:
        node.get_logger().info("Loading controllers...")
        node.ensure_loaded(node.jsb)
        node.ensure_loaded(node.diff)
        node.get_logger().info("Setting diff_drive parameters...")
        node.set_diff_params()
        node.get_logger().info("Activating controllers...")
        node.activate()
        node.get_logger().info("Controllers active")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
