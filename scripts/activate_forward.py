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


class ForwardActivator(Node):
    def __init__(self, yaml_path: str) -> None:
        super().__init__("forward_activator")
        self.cm = "/controller_manager"
        self.yaml_path = yaml_path
        self.left = "left_wheel_velocity_controller"
        self.right = "right_wheel_velocity_controller"

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
        req = LoadController.Request()
        req.name = name
        fut = self.cli_load.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=8.0)
        if not fut.done() or fut.result() is None or not fut.result().ok:
            raise RuntimeError(f"load_controller failed for {name}")

    @staticmethod
    def _to_param_msgs(mapping: dict) -> list[ParamMsg]:
        msgs: list[ParamMsg] = []
        for k, v in mapping.items():
            pv = ParameterValue()
            if isinstance(v, bool):
                pv.type = ParameterValue.TYPE_BOOL
                pv.bool_value = v
            elif isinstance(v, int):
                pv.type = ParameterValue.TYPE_INTEGER
                pv.integer_value = v
            elif isinstance(v, float):
                pv.type = ParameterValue.TYPE_DOUBLE
                pv.double_value = v
            elif isinstance(v, str):
                pv.type = ParameterValue.TYPE_STRING
                pv.string_value = v
            elif isinstance(v, list) and all(isinstance(x, str) for x in v):
                pv.type = ParameterValue.TYPE_STRING_ARRAY
                pv.string_array_value = v
            elif isinstance(v, list) and all(isinstance(x, float) for x in v):
                pv.type = ParameterValue.TYPE_DOUBLE_ARRAY
                pv.double_array_value = v
            elif isinstance(v, list) and all(isinstance(x, int) for x in v):
                pv.type = ParameterValue.TYPE_INTEGER_ARRAY
                pv.integer_array_value = v
            else:
                pv.type = ParameterValue.TYPE_STRING
                pv.string_value = str(v)
            pm = ParamMsg()
            pm.name = k
            pm.value = pv
            msgs.append(pm)
        return msgs

    def set_from_yaml(self, ctrl: str, data: dict) -> None:
        # Accept either manager-nested or top-level controller params
        params: dict
        if ctrl in data and isinstance(data[ctrl], dict):
            inner = data[ctrl]
            params = inner.get("ros__parameters", inner)
        elif "controller_manager" in data:
            cm = data["controller_manager"]
            if isinstance(cm, dict) and "ros__parameters" in cm:
                rp = cm["ros__parameters"]
                if isinstance(rp, dict) and ctrl in rp and isinstance(rp[ctrl], dict):
                    inner = rp[ctrl]
                    params = inner.get("ros__parameters", inner)
                else:
                    params = {}
            else:
                params = {}
        else:
            params = {}

        cli = self.create_client(SetParameters, f"/{ctrl}/set_parameters")
        self.wait(f"{ctrl} set_parameters", cli)
        req = SetParameters.Request()
        req.parameters = self._to_param_msgs(params)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=8.0)
        if not fut.done() or fut.result() is None or not all(r.successful for r in fut.result().results):
            raise RuntimeError(f"set_parameters failed for {ctrl}")

    def activate_both(self) -> None:
        self.wait("switch_controller", self.cli_switch)
        req = SwitchController.Request()
        req.activate_controllers = [self.left, self.right]
        req.strictness = SwitchController.Request.STRICT
        req.timeout = 3.0
        fut = self.cli_switch.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=8.0)
        if not fut.done() or fut.result() is None or not fut.result().ok:
            raise RuntimeError("switch_controller failed")


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: activate_forward.py <wheels_forward.yaml>")
        return 2
    yaml_path = sys.argv[1]
    with open(yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    rclpy.init()
    node = ForwardActivator(yaml_path)
    try:
        node.get_logger().info("Ensuring forward controllers loaded…")
        node.ensure_loaded(node.left)
        node.ensure_loaded(node.right)
        node.get_logger().info("Setting params from YAML…")
        node.set_from_yaml(node.left, data)
        node.set_from_yaml(node.right, data)
        node.get_logger().info("Activating controllers…")
        node.activate_both()
        node.get_logger().info("Forward controllers active")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

