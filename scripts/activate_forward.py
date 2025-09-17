#!/usr/bin/env python3
from __future__ import annotations

import sys
import time
import yaml

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter as ParamMsg
from ros2param.api import call_set_parameters
from controller_manager import set_controller_parameters_from_param_files
from controller_manager_msgs.srv import LoadController, SwitchController, ListControllers, ConfigureController
from controller_manager_msgs.msg import ControllerState
from builtin_interfaces.msg import Duration
from rclpy.parameter import Parameter


class ForwardActivator(Node):
    def __init__(self, yaml_path: str) -> None:
        super().__init__("forward_activator")
        self.cm = "/controller_manager"
        self.yaml_path = yaml_path
        self.left = "left_wheel_velocity_controller"
        self.right = "right_wheel_velocity_controller"

        self.cli_load = self.create_client(LoadController, f"{self.cm}/load_controller")
        self.cli_configure = self.create_client(ConfigureController, f"{self.cm}/configure_controller")
        self.cli_switch = self.create_client(SwitchController, f"{self.cm}/switch_controller")
        self.cli_list = self.create_client(ListControllers, f"{self.cm}/list_controllers")

    def wait(self, name: str, client, timeout: float = 10.0) -> None:
        end = time.time() + timeout
        while time.time() < end:
            if client.wait_for_service(timeout_sec=0.5):
                return
        raise RuntimeError(f"Service not available: {name}")

    def _list_controllers(self) -> list[ControllerState]:
        self.wait("list_controllers", self.cli_list)
        fut = self.cli_list.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.done() and fut.result():
            return list(fut.result().controller)
        return []

    def _controller_loaded(self, name: str, timeout: float = 0.0) -> bool:
        end = time.time() + timeout
        while True:
            for ctrl in self._list_controllers():
                if ctrl.name == name:
                    return True
            if time.time() >= end:
                return False
            time.sleep(0.2)

    def ensure_loaded(self, name: str) -> None:
        self.wait("load_controller", self.cli_load)
        if self._controller_loaded(name):
            return

        req = LoadController.Request()
        req.name = name
        fut = self.cli_load.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=8.0)
        ok = fut.done() and fut.result() is not None and fut.result().ok
        if not ok:
            self.get_logger().warn(f"load_controller returned not-ok for {name}; verifying via list_controllers")
        if self._controller_loaded(name, timeout=4.0):
            return
        raise RuntimeError(f"load_controller failed for {name}")

    @staticmethod
    def _to_param_msgs_with_prefix(ctrl: str, mapping: dict) -> list[ParamMsg]:
        msgs: list[ParamMsg] = []
        for key, value in mapping.items():
            name = f"{ctrl}.{key}"
            param = Parameter(name=name, value=value)
            msgs.append(param.to_parameter_msg())
        return msgs

    def set_from_yaml(self, ctrl: str, data: dict, yaml_path: str | None = None) -> None:
        # Accept controller-rooted or manager-nested params
        params: dict
        if ctrl in data and isinstance(data[ctrl], dict):
            inner = data[ctrl]
            params = inner.get("ros__parameters", inner)
        elif "controller_manager" in data and isinstance(data["controller_manager"], dict):
            rp = data["controller_manager"].get("ros__parameters", {})
            params = rp.get(ctrl, {}).get("ros__parameters", rp.get(ctrl, {})) if isinstance(rp, dict) else {}
        else:
            params = {}

        ok = True
        # Persist metadata for future shims/debugging.
        if yaml_path:
            ok = set_controller_parameters_from_param_files(self, self.cm, ctrl, [yaml_path])

        # Explicitly inject typed parameters to work around Humble bug where controllers see
        # empty declarations during configure (refs ADR-0006).
        if params:
            req_params = self._to_param_msgs_with_prefix(ctrl, params)
            resp = call_set_parameters(node=self, node_name=self.cm, parameters=req_params)
            failures = [res for res in resp.results if not res.successful]
            if failures:
                self.get_logger().error(f"Setting typed parameters for {ctrl} failed: {[res.reason for res in failures]}")
            ok = ok and not failures
        if not ok:
            raise RuntimeError(f"parameter injection failed for {ctrl}")

    def activate_both(self) -> None:
        # Ensure configured first
        self.wait("configure_controller", self.cli_configure)
        for name in [self.left, self.right]:
            reqc = ConfigureController.Request(); reqc.name = name
            futc = self.cli_configure.call_async(reqc)
            rclpy.spin_until_future_complete(self, futc, timeout_sec=6.0)
            if not futc.done() or futc.result() is None or not futc.result().ok:
                raise RuntimeError(f"configure_controller failed for {name}")

        self.wait("switch_controller", self.cli_switch)
        req = SwitchController.Request()
        req.activate_controllers = [self.left, self.right]
        req.strictness = SwitchController.Request.STRICT
        req.timeout = Duration(sec=3, nanosec=0)
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
        node.get_logger().info("Priming controller parameters from YAML…")
        # Prefer controller-scoped spawner file if present
        spawner_file = yaml_path
        # If given file is manager-scoped (wheels_forward.yaml), use configs/spawner/forward.yaml if present
        import os
        repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        spawner_default = os.path.join(repo_root, 'configs', 'spawner', 'forward.yaml')
        if os.path.exists(spawner_default):
            spawner_file = spawner_default
        node.set_from_yaml(node.left, data, spawner_file)
        node.set_from_yaml(node.right, data, spawner_file)

        node.get_logger().info("Ensuring forward controllers loaded…")
        node.ensure_loaded(node.left)
        node.ensure_loaded(node.right)

        # Some controller_manager builds wipe params during load; reapply to be safe.
        node.get_logger().info("Setting params from YAML…")
        node.set_from_yaml(node.left, data, spawner_file)
        node.set_from_yaml(node.right, data, spawner_file)

        node.get_logger().info("Activating controllers…")
        node.activate_both()
        node.get_logger().info("Forward controllers active")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
