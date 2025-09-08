#!/usr/bin/env python3
"""Import a ROS 2 launch file and ensure it returns a LaunchDescription.

This does not start ROS; it only imports and constructs the description.
"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: launch_smoke.py <path/to/launch.py>")
        return 2
    target = Path(sys.argv[1]).resolve()
    if not target.exists():
        print(f"Launch file not found: {target}")
        return 2

    spec = importlib.util.spec_from_file_location(target.stem, str(target))
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    if not hasattr(mod, "generate_launch_description"):
        print("No generate_launch_description() found")
        return 3
    ldesc = mod.generate_launch_description()
    try:
        from launch import LaunchDescription  # type: ignore
    except Exception:
        print("Warning: launch package not available; cannot type-check")
        return 0
    if not isinstance(ldesc, LaunchDescription):
        print("generate_launch_description() did not return LaunchDescription")
        return 4
    print("OK: LaunchDescription constructed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
