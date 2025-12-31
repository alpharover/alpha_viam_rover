#!/usr/bin/env python3
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def check_controllers_yaml(p: Path) -> list[str]:
    issues = []
    if not p.exists():
        issues.append(f"missing: {p}")
        return issues
    txt = p.read_text()
    if "params_file:" in txt:
        issues.append(
            "controllers.yaml: avoid manager-side 'params_file'; use spawner --param-file instead"
        )
    if "controller_manager:" not in txt or "ros__parameters:" not in txt:
        issues.append("controllers.yaml: expected controller_manager.ros__parameters root")
    # Require that either controller has inline params (Humble-safe) or we ship a diff_drive param file
    if "diff_drive_controller:" in txt and "left_wheel_names:" not in txt:
        issues.append(
            "controllers.yaml: diff_drive_controller missing inline params; provide them or --param-file with correct YAML"
        )
    return issues


def check_controller_params(p: Path) -> list[str]:
    issues = []
    if not p.exists():
        issues.append(f"missing: {p}")
        return issues
    txt = p.read_text().strip()
    if not (txt.startswith("/**:") or txt.startswith("diff_drive_controller:")):
        issues.append(
            f"{p.name}: expected '/**:' or 'diff_drive_controller:' root with 'ros__parameters' block"
        )
    # quick field presence hints
    for key in ["left_wheel_names", "right_wheel_names", "wheel_separation", "wheel_radius"]:
        if key not in txt:
            issues.append(f"{p.name}: missing '{key}'")
    return issues


def main() -> int:
    problems: list[str] = []
    problems += check_controllers_yaml(ROOT / "configs" / "controllers.yaml")

    # accept either filename; prefer new one
    new_p = ROOT / "configs" / "diff_drive.params.yaml"
    old_p = ROOT / "configs" / "diff_drive_params.yaml"
    if new_p.exists():
        problems += check_controller_params(new_p)
    elif old_p.exists():
        problems += check_controller_params(old_p)
    else:
        problems.append("missing: diff_drive parameter file (diff_drive.params.yaml)")

    if problems:
        print("Config lint: FAIL")
        for p in problems:
            print(f" - {p}")
        return 1
    print("Config lint: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
