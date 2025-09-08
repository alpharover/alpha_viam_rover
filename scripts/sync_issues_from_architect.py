#!/usr/bin/env python3
"""
Sync GitHub issues to architect's bundle: create milestones/labels, update or
create issues with titles/bodies/labels/milestones.

Usage:
  GITHUB_TOKEN=... python scripts/sync_issues_from_architect.py --repo alpharover/alpha_viam_rover

This script is idempotent: it updates existing issues matched by exact title.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional
import urllib.request


@dataclass
class IssueSpec:
    title: str
    body: str
    labels: List[str]
    milestone: str


MILESTONES = [
    "Milestone A: Phase 0 — Baseline bring‑up",
    "Milestone B: Phases 1–2 — Buses, IMU, Power",
    "Milestone C: Phases 4–5 — LiDAR, Teleop, Viz",
    "Milestone D: Tooling — CI, Provisioning",
]


LABELS: Dict[str, str] = {
    # phases
    "phase:0": "1f77b4",
    "phase:1": "aec7e8",
    "phase:2": "ff7f0e",
    "phase:3": "ffbb78",
    "phase:4": "2ca02c",
    "phase:5": "98df8a",
    "phase:infra": "9467bd",
    # areas
    "area:bringup": "17a2b8",
    "area:hardware": "8c564b",
    "area:drivers": "e377c2",
    "area:sensors": "7f7f7f",
    "area:telemetry": "bcbd22",
    "area:control": "d62728",
    "area:ci": "2ca02c",
    "area:provisioning": "6b6ecf",
    "area:networking": "1f77b4",
    # types
    "type:acceptance": "17a2b8",
    "type:setup": "8c564b",
    "type:test": "7f7f7f",
    "type:device": "e377c2",
    "type:tooling": "bcbd22",
    "type:feature": "d62728",
    "type:infra": "9467bd",
    # priority & size
    "prio:P0": "d73a4a",
    "prio:P1": "fbca04",
    "prio:P2": "0e8a16",
    "size:S": "cfd3d7",
    "size:M": "c2e0c6",
    "size:L": "bfdadc",
}


def gh_request(url: str, method: str = "GET", token: Optional[str] = None, data: Optional[dict] = None):
    req = urllib.request.Request(url, method=method)
    if token:
        req.add_header("Authorization", f"token {token}")
    req.add_header("Accept", "application/vnd.github+json")
    if data is not None:
        raw = json.dumps(data).encode("utf-8")
        req.add_header("Content-Type", "application/json")
    else:
        raw = None
    with urllib.request.urlopen(req, data=raw) as resp:
        return json.load(resp)


def ensure_milestones(repo: str, token: str) -> Dict[str, int]:
    url = f"https://api.github.com/repos/{repo}/milestones?state=all&per_page=100"
    existing = {m["title"]: m["number"] for m in gh_request(url, token=token)}
    out: Dict[str, int] = {}
    for title in MILESTONES:
        if title in existing:
            out[title] = existing[title]
        else:
            m = gh_request(f"https://api.github.com/repos/{repo}/milestones", method="POST", token=token, data={"title": title})
            out[title] = m["number"]
    return out


def ensure_labels(repo: str, token: str):
    # Fetch existing labels
    labels = gh_request(f"https://api.github.com/repos/{repo}/labels?per_page=100", token=token)
    existing = {l["name"]: l for l in labels}
    for name, color in LABELS.items():
        if name in existing:
            # Optionally update color
            if existing[name].get("color") != color:
                gh_request(
                    f"https://api.github.com/repos/{repo}/labels/{name}",
                    method="PATCH",
                    token=token,
                    data={"new_name": name, "color": color},
                )
        else:
            gh_request(
                f"https://api.github.com/repos/{repo}/labels",
                method="POST",
                token=token,
                data={"name": name, "color": color},
            )


def get_issues_by_title(repo: str, token: str) -> Dict[str, dict]:
    # Gather open issues (first few pages)
    out: Dict[str, dict] = {}
    page = 1
    while True:
        url = f"https://api.github.com/repos/{repo}/issues?state=open&per_page=100&page={page}"
        res = gh_request(url, token=token)
        if not res:
            break
        for it in res:
            if "pull_request" in it:
                continue
            out[it["title"]] = it
        page += 1
        if len(res) < 100:
            break
    return out


def upsert_issue(repo: str, token: str, spec: IssueSpec, milestones: Dict[str, int], existing_by_title: Dict[str, dict]):
    milestone_num = milestones[spec.milestone]
    labels = spec.labels
    if spec.title in existing_by_title:
        issue = existing_by_title[spec.title]
        number = issue["number"]
        gh_request(
            f"https://api.github.com/repos/{repo}/issues/{number}",
            method="PATCH",
            token=token,
            data={"title": spec.title, "body": spec.body, "labels": labels, "milestone": milestone_num},
        )
        return {"action": "updated", "number": number}
    else:
        it = gh_request(
            f"https://api.github.com/repos/{repo}/issues",
            method="POST",
            token=token,
            data={"title": spec.title, "body": spec.body, "labels": labels, "milestone": milestone_num},
        )
        return {"action": "created", "number": it["number"], "url": it.get("html_url")}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", required=True)
    args = ap.parse_args()
    token = os.environ.get("GITHUB_TOKEN")
    if not token:
        print("GITHUB_TOKEN required", file=sys.stderr)
        return 2

    # Architect bundle
    specs: List[IssueSpec] = [
        IssueSpec(
            title="Phase 0 — Finalize baseline bring‑up (time sync + talker/listener + proof bag)",
            labels=["phase:0", "area:bringup", "type:acceptance", "prio:P0", "size:S"],
            milestone="Milestone A: Phase 0 — Baseline bring‑up",
            body=(
                "Summary: Close out Phase 0 with explicit evidence: time sync verified, ROS 2 health checked, talker/listener demo recorded to MCAP, docs captured.\n\n"
                "Tasks\n"
                "- Verify system time sync (chrony or systemd-timesyncd) and record outputs in docs/bringup/time_sync.md.\n"
                "- Run ros2 doctor and capture results in the same doc.\n"
                "- Run talker/listener demo for 10–20 s and record a small MCAP in bags/phase0_talker_listener/.\n"
                "- Add a short ‘how to reproduce’ in docs/bringup/phase0.md with links to artifacts.\n\n"
                "Acceptance\n"
                "- docs/bringup/time_sync.md shows synchronized time and a passing ros2 doctor.\n"
                "- A <30 MB MCAP in bags/phase0_talker_listener/ plays back cleanly in Foxglove.\n"
                "- Checklist and links added to AGENTS_PROGRESS.md."
            ),
        ),
        IssueSpec(
            title="Phase 1 — Publish pinmap and enable overlays (I²C/SPI/PWM) + pigpio daemon",
            labels=["phase:1", "area:hardware", "type:setup", "prio:P0", "size:S"],
            milestone="Milestone B: Phases 1–2 — Buses, IMU, Power",
            body=(
                "Summary: Lock pin assignments and enable required kernel overlays. Launch pigpio as a system service for encoder edge timing and PWM.\n\n"
                "Tasks\n- Create hw/pinmap.yaml (I²C, SPI, PWM pins; encoder GPIOs; motor driver pins).\n- Enable I²C/SPI/PWM overlays and document the change in docs/hw/pinmap.md.\n- Install and enable pigpio service; capture a status screenshot in the doc.\n\n"
                "Acceptance\n- hw/pinmap.yaml exists, reviewed, and referenced by bring‑up.\n- Reboot shows I²C/SPI nodes present; pigpio active."
            ),
        ),
        IssueSpec(
            title="Phase 1 — Bus verification artifacts (prove I²C/SPI/uart are alive)",
            labels=["phase:1", "area:hardware", "type:test", "prio:P1", "size:S"],
            milestone="Milestone B: Phases 1–2 — Buses, IMU, Power",
            body=(
                "Summary: Produce quick, repeatable evidence that buses are functional.\n\n"
                "Tasks\n- Add a one‑pager docs/hw/bus_checks.md with screenshots/outputs from: i2cdetect -y 1, list of /dev/spidev*, and UART loopback or console check.\n- Save outputs under docs/hw/artifacts/phase1/.\n\n"
                "Acceptance\n- Doc shows expected devices (no phantom addresses), committed to repo, linked in AGENTS_PROGRESS.md."
            ),
        ),
        IssueSpec(
            title="Phase 2 — IMU (MPU‑6050) bring‑up + calibration & orientation sanity",
            labels=["phase:2", "area:drivers", "type:device", "prio:P0", "size:M"],
            milestone="Milestone B: Phases 1–2 — Buses, IMU, Power",
            body=(
                "Summary: Publish sensor_msgs/Imu at a stable rate with correct axis conventions; capture bias and verify gravity alignment.\n\n"
                "Tasks\n- Bring up IMU node (driver/config) and add to bringup/launch/.\n- Park the rover for 60 s; record MCAP; compute mean bias and noise.\n- Verify orientation in Foxglove while rotating the chassis; adjust transforms.\n- Document in docs/sensors/imu.md with plots/screenshots.\n\n"
                "Acceptance\n- /imu/data publishes at configured rate with bias noted in docs.\n- Gravity magnitude within ~±3% at rest; axes match URDF frames."
            ),
        ),
        IssueSpec(
            title="Phase 2 — Power telemetry (INA219) + calibration against bench meter",
            labels=["phase:2", "area:drivers", "type:device", "prio:P0", "size:S"],
            milestone="Milestone B: Phases 1–2 — Buses, IMU, Power",
            body=(
                "Summary: Provide accurate bus voltage/current; prove against a reference meter and log calibration constants.\n\n"
                "Tasks\n- Add INA219 node (or wrapper for hwmon) and publish voltage/current.\n- Cross‑check readings at idle and under load vs. a meter; capture data table.\n- Document in docs/sensors/power.md (include shunt value, expected ranges).\n\n"
                "Acceptance\n- Readings within ±5% of reference at two loads; doc shows methodology and results."
            ),
        ),
        IssueSpec(
            title="Phase 4 — LiDAR (YDLIDAR G4) bring‑up + udev + sanity scan",
            labels=["phase:4", "area:sensors", "type:device", "prio:P1", "size:M"],
            milestone="Milestone C: Phases 4–5 — LiDAR, Teleop, Viz",
            body=(
                "Summary: Run driver, set persistent device path via udev, record a short scan near a planar wall, and verify range/noise look sane.\n\n"
                "Tasks\n- Install and launch ydlidar_ros2_driver; bind serial by udev rule.\n- Record a 30 s MCAP facing a wall ~1.0–1.5 m away; screenshot Foxglove.\n- Add troubleshooting notes to docs/sensors/lidar.md.\n\n"
                "Acceptance\n- /scan stable; ranges in expected band; artifacts committed and linked."
            ),
        ),
        IssueSpec(
            title="Phase 5 — Foxglove layout + record/replay pipeline",
            labels=["phase:5", "area:telemetry", "type:tooling", "prio:P1", "size:S"],
            milestone="Milestone C: Phases 4–5 — LiDAR, Teleop, Viz",
            body=(
                "Summary: Ship a repo‑owned Foxglove layout and a scripted MCAP record/replay workflow.\n\n"
                "Tasks\n- Add foxglove_bridge to bring‑up; commit a layout JSON in configs/foxglove/.\n- Provide scripts/record_mcap.sh and a small sample bag in bags/samples/.\n- Doc: docs/tools/foxglove.md with screenshots and ‘how to replay.’\n\n"
                "Acceptance\n- One‑click launch shows the layout; sample bag replays cleanly."
            ),
        ),
        IssueSpec(
            title="Phase 5 — Teleop (keyboard + gamepad) and cmd_vel wiring",
            labels=["phase:5", "area:control", "type:feature", "prio:P1", "size:S"],
            milestone="Milestone C: Phases 4–5 — LiDAR, Teleop, Viz",
            body=(
                "Summary: Provide safe teleop for bench testing; verify topic flow end‑to‑end.\n\n"
                "Tasks\n- Add teleop_twist_keyboard and gamepad teleop; wire to cmd_vel.\n- Implement a rate limiter and dead‑man (no input → zero twist).\n- Document in docs/control/teleop.md.\n\n"
                "Acceptance\n- Echo of cmd_vel shows bounded rates; safety behavior verified and documented."
            ),
        ),
        IssueSpec(
            title="Phase 3 — ros2_control HW interface stub + watchdog (pre‑motor)",
            labels=["phase:3", "area:control", "type:infra", "prio:P2", "size:M"],
            milestone="Milestone B: Phases 1–2 — Buses, IMU, Power",
            body=(
                "Summary: Create the hardware interface skeleton and watchdog logic with ‘fake’ back‑end so the stack compiles/tests before physical motors.\n\n"
                "Tasks\n- Implement interface class and watchdog timer; expose diagnostics.\n- Unit tests for watchdog triggers; dry‑run launch.\n- Doc: docs/control/motor_interface.md.\n\n"
                "Acceptance\n- CI runs tests; launch succeeds; diagnostics topic present."
            ),
        ),
        IssueSpec(
            title="CI — Make quality gates strict (no ‘|| true’ escapes)",
            labels=["phase:infra", "area:ci", "type:infra", "prio:P0", "size:S"],
            milestone="Milestone D: Tooling — CI, Provisioning",
            body=(
                "Summary: Fail fast on lint/test/launch; add model and config checks.\n\n"
                "Tasks\n- Remove permissive || true in CI; Ruff and pytest fail on error.\n- Add check_urdf step and YAML schema tests for configs.\n- Add a smoke ros2 launch bringup job (headless) that returns success.\n\n"
                "Acceptance\n- CI blocks on any lint/test/launch error; badges green on main."
            ),
        ),
        IssueSpec(
            title="Provisioning — Ansible roles for overlays, pigpio, udev, services",
            labels=["phase:infra", "area:provisioning", "type:infra", "prio:P1", "size:M"],
            milestone="Milestone D: Tooling — CI, Provisioning",
            body=(
                "Summary: Replace manual steps with idempotent roles and linted playbooks.\n\n"
                "Tasks\n- Roles: overlays, pigpio, udev, systemd units, Foxglove bridge, Netplan.\n- Add ansible-lint and yamllint to CI; sample inventory in provisioning/.\n- Doc: docs/provisioning/ansible.md.\n\n"
                "Acceptance\n- Fresh Pi reaches Phase‑1 ‘bus checks’ in one play; CI lints pass."
            ),
        ),
        IssueSpec(
            title="Networking — Integrate external Alfa USB Wi‑Fi as primary (tracked)",
            labels=["phase:infra", "area:networking", "type:device", "prio:P1", "size:S"],
            milestone="Milestone D: Tooling — CI, Provisioning",
            body=(
                "Summary: Add a high‑gain Alfa USB adapter, make it the preferred interface, and harden connectivity for ROS 2 traffic.\n\n"
                "Tasks\n- Select in‑kernel‑supported Alfa model; verify chipset and kernel support; document.\n- Ensure power headroom and physical placement; add udev rule for persistent naming and NetworkManager profile.\n- Fix regulatory domain; prefer 5 GHz; test with iperf3 and ros2 pub/echo between machines.\n\n"
                "Acceptance\n- External adapter is default; throughput and packet loss documented; fallback path defined."
            ),
        ),
    ]

    milestones = ensure_milestones(args.repo, token)
    ensure_labels(args.repo, token)
    existing = get_issues_by_title(args.repo, token)

    results = []
    for spec in specs:
        results.append(upsert_issue(args.repo, token, spec, milestones, existing))
    print(json.dumps({"results": results}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

