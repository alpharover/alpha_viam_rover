Viam Rover v2 — Agent Operating Guide (ROS 2 Humble, RPi4)

Purpose
Teach coding agents (and humans) how to work on this robot. Use this file to find the right sub‑guides, follow safety rules, and record progress.

Repository shape

* `bringup/` — OS/ROS bring‑up and launch orchestration
* `drivers/` — IMU (MPU‑6050), power (INA219), encoders, motor HW (L298N)
* `nav/` — robot_localization, slam_toolbox, Nav2
* `perception/` — RGB/Depth camera stacks (future; may move to Jetson)
* `can/` — SocketCAN (Waveshare MCP2515) + servo control
* `configs/` — ros2_control, controller configs, QoS, DDS XML
* `urdf/` — robot model + ros2_control tags
* `docs/` — ADRs, bring‑up checklists, hardware, Foxglove/MCAP
* `tests/` — bench and acceptance scripts
* `bags/` — short MCAP samples and context readme

How agents should navigate

1. Read this file.
2. Open the nearest AGENTS.md in your working directory (drivers, nav, etc.). The nearest file overrides the root for local tasks. ([Agents][1])
3. Log progress in `AGENTS_PROGRESS.md` (root) after each task.
4. For any high‑impact change, follow the Escalation procedure below.

Build / test / evidence (always)

* Bootstrap with rosdep; build with colcon; tests green; `ros2 doctor` clean.
* Record a short MCAP bag for anything that moves, measures, or maps; add link in your PR and in `AGENTS_PROGRESS.md`. (Use Git LFS if bags exceed normal repo limits.) ([GitHub Docs][4], [Git Large File Storage][5])

Progress tracking (required)
Update `AGENTS_PROGRESS.md` after each unit of work with:

* Task ID (issue/PR), phase, subsystem, files touched
* Outcome (pass/fail against acceptance tests)
* Evidence links (MCAP in `bags/samples/`, Foxglove layout JSON, screenshots)
* Follow‑ups or risks
  Keep entries terse and append‑only.

Documentation rules

* Decisions → add/update a 1‑page ADR in `docs/ADR/` and link it in your PR.
* Configs → change the YAML in `configs/` (don’t scatter settings).
* Hardware/pin changes → update `hw/pinmap.yaml` and `docs/hw/*`.

Safety & guardrails (do not bypass)

* Motors disabled on boot; watchdog on `/cmd_vel`; E‑stop documented.
* Respect REP‑103/105 frames/units (`map/odom/base_link`, SI).
* Never hard‑code secrets.
* Don’t change topic names or message types without an ADR and approval.

Escalation procedure (architect approval required)
Mark PR with label `needs-architect` and request review from `@PROJECT_ARCHITECT_HANDLE` for any change that touches:

* Motor control (`ros2_control` HW, PWM ranges, watchdogs)
* GPIO/pin maps, I²C addresses, or power paths
* Message schemas / topic names / TF tree
* DDS networking, multi‑robot namespace/DOMAIN_ID, discovery config
* SLAM/Nav2 planners or costmaps
* Any hardware addition or driver swap
  Enforcement: CODEOWNERS requires architect review on protected paths, and branch protection requires an approving review before merge. ([GitHub Docs][3])

Monorepo pattern
Large repos may ship nested AGENTS.md in subfolders; the nearest one governs. Keep this root file short and link out. ([Agents][1])

Sub‑guides (agents open these next)

* `bringup/AGENTS.md` — install, launch, health checks
* `drivers/AGENTS.md` — IMU/INA219/encoders/L298N specifics
* `nav/AGENTS.md` — EKF, SLAM, Nav2
* `perception/AGENTS.md` — cameras/depth (future)
* `can/AGENTS.md` — SocketCAN + servos

[1]: https://agents.md/ "AGENTS.md"
[3]: https://docs.github.com/articles/about-code-owners?utm_source=chatgpt.com "About code owners"
[4]: https://docs.github.com/repositories/working-with-files/managing-large-files/about-git-large-file-storage?utm_source=chatgpt.com "About Git Large File Storage"
[5]: https://git-lfs.com/?utm_source=chatgpt.com "Git Large File Storage"

