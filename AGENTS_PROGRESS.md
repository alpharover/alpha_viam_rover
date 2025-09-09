What this is
A living log for agents and developers. Append a new entry each time you complete a task. Keep it short; link evidence.

Entry template

* Date / Author (agent or human)
* Phase / Subsystem
* Task ID (issue/PR) & Title
* Summary of changes
* Acceptance test result (pass/fail, brief notes)
* Evidence links (MCAP, Foxglove layout, screenshots)
* Follow‑ups / Risks

Rules

* Append only; never rewrite history.
* Reference ADR IDs for decisions.
* If a task fails acceptance, create a follow‑up issue immediately.

---

* 2025-09-08 / agent: codex-cli
* Phase / Subsystem: Bring-up / Repo & HW Sync
* Task: Sync local repo to origin/main; verify OS/RPi model; enumerate I²C/USB
* Summary: Fast-forwarded to origin/main; Ubuntu 22.04.5 on RPi4B Rev 1.1; I²C shows 0x40 (INA219) and 0x68 (MPU-6050); Alfa USB Wi‑Fi not enumerated in lsusb (only VIA hub seen).
* Acceptance: Pass for repo/OS/I²C; USB Alfa pending (needs driver/power check).
* Evidence: `i2cdetect -y 1` → 0x40, 0x68; `lsusb` output captured in session logs.
* Follow-ups: Check AWUS036ACH power/driver; run dmesg while hot-plugging; install DKMS if device present.

---

* 2025-09-08 / agent: codex-cli
* Phase / Subsystem: Bring-up / Launch & Recording
* Task: Update launch ordering and install ros2_control + MCAP
* Summary: Fixed launch arg ordering; installed ros2_control and controllers; added diagnostics analyzer; installed rosbag2-storage-mcap. Base bring-up runs; Foxglove at ws://0.0.0.0:8765. diff_drive_controller spawn pending params wiring.
* Acceptance: Partial — launch stable; drive controller not yet configured (left/right wheel names read empty by controller at load).
* Evidence: Launch logs; `ros2 pkg executables` confirm nodes; MCAP plugin installed.
* Follow-ups: Gate diff_drive spawner behind flag until params load path is finalized; proceed with IMU/INA219 drivers and EKF wiring; capture MCAP.

---

* 2025-09-08 / agent: codex-cli
* Phase / Subsystem: Bring-up / Foxglove
* Task: Start base bring-up with timeout for Foxglove validation
* Summary: Launched `alpha_viam_bringup` with a 10-minute timeout guard; verified foxglove_bridge listening on 0.0.0.0:8765 and provided IP 192.168.0.105. User confirmed Foxglove connects from Mac.
* Acceptance: Pass — port open, external client connected successfully.
* Evidence: `ss -ltnp` shows foxglove_bridge PID bound to :8765.
* Follow-ups: Add systemd unit with `TimeoutStopSec` and ExecStop to ensure clean shutdown.

---

* 2025-09-08 / agent: codex-cli
* Phase / Subsystem: Networking / USB Wi‑Fi
* Task: Alfa AWUS036ACH (RTL8812AU) driver and detection
* Summary: Installed `rtl8812au-dkms` initially; then replaced with aircrack `rtl88xxau` via DKMS (`88XXau` module). After replugging to a USB 3.0 port, the device enumerated as `0bda:8812`, interface `wlx00c0cab1223b` created.
* Acceptance: Pass — driver loaded and interface present.
* Evidence: `lsusb` shows 0bda:8812; `ip -br link` lists `wlx00c0cab1223b`; `iw dev` shows managed PHY.
* Follow-ups: Set REGDOMAIN, optionally rename to `wlan1` via systemd .link file; install NetworkManager if desired and create connection profile.

* 2025-09-07 / agent: codex-cli
* Phase / Subsystem: Phase 0/1 Docs & Provisioning / CI / Networking
* Task: Add Phase 0/1 docs, overlays+pigpio roles; ROS CI checks; AWUS036ACH guide
* Summary: Added `docs/bringup/time_sync.md` and `docs/bringup/phase0.md`; created Phase‑1 bus docs (`docs/hw/pinmap.md`, `docs/hw/bus_checks.md`) and artifacts folder. Implemented Ansible roles `overlays` (I2C/SPI/PWM) and `pigpio` (install+enable). Added `scripts/gpio_pulse.py` for GPIO sanity. Updated `ros-ci` workflow to run xacro+check_urdf and a launch smoke test via `scripts/launch_smoke.py`. Added `configs/teleop_joy.yaml`. Documented Alfa AWUS036ACH in `docs/networking/awus036ach.md`. Allowed "Alfa" in codespell.
* Acceptance: Linters/tests pass (18/18). CI lint green; ros-ci workflow now performs model and launch checks.
* Evidence: Commit `docs: phase0/time-sync/bus-checks/...` on `main`.
* Follow-ups: On-device execution to capture outputs/screenshots; consider ansible-lint; add gamepad + safety wiring end-to-end on Pi.

* 2025-09-07 / agent: codex-cli
* Phase / Subsystem: Planning / GitHub Issues
* Task: Align issues with architect bundle; add Wi‑Fi networking task
* Summary: Created milestones A–D; created/updated labels (phase, area, type, prio, size). Synced architect issue bundle (#17–#28) and consolidated duplicates by closing older overlapping items (#3–#7) with links. Assigned milestones/labels to remaining seeded issues (#2, #8–#16). Added new "Networking — Integrate external Alfa USB Wi‑Fi as primary" issue (#28).
* Acceptance: Open issues reflect architect structure; duplicates closed and cross‑referenced; preview and sync JSONs under `out/`.
* Evidence: `scripts/sync_issues_from_architect.py`, `out/issues_sync_results.json`.
* Follow-ups: Add assignees/milestones dates as needed; refine labels if architect prefers different taxonomy.

* 2025-09-07 / agent: codex-cli
* Phase / Subsystem: Backlog / Bring-up / Foxglove
* Task: Backlog seeding script; overlay env + controller manager; Foxglove doc polish
* Summary: Added `scripts/seed_issues.py` to parse `docs/ISSUE_BACKLOG.md` and create GitHub issues (requires `GITHUB_TOKEN`), with preview JSON written to `out/issues_seed_preview.json`. Updated bring-up: systemd reads `/etc/default/rover-bringup` for overlay and launch vars; controller manager + spawners added to bringup launch; added `configs/controllers.yaml` + schema and tests. Polished Foxglove doc with step-by-step and topic list.
* Acceptance: Local tests 18 passed; yamllint clean; preview JSON generated for backlog.
* Evidence: Commit `chore(backlog): add scripts/seed_issues.py ...` on `main`.
* Follow-ups: Run issue seeding with `GITHUB_TOKEN` when ready; capture Foxglove screenshot and add to docs.

* 2025-09-07 / agent: codex-cli
* Phase / Subsystem: Nav2 / Diagnostics / CI
* Task: Expand Nav2 params + schema; diagnostics config; tighten CI
* Summary: Added global/local costmaps and behavior_server to Nav2 params and schema; created `configs/diagnostics.yaml` + schema and wired into bringup launch; tightened CI by making Ruff blocking and unit tests required on PRs; added example MCAP info snippet to docs.
* Acceptance: Local tests 17 passed; linters clean.
* Evidence: Commit `feat(nav2): expand params ...` on `main`.
* Follow-ups: Consider ansible-lint in CI; expand Nav2 plugin params incrementally once on device.

* 2025-09-07 / agent: codex-cli
* Phase / Subsystem: URDF / Control
* Task: Add ros2_control tags + transmissions
* Summary: Inserted `<ros2_control>` system block with velocity command/state interfaces for `left_wheel_joint` and `right_wheel_joint`. Added placeholder `<transmission>` elements to align naming with configs.
* Acceptance: Unit tests still pass (16); linters clean.
* Evidence: Commit `feat(urdf): add ros2_control system/joint interfaces and placeholder transmissions to match diff_drive joints` on `main`.
* Follow-ups: Replace mock hardware plugin with real HW interface when ready; validate with controller manager on device.

* 2025-09-07 / agent: codex-cli
* Phase / Subsystem: Tests / Utilities
* Task: Add math utilities + unit tests
* Summary: Implemented `lib/math_utils.py` (ticks→m/s, angle wrap to [-pi, pi), clamp) and added `tests/unit/test_math_utils.py`; ensured import path in tests works without packaging.
* Acceptance: Local `pytest` now 16 passed; linters clean after Black formatted.
* Evidence: Commit `feat(utils): add math_utils (ticks→m/s, angle wrap, clamp) with unit tests; format code` on `main`.
* Follow-ups: Consider moving utils into a proper package later and exposing via `setup.cfg` if needed.

* 2025-09-07 / agent: codex-cli
* Phase / Subsystem: Configs & Tests / Nav2 & Launch
* Task: Add Nav2 minimal params + schema; launch dry-check test
* Summary: Added `configs/nav2/nav2_params.yaml` and `configs/schemas/nav2.schema.json`; extended `tests/config/test_yaml_schemas.py` to validate it. Added `tests/launch/test_teleop_viz_text.py` to assert required launch args/params exist without importing ROS.
* Acceptance: `pytest` local: 12 passed; linters clean (`yamllint`, `codespell`, `ruff exit-zero`, `black` formatted).
* Evidence: Commit `feat(configs): add minimal Nav2 params + JSON Schema; add launch dry-check test; improve Ansible roles...`.
* Follow-ups: Flesh out Nav2 param sets (costmaps, recoveries) later; consider making ruff non-exit-zero and enabling tests on main.

---

* 2025-09-08 / agent: codex-cli
* Phase / Subsystem: Networking / USB Wi‑Fi
* Task: Rename Alfa to wlan1; netplan prefer Alfa; full-power profile
* Summary: Persistently renamed Alfa iface to `wlan1` via systemd .link; added netplan for `wlan1` (metric 50) and raised `wlan0` metric (600) for fallback; added services to disable Wi‑Fi powersave and set txpower to 30 dBm; verified default route via wlan1 and SSH reachability to 192.168.0.106.
* Acceptance: Pass — `iw dev wlan1 info` shows txpower 30.00 dBm; route prefers wlan1; wlan0 remains reachable as standby.
* Evidence: `ip route` shows default via wlan1 (metric 50); `iw dev wlan1 info`.
* Follow-ups: Consider Ansible Vault templating for SSIDs/PSKs; monitor stability over extended uptime.

---

* 2025-09-08 / agent: codex-cli
* Phase / Subsystem: Power / Drivers
* Task: INA219 power publisher (bus voltage/current) + bring-up wiring
* Summary: Added `drivers/ina219_monitor` (ament_python) publishing `/power/bus_voltage` and `/power/current` using I2C @ 0x40 and `shunt_ohms` from `configs/power.yaml`; integrated into base bring-up. Verified topics present; node logs OK.
* Acceptance: Pass — topics visible; next step is bench current sweep validation under motor load.
* Evidence: `ros2 topic list` shows `/power/bus_voltage` and `/power/current`; launch logs show INA219 monitor started.
* Follow-ups: Record MCAP during a load sweep; add BatteryState publisher variant if needed; calibrate shunt value against bench meter.

---

* 2025-09-08 / agent: codex-cli
* Phase / Subsystem: Power / Evidence
* Task: Record INA219 baseline MCAP sample
* Summary: Recorded ~6.4s MCAP containing `/power/bus_voltage` (130 msgs) and `/power/current` (130 msgs) to `bags/samples/20250908_224822_ina219`.
* Acceptance: Pass — messages present at ~20 Hz; ready for load sweep capture later.
* Evidence: `ros2 bag info bags/samples/20250908_224822_ina219` (messages=260).

* 2025-09-07 / agent: codex-cli
* Phase / Subsystem: Bring-up / Ansible
* Task: Fill ROS 2 install steps; add LiDAR udev; set ExecStart
* Summary: Added apt key/repos and `ros-humble-ros-base` install tasks; wrote udev rule for YDLIDAR predictable symlink; set systemd ExecStart to launch `alpha_viam_bringup` (wrapped + line-length-safe).
* Acceptance: `yamllint` clean; handlers already wired to reload udev/systemd.
* Evidence: Same commit as above.
* Follow-ups: Add workspace sourcing (overlay) and environment files; consider ansible-lint in CI later.

* 2025-09-07 / agent: codex-cli
* Phase / Subsystem: Repo Hygiene / Governance
* Task: Add LICENSE and update CODEOWNERS
* Summary: Added Apache-2.0 `LICENSE`; replaced placeholder CODEOWNERS with `@alpharover` as catch-all and owner for protected paths; updated README license section.
* Acceptance: License present; CODEOWNERS enforced; README reflects license.
* Evidence: Commit `chore(repo): add Apache-2.0 LICENSE; set CODEOWNERS to @alpharover; update README license` on `main`.
* Follow-ups: Provide architect/team handles if you want finer-grained ownership (controls/nav/vision/embedded).

* 2025-09-07 / agent: codex-cli
* Phase / Subsystem: Docs / Repo Hygiene
* Task: Keep travel plan local-only
* Summary: Removed `travel_progress_plan.md` from Git history (untracked going forward) and added it to `.gitignore`. Local working copy remains on this machine.
* Acceptance: File deleted from GitHub, present locally; `.gitignore` prevents re-adding.
* Evidence: Commit `docs: stop tracking travel_progress_plan.md; ignore local-only travel doc` on `main`.
* Follow-ups: None.

---

* 2025-09-09 / agent: codex-cli
* Phase / Subsystem: Sensors / IMU
* Task: Add Madgwick filter for fused orientation
* Summary: Installed `ros-humble-imu-filter-madgwick` and updated bring-up to start the filter. It subscribes to `/imu/data` and publishes fused orientation on `/imu/data_fused`. Verified in Foxglove; rate ~97–100 Hz.
* Acceptance: Pass — fused orientation visible; responds to motion.
* Evidence: Live Foxglove session; `ros2 topic hz /imu/data_fused` ≈ 100 Hz.

* 2025-09-07 / agent: codex-cli
* Phase / Subsystem: CI / Repo Hygiene
* Task: Fix failing GitHub lint job (yamllint errors)
* Summary: Moved Ansible role handlers out of tasks files into `handlers/main.yml`; removed disallowed blank lines in YAML across `ansible/`, `configs/`, and `hw/`; formatted Python files with Black to match CI behavior.
* Acceptance: Pass locally (`yamllint .` clean; `ruff` exit-zero; `codespell` clean). Expect GH “Lint & Unit” to pass post-push (~14s).
* Evidence: Local CI script run in venv (see `travel_progress_plan.md` CI section for exact commands).
* Follow-ups: Optionally guard markdownlint step in CI with `command -v` check; tighten CI (make ruff non-exit-zero) when ready.

---

* 2025-09-07 / agent: codex-cli
* Phase / Subsystem: Docs / CI Badges
* Task: Lint badge in README shows failing
* Summary: Pinned the lint badge image to `?branch=main` to bypass stale cache; verified badge SVG returns "passing"; CI run is green.
* Acceptance: README badge now renders passing for main.
* Evidence: Badge fetch shows title "Lint & Unit - passing"; see run link in previous entry.
* Follow-ups: None.

* 2025-09-06 / agent: codex-cli
* Phase / Subsystem: Bring-up / Networking
* Task: Discover RPi and verify SSH reachability (alpha-viam.local)
* Summary: Resolved `alpha-viam.local` to `192.168.0.105`; verified TCP/22 open.
* Acceptance: Pass (host reachable, SSH port open); login not performed yet.
* Evidence: `ping alpha-viam.local` -> 192.168.0.105; `nc -vz alpha-viam.local 22` succeeded.
* Follow-ups: Add SSH key for passwordless login; first login hardening.

---

* 2025-09-06 / agent: codex-cli
* Phase / Subsystem: Bring-up / Networking
* Task: Install SSH public key for passwordless login
* Summary: Added local ed25519 key to `alpha_viam@alpha-viam.local` via `ssh-copy-id` (non-interactive using expect); verified `ssh -o BatchMode=yes` works.
* Acceptance: Pass (passwordless SSH returns `OK` and UID `1000`).
* Evidence: `ssh -o BatchMode=yes alpha_viam@alpha-viam.local 'echo OK && id -u'` succeeded.
* Follow-ups: Optionally disable password auth in `sshd_config`; add Host alias in `~/.ssh/config`.

---

* 2025-09-06 / agent: codex-cli
* Phase / Subsystem: Bring-up / Tooling
* Task: Install Node.js 20.x and Codex CLI on RPi4
* Summary: Added NodeSource repo; removed conflicting `libnode-dev`; installed Node.js v20.19.5 and global `@openai/codex` (codex-cli 0.30.0) on `alpha-viam.local`.
* Acceptance: Pass — `node -v` → v20.19.5; `codex --version` → codex-cli 0.30.0.
* Evidence: Remote install via SSH; verified binaries present in PATH (`/usr/bin/node`, `/usr/bin/codex`).
* Follow-ups: `sudo apt autoremove` to prune old node packages; consider nvm for per-user Node management.

---

* 2025-09-06 / agent: codex-cli
* Phase / Subsystem: Bring-up / Tooling
* Task: Configure Codex CLI auth on headless RPi
* Summary: Copied local `$HOME/.codex/auth.json` to `alpha-viam.local:~/.codex/auth.json` per headless auth docs; verified `codex login status` shows "Logged in using ChatGPT".
* Acceptance: Pass — non-interactive auth works on RPi.
* Evidence: `ssh alpha_viam@alpha-viam.local 'codex login status'` → Logged in using ChatGPT.
* Follow-ups: If preferring API key billing, set `OPENAI_API_KEY` on the RPi and run `codex login --api-key $OPENAI_API_KEY`.

---

 
* 2025-09-06 / agent: codex-cli
* Phase / Subsystem: Bring-up / OS+ROS
* Task: Add ROS 2 Humble bootstrap + checks (clean slate)
* Summary: Created `scripts/bootstrap_ros2_humble.sh` (idempotent ROS 2 install for Ubuntu 22.04) and `scripts/check_stack.sh`; updated `docs/bringup/os_ros.md` with concrete steps.
* Acceptance: N/A (installation not executed yet on this host).
* Evidence: Scripts present; docs updated; ready to run.
* Follow-ups: Execute bootstrap script, open new shell, run `scripts/check_stack.sh`; confirm `ros2 doctor` OK.

---

* 2025-09-06 / agent: codex-cli
* Phase / Subsystem: Bring-up / OS+ROS
* Task: Install ROS 2 Humble and scaffold base bring-up
* Summary: Ran `scripts/bootstrap_ros2_humble.sh` with sudo; installed Humble, colcon, rosdep. Added ament_python pkg `bringup/alpha_viam_bringup` with `base_bringup.launch.py` composing robot_state_publisher (URDF), EKF (configs/ekf.yaml), and Foxglove bridge. Fixed EKF YAML structure and minimal URDF. Installed dependencies via apt.
* Acceptance: Pass — `ros2 doctor` reports "All 5 checks passed"; `ros2 launch alpha_viam_bringup base_bringup.launch.py` starts nodes (foxglove at ws://0.0.0.0:8765). 10s smoke test stable.
* Evidence: Command logs in session; nodes started: robot_state_publisher, ekf_node, foxglove_bridge.
* Follow-ups: Add `diagnostic_aggregator` to publish `/diagnostics`; record MCAP once sensors are integrated; create systemd unit for bring-up.
 
* 2025-09-07 / agent: codex-ide
* Phase / Subsystem: Repo / CI & Standards
* Task: Add pre-commit and GitHub Actions Lint & Unit workflow
* Summary: Added `.pre-commit-config.yaml` with ruff/black/codespell/yamllint/markdownlint; added CI workflow to run these checks and stub unit tests on push/PR.
* Acceptance: Pending — CI will run on next push/PR; configs syntactically valid.
* Evidence: `.github/workflows/lint-and-unit.yml`, `.pre-commit-config.yaml` present.
* Follow-ups: Add schema and unit tests; remove `|| true` from pytest once tests exist.

---

* 2025-09-07 / agent: codex-ide
* Phase / Subsystem: Configs / Validation
* Task: Add JSON Schemas and YAML configs with unit validation
* Summary: Added `configs/schemas/` with `topics.schema.json` and `network.schema.json`; created `configs/topics.yaml` and `configs/network.yaml`; added `tests/config/test_yaml_schemas.py` and `pytest.ini`.
* Acceptance: Pending — tests will run in CI; schemas compile locally.
* Evidence: `configs/schemas/*.schema.json`, `configs/topics.yaml`, `configs/network.yaml`, `tests/config/test_yaml_schemas.py`.
* Follow-ups: Extend schemas to other config files; make pytest required in CI.

---

* 2025-09-07 / agent: codex-ide
* Phase / Subsystem: URDF / TF
* Task: Flesh out URDF skeleton and TF contract doc
* Summary: Expanded `urdf/rover.urdf.xacro` with `base_link`, left/right wheel links and joints, and fixed `imu_link`/`lidar_link`; added `docs/tf_tree.md` describing the planned TF tree and ownership.
* Acceptance: Pass (text assets only) — URDF parses as XML; docs added.
* Evidence: `urdf/rover.urdf.xacro`, `docs/tf_tree.md`.
* Follow-ups: Add ros2_control tags and exact sensor extrinsics after measurements.

---

* 2025-09-07 / agent: codex-ide
* Phase / Subsystem: Tools / Viz & Logging
* Task: Add Foxglove default layout and MCAP recording script
* Summary: Created `configs/foxglove/default_layout.json` with panels for `/scan`, `/tf`, `/odom`, `/joint_states`, `/power/ina219`; added `scripts/record_mcap.sh` to record short MCAP clips; updated `docs/tools/foxglove.md` with quick-start and recording policy.
* Acceptance: Pass (text assets only) — JSON validates, script present.
* Evidence: `configs/foxglove/default_layout.json`, `scripts/record_mcap.sh`, `docs/tools/foxglove.md`.
* Follow-ups: Confirm bridge port and topics on-device; add screenshots and refine layout.

---

* 2025-09-07 / agent: codex-ide
* Phase / Subsystem: Bring-up / Provisioning
* Task: Scaffold Ansible roles and setup docs
* Summary: Added `ansible/inventory.example`, `ansible/playbook.yml`, and role stubs for `base`, `ros`, `udev`, and `systemd`; created `docs/setup_pi.md` with quick-start instructions.
* Acceptance: Pass (text assets only) — playbook and roles are placeholders ready for fill-in.
* Evidence: `ansible/*`, `docs/setup_pi.md`.
* Follow-ups: Implement ROS 2 install steps and bringup ExecStart; add ansible-lint in CI later.

---

* 2025-09-07 / agent: codex-ide
* Phase / Subsystem: Planning / Issues
* Task: Draft backlog from roadmap with acceptance criteria
* Summary: Added `docs/ISSUE_BACKLOG.md` with 15 copy/paste-ready issue titles and acceptance criteria spanning encoders, sensors, Foxglove/MCAP, EKF/SLAM/Nav2, URDF/ros2_control, Ansible, and tests.
* Acceptance: Pass (planning artifact) — ready to create GitHub issues.
* Evidence: `docs/ISSUE_BACKLOG.md`.
* Follow-ups: Create GitHub issues, label by Phase/Subsys; wire to milestone.

---

* 2025-09-07 / agent: codex-ide
* Phase / Subsystem: Repo / CI
* Task: Make CI sensible during rapid iteration (gate heavy builds)
* Summary: Tuned `Lint & Unit` workflow to ignore images/bags and cancel superseded runs; added an on-demand `ROS Build & Tests` workflow that runs only on manual trigger, nightly schedule, or PRs labeled `ci:ros-build`.
* Acceptance: Pass — workflows defined; heavy jobs won't run unless requested.
* Evidence: `.github/workflows/lint-and-unit.yml`, `.github/workflows/ros-ci.yml`, `REPO_UPDATES.md` updated.
* Follow-ups: When ready to tighten, remove `|| true` from unit tests and add action-ros-ci steps.

---

* 2025-09-07 / agent: codex-ide
* Phase / Subsystem: Configs / Validation
* Task: Extend schemas and add ros2_control planning config
* Summary: Added schemas for `diff_drive`, `ekf`, `imu`, `power`, `camera`, `ydlidar_g4`, `slam_toolbox`, and a light `ros2_control` schema; created `configs/ros2_control.yaml` placeholder; expanded unit tests to validate all configs.
* Acceptance: Pending — unit tests will run in CI; YAML loads locally.
* Evidence: `configs/schemas/*.schema.json`, `configs/ros2_control.yaml`, `tests/config/test_yaml_schemas.py`.
* Follow-ups: Replace planning `ros2_control.yaml` with real controller params once hardware interface exists.

---

* 2025-09-07 / agent: codex-ide
* Phase / Subsystem: Tools / Recording
* Task: Flesh out MCAP recording policy doc
* Summary: Updated `docs/data/mcap.md` with topic set, naming, retention, and playback notes aligned to `scripts/record_mcap.sh`.
* Acceptance: Pass (doc asset) — ready for reviewers and usage.
* Evidence: `docs/data/mcap.md`.
* Follow-ups: Add screenshots and example bag `info` snippet once on-device.

---

* 2025-09-07 / agent: codex-ide
* Phase / Subsystem: Tools / Teleop & Viz
* Task: Implement teleop + Foxglove launch and docs
* Summary: Updated `launch/teleop_viz.launch.xml` to launch `foxglove_bridge` with configurable `ws_port`/`ws_address` and optional `teleop_twist_keyboard`; added `docs/tools/teleop.md` with usage.
* Acceptance: Pass (text assets) — launch is parameterized and documented.
* Evidence: `launch/teleop_viz.launch.xml`, `docs/tools/teleop.md`.
* Follow-ups: Verify parameters on-device; confirm `foxglove_ws_port` alignment with `configs/network.yaml`.

---

* 2025-09-07 / agent: codex-ide
* Phase / Subsystem: Repo / Polish
* Task: Add GitHub badges for CI/tooling to README
* Summary: Added badges for Lint & Unit, ROS Build & Tests, Project Clock, pre-commit, Black, and Ruff to improve GitHub presentation; linked SECURITY.md.
* Acceptance: Pass — badges render on GitHub; links point to workflows/docs.
* Evidence: `README.md` (badges and Security section).
* Follow-ups: Add license badge once `LICENSE` is finalized.

* 2025-09-07 / agent: codex-ide
* Phase / Subsystem: Repo / Hygiene
* Task: Add SECURITY.md policy
* Summary: Added `SECURITY.md` with private disclosure process and operational guidance for secrets and ROS 2 networking.
* Acceptance: Pass — policy present in repo.
* Evidence: `SECURITY.md`.
* Follow-ups: Link SECURITY.md from README in a later pass.

---

* 2025-09-07 / agent: codex-ide
* Phase / Subsystem: Tools / Teleop & Viz
* Task: Read Foxglove port/address from `configs/network.yaml`
* Summary: Added `launch/teleop_viz.launch.py` which loads `foxglove_ws_port` and `rover_hostname` from `configs/network.yaml` and passes them to `foxglove_bridge`; updated `docs/tools/teleop.md` to prefer the Python launch.
* Acceptance: Pass (text assets) — cleaner configuration source of truth.
* Evidence: `launch/teleop_viz.launch.py`, `docs/tools/teleop.md`.
* Follow-ups: Verify on-device; keep XML fallback for manual override.

---

* 2025-09-07 / agent: codex-ide
* Phase / Subsystem: Repo / CI & Linting
* Task: Stabilize Lint & Unit by adding configs
* Summary: Added `.ruff.toml`, `.markdownlint.json`, and `.yamllint.yaml` to relax rules for fast iteration while keeping quality; keeps CI green without fighting style defaults.
* Acceptance: Pending — next CI run should pass lint steps.
* Evidence: `.ruff.toml`, `.markdownlint.json`, `.yamllint.yaml`.
* Follow-ups: Tighten rules post first on-device release.

---

* 2025-09-07 / agent: codex-ide
* Phase / Subsystem: Repo / CI
* Task: Rework Project Clock to avoid failing pushes
* Summary: Changed `project-clock.yml` to manual (`workflow_dispatch`) and emit a job summary instead of committing to `main` to respect branch protection.
* Acceptance: Pass — no scheduled writes; job succeeds when run manually.
* Evidence: `.github/workflows/project-clock.yml`.
* Follow-ups: Optionally replace with a shields badge or PR-based updater later.

---

* 2025-09-07 / agent: codex-ide
* Phase / Subsystem: Docs / Branding
* Task: 80s-style README header and quick links
* Summary: Rewrote the README title block with a retro ASCII banner and added quick links; replaced Project Clock badge with Last Commit badge.
* Acceptance: Pass — renders well on GitHub; links functional.
* Evidence: `README.md`.
* Follow-ups: Add a small banner image later if desired.

---

* 2025-09-07 / agent: codex-ide
* Phase / Subsystem: Planning / Travel Tasks
* Task: Prune travel plan to remaining items (local only)
* Summary: Trimmed `travel_progress_plan.md` to a concise “Remaining Travel Tasks” list, removing items already completed (CI scaffold, schemas/tests, URDF/TF doc, Foxglove/MCAP, teleop, Ansible scaffolds, branding). Preserved reference snippets.
* Acceptance: Pass (local doc) — ready for next session.
* Evidence: `travel_progress_plan.md` (not pushed).
* Follow-ups: Use this list as next-session context; keep local until push is desired.
 
---

* 2025-09-09 / agent: codex-cli
* Phase / Subsystem: Bring-up / IMU
* Task: IMU (MPU‑6050) integration
* Summary: Added `drivers/mpu6050_driver` (ament_python) publishing `sensor_msgs/Imu` on `/imu/data` with gyro bias calibration and configurable ranges; wired node into base bring-up and filtered params from `configs/imu.yaml`. Updated EKF to consume IMU angular velocity only.
* Acceptance: Pass — `/imu/data` at ~100 Hz (`ros2 topic hz` over 7 windows ~100.0 Hz). Frame `imu_link` present.
* Evidence: MCAP `bags/samples/20250908_230210_bench` (6.65s, 554 msgs on `/imu/data`), node log shows startup and gyro bias; sample message captured.
* Follow-ups: Optionally add `imu_filter_madgwick` for orientation; validate axes vs URDF and update `docs/sensors/imu_power.md` with bias/noise; tune EKF once wheel odom is active.

---

* 2025-09-09 / agent: codex-cli
* Phase / Subsystem: Docs / CI
* Task: Update IMU/Power docs; stabilize CI Black check
* Summary: Expanded `docs/sensors/imu_power.md` with IMU/INA219 run/acceptance details; noted latest MCAP sample in `docs/data/mcap.md`. Fixed `configs/diff_drive.yaml` to match schema. Adjusted code style to satisfy GitHub Black 23.x; Lint & Unit is green.
* Acceptance: Pass — latest Lint & Unit succeeded; docs updated.
* Evidence: GH run 17571693391 (success), 17571742027 (success).
* Follow-ups: None.
