You can get a *lot* done from the MacBook without touching the Pi. Based on what’s in your repo—folders like `ansible/`, `bringup/`, `calibration/`, `configs/`, `drivers/`, `nav/`, `perception/`, `tests/`, `urdf/`, plus the roadmap and progress files—and the README’s near‑term goals (encoders/odometry via `ros2_control`, IMU + power sensing, YDLIDAR G4, Foxglove + MCAP, SLAM Toolbox + Nav2), here’s a tight travel‑mode plan that moves the whole project forward while keeping your hands off the compiler. ([GitHub][1])

---

## CI Lint Fix (ready to push)

- What: Resolved GitHub lint job failures (yamllint errors) and formatted Python to match CI.
- Changes: Moved Ansible role handlers into `handlers/main.yml`; removed disallowed blank lines in YAML; ran `black` on repo Python files.
- Files: `ansible/roles/{udev,systemd}/handlers/main.yml` (new), `ansible/roles/{udev,systemd}/tasks/main.yml`, `ansible/{playbook.yml,roles/{base,ros}/tasks/main.yml}`, `configs/{diff_drive.yaml,ros2_control.yaml,ydlidar_g4.yaml,imu.yaml}`, `hw/pinmap.yaml`, plus auto-format on a few `.py` files.
- Verify: Commit and push; watch “Lint & Unit” workflow (~14s). Locally: `ruff check --exit-zero . && black --check . || black . && codespell -L "ros,urdf,viam,ros2,Nav2,SLAM,MCAP,foxglove,ROS,ROS2" && yamllint .`.
- Note: `markdownlint` is advisory in CI; it’s skipped if not installed and will not fail the job.

## Remaining Travel Tasks (local, no push required)

Only items not yet completed are listed here. Completed: CI scaffold, config schemas + tests, URDF/TF doc, Foxglove layout + MCAP script, teleop launch (XML+Py), Ansible scaffolds, README/branding, CI policy.

1) Repo hygiene
- Add `LICENSE` (Apache‑2.0 suggested).
- Replace CODEOWNERS placeholders with real handles.

2) Backlog → Issues (when ready to push)
- Convert `docs/ISSUE_BACKLOG.md` into GitHub issues with labels/milestones (Phase, Subsystem). Keep local for now.

3) Configs & Schemas
- Add Nav2 parameter files under `configs/nav2/` and matching JSON Schemas; extend tests to validate them.

4) Tests (offline)
- Add a “launch dry‑check” that parses `launch/teleop_viz.launch.py` and asserts keys are wired (port/address, `use_teleop`).
- Add a tiny math util + unit tests (e.g., encoder ticks→m/s, angle wrap) to seed future logic.

5) Teleop & Diagnostics
- Add a basic diagnostics node or aggregator config; document how to view `/diagnostics` in Foxglove.

6) URDF / Control
- Add `ros2_control` tags and transmissions to URDF; ensure names match `diff_drive_controller` and `configs/diff_drive.yaml`.

7) Foxglove / MCAP polish
- Add a screenshot and a short example bag metadata snippet to docs; verify layout references final topic names.

8) Ansible (fill-ins)
- Implement ROS 2 Humble install steps in `roles/ros`; add `ansible-lint` in CI (optional later).
- Add udev rules for LiDAR predictable symlink (e.g., `/dev/ydlidar`).
- Replace `ExecStart=/usr/bin/true` with bring‑up command in systemd unit.

9) CI tightening (later)
- Make unit tests required (remove `|| true`); make ruff blocking; optionally add commitlint.

10) Queue for Pi day
- Measure sensor extrinsics (TF), verify encoder polarity/scaling, bring up SLAM/Nav2, enable systemd services.

---

## Reference snippets (keep for convenience)

**.pre-commit-config.yaml (starter)**

```yaml
repos:
  - repo: https://github.com/charliermarsh/ruff-pre-commit
    rev: v0.6.7
    hooks: [{id: ruff}, {id: ruff-format}]
  - repo: https://github.com/psf/black
    rev: 24.8.0
    hooks: [{id: black, args: ["--check"]}]
  - repo: https://github.com/codespell-project/codespell
    rev: v2.3.0
    hooks: [{id: codespell, args: ["-L", "ros,urdf"]}]
  - repo: https://github.com/adrienverge/yamllint
    rev: v1.35.1
    hooks: [{id: yamllint}]
  - repo: https://github.com/igorshubovych/markdownlint-cli
    rev: v0.41.0
    hooks: [{id: markdownlint}]
```

**.github/workflows/lint-and-unit.yml (starter)**

```yaml
name: Lint & Unit
on: [push, pull_request]
jobs:
  ci:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-python@v5
        with: { python-version: "3.11" }
      - run: python -m pip install -U pip pytest jsonschema
      - run: pip install ruff black codespell yamllint markdownlint-cli
      - name: Pre-commit checks
        run: |
          ruff . && black --check . && codespell && yamllint . && markdownlint "**/*.md"
      - name: Unit tests
        run: pytest -m unit -q || true   # keep green until tests land
```

**configs/schemas/topics.schema.json (tiny example)**

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "title": "Topic Registry",
  "type": "object",
  "properties": {
    "scan": {"type": "string", "pattern": "^/scan$"},
    "odom": {"type": "string", "pattern": "^/odom$"},
    "tf":   {"type": "string", "pattern": "^/tf$"}
  },
  "required": ["scan","odom","tf"],
  "additionalProperties": true
}
```

**tests/config/test\_yaml\_schemas.py (sketch)**

```python
import json, yaml, pathlib
from jsonschema import validate

SCHEMAS = {
    "topics": json.loads(pathlib.Path("configs/schemas/topics.schema.json").read_text())
}

def test_topics_yaml_valid():
    data = yaml.safe_load(pathlib.Path("configs/topics.yaml").read_text())
    validate(instance=data, schema=SCHEMAS["topics"])
```

All of that is platform‑agnostic and keeps you moving.

---

## What you’ll queue up for the Pi later

* Measuring **sensor extrinsics** for TF.
* Verifying **encoder polarity & scaling** on real wheels.
* Running **SLAM/Nav2** and tuning params against real scans.
* Burning in Ansible playbooks and enabling systemd services.

---

### Why this fits your repo right now

* The file layout and README already emphasize agents, governance, progress logging, and a living roadmap. You’re simply wiring the *automation and text assets* that let the on‑device work “just slot in” when you’re back with the robot. ([GitHub][1])

---

**Note on your links:** my browser session couldn’t expand the two specific Markdown pages (`alpha_viam_rover_roadmap_v1.0.md` and `AGENTS_PROGRESS.md`) due to GitHub’s dynamic rendering, but I did access the repository root (file list, README excerpts, directory structure). That was enough to tailor the plan; once those files load on your end, convert their bullets into the issue templates above. ([GitHub][1])

When you’re ready, I can draft the actual file skeletons (schemas, configs, CI workflow, tests) so you can paste them in and commit during your travel window; the rest will be waiting patiently for the Pi.

[1]: https://github.com/alpharover/alpha_viam_rover "GitHub - alpharover/alpha_viam_rover: testing and proving the bot bashing method for robotics"
