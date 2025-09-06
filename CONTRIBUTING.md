# Contributing

Thanks for jumping in. This repo is set up for small, verifiable changes with clear evidence, and it’s friendly to both humans and coding agents.

- Start here: `AGENTS.md` (root) for how to work and where to look next
- Ops/playbook: `REPO_UPDATES.md` (flow, CI, releases)
- Progress log: `AGENTS_PROGRESS.md` (append-only evidence after merge)

## Prerequisites

- GitHub account and SSH access set up
- Git + a modern Python, and ROS 2 Humble on Ubuntu 22.04 for ROS work
- For large artifacts (bags, media): Git LFS installed and configured

## Workflow

1) Create a short-lived feature branch from `main`.
2) Make a focused change (docs, config, code). Keep diffs small.
3) Run local checks relevant to your change (build/tests/linters).
4) Open a PR using the template; fill in acceptance results and attach evidence.
5) After merge, append an entry to `AGENTS_PROGRESS.md` with links.

See `REPO_UPDATES.md` for planning details (Projects/Milestones).

## Commit Messages

Use Conventional Commits to enable automatic changelogs and releases:

- feat(drivers): add imu bias correction
- fix(control): clamp pwm at 95%
- docs(nav): update nav2 params
- chore: tidy scripts

Breaking changes use a footer: `BREAKING CHANGE: <explanation>`.

## Evidence Requirements

For sensors/control/nav changes:
- Short MCAP clip (use MCAP rosbag2 storage) and/or Foxglove layout/screenshot
- Link evidence in the PR and later in `AGENTS_PROGRESS.md`
- Keep clips short; use Git LFS for large binaries

## Docs & Decisions

- Add/update a 1-page ADR in `docs/ADR/` when changing interfaces, drivers, or architecture
- Update configs under `configs/` (don’t scatter settings)
- Update `hw/pinmap.yaml` and relevant docs on any wiring/pin change

## High-Impact Changes (Escalation)

Require architect approval before merge. Label the PR `needs-architect` when touching:
- Motor control (`ros2_control` HW, PWM ranges, watchdogs)
- GPIO/pin maps, I²C addresses, or power paths
- Message schemas / topic names / TF tree
- DDS networking, multi-robot namespace/DOMAIN_ID, discovery config
- SLAM/Nav2 planners or costmaps
- Any hardware addition or driver swap

Enforcement is via `CODEOWNERS` and branch protection.

## CI Expectations

- PRs must pass build/tests/lint (see Actions) and commit message checks
- Use the PR template; complete the checklist and include evidence links

## Large Files (LFS)

Track large binaries in Git LFS (e.g., `.mcap`, long videos, large images). Be mindful of quotas. Short samples can live under `bags/`.

## Community & Support

- Questions/ideas: open a Discussion/Issue, or tag @alpha_rover on X (https://x.com/Alpha10six)
- Security/secret handling: never commit secrets; scrub logs and bags

Happy hacking and bot bashing.
