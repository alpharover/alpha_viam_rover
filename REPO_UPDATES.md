# REPO_UPDATES.md — Updates, Progress & Releases

**Scope.** This document defines how work is planned, tracked, reviewed, and released for the Viam Rover v2 project. It complements `AGENTS.md` (how to work), `AGENTS_PROGRESS.md` (what happened), and the phase roadmap.

---

## 1) Working model (daily flow)

* **Branching:** Use **short‑lived feature branches** off `main` with small, frequent merges. This is essentially **GitHub Flow** with trunk‑based habits (avoid long‑running branches; merge behind PRs). ([GitHub Docs][1], [trunkbaseddevelopment.com][2], [Atlassian][3])
* **Commits:** Follow **Conventional Commits** to enable automated changelogs and releases. Examples: `feat(drivers): add imu bias correction`, `fix(control): clamp pwm`. Use `BREAKING CHANGE:` in the footer when applicable. ([Conventional Commits][4])
* **Versioning:** Use **Semantic Versioning** for repository‑level releases (or per‑package if you later split). Major = breaking, Minor = feature, Patch = fix. ([Semantic Versioning][5])
* **Changelog:** Maintain `CHANGELOG.md` in **Keep a Changelog** format; releases reference it. ([Keep a Changelog][6])

---

## 2) Planning & progress tracking

Authoritative status lives in a **GitHub Project (Projects v2)** at the org/repo level.

* **Project board:** Columns: Backlog → Ready → In Progress → In Review → Blocked → Done. Use built‑in **Project automations** to move items on issue/PR events. ([GitHub Docs][7])
* **Fields:** Add custom fields: Phase, Subsystem, Risk, Size, Evidence URL (MCAP), Demo Date. Projects support custom fields and views. ([GitHub Docs][8])
* **Automation:** Enable “auto‑add to project” and “item closed = Done” workflows; optionally add an Action to attach PRs to the Project when marked ready for review. ([GitHub Docs][9])
* **Milestones:** Create one **milestone per phase** (P0, P1, …). Associate issues/PRs; use milestone burndown as the phase dashboard. ([GitHub Docs][10])
* **Progress ledger:** After each merged PR, append an entry to `AGENTS_PROGRESS.md` with Task ID, outcome, and evidence links (bags/screens). This stays append‑only as the human/agent activity log, and it pairs with the Project board.

---

## 3) PR policy & quality gates

All changes land by Pull Request into `main`. PRs must:

* Use the **PR template** and include: problem, approach, acceptance‑test results, and **evidence** (MCAP link, Foxglove screenshot/layout). ([GitHub Docs][11])
* **Pass CI**: build, tests, linters; and pass commit‑message checks (Conventional Commits). ([GitHub][12])
* Respect **CODEOWNERS**: architect review is required for protected paths and high‑impact areas. Enforce via branch protection rules. ([GitHub Docs][13])
* Update **docs/configs** affected (ADRs, `configs/`, `hw/pinmap.yaml`) and **append** to `AGENTS_PROGRESS.md`.

**Branch protection:** Require at least one approving review (architect on protected paths), status checks, and linear history. Use GitHub’s branch protection rules. ([GitHub Docs][14])

---

## 4) CI/CD & automation (what runs on every PR)

Use GitHub Actions to enforce the same rituals on all contributors:

* **ROS build/test:** `ros-tooling/setup-ros` to provision ROS 2 Humble and `action-ros-ci` to build with colcon and run tests. Matrix on Ubuntu (and optional macOS). ([GitHub][15])
* **Linting:** `action-ros-lint` (ament linters) for fast feedback. ([GitHub][16])
* **Commit message check:** commitlint or a “Conventional Commit Lint” action. ([GitHub][17])
* **Project automation:** optional action to auto‑attach PRs/issues to the Projects board and set status fields. ([GitHub][18])

Result: PR cannot merge without green checks; cards move automatically; release notes write themselves later.

---

## 5) Releases & changelog

Every **phase gate** or meaningful waypoint yields a GitHub Release.

* **Changelog & version bump:** Automate with **release‑please** so Conventional Commits drive the next version and `CHANGELOG.md` sections. This creates a release PR you can review. ([GitHub][19])
* **Release notes:** If you prefer, use GitHub’s **auto‑generated release notes** (summaries of merged PRs, contributors, and a full changelog link). ([GitHub Docs][20])
* **SemVer cadence:**

  * Major: breaking changes to interfaces, topic schemas, or TF.
  * Minor: new features (e.g., added sensor or planner).
  * Patch: bug fixes and doc updates. ([Semantic Versioning][5])
* **Attachments:** Add small artifacts (config snapshots, maps, layout JSON). Large data goes to LFS (see below).

---

## 6) Data & large files (bags, maps, media)

* **Default format:** Use **MCAP** for rosbag2 recordings to attach evidence links in PRs; store short samples in `bags/samples/`.
* **Git LFS:** Track large binaries (e.g., `.mcap`, long videos, large images) with **Git LFS** to keep clone size modest. LFS stores pointer files in Git and the payloads separately; be mindful of quota. ([GitHub Docs][21])
* **Policy:** Keep evidence clips short. Long sessions belong in external storage or release assets; if you move files into LFS after the fact, follow GitHub’s “move to LFS” guidance. ([GitHub Docs][22])

---

## 7) Labels & taxonomy (for filters and automation)

Adopt a compact, stable label set so filters and charts stay useful:

* **Phase:** `phase:P0` … `phase:P9`
* **Subsystem:** `subsystem:drivers`, `subsystem:bringup`, `subsystem:nav`, `subsystem:perception`, `subsystem:can`, `subsystem:config`
* **Type:** `type:feat`, `type:fix`, `type:chore`, `type:docs`, `type:test` (mirrors Conventional Commits) ([Conventional Commits][4])
* **Risk:** `risk:low|medium|high`
* **Status:** used by Project workflows (Status field, not a label)

Use **Milestones** for phases; labels for intent/risk; Projects for status/fields. ([GitHub Docs][10])

---

## 8) Documentation & evidence

* **ADRs:** Any decision that changes interfaces, drivers, or architecture gets a 1‑page ADR in `docs/ADR/`. Use the Nygard ADR style (“Context, Options, Decision, Consequences”).
* **PR evidence:** Short MCAP + Foxglove layout/screenshot for sensor/control/Nav changes.
* **Progress log:** Append to `AGENTS_PROGRESS.md` after merge with links to the evidence and milestone.

---

## 9) Escalation & approvals

High‑impact changes require **architect sign‑off** before merge:

* **What counts as high‑impact:** motor control or safety logic, GPIO/pin maps, topic/TF schemas, DDS/discovery and multi‑robot config, SLAM/Nav planner changes, and any hardware driver swap.
* **How enforced:** `CODEOWNERS` assigns the architect to protected paths; **branch protection rules** require an approving review. Tag PRs with `needs-architect`. ([GitHub Docs][13])
* **When in doubt:** open an ADR draft and a milestone‑scoped issue; link both in the PR for review.

---

## 10) Roles & responsibilities

* **Project architect:** owns the roadmap, approves high‑impact changes, curates ADRs, and is the escalation endpoint.
* **Subsystem maintainers:** own `drivers/`, `nav/`, `perception/`, `can/`, `configs/`, `urdf/` paths.
* **Contributors/agents:** follow AGENTS chain, keep diffs small, attach evidence, and update progress.

---

## 11) Implementation checklist (one‑time setup)

Repository owners should put these in place:

1. **Projects:** create a Project board with fields and built‑in automations for status. ([GitHub Docs][23])
2. **Milestones:** create P0…P9 milestones and link issues/PRs. ([GitHub Docs][24])
3. **CODEOWNERS + branch protection:** require reviews and status checks on `main`. ([GitHub Docs][13])
4. **PR/Issue templates:** standardize evidence requirements. ([GitHub Docs][25])
5. **CI:**

   * ROS setup + build/test via ROS Tooling actions. ([GitHub][15])
   * Lint via `action-ros-lint`. ([GitHub][16])
   * Commit message linting (commitlint or equivalent). ([GitHub][17])
6. **Release automation:** choose **release‑please** (Conventional Commits → version + changelog + GitHub Release). ([GitHub][19])
7. **LFS:** enable Git LFS for `.mcap` and other large binaries; document quotas. ([GitHub Docs][21])

---

## 12) Definitions of done (DoD)

A change is **done** when:

* Linked issue is in **Done** on the Project board; milestone reflects progress. ([GitHub Docs][9])
* PR merged with green checks, architect sign‑off if high‑impact. ([GitHub Docs][14])
* Evidence attached and `AGENTS_PROGRESS.md` updated.
* Docs/configs updated; ADR added if applicable.

---

## 13) Appendix — Tools we align with

* **Conventional Commits** (commit messages) → enables automation and SemVer/notes. ([Conventional Commits][4])
* **SemVer** (release numbering) and **Keep a Changelog** (human‑readable changes). ([Semantic Versioning][5], [Keep a Changelog][6])
* **GitHub Projects** for planning, **Milestones** for phase tracking, **auto release notes** for summaries. ([GitHub Docs][7])
* **ROS Tooling GitHub Actions** for consistent build/test/lint across contributors. ([GitHub][15])
* **Release Please** for automated versioning and changelog PRs; **commitlint** for enforcement. ([GitHub][19])
* **Git LFS** to keep the repo lean while preserving MCAP evidence and media. ([GitHub Docs][21])

---

### Operational note

This file sets **how** we work; the **what** remains the phase roadmap. When the roadmap shifts, update milestones, labels, and the Project board immediately, then open or update ADRs to record the decision.

---

If you want, I can also generate a concise CONTRIBUTING.md that mirrors this policy and links to AGENTS.md, or add a lightweight label taxonomy file your devs can import.

[1]: https://docs.github.com/en/get-started/using-github/github-flow?utm_source=chatgpt.com "GitHub flow"
[2]: https://trunkbaseddevelopment.com/?utm_source=chatgpt.com "Trunk Based Development"
[3]: https://www.atlassian.com/continuous-delivery/continuous-integration/trunk-based-development?utm_source=chatgpt.com "Trunk-based Development"
[4]: https://www.conventionalcommits.org/en/v1.0.0/?utm_source=chatgpt.com "Conventional Commits"
[5]: https://semver.org/?utm_source=chatgpt.com "Semantic Versioning 2.0.0 | Semantic Versioning"
[6]: https://keepachangelog.com/en/1.1.0/?utm_source=chatgpt.com "Keep a Changelog"
[7]: https://docs.github.com/issues/planning-and-tracking-with-projects/learning-about-projects/about-projects?utm_source=chatgpt.com "About Projects - GitHub Docs"
[8]: https://docs.github.com/en/issues/planning-and-tracking-with-projects/understanding-fields?utm_source=chatgpt.com "Understanding fields"
[9]: https://docs.github.com/en/issues/planning-and-tracking-with-projects/learning-about-projects/best-practices-for-projects?utm_source=chatgpt.com "Best practices for Projects"
[10]: https://docs.github.com/issues/using-labels-and-milestones-to-track-work/about-milestones?utm_source=chatgpt.com "About milestones"
[11]: https://docs.github.com/en/communities/using-templates-to-encourage-useful-issues-and-pull-requests/creating-a-pull-request-template-for-your-repository?utm_source=chatgpt.com "Creating a pull request template for your repository"
[12]: https://github.com/ros-tooling/action-ros-ci?utm_source=chatgpt.com "ros-tooling/action-ros-ci"
[13]: https://docs.github.com/articles/about-code-owners?utm_source=chatgpt.com "About code owners"
[14]: https://docs.github.com/repositories/configuring-branches-and-merges-in-your-repository/managing-protected-branches/about-protected-branches?utm_source=chatgpt.com "About protected branches"
[15]: https://github.com/ros-tooling/setup-ros?utm_source=chatgpt.com "ros-tooling/setup-ros: Github Action to set up ROS 2 on hosts"
[16]: https://github.com/ros-tooling/action-ros-lint?utm_source=chatgpt.com "ros-tooling/action-ros-lint"
[17]: https://github.com/conventional-changelog/commitlint?utm_source=chatgpt.com "conventional-changelog/commitlint: 📓 Lint commit messages"
[18]: https://github.com/marketplace/actions/project-beta-automations?utm_source=chatgpt.com "Actions · GitHub Marketplace - project beta automations"
[19]: https://github.com/googleapis/release-please-action?utm_source=chatgpt.com "googleapis/release-please-action: automated ..."
[20]: https://docs.github.com/en/repositories/releasing-projects-on-github/automatically-generated-release-notes?utm_source=chatgpt.com "Automatically generated release notes"
[21]: https://docs.github.com/repositories/working-with-files/managing-large-files/about-git-large-file-storage?utm_source=chatgpt.com "About Git Large File Storage"
[22]: https://docs.github.com/en/repositories/working-with-files/managing-large-files/moving-a-file-in-your-repository-to-git-large-file-storage?utm_source=chatgpt.com "Moving a file in your repository to Git Large File Storage"
[23]: https://docs.github.com/en/issues/planning-and-tracking-with-projects/automating-your-project/using-the-built-in-automations?utm_source=chatgpt.com "Using the built-in automations"
[24]: https://docs.github.com/en/issues/using-labels-and-milestones-to-track-work/creating-and-editing-milestones-for-issues-and-pull-requests?utm_source=chatgpt.com "Creating and editing milestones for issues and pull requests"
[25]: https://docs.github.com/en/communities/using-templates-to-encourage-useful-issues-and-pull-requests/about-issue-and-pull-request-templates?utm_source=chatgpt.com "About issue and pull request templates"

