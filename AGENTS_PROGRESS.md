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
