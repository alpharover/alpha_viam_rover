# Drive Bring-up Status — 2025-09-17

Summary
- Manual `pigpio` path confirmed both wheels operate forward, reverse, and counter-rotation.
- `scripts/direct_smoke.sh` now completes cleanly; `l298n_direct` no longer crashes on shutdown when pigpiod drops the socket.
- Forward/diff-drive controllers still fail at `configure` because `/controller_manager` ignores typed `joints` parameters on this Humble image.

What works now
- `scripts/direct_smoke.sh 8` → records MCAP (`bags/direct_20250917_145040/`) with forward burst; motors spin as commanded.
- `pigs` bursts on BCM 19/13/26 and 6/5/22 (duty 180 for 1.2 s) move both wheels forward, reverse, and in opposition.
- `scripts/activate_forward.py` primes controller params before/after `load_controller` and tolerates the buggy return codes.

What’s blocked
- `ros2 control configure_controller` still reports "'joints' parameter was empty" for forward controllers even after param injection.
- `diff_drive_controller` path blocked on same issue; controller remains `unconfigured` and hardware watchdog keeps braking.

Plan
1. Instrument parameter state: after activator runs, capture `ros2 param get /controller_manager <controller>.joints` and include in ADR-0006.
2. Implement shim to call the low-level SetControllerParameters service (bypassing the broken bridge) or pin controller_manager to a patched release; request architect review.
3. Once parameters hold, rerun `scripts/forward_smoke.sh` and `scripts/diff_drive_validate.sh` to collect MCAP evidence and update `AGENTS_PROGRESS.md`.

Evidence
- `log/agent_runs/20250917_145040/` (direct smoke) with clean shutdown logs.
- `log/agent_runs/20250917_142147/drive_forward.log` showing current configure failure trace.
- Manual `pigs` command history in shell session (1.2 s bursts at duty 180).
