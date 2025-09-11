# ADR-0006: Controller Parameter Compatibility on ROS 2 Humble

Status: Proposed  
Date: 2025-09-11

Context
- On the rover’s Humble image, `controller_manager` does not apply typed controller parameters at controller creation time. The `spawner --param-file` option sets a `params_file` string on `/controller_manager`, but the controller’s `on_init` still sees empty required params (e.g., `left_wheel_names`).
- As a result `diff_drive_controller` fails at init with `Invalid value … 'left_wheel_names': cannot be empty`.

Decision (proposed)
1) Short term: use a forward-controllers smoke test for motion evidence. Provide controller‑scoped YAML via spawner and/or set params on `/controller_manager` before calling `configure_controller`.
2) Medium term (preferred): add a bring‑up shim that reads `*controller*.params_file` from `/controller_manager` and applies typed parameters to the controller node before `load_controller` returns, or pin to a controller_manager/spawner version that restores the expected behavior on Humble.

Consequences
- Enables deterministic bring-up on the rover’s current Humble stack and unblocks motion smoke.
- DiffDrive can be re‑enabled after shim or version pin is accepted and verified.

Alternatives considered
- Rely solely on `--param-file`: rejected (no effect on this image).
- Embed all controller params inline under `controller_manager.<ctrl>`: attempted; manager still didn’t surface params at controller `on_init`.

Acceptance criteria
- `diff_drive_controller` loads, configures, and activates on device without manual parameter injection; bag shows odom + cmd topics.

