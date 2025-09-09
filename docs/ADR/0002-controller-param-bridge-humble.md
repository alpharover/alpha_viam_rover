# ADR-0002: Controller Parameter Bridge for Humble (Spawner Param Quirk)

Status: Proposed (2025-09-09)

Context
- On the rover’s ROS 2 Humble image, using `ros2 run controller_manager spawner <controller> --param-file <file.yaml>` sets a string parameter `params_file` on the controller node but does not apply the YAML contents as controller parameters. As a result controllers fail to configure:
  - `diff_drive_controller`: "left_wheel_names cannot be empty"
  - `forward_command_controller`: "'joints' parameter was empty"
- Controller parameters under `/controller_manager` also are not propagated to controllers during `load_controller` on this image.

Decision
- Introduce a small helper (node/script) which:
  1. Calls `/controller_manager/load_controller` for the target controller.
  2. Calls `/<controller>/set_parameters` with the YAML content (maps to typed values).
  3. Calls `/controller_manager/switch_controller` to activate the controller.
- Keep YAMLs in `configs/` as the source of truth; the helper reads them and sets parameters directly.

Consequences
- Works across Humble variants and avoids reliance on spawner param-file semantics.
- Adds one small utility maintained in repo (`scripts/activate_diff_drive.py`).

Alternatives Considered
- Embedding parameters under `controller_manager.ros__parameters.<controller>`: still not applied to controller at load time on this image.
- Using spawner `--param-file` only: not reliable on this image.

Follow-ups
- Implement a generic `activate_controller.py` that accepts controller name and param YAML for reuse.
- Document this quirk in bring-up docs.

