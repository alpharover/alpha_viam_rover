# ADR-0002: Controller Parameter Bridge for Humble (Spawner Param Quirk)

Status: Superseded (see ADR-0006)
Date: 2025-09-09 (superseded 2025-12-26)

## Context

- On the rover’s ROS 2 Humble image, early bring-up attempts using `ros2 run controller_manager spawner <controller> --param-file <file.yaml>` did not result in typed controller parameters being present at controller init/configure time.
- Controllers failed to configure:
  - `diff_drive_controller`: "left_wheel_names cannot be empty"
  - `forward_command_controller`: "'joints' parameter was empty"

## Proposed decision (historical)

Introduce a small helper (node/script) which:

1) Calls `/controller_manager/load_controller` for the target controller.
2) Calls `/<controller>/set_parameters` with the YAML content (typed values).
3) Calls `/controller_manager/switch_controller` to activate the controller.

Keep YAMLs in `configs/` as the source of truth; the helper reads them and sets parameters directly.

## Update (2025-12-26)

On the alpha-viam rover, the issue was resolved without a parameter bridge by setting `/controller_manager` parameter `diff_drive_controller.params_file` to the installed wildcard YAML `configs/diff_drive_params.yaml` and spawning diff drive without `--param-file` (see ADR-0006).

The helper scripts remain useful as a fallback for other Humble variants, but are not required to achieve wheel spin on this rover as of 2025-12-26.
