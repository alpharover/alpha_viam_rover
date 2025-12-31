# ADR-0006: Controller Parameter Compatibility on ROS 2 Humble

Status: Accepted
Date: 2025-12-26

## Context

- On the alpha-viam rover Humble image, the diff-drive controller’s topics live under `/controller_manager/*` (e.g. `/controller_manager/cmd_vel_unstamped`, `/controller_manager/odom`).
- Passing a controller-scoped YAML via `spawner --param-file` (root `diff_drive_controller:`) did not apply parameters with the right node name on this image, and `diff_drive_controller` failed at init with `left_wheel_names` empty.
- Setting `/controller_manager` parameter `diff_drive_controller.params_file` to a wildcard YAML (`/**:`) allows `controller_manager` to pass the YAML into the controller node at load time, so required params are present during `on_init`.

## Decision

1) Use `configs/diff_drive_params.yaml` (wildcard root `/**:`) as the diff-drive parameter source of truth for this rover.
2) Install it into the bringup package share and inject it at controller-manager startup as:

```yaml
# passed as a parameter override to ros2_control_node
{
  "diff_drive_controller.params_file": ["<path>/diff_drive_params.yaml"]
}
```

3) Spawn `diff_drive_controller` with:

```bash
ros2 run controller_manager spawner diff_drive_controller \
  --controller-manager /controller_manager \
  --activate --unload-on-kill
```

## Validation (off-ground)

- Command (example): `LINEAR_X=0.75 FWD_SEC=5 REV_SEC=5 scripts/drive_smoke.sh 30`
- Evidence: `bags/phase3_offground_20251226_230317/phase3_offground_0.mcap`
- Observed: wheels spin (user-confirmed); `l298n_hardware` reports duty `target≈159/255` during bursts.

## Notes

- The Humble spawner can stringify list-like `params_file` values; keep `params_file` a proper string array on `/controller_manager` (or just one file) if you set it dynamically.
- Expect diff-drive input topic on this rover to be `/controller_manager/cmd_vel_unstamped` when `use_stamped_vel=false`.
