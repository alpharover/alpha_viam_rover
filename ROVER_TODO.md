# Rover Session To-Do List

> Hand this file to the agent when rover is back online.
> Updated: 2025-12-31

---

## Session Context (2025-12-31)

**What was accomplished today:**
- PR #32 merged to main (92 files, +7948 lines)
- Mac and rover synced to `main` @ commit `e4270b6`
- Diff_drive velocity limits tuned: linear 0.8→1.0 m/s, angular 3.0→4.0 rad/s
- User confirmed improved drive speed/turning
- Driver station verified working

**Current branch:** `main` (both Mac and rover)

**Key insight from today:** Launch loads configs from installed package share dir (`install/alpha_viam_bringup/share/...`), not source. To apply config changes:
1. Edit source file
2. Either rebuild (`colcon build`) OR directly edit installed file
3. Restart driver station session

---

## Phase 1: Further Speed Tuning (if needed)

If movement still feels limited, apply Step 2 from Oracle analysis:

### Lower `max_wheel_rad_s` for more PWM duty per rad/s

```bash
# On rover, edit URDF (source):
nano ~/alpha_viam_rover/urdf/rover.urdf.xacro
# Find line ~89: <param name="max_wheel_rad_s">20.0</param>
# Change to: <param name="max_wheel_rad_s">12.0</param>

# Rebuild l298n_hardware:
cd ~/alpha_viam_rover
source /opt/ros/humble/setup.bash
colcon build --packages-select l298n_hardware

# Restart driver station:
scripts/driver_station_session.sh
```

**Why this helps:** At 20 rad/s max, commanded 4 rad/s only produces ~20% PWM duty. Lowering to 12 rad/s means same command produces ~33% duty.

---

## Phase 2: Encoder Validation

**Context**: Single-channel encoders (no B channel). Direction inferred from command sign.

### Check encoder readings:
```bash
# While wheels spin:
ros2 topic echo /joint_states --field velocity

# Expected: non-zero values that correlate with wheel motion
```

### Parameters to note/tune (in l298n_hardware via URDF):
- `ticks_per_rev` - encoder ticks per wheel revolution
- `speed_kp` - PI controller proportional gain
- `speed_ki` - PI controller integral gain  
- `speed_i_max` - integral windup limit

### Validation:
1. Spin wheels at constant cmd_vel
2. Check `/joint_states` velocity is stable (not jumping wildly)
3. Check `WHEEL_DIFF` in driver station UI is near 0 for straight driving

---

## Phase 3: MCAP Capture (validation evidence)

```bash
# While driver_station is running, in another terminal:
ros2 bag record -o bags/validation_$(date +%Y%m%d_%H%M%S) \
  /controller_manager/cmd_vel_unstamped \
  /controller_manager/odom \
  /joint_states \
  /tf \
  --max-cache-size 100000000 \
  -d 30

# Test sequence (off-ground):
# 1. Forward burst (5s)
# 2. Reverse burst (5s) 
# 3. Gentle left turn (small joystick X)
# 4. Gentle right turn
# 5. Stop
```

**Success criteria**:
- Reverse produces wheel spin (not just forward)
- Small joystick X produces small angular.z (not snapping to max)
- `/joint_states` shows wheel velocities

---

## Phase 4: Documentation Updates

After validation passes:

1. Update `AGENTS_PROGRESS.md` with:
   - MCAP bag location
   - Encoder validation results
   - Any additional tuning applied

2. Close/update GitHub issues if applicable:
   - #2 (Encoders) - partial progress
   - #24 (Teleop) - likely closeable

---

## Quick Reference

### Rover access:
```bash
ssh alpha_viam@alpha-viam.local
# or
ssh alpha_viam@192.168.0.87
cd ~/alpha_viam_rover
```

### Driver station workflow:
```bash
scripts/driver_station_session.sh
# Browser: http://192.168.0.87:8090/
# Video: http://192.168.0.87:8080/stream
```

### Build after changes:
```bash
source /opt/ros/humble/setup.bash
source ~/alpha_viam_rover/install/setup.bash
colcon build --packages-select l298n_hardware
```

### Clean stale processes:
```bash
scripts/ros_clean.sh
```

### Check current velocity limits:
```bash
cat ~/alpha_viam_rover/install/alpha_viam_bringup/share/alpha_viam_bringup/configs/diff_drive_params.yaml | grep max_velocity
```

---

## Notes

- Encoders are single-channel only (hardware limitation)
- Deprecated teleop_keyboard.py/sh files were deleted (use driver_station instead)
- Rover backup exists at `~/alpha_viam_rover_working_20251231_095936.tgz` (can delete if not needed)
- PR #32 merged - all drive stabilization work is now on main
