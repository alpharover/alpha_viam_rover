# Rover Session To-Do List

> Hand this file to the agent when rover is back online.
> Created: 2025-12-29

---

## Phase 1: File Sync (do first)

**Goal**: Ensure local repo matches rover's authoritative files, then commit.

### Critical files to verify/sync FROM rover:

```bash
# Run from Mac - compare rover vs local
CRITICAL_FILES=(
  "lib/driver_station_mapping.py"
  "scripts/driver_station_server.py"
  "scripts/driver_station_session.sh"
  "scripts/driver_station.sh"
  "scripts/ros_clean.sh"
  "tests/unit/test_driver_station_mapping.py"
  "web/driver_station/app.js"
  "web/driver_station/index.html"
  "web/driver_station/joystick.js"
  "web/driver_station/style.css"
  "web/driver_station/ws.js"
)

# Quick diff check (run for each file):
ssh alpha_viam@alpha-viam.local "cat ~/alpha_viam_rover/<file>" | diff - <file>
```

### Also check these modified files (bringup/drivers):
- `bringup/alpha_viam_bringup/alpha_viam_bringup/launch/base_bringup.launch.py`
- `drivers/l298n_hardware/src/l298n_system.cpp`
- `drivers/l298n_hardware/include/l298n_hardware/l298n_system.hpp`

### Actions:
1. SSH to rover, check `git status` there
2. If rover has uncommitted changes that are newer, `scp` them to local
3. If local is newer/same, proceed to commit
4. Commit all driver_station + supporting files
5. Push to origin
6. On rover: `git pull` to sync

---

## Phase 2: Validation Testing (off-ground, E-stop ready)

### 2a. Basic teleop validation
```bash
# On rover:
scripts/driver_station_session.sh

# On Mac browser:
# Open http://alpha-viam.local:8090/
# Verify: video feed, joystick control, arm/disarm toggle
```

### 2b. MCAP capture - reverse + gentle turn
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

## Phase 3: Encoder Validation

**Context**: Single-channel encoders only (no B channel). Direction inferred from command sign.

### Check encoder readings:
```bash
# While wheels spin:
ros2 topic echo /joint_states --field velocity

# Expected: non-zero values that correlate with wheel motion
```

### Parameters to note/tune (in l298n_hardware):
- `ticks_per_rev` - encoder ticks per wheel revolution
- `speed_kp` - PI controller proportional gain
- `speed_ki` - PI controller integral gain  
- `speed_i_max` - integral windup limit

### Validation:
1. Spin wheels at constant cmd_vel
2. Check `/joint_states` velocity is stable (not jumping wildly)
3. Check `WHEEL_DIFF` in driver station UI is near 0 for straight driving

---

## Phase 4: Documentation Updates

After validation passes:

1. Update `AGENTS_PROGRESS.md` with:
   - File sync completed
   - MCAP bag location
   - Encoder validation results

2. Verify `docs/hw/pinmap.md` is accurate (already notes single-channel encoders)

3. Close/update GitHub issues if applicable:
   - #2 (Encoders) - partial progress
   - #24 (Teleop) - likely closeable

---

## Quick Reference

### Rover access:
```bash
ssh alpha_viam@alpha-viam.local
cd ~/alpha_viam_rover
```

### Driver station workflow:
```bash
scripts/driver_station_session.sh
# Browser: http://alpha-viam.local:8090/
# Video: http://alpha-viam.local:8080/stream
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

---

## Notes

- Encoders are single-channel only (hardware limitation)
- Deprecated teleop_keyboard.py/sh files were deleted (use driver_station instead)
- Local repo branch: `chore/lint-fixes`
