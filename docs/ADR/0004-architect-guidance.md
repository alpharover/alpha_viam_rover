Alright, here’s a tight read on your dev’s changes, why you’re still seeing motor-control weirdness on Humble, and exactly what I’d do next. I’ll also tune up the “optimized prompt” so new sessions start smooth and safe.

---

## What looks solid in the branch

From your summary, a lot is heading the right way:

* **Two-track plan**: a minimal, **direct `/cmd_vel` → PWM** driver for demos, plus a path to get **`diff_drive_controller` under `ros2_control`**. That’s pragmatic.
* **Daemon‑mode pigpio is standardized** (systemd or manual start), and your scripts refuse to run when `controller_manager` is alive—good guardrails against GPIO contention.
* **Helper scripts** (`ros_clean.sh`, smoke tests, bursts) show healthy operational hygiene.

Given Humble’s quirks, this is a sensible staging strategy.

---

## Why Humble bites you here (the real culprits)

1. **Controller params not actually landing on the controller at configure time.**
   `diff_drive_controller` enforces **non‑empty** `left_wheel_names`/`right_wheel_names` using generated parameter validation. If those don’t reach the controller node, you get the exact “cannot be empty” failure you’ve reported. This isn’t optional; the controller won’t configure without them. ([control.ros.org][1])

2. **URDF delivery to controller\_manager.**
   On Humble, the **recommended** pattern is *do not* pass `robot_description` as a parameter to the manager; **subscribe to `~/robot_description` from `robot_state_publisher`**. If the description isn’t found (or the topic remap is wrong), spawners can appear to “hang” or fail late. Some teams still patch around this on Humble by providing the parameter directly when they hit discovery timing issues. ([control.ros.org][2], [Robotics Stack Exchange][3])

3. **Spawner+param‑file edge cases.**
   The **controller spawner** wants the param file **on the machine that runs `controller_manager`**, and on Humble it’s easier to trip over YAML shapes/namespacing. In newer releases the spawner grew better wildcard support and multi‑file parsing, but Humble is touchier. Treat the param‑file path as local-to-manager, and if you still see mis‑applied params, use a **shim** that loads, sets parameters via service, then activates. There’s even a fresh issue noting the spawner’s parameter-file locality assumption. ([GitHub][4], [control.ros.org][5])

4. **Interface expectations.**
   `diff_drive_controller` **outputs wheel `velocity` commands** and expects **`velocity` command interfaces** on your wheel joints. For feedback it uses **position or velocity** (or **`open_loop=true`** if you temporarily want odom from commands). If your hardware plugin exports the wrong interfaces, activation will fail or the robot won’t move sanely. ([control.ros.org][1])

5. **GPIO contention and PWM channels.**
   With **pigpio**, the daemon allows **multiple clients**, but **only one process may “own” pigpio** if you link the C library directly; daemon mode avoids that. Separately, **hardware PWM pins** are grouped into two channels (GPIO 12/18 and 13/19 share channel pairs), so pins on the same channel share frequency. That’s fine for two motors (two channels = two PWMs), just be aware of the coupling. ([Raspberry Pi Stack Exchange][6], [Abyz][7], [Raspberry Pi Forums][8])

6. **L298N’s physics tax.**
   The L298N is easy but lossy: **\~1.8–3.2 V typical drop per bridge at 1–2 A**; worst case \~4.9 V. That means visible sag and heat at load, which can look like “software flakiness.” 20 kHz PWM is generally okay (datasheets and vendor pages list **\~25 kHz commutation** typical), but it’s not a MOSFET driver—don’t expect miracles. ([SparkFun][9], [STMicroelectronics][10], [cpcireland.farnell.com][11])

---

## Concrete bring‑up that works on Humble (copy/paste order)

1. **Start description → manager (with correct remap)**
   Ensure `ros2_control_node` gets the robot description **from the topic**. If needed on your image, also pass the deprecated param as a belt‑and‑suspenders fallback until you verify the topic path. Docs emphasize the topic route; some Humble setups still behave better with the param present. ([control.ros.org][2], [Robotics Stack Exchange][3])

2. **Spawn broadcasters first, then controllers, with params truly present**

   * `joint_state_broadcaster` **activate** before diff drive.
   * For `diff_drive_controller`, prefer the spawner **with `--param-file`** that uses the **controller’s exact node name** at the YAML root. If Humble still ignores it on your machine, use your **activation shim**:

     1. load, 2) `ros2 param load`/`set_parameters` to that node, 3) `switch_controllers` to activate. ([control.ros.org][12])

3. **Introspect aggressively**

   * `ros2 control list_hardware_interfaces` → confirm **command: velocity** on both wheel joints, **state: position, velocity** present (or set `open_loop=true` until encoders are wired).
   * `ros2 control list_controllers` → expect JSB **active**, diff drive **active**. If not, `set_controller_state` / `switch_controllers`. ([control.ros.org][12])

4. **Tune controller params minimally at first**
   Start with: `left_wheel_names/right_wheel_names`, `wheel_radius`, `wheel_separation`, `cmd_vel_timeout=0.5`, `publish_rate=50`, `use_stamped_vel=false` (so you can use plain `/cmd_vel`). Only flip `open_loop=true` while encoders are not trusted. The Humble docs show canonical parameters and even a working example block. ([control.ros.org][1])

5. **Keep pigpio consistent**
   Stick to **daemon mode** across all clients, and don’t run standalone pigpio tests while the hardware plugin is active. Use **separate PWM channels** (e.g., 18 for left, 19 for right) so both sides can share 20 kHz without cross‑talk on the same channel. ([Abyz][7])

---

## Small but high‑leverage fixes inside your repo plan

* **Activation shim** (your `activate_controllers.py` scaffold):
  Implement a deterministic sequence:

  1. `load_controller` `joint_state_broadcaster` and `diff_drive_controller`
  2. `set_parameters` on `diff_drive_controller` (your YAML)
  3. `switch_controllers` with `start=[jsb, diff]` `strict=true`

  This avoids the param‑file edge case on Humble while keeping your YAML the source of truth. The CLI and service APIs match the docs. ([control.ros.org][12])

* **Use `open_loop=true` until encoders are verified.**
  That gives you odom from commands and removes a dependency while you wire and de‑bounce A/B lines (add pigpio glitch filtering when you do wire them). The parameter is explicitly supported. ([control.ros.org][1])

* **PWM mapper hygiene** (your docs already mention deadband/slew):
  Keep a **deadband near 0** to prevent buzzing and add a **slew limit** to contain current spikes. With L298N’s drop, spikes can induce sag; this is not superstition, it’s BJT heat. The datasheet values explain why. ([SparkFun][9])

---

## Quick checks when “it moved once, then not again”

* **Is the controller actually active?**
  `ros2 control list_controllers` should show `diff_drive_controller [active]`. If it’s `inactive`, your spawner/params didn’t apply. Use the shim. ([control.ros.org][13])

* **Do you export the right interfaces?**
  `ros2 control list_hardware_interfaces` should list **command\_interfaces: velocity** for each wheel joint. If not, adjust your `SystemInterface::export_command_interfaces`. `diff_drive_controller` demands velocity output. ([control.ros.org][1])

* **Is `robot_description` actually seen by controller\_manager?**
  If logs say it’s “waiting for robot\_description,” ensure you’re remapping `~/robot_description` correctly or, as a temporary Humble‑only crutch, pass the param too. ([control.ros.org][2], [Robotics Stack Exchange][3])

* **Did another process grab GPIO?**
  In daemon mode, many clients can connect—**but don’t** run pigpio scripts while the driver runs. Your guard is good; keep it strict. ([Abyz][7])

* **Is the power rail sagging?**
  L298N’s drop means the motors see less voltage than you think, especially under load. If you can, log pack voltage during bursts to separate software faults from brownout‑looking behavior. Datasheet numbers are… unambiguous. ([SparkFun][9])

---

## Your setup prompt


> **Context & defaults**
>
> * Host: Raspberry Pi 4, Ubuntu 22.04, ROS 2 Humble. Repo: `alpha_viam_rover` (cwd = repo root).
> * Sudo: password = 9909 . Don’t prompt me for it, use when needed. 
> * Read first: `AGENTS.md` (root), `bringup/AGENTS.md`, `drivers/AGENTS.md`, `docs/control/phase3_drive_report.md`, ADR‑0001..0004.
>
> **Safety**
>
> * Wheels off the ground.
> * Never run pigpio test scripts when `controller_manager` is running.
> * Always `scripts/ros_clean.sh --force` before/after launches.
>
> **Known Humble quirk**
>
> * `controller_manager` spawner with `--param-file` can silently drop typed params on our image. Symptom: diff drive dies with **“left\_wheel\_names cannot be empty.”** Use `drive_direct.launch.py` for quick demos; for `ros2_control`, use the **activation shim** (load → set parameters → switch). ([control.ros.org][1])
>
> **Bring‑up (direct demo, off‑ground)**
>
> 1. `scripts/ros_clean.sh --force`
> 2. `sudo systemctl start pigpiod || sudo /usr/local/bin/pigpiod` (daemon mode) ([Abyz][7])
> 3. `source /opt/ros/humble/setup.bash && source install/setup.bash`
> 4. `ros2 launch alpha_viam_bringup drive_direct.launch.py`
> 5. In a second terminal:
>
>    * `timeout 4s ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}}'`
>    * `timeout 1s ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}}'`
>
> **Bring‑up (`ros2_control` path)**
>
> * Ensure `robot_state_publisher` is publishing `/robot_description`; `ros2_control_node` remap is correct (Humble expects topic; param fallback allowed). ([control.ros.org][2])
> * `ros2 run controller_manager spawner joint_state_broadcaster --activate`
> * **Either**: `spawner diff_drive_controller --inactive --param-file configs/diff_drive_params.yaml` **then** `spawner diff_drive_controller --activate`
>   **Or**: run `scripts/activate_controllers.py` (load → set params → switch).
> * Checks:
>
>   * `ros2 control list_controllers` → `joint_state_broadcaster[active]`, `diff_drive_controller[active]`
>   * `ros2 control list_hardware_interfaces` → command: `velocity` for left/right, state: `position, velocity` (or `open_loop=true` temporarily) ([control.ros.org][12])
>
> **Quick commands**
>
> * Clean: `scripts/ros_clean.sh --force`
> * Start pigpio: `sudo systemctl start pigpiod || sudo /usr/local/bin/pigpiod`
> * Minimal demo: `ros2 launch alpha_viam_bringup drive_direct.launch.py`
> * Drive burst: `timeout 4s ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}}'`
>
> **Evidence policy**
>
> * Record short MCAPs for anything that moves; stash in `bags/` and append context to `AGENTS_PROGRESS.md`.

**Why these changes?** The docs are explicit about parameter validation in `diff_drive_controller`, the robot description delivery pattern on Humble, and the controller/switch CLI. Using the shim and those checks eliminates the two biggest silent failure modes you’re seeing. ([control.ros.org][1])

---

## Hardware/plugin nits worth tightening now

* **Stick to pigpio daemon mode everywhere** (you’ve documented this). Multiple clients can attach, but don’t run tests alongside the driver. If you choose hardware PWM, keep one wheel on PWM channel 0 (GPIO 12/18) and the other on channel 1 (GPIO 13/19), since pins on the same channel share a frequency. ([Abyz][7])
* **Watchdog behavior**: timeout ⇒ **brake** (IN1=IN2) with EN high, command=0 ⇒ **coast** (EN low). That gives you safe stops and quiet idles.
* **Encoder prep**: add pigpio glitch filtering on A/B (100–150 µs), confirm ticks/rev and sign; when uncertain, use `open_loop=true` so odom is still stable for demos. ([control.ros.org][1])

---

## If you want one more “hardening” step

Enable **real‑time scheduling** for controller\_manager (Humble doc has guidance). It’s not mandatory on Pi, but it reduces jitter in the main loop and makes your PWM mapping feel less “spongy” under CPU load. ([control.ros.org][2])

---

### TL;DR action list for your dev

1. Finish the **activation shim** (load → param‑set → switch) and make it the default path on Humble.
2. Verify **interfaces** via `ros2 control list_hardware_interfaces`; adjust `SystemInterface` export if needed. ([control.ros.org][12])
3. Use `open_loop=true` until encoders are clean; flip back once A/B are filtered and signed. ([control.ros.org][1])
4. Keep pigpio **daemon‑only** during runs; forbid test scripts while the driver is loaded. ([Abyz][7])



[1]: https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html "diff_drive_controller — ROS2_Control: Humble Sep 2025 documentation"
[2]: https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html?utm_source=chatgpt.com "Controller Manager — ROS2_Control: Humble Sep 2025 ..."
[3]: https://robotics.stackexchange.com/questions/112698/ros2-control-spawner-not-working-spawner-6-info-waiting-for-controller?utm_source=chatgpt.com "ros2 control spawner not working : [spawner-6] [INFO]"
[4]: https://github.com/ros-controls/ros2_control/issues/2403?utm_source=chatgpt.com "[spawner] Controller spawner assumes parameter file is on ..."
[5]: https://control.ros.org/jazzy/doc/ros2_control/doc/release_notes.html?utm_source=chatgpt.com "Humble to Jazzy — ROS2_Control"
[6]: https://raspberrypi.stackexchange.com/questions/65970/2-pigpio-programs-at-the-same-time?utm_source=chatgpt.com "2 pigpio programs at the same time"
[7]: https://abyz.me.uk/rpi/pigpio/pigpiod.html?utm_source=chatgpt.com "pigpio Daemon"
[8]: https://forums.raspberrypi.com/viewtopic.php?t=256676&utm_source=chatgpt.com "PWM with pigpio"
[9]: https://cdn.sparkfun.com/assets/7/1/d/6/c/Full-Bridge_Motor_Driver_Dual_-_L298N.pdf?utm_source=chatgpt.com "l298 - dual full-bridge driver"
[10]: https://www.st.com/resource/en/datasheet/l298.pdf?utm_source=chatgpt.com "Datasheet - L298 - Dual full-bridge driver"
[11]: https://cpcireland.farnell.com/stmicroelectronics/l298n/dual-full-bridge-driver-4a-o-p/dp/SCL298?utm_source=chatgpt.com "L298N - Stmicroelectronics - Dual Full-Bridge Driver 4A Output"
[12]: https://control.ros.org/humble/doc/ros2_control/ros2controlcli/doc/userdoc.html?utm_source=chatgpt.com "Command Line Interface — ROS2_Control"
[13]: https://control.ros.org/foxy/doc/ros2_control/ros2controlcli/doc/userdoc.html?utm_source=chatgpt.com "Command Line Interface — ROS2_Control: Foxy Jul 2025 ..."
[14]: https://control.ros.org/humble/doc/ros2_controllers/forward_command_controller/doc/userdoc.html?utm_source=chatgpt.com "forward_command_controller — ROS2_Control: Humble ..."
