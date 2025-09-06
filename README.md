# alpha_viam_rover -- bot bashing demonstration

Testing and proving a new way to build robots: “bot bashing” — vibe-coding a robot by pairing a high-level AI planner with an on-device coding agent.

![Viam Rover 2 chassis](viam-rover-hero.jpg)

## Project Clarity
- Started with an out‑of‑box Viam Rover v2 kit; additional parts were on‑hand.
- Compute is a Raspberry Pi 4B for now (may upgrade later).
- Viam’s chassis carrier board schematic isn’t published; I manually traced the pinout.
- Wheel encoders are not fully sorted yet — work in progress.

## What Is This?
I’ve been building robots since the Basic Stamp 2 days. Robotics is my passion, but the barrier to entry can be steep — especially on the software side. You need to wrangle Linux, networking, Git, and the whole robotics stack.

For this project, I’m experimenting with a workflow where I use GPT‑5‑Pro to generate an end‑to‑end robotics architecture based on my exact hardware configuration, and then a local Codex CLI agent (running GPT‑5‑thinking‑high) on the robot implements the plan. In short: I’m vibe coding a robot. I call it “bot bashing,” and I think it’s the future.

## Hardware
- Chassis: Viam Rover v2
  - Resource page: https://www.viam.com/resources/rover
  - Reference repo: https://github.com/viamrobotics/Viam-Rover-2

## Future Build Plans (Short)
- Finalize encoder wiring and odometry via `ros2_control`.
- Integrate IMU, INA219 power sensing, and YDLIDAR G4.
- Teleop + visualization with Foxglove; MCAP logging.
- Mapping and basic navigation with SLAM Toolbox and Nav2.
- See `alpha_viam_rover_roadmap_v1.0` for the living plan.

## Repo Contents
- `alpha_viam_rover_roadmap_v1.0` — The current high-level roadmap that guides implementation.
- `README.md` — You’re here. Context, links, and how to follow along.
- Images and assets for the project log.

More will arrive as the build progresses (modules, configs, scripts, notes).

## Bot Bashing Framework (Coming Soon)
I’m formalizing “bot bashing” as a dedicated, open framework with clear roles, permissions, safety gates, escalation protocols, backup policies, and operational conventions. Once fully defined and tested, it will live in its own repo with docs, templates, and examples.

## Follow Along
- Build updates and commentary on X: @alpha_rover — https://x.com/Alpha10six
- Issues will track milestones and blockers; PRs will show diff-based progress.

## Getting Started (Observers)
If you’re following along or want to try something similar:
- Read the roadmap: `alpha_viam_rover_roadmap_v1.0` to see the current plan.
- Watch issues and PRs for day-to-day progress.
- If you have a Viam Rover v2, explore the links above to get your hardware ready.

## Contributing
Ideas, discussions, and lightweight PRs are welcome. Please open an issue to propose changes or share ideas that could accelerate the “bot bashing” method.

## License
TBD (will be added once the public scope is finalized).
