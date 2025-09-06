# Alpha Viam Rover — Bot Bashing Demo

Testing and proving a new way to build robots: “bot bashing” — vibe-coding a robot by pairing a high-level AI planner with an on-device coding agent.

![Viam Rover 2 chassis](638e3de6e9575cf86a297fb7_viam-open-graph-image-rover%20copy.jpg)

## What Is This?
I’ve been building robots since the Basic Stamp 2 days. Robotics is my passion, but the barrier to entry can be steep — especially on the software side. You need to wrangle Linux, networking, Git, and the whole robotics stack.

For this project, I’m experimenting with a workflow where:
- I use GPT-5-Pro to generate an end-to-end robotics architecture based on my exact hardware configuration.
- I hand that roadmap to a local Codex CLI agent (running GPT‑5‑thinking‑high) on the robot’s computer to implement the plan.

In short: I’m vibe coding a robot. I call it “bot bashing,” and I think it’s the future.

## Hardware
- Chassis: Viam Rover v2
  - Resource page: https://www.viam.com/resources/rover
  - Reference repo: https://github.com/viamrobotics/Viam-Rover-2

## Repo Contents
- `alpha_viam_rover_roadmap_v1.0` — The current high-level roadmap that guides implementation.
- `README.md` — You’re here. Context, links, and how to follow along.
- Images and assets for the project log.

More will arrive as the build progresses (modules, configs, scripts, notes).

## How “Bot Bashing” Works
1. Profile hardware: enumerate sensors, actuators, compute, networking, power.
2. Generate architecture: GPT‑5‑Pro proposes stack, interfaces, and milestones.
3. Plan to actions: turn the roadmap into concrete tasks and checklists.
4. On-device agent: Codex CLI agent implements changes, files, and glue code.
5. Tight loop: validate on hardware, adjust the plan, iterate quickly.

## Follow Along
- Build updates and commentary on X: [add handle/link]
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
