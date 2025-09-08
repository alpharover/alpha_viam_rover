# Phase 0 — Baseline Bring-up (Evidence)

Complete these steps and attach artifacts.

Checklist

- [ ] Time sync verified (timedatectl + chronyc)
- [ ] `ros2 doctor --report` captured
- [ ] Talker/listener MCAP (10–20 s) recorded in `bags/phase0_talker_listener/`

How to reproduce

1) Time sync
```
timedatectl
chronyc sources -v
```

2) ROS 2 health
```
ros2 doctor --report
```

3) Talker/listener proof bag
```
# Terminal A
ros2 run demo_nodes_cpp talker

# Terminal B
ros2 run demo_nodes_cpp listener

# Terminal C (record 10–20 s)
ros2 bag record -s mcap -o bags/phase0_talker_listener /chatter
```

4) Add links in `AGENTS_PROGRESS.md`

