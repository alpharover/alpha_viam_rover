Scope
SocketCAN (Waveshare MCP2515) and CAN‑servo control.

You will

* Enable `can0` via device‑tree overlay (SPI + oscillator/INT pins matching the HAT).
* Bridge CAN frames to ROS 2; integrate device‑specific servo control.

Acceptance

* `can0` up at target bitrate; loopback OK; servo responds to setpoint
* 30‑minute endurance without bus errors
* Evidence: logs/snippets + video or MCAP of commanded motions (if applicable)

Escalation
Bitrate/termination or driver swaps require architect approval.

