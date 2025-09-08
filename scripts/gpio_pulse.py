#!/usr/bin/env python3
"""Quick GPIO pulse using pigpio (Phase 1 sanity check).

Usage:
  sudo pigpiod  # ensure daemon is running (or via systemd)
  python3 scripts/gpio_pulse.py --pin 12 --hz 10 --cycles 20
"""
from __future__ import annotations

import argparse
import time

import pigpio  # type: ignore


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pin", type=int, required=True, help="BCM pin number (e.g., 12)")
    ap.add_argument("--hz", type=float, default=10.0, help="Pulse frequency in Hz")
    ap.add_argument("--cycles", type=int, default=20, help="Number of on/off cycles")
    args = ap.parse_args()

    period = 1.0 / max(args.hz, 0.1)
    pi = pigpio.pi()
    if not pi.connected:
        raise SystemExit("pigpio daemon not running. Start with 'sudo pigpiod'.")
    try:
        pi.set_mode(args.pin, pigpio.OUTPUT)
        for _ in range(args.cycles):
            pi.write(args.pin, 1)
            time.sleep(period / 2)
            pi.write(args.pin, 0)
            time.sleep(period / 2)
    finally:
        pi.write(args.pin, 0)
        pi.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
