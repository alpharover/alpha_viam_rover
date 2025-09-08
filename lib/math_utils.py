"""Math utilities for simple robotics calculations.

Functions are hardware-agnostic and safe for unit testing.
"""

from __future__ import annotations

import math


def ticks_to_mps(ticks: float, ticks_per_rev: float, wheel_radius_m: float, dt_s: float) -> float:
    """Convert encoder ticks over a time window to linear velocity in m/s.

    v = (2 * pi * R) * (ticks / ticks_per_rev) / dt

    - ``ticks`` may be negative (direction).
    - ``ticks_per_rev`` and ``dt_s`` must be > 0.
    - ``wheel_radius_m`` must be > 0.
    """
    if ticks_per_rev <= 0:
        raise ValueError("ticks_per_rev must be > 0")
    if wheel_radius_m <= 0:
        raise ValueError("wheel_radius_m must be > 0")
    if dt_s <= 0:
        raise ValueError("dt_s must be > 0")
    circumference = 2.0 * math.pi * wheel_radius_m
    revolutions = ticks / float(ticks_per_rev)
    return (circumference * revolutions) / dt_s


def angle_wrap_rad(angle: float) -> float:
    """Wrap angle in radians to the range [-pi, pi).

    Exact +pi maps to -pi.
    """
    two_pi = 2.0 * math.pi
    wrapped = (angle + math.pi) % two_pi
    return wrapped - math.pi


def clamp(value: float, lo: float, hi: float) -> float:
    """Clamp value to the inclusive range [lo, hi]."""
    if lo > hi:
        lo, hi = hi, lo
    return min(hi, max(lo, value))
