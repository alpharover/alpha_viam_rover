"""Pure math helpers for the web driver station.

Keep this module ROS-free so it can be unit-tested in CI.
"""

from __future__ import annotations

import math

from lib.math_utils import clamp


def scale_norm_to_range(
    norm: float,
    *,
    min_abs: float,
    max_abs: float,
    deadband: float = 0.08,
    expo: float = 1.0,
) -> float:
    """Scale a normalized command in [-1, 1] into a physical range with stiction floor.

    - Applies a symmetric deadband around 0.
    - Maps (deadband..1] smoothly onto [min_abs..max_abs].
    - Preserves sign.

    This is intended for mapping a joystick axis to linear.x or angular.z where a
    minimum magnitude is required to overcome drivetrain stiction.
    """

    norm = float(clamp(norm, -1.0, 1.0))

    min_abs = max(0.0, float(min_abs))
    max_abs = max(min_abs, float(max_abs))

    deadband = float(clamp(deadband, 0.0, 0.99))

    magnitude = abs(norm)
    if magnitude <= deadband:
        return 0.0

    # Normalize magnitude to [0, 1] after deadband.
    t = (magnitude - deadband) / (1.0 - deadband)
    t = clamp(t, 0.0, 1.0)

    # Exponential response curve (expo > 1 reduces sensitivity near center).
    expo = max(1e-6, float(expo))
    if not math.isclose(expo, 1.0):
        t = t**expo

    scaled = min_abs + t * (max_abs - min_abs)
    return math.copysign(float(scaled), norm)


def scale_norm_twist(
    *,
    lin_norm: float,
    ang_norm: float,
    min_speed: float,
    min_turn: float,
    max_speed: float,
    max_turn: float,
    deadband: float = 0.08,
    expo: float = 1.0,
) -> tuple[float, float]:
    """Map normalized (lin, ang) in [-1, 1] to (linear_x, angular_z)."""

    deadband = float(clamp(deadband, 0.0, 0.99))
    lin_norm = float(clamp(lin_norm, -1.0, 1.0))

    linear_x = scale_norm_to_range(
        lin_norm,
        min_abs=min_speed,
        max_abs=max_speed,
        deadband=deadband,
        expo=expo,
    )

    angular_z = scale_norm_to_range(
        ang_norm,
        min_abs=0.0,
        max_abs=max_turn,
        deadband=deadband,
        expo=expo,
    )
    return linear_x, angular_z
