import math
import sys
import pathlib
import pytest

# Ensure repo root is on path to import local lib/
ROOT = pathlib.Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from lib.math_utils import ticks_to_mps, angle_wrap_rad, clamp


@pytest.mark.unit
def test_ticks_to_mps_full_rotation_in_one_second():
    r = 0.06  # meters
    tpr = 20  # ticks per revolution (example)
    v = ticks_to_mps(ticks=tpr, ticks_per_rev=tpr, wheel_radius_m=r, dt_s=1.0)
    assert math.isclose(v, 2 * math.pi * r, rel_tol=1e-6)


@pytest.mark.unit
def test_ticks_to_mps_negative_direction():
    r = 0.05
    tpr = 100
    v = ticks_to_mps(ticks=-50, ticks_per_rev=tpr, wheel_radius_m=r, dt_s=0.5)
    # Half a revolution in 0.5s -> one rev per second, but negative.
    expected = -(2 * math.pi * r)
    assert math.isclose(v, expected, rel_tol=1e-6)


@pytest.mark.unit
def test_angle_wrap_rad_boundaries():
    assert angle_wrap_rad(0.0) == 0.0
    # +pi -> -pi
    assert math.isclose(angle_wrap_rad(math.pi), -math.pi, rel_tol=1e-12)
    # -pi stays -pi
    assert math.isclose(angle_wrap_rad(-math.pi), -math.pi, rel_tol=1e-12)
    # 3*pi -> -pi
    assert math.isclose(angle_wrap_rad(3 * math.pi), -math.pi, rel_tol=1e-12)
    # 4.5*pi -> 0.5*pi
    assert math.isclose(angle_wrap_rad(4.5 * math.pi), 0.5 * math.pi, rel_tol=1e-12)


@pytest.mark.unit
def test_clamp_basic():
    assert clamp(5, 0, 10) == 5
    assert clamp(-1, 0, 10) == 0
    assert clamp(11, 0, 10) == 10
    # inverted bounds tolerated
    assert clamp(5, 10, 0) == 5
