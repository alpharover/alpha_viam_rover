import math
import pathlib
import sys

import pytest

# Ensure repo root is on path to import local lib/
ROOT = pathlib.Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from lib.driver_station_mapping import scale_norm_to_range, scale_norm_twist


@pytest.mark.unit
def test_scale_norm_to_range_zero_returns_zero():
    assert scale_norm_to_range(0.0, min_abs=0.72, max_abs=1.0, deadband=0.08) == 0.0


@pytest.mark.unit
def test_scale_norm_to_range_inside_deadband_returns_zero():
    assert scale_norm_to_range(0.05, min_abs=0.72, max_abs=1.0, deadband=0.08) == 0.0
    assert scale_norm_to_range(-0.079, min_abs=0.72, max_abs=1.0, deadband=0.08) == 0.0


@pytest.mark.unit
def test_scale_norm_to_range_endpoints_map_to_max_abs():
    out_pos = scale_norm_to_range(1.0, min_abs=0.72, max_abs=1.0, deadband=0.08)
    out_neg = scale_norm_to_range(-1.0, min_abs=0.72, max_abs=1.0, deadband=0.08)
    assert math.isclose(out_pos, 1.0, rel_tol=1e-12)
    assert math.isclose(out_neg, -1.0, rel_tol=1e-12)


@pytest.mark.unit
def test_scale_norm_to_range_just_outside_deadband_at_least_min_abs():
    out = scale_norm_to_range(0.081, min_abs=0.72, max_abs=1.0, deadband=0.08)
    assert out >= 0.72
    assert out <= 1.0


@pytest.mark.unit
def test_scale_norm_to_range_clamps_norm_and_bounds():
    # norm clamps to [-1, 1]
    assert scale_norm_to_range(99.0, min_abs=0.1, max_abs=0.2) == 0.2
    assert scale_norm_to_range(-99.0, min_abs=0.1, max_abs=0.2) == -0.2

    # max_abs below min_abs is tolerated (treated as equal)
    out = scale_norm_to_range(1.0, min_abs=0.5, max_abs=0.1)
    assert out == 0.5


@pytest.mark.unit
def test_scale_norm_twist_maps_both_axes():
    lin, ang = scale_norm_twist(
        lin_norm=1.0,
        ang_norm=-1.0,
        min_speed=0.72,
        min_turn=4.86,
        max_speed=1.0,
        max_turn=6.0,
        deadband=0.08,
        expo=1.0,
    )
    assert math.isclose(lin, 1.0, rel_tol=1e-12)
    assert math.isclose(ang, -6.0, rel_tol=1e-12)


@pytest.mark.unit
def test_scale_norm_twist_turn_is_smooth_near_center():
    lin, ang = scale_norm_twist(
        lin_norm=0.0,
        ang_norm=0.081,
        min_speed=0.72,
        min_turn=4.86,
        max_speed=1.0,
        max_turn=6.5,
        deadband=0.08,
        expo=1.0,
    )
    assert lin == 0.0
    assert 0.0 < ang < 0.1


@pytest.mark.unit
def test_scale_norm_twist_turn_floor_not_applied_when_moving():
    lin, ang = scale_norm_twist(
        lin_norm=0.081,
        ang_norm=0.081,
        min_speed=0.72,
        min_turn=4.86,
        max_speed=1.0,
        max_turn=6.5,
        deadband=0.08,
        expo=1.0,
    )
    assert lin >= 0.72
    assert 0.0 < ang < 1.0
