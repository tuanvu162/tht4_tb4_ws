"""Check lidar saturation detection without suppressing actual close returns."""

import math
from types import SimpleNamespace

import pytest

from tht5_tb4_perception.scan_health import scan_problem


def scan(ranges):
    return SimpleNamespace(ranges=ranges, range_min=0.164, range_max=12.0)


def test_saturated_gazebo_scan_is_rejected():
    assert 'range_min' in scan_problem(scan([0.1640000045] * 640))


@pytest.mark.parametrize('ranges', [[], [float('nan')] * 640, [0.0] * 640])
def test_empty_or_invalid_scan_is_rejected(ranges):
    assert scan_problem(scan(ranges))


def test_real_close_obstacle_with_other_returns_is_preserved():
    assert not scan_problem(scan([0.164, 0.18, 0.2, 1.5, math.inf]))


def test_open_space_scan_is_valid():
    assert not scan_problem(scan([math.inf] * 640))
