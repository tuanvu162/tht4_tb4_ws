"""Check angular interpretation and wall avoidance transitions."""

import math
from types import SimpleNamespace

import pytest

from tht4_tb4_nav.avoidance import WallAvoidance


def scan(front=3.0, left=3.0, right=3.0, rear=3.0, start=-math.pi):
    ranges = []
    for index in range(360):
        angle = math.atan2(math.sin(start + index * math.pi / 180),
                           math.cos(start + index * math.pi / 180))
        if abs(angle) <= math.radians(40):
            ranges.append(front)
        elif math.radians(40) < angle <= math.radians(110):
            ranges.append(left)
        elif -math.radians(110) <= angle < -math.radians(40):
            ranges.append(right)
        else:
            ranges.append(rear)
    return SimpleNamespace(ranges=ranges, angle_min=start,
                           angle_increment=math.pi / 180,
                           range_min=0.1, range_max=10.0)


@pytest.mark.parametrize('start', [-math.pi, 0.0])
def test_front_wall_stops_and_turns_toward_open_side(start):
    controller = WallAvoidance()
    linear, angular = controller.command(scan(front=0.5, left=2, right=0.4, start=start))
    assert linear == 0
    assert angular > 0


def test_rear_wall_does_not_replace_front_measurement():
    linear, _ = WallAvoidance().command(scan(rear=0.5))
    assert linear > 0


def test_turn_direction_is_held_until_three_clear_scans():
    controller = WallAvoidance()
    assert controller.command(scan(front=0.5, left=0.4, right=2))[1] < 0
    assert controller.command(scan(front=0.7, left=2, right=0.4))[1] < 0
    assert controller.command(scan())[0] == 0
    assert controller.command(scan())[0] == 0
    assert controller.command(scan())[0] > 0


@pytest.mark.parametrize('value', [float('nan'), 0.0, float('-inf')])
def test_unknown_front_stops(value):
    assert WallAvoidance().command(scan(front=value)) == (0, 0)


def test_infinite_returns_mean_open_space():
    assert WallAvoidance().command(scan(front=float('inf')))[0] > 0


def test_wall_inside_rotation_clearance_stops():
    assert WallAvoidance().command(scan(left=0.2)) == (0, 0)


def test_slows_and_steers_away_from_right_wall():
    linear, angular = WallAvoidance().command(scan(front=0.8, right=0.4))
    assert 0 < linear < 0.16
    assert angular > 0


def test_front_contact_backs_out_then_turns():
    controller = WallAvoidance()
    assert controller.command(scan(front=0.25, left=2, right=0.5)) == (-0.06, 0)
    assert controller.command(scan(front=0.4, left=2, right=0.5)) == (-0.06, 0)
    linear, angular = controller.command(scan(front=0.5, left=2, right=0.5))
    assert linear == 0
    assert angular > 0


def test_no_reverse_when_rear_is_blocked_or_unknown():
    assert WallAvoidance().command(scan(front=0.25, rear=0.4)) == (0, 0)
    assert WallAvoidance().command(scan(front=0.25, rear=float('nan'))) == (0, 0)


def test_turtlebot_lidar_rotation_maps_sensor_right_to_robot_front():
    controller = WallAvoidance(scan_yaw=math.pi / 2)
    # Sensor -90 degrees is robot forward; sensor forward is robot left.
    distances = controller.sectors(scan(right=0.5))
    assert distances['front'] == 0.5
    linear, angular = controller.command(scan(right=0.5))
    assert linear == 0
    assert angular != 0
