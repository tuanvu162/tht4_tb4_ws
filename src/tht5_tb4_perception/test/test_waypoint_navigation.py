"""Verify sequential navigation without starting a ROS node or robot."""

import math
from unittest.mock import Mock

from builtin_interfaces.msg import Time
from nav2_simple_commander.robot_navigator import TaskResult
import pytest

from tht5_tb4_perception.waypoint_navigation import goal_pose, run_route, validate_goals


GOALS = [(1.0, 2.0, 0.0), (3.0, 4.0, 1.0), (5.0, 6.0, -1.0)]


def navigator(results):
    node = Mock()
    node.get_clock.return_value.now.return_value.to_msg.return_value = Time()
    node.goToPose.return_value = True
    node.isTaskComplete.return_value = True
    node.getResult.side_effect = results
    return node


def test_success_sends_all_goals_in_order():
    node = navigator([TaskResult.SUCCEEDED] * 3)
    assert run_route(node, GOALS, 180.0)
    assert [(call.args[0].pose.position.x, call.args[0].pose.position.y)
            for call in node.goToPose.call_args_list] == [(1, 2), (3, 4), (5, 6)]


@pytest.mark.parametrize('result', [TaskResult.FAILED, TaskResult.CANCELED, TaskResult.UNKNOWN])
def test_failure_stops_before_next_goal(result):
    node = navigator([TaskResult.SUCCEEDED, result])
    assert not run_route(node, GOALS, 180.0)
    assert node.goToPose.call_count == 2


def test_rejected_goal_stops_route():
    node = navigator([])
    node.goToPose.return_value = False
    assert not run_route(node, GOALS, 180.0)
    assert node.goToPose.call_count == 1


def test_timeout_cancels_without_sending_next_goal():
    node = navigator([])
    node.isTaskComplete.return_value = False
    assert not run_route(node, GOALS, 1.0, now=iter([0.0, 2.0]).__next__)
    node.cancelTask.assert_called_once()
    assert node.goToPose.call_count == 1


def test_bad_scan_prevents_start():
    node = navigator([])
    assert not run_route(node, GOALS, 180.0, health_check=lambda: 'Bad scan')
    node.goToPose.assert_not_called()


def test_scan_failure_during_goal_cancels_route():
    node = navigator([])
    node.isTaskComplete.return_value = False
    checks = iter(['', 'All rays at range_min'])
    assert not run_route(node, GOALS, 180.0, health_check=lambda: next(checks))
    node.cancelTask.assert_called_once()
    assert node.goToPose.call_count == 1


def test_yaw_becomes_normalized_quaternion():
    pose = goal_pose(1, 2, math.pi / 2, Time())
    assert pose.header.frame_id == 'map'
    assert pose.pose.orientation.z == pytest.approx(math.sqrt(0.5))
    assert pose.pose.orientation.w == pytest.approx(math.sqrt(0.5))


@pytest.mark.parametrize('xs,ys,yaws', [
    ([1.0], [2.0], [0.0]),
    ([1.0] * 3, [2.0] * 2, [0.0] * 3),
    ([float('nan')] * 3, [2.0] * 3, [0.0] * 3),
])
def test_bad_goal_arrays_are_rejected(xs, ys, yaws):
    with pytest.raises(ValueError):
        validate_goals(xs, ys, yaws)
