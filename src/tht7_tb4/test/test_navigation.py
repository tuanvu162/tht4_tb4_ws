"""Real ROS action transport against a controlled fake Nav2 server.

Requires sourced ROS Humble and a built tht7_tb4 package. No simulator needed.
"""
# flake8: noqa: E402

from collections import deque
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from tht7_tb4.voice_navigation import VoiceNavigation


class FakeNav2(Node):
    def __init__(self):
        super().__init__('fake_nav2_tht7_test')
        self.received = []
        self.results = deque()
        self.release = threading.Event()
        self.reject = False
        self.scan_enabled = True
        self.cancel_seen = False
        self.server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose', self.execute,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=lambda _: GoalResponse.REJECT if self.reject else GoalResponse.ACCEPT,
            cancel_callback=self.cancel_goal)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.amcl = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', latched)
        self.scan = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.command = self.create_publisher(String, '/voice_command', 10)
        self.tf = TransformBroadcaster(self)
        self.statuses = []
        self.status_sub = self.create_subscription(
            String, '/voice_navigation/status',
            lambda msg: self.statuses.append(msg.data), latched)
        self.timer = self.create_timer(0.05, self.sensors)

    def sensors(self):
        stamp = self.get_clock().now().to_msg()
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id, pose.header.stamp = 'map', stamp
        pose.pose.pose.orientation.w = 1.0
        self.amcl.publish(pose)
        transform = TransformStamped()
        transform.header.frame_id, transform.header.stamp = 'map', stamp
        transform.child_frame_id = 'base_link'
        transform.transform.rotation.w = 1.0
        self.tf.sendTransform(transform)
        if self.scan_enabled:
            scan = LaserScan()
            scan.header.frame_id, scan.header.stamp = 'base_link', stamp
            scan.range_min, scan.range_max = 0.1, 10.0
            scan.ranges = [2.0] * 10
            self.scan.publish(scan)

    def cancel_goal(self, _):
        self.cancel_seen = True
        return CancelResponse.ACCEPT

    def execute(self, handle):
        self.received.append(handle.request.pose)
        deadline = time.monotonic() + 8
        while not self.release.is_set() and not handle.is_cancel_requested:
            if time.monotonic() > deadline:
                handle.abort()
                return NavigateToPose.Result()
            time.sleep(0.01)
        if handle.is_cancel_requested:
            handle.canceled()
        elif self.results and self.results.popleft() == GoalStatus.STATUS_ABORTED:
            handle.abort()
        else:
            handle.succeed()
        return NavigateToPose.Result()


@pytest.fixture
def system():
    rclpy.init()
    fake, navigator = FakeNav2(), VoiceNavigation()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(fake)
    executor.add_node(navigator)

    def wait(predicate, timeout=6):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert predicate(), f'Timed out. Statuses: {fake.statuses}'

    try:
        wait(lambda: not navigator.navigation_problem()
             and fake.command.get_subscription_count() > 0)
        yield fake, navigator, wait
    finally:
        fake.release.set()
        if navigator.goal_handle is not None:
            navigator.cancel('test teardown')
            deadline = time.monotonic() + 1
            while navigator.goal_handle is not None and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.02)
        executor.shutdown(timeout_sec=2)
        fake.server.destroy()
        navigator.client.destroy()
        fake.destroy_node()
        navigator.destroy_node()
        rclpy.shutdown()


def test_source_then_destination_only_after_success(system):
    fake, navigator, wait = system
    fake.command.publish(String(data='Di chuyển từ A đến B'))
    wait(lambda: len(fake.received) == 1 and navigator.state == 'active')
    assert fake.received[0].pose.position.x == navigator.by_id['A'].x
    assert navigator.state == 'active'
    # The first action is blocked until released; no second goal yet.
    assert len(fake.received) == 1
    fake.release.set()
    wait(lambda: len(fake.received) == 2 and navigator.state == 'idle')
    assert fake.received[1].pose.position.x == navigator.by_id['B'].x
    wait(lambda: any(status.startswith('SUCCEEDED:') for status in fake.statuses))


def test_abort_source_never_sends_destination(system):
    fake, navigator, wait = system
    fake.results.append(GoalStatus.STATUS_ABORTED)
    fake.release.set()
    fake.command.publish(String(data='Di chuyển từ A đến B'))
    wait(lambda: any(status.startswith('FAILED:') for status in fake.statuses))
    assert len(fake.received) == 1 and navigator.state == 'idle'


def test_unknown_source_dispatches_nothing(system):
    fake, navigator, wait = system
    fake.command.publish(String(data='Di chuyển từ Z đến B'))
    wait(lambda: any(status.startswith('INVALID:') for status in fake.statuses))
    assert not fake.received and navigator.state == 'idle'


def test_rejected_goal_stops_route(system):
    fake, navigator, wait = system
    fake.reject = True
    fake.command.publish(String(data='Di chuyển từ A đến B'))
    wait(lambda: any(status.startswith('REJECTED:') for status in fake.statuses))
    assert not fake.received and navigator.state == 'idle'


def test_busy_then_cancel_clears_destination(system):
    fake, navigator, wait = system
    fake.command.publish(String(data='Di chuyển từ A đến B'))
    wait(lambda: navigator.state == 'active' and len(fake.received) == 1)
    fake.command.publish(String(data='Đi đến C'))
    wait(lambda: any(status.startswith('BUSY:') for status in fake.statuses))
    fake.command.publish(String(data='Dừng lại'))
    wait(lambda: navigator.state == 'idle' and fake.cancel_seen)
    assert len(fake.received) == 1 and not navigator.route
    wait(lambda: any(status.startswith('STOPPED:') for status in fake.statuses))


@pytest.mark.parametrize('reason', ['timeout', 'stale_scan'])
def test_watchdog_cancels_active_route(system, reason):
    fake, navigator, wait = system
    fake.command.publish(String(data='Di chuyển từ A đến B'))
    wait(lambda: navigator.state == 'active' and len(fake.received) == 1)
    if reason == 'timeout':
        navigator.started = time.monotonic() - 200
    else:
        fake.scan_enabled = False
        navigator.last_scan = time.monotonic() - 10
    wait(lambda: navigator.state == 'idle' and fake.cancel_seen)
    assert len(fake.received) == 1 and not navigator.route
